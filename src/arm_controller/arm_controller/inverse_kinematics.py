from rclpy.node import Node
from rclpy.action import ActionServer
import rclpy
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Int16MultiArray, MultiArrayDimension, Float32MultiArray, Int16

import numpy as np

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_matrix, quaternion_matrix
from geometry_msgs.msg import TransformStamped, PoseStamped, PointStamped
import tf2_geometry_msgs
from visualization_msgs.msg import MarkerArray

from action_msgs.msg import GoalStatus
from action_interfaces.action import IK

# "{layout: {dim: [{label: '', size: 0, stride: 0}], data_offset: 0}, data: [12000,12000,12000,12000,12000,12000,500,500,500,500,500,500]}"

ANGLE_LIMITS_LOW = [1000, 3100, 2000, 7000, 3500, 8200]
ANGLE_LIMITS_HIGH = [12050, 21500, 20500, 23000, 16000, 16500]
STRAIGHT = [1000, 12000, 12000, 12000, 12000, 12000]

PICK_READY = [1000, 12000, 5000, 19000, 10000, 12000]
MOVE_W_OBJ = [11050, 12000, 5000, 19000, 10000, 12000]


ANGLE_HOMES = [12000, 12000, 12000, 12000, 12000] #keeping all but the ee
PERIOD = 0.1

DELAY = 5000 #ms

ENCODER_2_DEG = 0.01 * (np.pi/180)
DEG_2_ENCODER = 100 * (180/np.pi)
D1 = 0.065
A2 = 0.101
A3 = 0.094
D5 = 0.137 # this is tip of ee when it is at encoder value of 1000

'''
LX16AServo servo1(&servoBus, 1); // 0-16800 (0-168 degrees)
LX16AServo servo2(&servoBus, 2); // 0-24000 (0-240 degrees)
LX16AServo servo3(&servoBus, 3); // 0-24000 (0-240 degrees)
LX16AServo servo4(&servoBus, 4); // 0-24000 (0-240 degrees)
LX16AServo servo5(&servoBus, 5); // 0-24000 (0-240 degrees)
LX16AServo servo6(&servoBus, 6); // 0-24000 (0-240 degrees)
'''

class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')
        
        self.joint_pos_sub = self.create_subscription(JointState, topic='/servo_pos_publisher', callback=self.joint_pos_cb, qos_profile=10)
        self.joint_cmd_sub = self.create_subscription(Int16MultiArray, '/multi_servo_cmd_sub', callback=self.joint_cmd_cb, qos_profile=10)
        
        self.ik_sub = self.create_subscription(PoseStamped, '/ik_publisher', callback=self.ik_cb, qos_profile=10)
        
        self.kinematics_pub = self.create_publisher(Int16MultiArray, '/kinematic_control', 10)
        
        self.joycon_sub = self.create_subscription(Joy, topic='/joy', callback=self.joy_cb, qos_profile=10)
        self.joint_cmd_pub = self.create_publisher(Int16MultiArray, '/multi_servo_cmd_sub', 10)
        
        
        
        self._action_server = ActionServer(self, IK, 'ik', self.ik_cb_as) #creating an action server!
        
        
        # Initialize the transform listener and assign it a buffer
        self.tf2Buffer = Buffer(cache_time=None)

        #transform listener fills the buffer
        self.listener = TransformListener(self.tf2Buffer, self)
        
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.last_joint_cmd_pub_time = self.get_clock().now()
        
        self.joint_cmd = Int16MultiArray()
        dim = MultiArrayDimension()
        dim.label = ''
        dim.size = 0
        dim.stride = 0
        self.joint_cmd.layout.dim.append(dim)
        self.joy_cmd = Joy()
        
        self.joint_angles = ANGLE_HOMES
        self.joint_times = [1500] * 6
        self.start_time = self.get_clock().now()
        self.init_cnt = 0
        
        self.joint_pos_sub
        
        self.test = np.identity(2)
        
        self.prev_state = []
        
        self.zero = ANGLE_HOMES[4] # they are all the same right now.. but will want to change this!!!!!
        
        
        self.zero_0 = ANGLE_HOMES[0] # bottom joint
        self.zero_1 = ANGLE_HOMES[1] # second joint
        self.zero_2 = ANGLE_HOMES[2]
        self.zero_3 = ANGLE_HOMES[3]
        self.zero_4 = ANGLE_HOMES[4] # top joint before end effector
        
        self.current_qs = [0, (np.pi/2), 0, (np.pi/2), 0] #this is our home for angles. this may need to change!!!!!!!!!!!!!!!!
        
        self.desired_encoder_vals = [12000, 12000, 12000, 12000, 12000]
        
        #BEFORE ANY IK, NEED TO SEND COMMAND HOME. THIS CREATES ALL TRANSFORMS AS WELL
        # self.move_arm(STRAIGHT)
        
        # self.object_sub = self.create_subscription(MarkerArray, '/object_centers', self.obj_cb, 1)
        self.update = True
        
        self.pick_sub = self.create_subscription(PointStamped, '/pick_ik', self.obj_cb, 10)
        self.ik_res = self.create_publisher(Int16, '/ik_res', 1)
        
    def obj_cb(self, msg:PointStamped):
        self.test_this(msg)
        
    
    def ik_cb_as(self, goal_handle):
        self.get_logger().info('Executing goal...') # it is in executing state.. for now, go to succeeded but response may still be fail.
        
        # print(goal_handle.__dir__()) # 'request', 'goal_id', 'is_active', 'is_cancel_requested', 'status', '_update_state', 'execute', 'publish_feedback', 'succeed', 'abort', 'canceled', 'destroy'
        
        goal_handle.success()
        result = IK.Result()
        
        obj = goal_handle.request.goal_point # this is goal from action client
        
        print("Received Goal for IK!")
        
        obj_stamp = obj.header.stamp
        obj_frame = obj.header.frame_id
        
        if self.tf2Buffer.can_transform('arm_base', obj_frame, obj_stamp):
            transform = self.tf2Buffer.lookup_transform('arm_base', obj_frame, obj_stamp)
            
            trans_obj = tf2_geometry_msgs.do_transform_point(obj.point, transform)
            
            #at this point should have Pose object
            
            if (trans_obj.point.z < -0.114) or (trans_obj.point.x > 0.290) or (trans_obj.point.y > 0.210) or (trans_obj.point.y < -0.214):
                # if it is below floor, too far ahead, too far left or right
                print("Outside of reachable workspace!!")
                result.result = 1
                return result
            
            #in bounds so let's process! 
            
            print("Object within workspace. Moving arm to init position...")
            
            self.move_arm(PICK_READY)
            time_delay = self.get_clock().now()
            
            while ((self.get_clock().now() - time_delay).nanoseconds / 1e6 <= DELAY):
                continue
            
            print("Done moving arm, now onto ik!")
            
            # this just takes the rotation matrix of the pose and keeps it. This will change when we know PoseStamped. Currently have PointStamped
            # trans_obj_r = self.t1 @ self.t2 @ self.t3 @ self.t4 @ self.t5
            # trans_obj_r = quaternion_matrix([trans_obj.orientation.x, trans_obj.orientation.y, trans_obj.orientation.z, trans_obj.orientation.w])
            
            trans_obj_r = np.array([[1.0, 0.0, 0.0, 0.0],
                                    [0.0, -1.0, 0.0, 0.0],
                                    [0.0, 0.0, -1.0, 0.0],
                                    [0.0, 0.0, 0.0, 1.0]])
            
            trans_obj_r[0,3] = trans_obj.point.x
            trans_obj_r[1,3] = trans_obj.point.y
            trans_obj_r[2,3] = trans_obj.point.z + 0.025 #want to be slightly higher
            
            trans_obj_r[2,3] = -0.059

            
            desired_pose = np.array(trans_obj_r).reshape((4,4))
            
            r_des = desired_pose[0:3, 0:3]
            position_des = [desired_pose[0,3],desired_pose[1,3], desired_pose[2,3]]            
            
            print("Solving now...")    
                
            res , des_qs = self.solve_ik(position_des, r_des, self.current_qs)
            
            if not res:
                print("IK did not converge in time!")
                result.result = 1
                return result
            
            des_qs = [round(elem,2) for elem in des_qs]
                
            print([elem* (180/np.pi) for elem in des_qs])
            
            self.desired_encoder_vals = self.angle_to_encoder(des_qs)
            
            # print(self.desired_encoder_vals)
            
            print("Feasible: {}".format(self.sol_feasbile()))
            
            if not self.sol_feasbile():
                print("Not feasible so exiting!")
                result.result = 1
                return result
            
            
            #this will move the arm
            self.kinematic_go()
            time_delay = self.get_clock().now()
            while ((self.get_clock().now() - time_delay).nanoseconds / 1e6 <= DELAY):
                continue
            
                       
            # close ee
            self.kinematic_go(ee_value=11050)
            time_delay = self.get_clock().now()
            while ((self.get_clock().now() - time_delay).nanoseconds / 1e6 <= DELAY):
                continue
            
            
            # move to carrying position
            self.move_arm(MOVE_W_OBJ)
            time_delay = self.get_clock().now()
            while ((self.get_clock().now() - time_delay).nanoseconds / 1e6 <= DELAY):
                continue
            
        
        else:
            print("No transform available!")
            result.result = 1
            return result
        
        
        #return success from action server
        result.result = 0
        return result
    
    
    
    
    
    def test_this(self, p):
        self.get_logger().info('Executing goal...') # it is in executing state.. for now, go to succeeded but response may still be fail.
        
        # print(goal_handle.__dir__()) # 'request', 'goal_id', 'is_active', 'is_cancel_requested', 'status', '_update_state', 'execute', 'publish_feedback', 'succeed', 'abort', 'canceled', 'destroy'
        
        res_send = Int16()

        print("Received Goal for IK!")
        
        obj_stamp = p.header.stamp
        obj_frame = p.header.frame_id
        print(obj_frame)
        
        if self.tf2Buffer.can_transform('arm_base', obj_frame, obj_stamp):
            transform = self.tf2Buffer.lookup_transform('arm_base', obj_frame, obj_stamp)
            
            # p = PointStamped()
            # p.point.x = self.obj_positionx
            # p.point.y = self.obj_positiony
            # p.point.z = self.obj_positionz
            
            self.update = True
            
            trans_obj = tf2_geometry_msgs.do_transform_point(p, transform)
            
            #at this point should have Pose object

            trans_obj.point.z = -0.059
            print("Point is at:")
            print(trans_obj.point)
            
            if (trans_obj.point.z < -0.114) or (trans_obj.point.x > 0.310) or (trans_obj.point.y > 0.210) or (trans_obj.point.y < -0.214): #or (trans_obj.point.x > 0.290)
                # if it is below floor, too far ahead, too far left or right
                print("Outside of reachable workspace!!")
                res_send.data = 1
                self.ik_res.publish(res_send)
                return
            
            #in bounds so let's process! 
            
            print("Object within workspace. Moving arm to init position...")
            
            self.move_arm(PICK_READY)
            time_delay = self.get_clock().now()
            
            while ((self.get_clock().now() - time_delay).nanoseconds / 1e6 <= DELAY):
                continue
            
            print("Done moving arm, now onto ik!")
            
            # this just takes the rotation matrix of the pose and keeps it. This will change when we know PoseStamped. Currently have PointStamped
            # trans_obj_r = self.t1 @ self.t2 @ self.t3 @ self.t4 @ self.t5
            # trans_obj_r = quaternion_matrix([trans_obj.orientation.x, trans_obj.orientation.y, trans_obj.orientation.z, trans_obj.orientation.w])
            
            trans_obj_r = np.array([[1.0, 0.0, 0.0, 0.0],
                                    [0.0, -1.0, 0.0, 0.0],
                                    [0.0, 0.0, -1.0, 0.0],
                                    [0.0, 0.0, 0.0, 1.0]])
            
            trans_obj_r[0,3] = trans_obj.point.x
            trans_obj_r[1,3] = trans_obj.point.y
            trans_obj_r[2,3] = trans_obj.point.z

            
            desired_pose = np.array(trans_obj_r).reshape((4,4))
            
            r_des = desired_pose[0:3, 0:3]
            position_des = [desired_pose[0,3],desired_pose[1,3], desired_pose[2,3]]            
            
            print("Solving now...")    
                
            res , des_qs = self.solve_ik(position_des, r_des, self.current_qs)
            
            if not res:
                print("IK did not converge in time!")
                res_send.data = 1
                self.ik_res.publish(res_send)
                return
            
            des_qs = [round(elem,2) for elem in des_qs]
                
            print([elem* (180/np.pi) for elem in des_qs])
            
            self.desired_encoder_vals = self.angle_to_encoder(des_qs)
            
            # print(self.desired_encoder_vals)
            
            print("Feasible: {}".format(self.sol_feasbile()))
            
            if not self.sol_feasbile():
                print("Not feasible so exiting!")
                res_send.data = 1
                self.ik_res.publish(res_send)
                return
            
            
            #this will move the arm
            self.kinematic_go()
            time_delay = self.get_clock().now()
            while ((self.get_clock().now() - time_delay).nanoseconds / 1e6 <= DELAY):
                continue
            
                       
            # close ee
            self.kinematic_go(ee_value=11050)
            time_delay = self.get_clock().now()
            while ((self.get_clock().now() - time_delay).nanoseconds / 1e6 <= DELAY):
                continue
            
            
            # move to carrying position
            self.move_arm(MOVE_W_OBJ)
            time_delay = self.get_clock().now()
            while ((self.get_clock().now() - time_delay).nanoseconds / 1e6 <= DELAY):
                continue
            
        
        else:
            print("No transform available!")
            self.update = True
            res_send.data = 1
            self.ik_res.publish(res_send)
            return
        
        
        #return success from action server
        print("Success!")
        self.update = True
        res_send.data = 0
        self.ik_res.publish(res_send)
        return
    
    
    
    def joy_cb(self, msg:Joy):
        # axes: left-stick horizontal, left-stick verticle, LT, right-stick horizontal, right-stick verticle, RT, left-cross horizontal, left-cross verticle
        # buttons: A, B, X, Y, LB, RB, SACK, START
        self.joy_cmd = msg
        
        if self.joy_cmd.buttons[1]: # red button is pressed.. RESET!!!!!!!!
        
            self.move_arm(STRAIGHT)
            print("Resetting home")
            
        
        if self.joy_cmd.buttons[2]: # blue button is pressed.. PICKUP CONFIG
        
            self.move_arm(PICK_READY)
            print("Init pickup")
        
        # if self.joy_cmd.buttons[3]: # if Y is pressed. Start IK
            
        #     self.update = False
        #     self.test_this()
            

            
        #     # desired_pose = np.array([[5.11022852e-01,  2.71393271e-02,  8.59138581e-01,  3.03887530e-01], #straight out, but down a lot.
        #     #                          [ 5.30331184e-02, -9.98592754e-01, -2.07850033e-16, -4.49749144e-17],
        #     #                          [8.57929562e-01,  4.55627981e-02, -5.11743000e-01, -5.34450878e-03],
        #     #                          [0,          0,          0,          1 ]]).reshape((4,4))
            
            
        #     desired_pose = np.array([[0.25822102,  0.22619024,  0.93923366,  0.3046841], #down a lot, and to the right.
        #                              [ 0.11033404, -0.97274864,  0.20392762,  0.0661534],
        #                              [0.95976471,  0.05097104, -0.27614063,  0.07434399],
        #                              [0,          0,          0,          1 ]]).reshape((4,4))
            
            
            
        #     desired_pose = np.array([[9.45630384e-01,  3.04698070e-01,  1.13764065e-01,  1.97579641e-01], #down to pickup an object
        #                             [ 3.02516728e-01, -9.52448994e-01,  3.63942753e-02,  6.32077263e-02],
        #                             [1.19443734e-01,  3.57408181e-16, -9.92840971e-01, -8.67390514e-02],
        #                             [0,          0,          0,          1 ]]).reshape((4,4))
            
            
            
        #     desired_pose = np.array([[0.90094339,  0.27990731,  0.33159146,  0.20733415], #down to pickup an object to the left
        #                             [ 0.3201068,  -0.94461409, -0.07235926, -0.04524406],
        #                             [0.29297208,  0.17133628, -0.94064406, -0.07432066],
        #                             [0,          0,          0,          1 ]]).reshape((4,4))
            
            
            
        #     # desired_pose = np.array([[8.37858517e-01, -4.03545296e-01,  3.67619776e-01,  2.31521143e-01], #down and far left (when looking at robot) on one side to pick object
        #     #                         [ -3.69539637e-01, -9.14959668e-01, -1.62139640e-01, -1.02112991e-01],
        #     #                         [4.01787957e-01,  3.49760316e-16, -9.15732733e-01, -9.33479706e-02],
        #     #                         [0,          0,          0,          1 ]]).reshape((4,4))
            
            
        #     desired_pose = np.array([[0.83803405,  0.49903877,  0.22058841,  0.17665445], #far on the right (when looking at robot) and close to body
        #                             [ 0.47165006, -0.86583697,  0.16695078,  0.13369967],
        #                             [0.27430852, -0.0358699,  -0.96097252, -0.09478877],
        #                             [0,          0,          0,          1 ]]).reshape((4,4))
            
            
            
        #     r_des = desired_pose[0:3, 0:3]
        #     position_des = [desired_pose[0,3],desired_pose[1,3], desired_pose[2,3]]
            
        #     print(r_des)
        #     print(position_des)
            
        #     print("Solving now...")
            
        #     res , des_qs = self.solve_ik(position_des, r_des, self.current_qs)
        
        #     if not res:
        #         print("IK did not converge in time!")
        #         return
            
        #     des_qs = [round(elem,2) for elem in des_qs]
            
        #     print([elem* (180/np.pi) for elem in des_qs])
            
        #     self.desired_encoder_vals = self.angle_to_encoder(des_qs)
            
        #     print(self.desired_encoder_vals)
            
        #     print("Feasible: {}".format(self.sol_feasbile()))
            
        
        # if self.joy_cmd.buttons[0]: # if A is pressed. Move arm to position!
            
        #     if self.sol_feasbile():
        #         self.kinematic_go()
        #     else:
        #         print("It ain't feasible so I ain't goin!")
    
    def joint_cmd_cb(self, msg:Int16MultiArray):
        assert len(msg.data) == 12
        
        self.joint_pos = [msg.data[5],msg.data[4],msg.data[3],msg.data[2],msg.data[1]]
        
        
        if self.prev_state == self.joint_pos:
            return
        
        self.q1 = (self.joint_pos[0] - self.zero) * ENCODER_2_DEG
        self.q2 = ((self.joint_pos[1] - self.zero) * ENCODER_2_DEG) + (np.pi/2) # adding this because it starts 90 degrees positive
        self.q3 = ((self.joint_pos[2] - self.zero) * ENCODER_2_DEG)
        self.q4 = ((self.joint_pos[3] - self.zero) * ENCODER_2_DEG) + (np.pi/2) # adding this because it starts 90 degrees positive
        self.q5 = ((self.joint_pos[4] - self.zero) * ENCODER_2_DEG)
        
        self.prev_state = self.joint_pos
        self.current_qs = [self.q1, self.q2, self.q3, self.q4, self.q5]
        
        self.t1 = self.homo_trans(self.q1, np.pi/2, 0, D1)
        self.t2 = self.homo_trans(self.q2, np.pi, A2, 0)
        self.t3 = self.homo_trans(self.q3, np.pi, A3, 0)
        self.t4 = self.homo_trans(self.q4, np.pi/2, 0, 0)
        self.t5 = self.homo_trans(self.q5, 0, 0, D5) 
        
        print(self.t1 @ self.t2 @ self.t3 @ self.t4 @ self.t5)
        print()
        print()
        
        self.broadcast_transform(self.t1, 1)
        self.broadcast_transform(self.t1 @ self.t2, 2)
        self.broadcast_transform(self.t1 @ self.t2 @ self.t3, 3)
        self.broadcast_transform(self.t1 @ self.t2 @ self.t3 @ self.t4, 4)
        self.broadcast_transform(self.t1 @ self.t2 @ self.t3 @ self.t4 @ self.t5, 5)
        
        
    def broadcast_transform(self, trans, joint_num):
        """
        Broadcasts each joint pose
        """

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'arm_base'
        t.child_frame_id = 'joint_' + str(joint_num)

        t.transform.translation.x = trans[0,3]   
        t.transform.translation.y = trans[1,3]
        t.transform.translation.z = trans[2,3]

        q = quaternion_from_matrix(trans)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        if(joint_num == 5):
            print("Rotation")
            print(t.transform.rotation)
            print()
            print("Position")
            print(t.transform.translation)

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
        
    def homo_trans(self, theta, alpha, a, d):
        # theta = np.deg2rad(theta)
        return np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                        [0, np.sin(alpha), np.cos(alpha), d],
                        [0, 0, 0, 1]])
        
        
        
    
    def ik_cb(self, msg:PoseStamped):
        
        #BEFORE THIS IS EVEN CALLED, NEED TO INIT THE TRANSFORMS FOR T1, T2, etc...
        
        print("Received Message for IK!")
        
        obj_stamp = msg.header.stamp
        obj_frame = msg.header.frame_id
        
        if self.tf2Buffer.can_transform('arm_base', obj_frame, obj_stamp):
            transform = self.tf2Buffer.lookup_transform('arm_base', obj_frame, obj_stamp)
            
            trans_obj = tf2_geometry_msgs.do_transform_pose(msg.pose, transform)
            
            #at this point should have Pose object
            
            if (trans_obj.position.z < -0.114) or (trans_obj.position.x > 0.290) or (trans_obj.position.y > 0.210) or (trans_obj.position.y < -0.214):
                # if it is below floor, too far ahead, too far left or right
                print("Outside of reachable workspace!!")
                return
            
            #in bounds so let's process! 
            
            trans_obj_r = quaternion_matrix([trans_obj.orientation.x, trans_obj.orientation.y, trans_obj.orientation.z, trans_obj.orientation.w])
            
            trans_obj_r[0,3] = trans_obj.position.x
            trans_obj_r[1,3] = trans_obj.position.y
            trans_obj_r[2,3] = trans_obj.position.z

            
            desired_pose = np.array(trans_obj_r).reshape((4,4))
            
            r_des = desired_pose[0:3, 0:3]
            position_des = [desired_pose[0,3],desired_pose[1,3], desired_pose[2,3]]
            
            # print(r_des)
            # print(position_des)
            
            
            print("Object within workspace. Moving arm to init position...")
            
            self.joint_angles = [1000, 12000, 5000, 19000, 10000, 12000] #[4200, 12000, 4800, 18574, 11151, 12000]
            self.joint_times = [DELAY] * 6
            self.publish_joint_cmd(self.joint_angles, self.joint_times)
            time_delay = self.get_clock().now()
            
            while ((self.get_clock().now() - time_delay).nanoseconds / 1e6 <= DELAY):
                continue
            
            print("Done moving arm, now onto ik!")
            
            print("Solving now...")    
                
            res , des_qs = self.solve_ik(position_des, r_des, self.current_qs)
            
            if not res:
                print("IK did not converge in time!")
                return
            
            des_qs = [round(elem,2) for elem in des_qs]
                
            print([elem* (180/np.pi) for elem in des_qs])
            
            self.desired_encoder_vals = self.angle_to_encoder(des_qs)
            
            # print(self.desired_encoder_vals)
            
            print("Feasible: {}".format(self.sol_feasbile()))
            
            if self.sol_feasbile():
            
                vals = Int16MultiArray()
                vals.data = self.desired_encoder_vals

                # ADD THIS WHEN WANT TO MOVE THE ARM
                # self.kinematics_pub.publish(vals)
            else:
                print("Not feasible so exiting!")
                return
        
        else:
            print("No transform available!")
            return
  
        
        
    
    def angle_to_encoder(self, angles):
        
        encoder_val = []
        encoder_val.append((angles[0]*DEG_2_ENCODER) + self.zero)
        encoder_val.append(((angles[1] - (np.pi/2))* DEG_2_ENCODER) + self.zero)
        encoder_val.append((angles[2]*DEG_2_ENCODER) + self.zero)
        encoder_val.append(((angles[3] - (np.pi/2))*DEG_2_ENCODER) + self.zero)
        encoder_val.append((angles[4]*DEG_2_ENCODER) + self.zero)
        
        return [int(elem) for elem in encoder_val]
        
        
        
    def get_jacob (self, t1, t2, t3, t4, t5):

        z0 = np.array([[0],
                    [0],
                    [1]])
        p0 = np.array([[0],
                    [0],
                    [0],
                    [1]])
        
        pe = (t1 @ t2 @ t3 @ t4 @ t5 @ p0)[:3]

        p0_cross = np.cross(z0, pe, axis=0)
        c1 = np.vstack((p0_cross, z0))


        z1 = t1[0:3,0:3] @ z0
        p1 = (t1 @ p0)[:3]

        p1_cross = np.cross(z1, pe - p1, axis=0)
        c2 = np.vstack((p1_cross, z1))


        z2 = (t1 @ t2)[0:3,0:3] @ z0
        p2 = (t1 @ t2 @ p0)[:3]

        p2_cross = np.cross(z2, pe - p2, axis=0)
        c3 = np.vstack((p2_cross, z2))


        z3 = (t1 @ t2 @ t3)[0:3,0:3] @ z0
        p3 = (t1 @ t2 @ t3 @ p0)[:3]
        
        p3_cross = np.cross(z3, pe - p3, axis=0)
        c4 = np.vstack((p3_cross, z3))


        z4 = (t1 @ t2 @ t3 @ t4)[0:3,0:3] @ z0
        p4 = (t1 @ t2 @ t3 @ t4 @ p0)[:3]

        p4_cross = np.cross(z4, pe - p4, axis=0)
        c5 = np.vstack((p4_cross, z4))

        jacob = np.hstack((c1, c2, c3, c4, c5))

        return jacob



    def R_error (self, R, R_est):

        R = np.array(R)

        n1 = R[:,0]
        o1 = R[:,1]
        a1 = R[:,2]

        n2 = R_est[:,0]
        o2 = R_est[:,1]
        a2 = R_est[:,2]

        error = 0.5*(np.cross(n1, n2) + np.cross(o1, o2) + np.cross(a1, a2)) # ALESSANDRO ADDED THIS

        print(error.shape)

        return np.reshape(error, (3,1))


    def solve_ik(self, point, R, joint_positions, timeout=10000):
        
        '''
        This takes x,y,z point, rotation matrix R, and the current joint_positions
        
        '''
        x = point[0]
        y = point[1]
        z = point[2]
        q = joint_positions #this has to be 5 elements. These are the current join positions

        q = np.reshape(np.asarray(q), (5,1))
        
        X = np.array([[x],
                    [y],
                    [z],
                    [np.arctan2(R[1][2],R[0][2])],
                    [np.arctan2(np.sqrt((R[0][2]**2) + (R[1][2]**2)),R[2][2])],
                    [np.arctan2(R[2][1],-R[2][0])]])
        
        X = np.array([[x],
                    [y],
                    [z]])


        max_error = 1
        tolerance = 0.5
        
        delay_time = self.get_clock().now()
        
    
        # while loop breaks when error below tolerance or timeout occurs. Default 10 seconds
        while(max_error >= tolerance) and ((self.get_clock().now() - delay_time).nanoseconds / 1e6 <= timeout):
            
            q1 = q[0,0]
            q2 = q[1,0]
            q3 = q[2,0]
            q4 = q[3,0]
            q5 = q[4,0]
    
            t1 = self.t1
            t2 = self.t2
            t3 = self.t3
            t4 = self.t4
            t5 = self.t5
            
            
            t1 = self.homo_trans(q1, np.pi/2, 0, D1)
            t2 = self.homo_trans(q2, np.pi, A2, 0)
            t3 = self.homo_trans(q3, np.pi, A3, 0)
            t4 = self.homo_trans(q4, np.pi/2, 0, 0)
            t5 = self.homo_trans(q5, 0, 0, D5) 

            K = t1 @ t2 @ t3 @ t4 @ t5

            R_est = K[0:3,0:3]

            estimate = np.array([[K[0,3]],
                                [K[1,3]],
                                [K[2,3]],
                                [np.arctan2(R_est[1,2],R_est[0,2])],
                                [np.arctan2(np.sqrt((R_est[0,2]**2) + (R_est[1,2]**2)),R_est[2,2])],
                                [np.arctan2(R_est[2,1],-R_est[2,0])]])
            
            estimate = np.array([[K[0,3]],
                                [K[1,3]],
                                [K[2,3]]])

            error_X = estimate - X

            error_R = self.R_error(R, R_est)

            error = np.concatenate((error_X, error_R))

            jacob = self.get_jacob(t1, t2, t3, t4, t5)

            e_theta = np.linalg.pinv(jacob) @ error

            q = q - e_theta

            max_error = np.linalg.norm(error)

        
            
             
        q = np.reshape(q,(5,))
        q = q.tolist()
        
        if ((self.get_clock().now() - delay_time).nanoseconds / 1e6 > timeout):
            print(max_error)
            return False , q

        #return the desired joint params
        print(max_error)
        return True , q



    def move_arm(self, vals):
        self.joint_angles = vals
        self.joint_times = [DELAY] * 6
        self.publish_joint_cmd(self.joint_angles, self.joint_times)


    def sol_feasbile(self):
        
        # CURRENTLY I DID NOT ADD THE GRIPPER SO I PASS TO THE NEXT VALUE! MAY WANT TO CHANGE
        encoder_vals = self.desired_encoder_vals[::-1]
        for i in range(5):
            if (encoder_vals[i] < ANGLE_LIMITS_LOW[i + 1]) or (encoder_vals[i] > ANGLE_LIMITS_HIGH[i + 1]):
                return False
        return True



    def kinematic_go(self, ee_value=1000):
        
        self.joint_angles = [ee_value, self.desired_encoder_vals[4], self.desired_encoder_vals[3], self.desired_encoder_vals[2], self.desired_encoder_vals[1], self.desired_encoder_vals[0]]
        self.joint_times = [DELAY] * 6
        print("Moving arm to {}".format(self.joint_angles))
        
        # return
        self.publish_joint_cmd(self.joint_angles, self.joint_times)
        return
        
        
    def joint_pos_cb(self, msg:JointState):
        assert len(msg.name) == 6
        
        self.positions = msg.position
        self.velocities = msg.velocity
        self.efforts = msg.effort
        
    def publish_joint_cmd(self, joint_angles, joint_times):
        self.joint_cmd.data = []
        assert len(joint_angles) == len(joint_times) and len(joint_times) == 6
        
        dt = (self.get_clock().now() - self.last_joint_cmd_pub_time).nanoseconds / 1e9 # in seconds

        if dt < PERIOD:
            return 
        self.last_joint_cmd_pub_time = self.get_clock().now()

        # print(dt)
        
        for i in range(6):
            if joint_angles[i] < ANGLE_LIMITS_LOW[i]:
                joint_angles[i] = ANGLE_LIMITS_LOW[i]
            elif joint_angles[i] > ANGLE_LIMITS_HIGH[i]:
                joint_angles[i] = ANGLE_LIMITS_HIGH[i]
            self.joint_cmd.data.append(joint_angles[i])
            
        for i in range(6):
            if joint_times[i] < 1000:
                joint_times[i] = 1000
            self.joint_cmd.data.append(joint_times[i])
                
        print(self.joint_cmd)
            
        self.joint_cmd_pub.publish(self.joint_cmd)

def main():
    print('Hi from inverse_kinematics.')
    
    rclpy.init()
    
    inverse_kinematics = InverseKinematics()
    
    rclpy.spin(inverse_kinematics)
    
    inverse_kinematics.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
