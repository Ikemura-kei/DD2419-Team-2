from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Int16MultiArray, MultiArrayDimension, Float32MultiArray, Int16, Bool

import numpy as np

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_matrix, quaternion_matrix
from geometry_msgs.msg import TransformStamped, PoseStamped, PointStamped
import tf2_geometry_msgs
from visualization_msgs.msg import MarkerArray


ANGLE_LIMITS_LOW = [1000, 3100, 2000, 7000, 3500, 8200]
ANGLE_LIMITS_HIGH = [12050, 21500, 20500, 23000, 16000, 16500]
STRAIGHT = [1000, 12000, 12000, 12000, 12000, 12000]

PICK_READY = [1000, 12000, 5000, 19000, 10000, 12000]
MOVE_W_OBJ = [11050, 12000, 12000, 17000, 12000, 12000]

# ANGLE_PLACE_DOWN = [11050, 12000, 12000, 17000, 10000, 12000]
ANGLE_OPEN_GRIPPER = [4200, 12000, 12000, 17000, 12000, 12000]


ANGLE_HOMES = [12000, 12000, 12000, 12000, 12000] #keeping all but the end effector(ee)
PERIOD = 0.1

DELAY = 3500 #ms

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
        
        # arm motor subscribers and publisher
        self.joint_pos_sub = self.create_subscription(JointState, topic='/servo_pos_publisher', callback=self.joint_pos_cb, qos_profile=10)
        self.joint_cmd_sub = self.create_subscription(Int16MultiArray, '/multi_servo_cmd_sub', callback=self.joint_cmd_cb, qos_profile=10)
        self.joint_cmd_pub = self.create_publisher(Int16MultiArray, '/multi_servo_cmd_sub', 10)
        
        # joystick subscriber
        self.joycon_sub = self.create_subscription(Joy, topic='/joy', callback=self.joy_cb, qos_profile=10)
        
        # ik request and response topics. Future can be implemented as service
        self.pick_sub = self.create_subscription(PointStamped, '/pick_ik', self.pick_cb, 10)
        self.ik_res = self.create_publisher(Bool, '/ik_res', 10)
        
        # drop request and reponse topics. Future can be implemented as service
        self.drop_sub = self.create_subscription(Bool, '/drop_obj', self.drop_obj_cb, 10)
        self.drop_res_pub = self.create_publisher(Bool, '/drop_obj_res', 10)
        
        # TF initialization
        self.tf2Buffer = Buffer(cache_time=None)
        self.listener = TransformListener(self.tf2Buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize variables
        self.last_joint_cmd_pub_time = self.get_clock().now()
        self.joint_cmd = Int16MultiArray()
        dim = MultiArrayDimension()
        dim.label = ''
        dim.size = 0
        dim.stride = 0
        self.joint_cmd.layout.dim.append(dim)
        
        self.joint_angles = ANGLE_HOMES
        self.joint_times = [1500] * 6
        
        self.prev_state = []
        
        self.zero = ANGLE_HOMES[4] # they are all the same right now.. but will want to change this
        
        self.zero_0 = ANGLE_HOMES[0] # bottom joint
        self.zero_1 = ANGLE_HOMES[1] # second joint
        self.zero_2 = ANGLE_HOMES[2]
        self.zero_3 = ANGLE_HOMES[3]
        self.zero_4 = ANGLE_HOMES[4] # top joint before end effector
        
        self.current_qs = [0, (np.pi/2), 0, (np.pi/2), 0] #this is our home for angles. this may need to change
        
        self.desired_encoder_vals = [12000, 12000, 12000, 12000, 12000]
        
        self.update = True
        
        self.init_joints()
        
        self.timer = self.create_timer(0.1, self.publish_joints)
        
    
    def drop_obj_cb(self, msg:Bool):
        if msg.data:
            self.drop_object()
        
    
    def drop_object(self):
        print("Dropping object!")
        self.move_arm(ANGLE_OPEN_GRIPPER)
        time_delay = self.get_clock().now()
        while ((self.get_clock().now() - time_delay).nanoseconds / 1e6 <= (DELAY + 1000)):
            continue
        
        drop = Bool()
        drop.data = True
        self.drop_res_pub.publish(drop)
        
        
    def pick_cb(self, msg:PointStamped):
        self.pick_from_point(msg)
    
    
    def pick_from_point(self, p):
        
        res_send = Bool()
        print("Received Goal for IK!")
        
        obj_stamp = p.header.stamp
        obj_stamp = rclpy.time.Time()
        obj_frame = p.header.frame_id
        print(obj_frame)
        
        print("Checking transform...")
        timeout = rclpy.duration.Duration(seconds=0.5)
        if self.tf2Buffer.can_transform('arm_base', obj_frame, obj_stamp, timeout):
            transform = self.tf2Buffer.lookup_transform('arm_base', obj_frame, obj_stamp, timeout)
            
            self.update = True
            
            trans_obj = tf2_geometry_msgs.do_transform_point(p, transform)
            
            #at this point should have Pose object

            trans_obj.point.z = -0.0729
            print("Point is at:")
            print(trans_obj.point)
            
            if (trans_obj.point.z < -0.114) or (trans_obj.point.x > 0.310) or (trans_obj.point.y > 0.210) or (trans_obj.point.y < -0.214): #or (trans_obj.point.x > 0.290)
                # if it is below floor, too far ahead, too far left or right
                print("Outside of reachable workspace!!")
                res_send.data = False
                self.ik_res.publish(res_send)
                return
            
            #in bounds so let's process! 
            
            print("Object within workspace. Moving arm to init position...")
            
            self.move_arm(PICK_READY)
            time_delay = self.get_clock().now()
            
            while ((self.get_clock().now() - time_delay).nanoseconds / 1e6 <= (DELAY + 1000)):
                continue
            
            
            # added this because there was a time delay and current_qs was not getting updated in time (because of callback using for current_qs)
            q1 = (PICK_READY[5] - self.zero) * ENCODER_2_DEG
            q2 = ((PICK_READY[4] - self.zero) * ENCODER_2_DEG) + (np.pi/2) # adding this because it starts 90 degrees positive
            q3 = ((PICK_READY[3] - self.zero) * ENCODER_2_DEG)
            q4 = ((PICK_READY[2] - self.zero) * ENCODER_2_DEG) + (np.pi/2) # adding this because it starts 90 degrees positive
            q5 = ((PICK_READY[1] - self.zero) * ENCODER_2_DEG)
            
            self.current_qs = [q1, q2, q3, q4, q5]
            
            
            print("Done moving arm, now onto ik!")
            
            # this just takes the rotation matrix of the pose and keeps it. This will change when we know PoseStamped. Currently have PointStamped
            # trans_obj_r = self.t1 @ self.t2 @ self.t3 @ self.t4 @ self.t5
            # trans_obj_r = quaternion_matrix([trans_obj.orientation.x, trans_obj.orientation.y, trans_obj.orientation.z, trans_obj.orientation.w])
            
            trans_obj_r = np.array([[1.0, 0.0, 0.0, 0.0],
                                    [0.0, -1.0, 0.0, 0.0],
                                    [0.0, 0.0, -1.0, 0.0],
                                    [0.0, 0.0, 0.0, 1.0]])
            
            trans_obj_r[0,3] = trans_obj.point.x + 0.04  # may want to adjust this
            trans_obj_r[1,3] = trans_obj.point.y - 0.015 # may want to adjust this
            trans_obj_r[2,3] = trans_obj.point.z

            
            desired_pose = np.array(trans_obj_r).reshape((4,4))
            
            r_des = desired_pose[0:3, 0:3]
            position_des = [desired_pose[0,3],desired_pose[1,3], desired_pose[2,3]]            
            
            print("self: Current qs: {}".format(self.current_qs))
            print("Solving now...")    
            current_q = self.current_qs.copy()
            res , des_qs = self.solve_ik(position_des, r_des, current_q)
            
            if not res:
                print("IK did not converge in time!")
                res_send.data = False
                self.ik_res.publish(res_send)
                return
            
            des_qs = [round(elem,2) for elem in des_qs]
                
            print([elem* (180/np.pi) for elem in des_qs])
            
            self.desired_encoder_vals = self.angle_to_encoder(des_qs)
            
            print(self.desired_encoder_vals)
            
            print("Feasible: {}".format(self.sol_feasbile()))
            
            if not self.sol_feasbile():
                print("Not feasible so exiting!")
                res_send.data = False
                self.ik_res.publish(res_send)
                return
            
            
            #this will move the arm
            print("Moving arm to goal")
            self.kinematic_go()
            time_delay = self.get_clock().now()
            while ((self.get_clock().now() - time_delay).nanoseconds / 1e6 <= (DELAY + 1000)):
                continue
            
                       
            # close ee
            print("Picking up!")
            self.kinematic_go(ee_value=11050)
            time_delay = self.get_clock().now()
            while ((self.get_clock().now() - time_delay).nanoseconds / 1e6 <= (DELAY + 1000)):
                continue
            
            
            # move to carrying position
            print("Moving to carrying position")
            self.move_arm(MOVE_W_OBJ)
            time_delay = self.get_clock().now()
            while ((self.get_clock().now() - time_delay).nanoseconds / 1e6 <= (DELAY + 1000)):
                continue
            
        
        else:
            print("No transform available!")
            self.update = True
            res_send.data = False
            self.ik_res.publish(res_send)
            return
        
        
        #publish success
        print("Success!")
        self.update = True
        res_send.data = True
        self.ik_res.publish(res_send)
        return
    
    
    
    def joy_cb(self, msg:Joy):
        # axes: left-stick horizontal, left-stick verticle, LT, right-stick horizontal, right-stick verticle, RT, left-cross horizontal, left-cross verticle
        # buttons: A, B, X, Y, LB, RB, SACK, START
        
        if msg.buttons[1]: # red button is pressed.. RESET
        
            self.move_arm(STRAIGHT)
            print("Resetting home")
            
    
    def init_joints(self):
        
        self.q1 = self.current_qs[0]
        self.q2 = self.current_qs[1]
        self.q3 = self.current_qs[2]
        self.q4 = self.current_qs[3]
        self.q5 = self.current_qs[4]
        
        self.t1 = self.homo_trans(self.q1, np.pi/2, 0, D1)
        self.t2 = self.homo_trans(self.q2, np.pi, A2, 0)
        self.t3 = self.homo_trans(self.q3, np.pi, A3, 0)
        self.t4 = self.homo_trans(self.q4, np.pi/2, 0, 0)
        self.t5 = self.homo_trans(self.q5, 0, 0, D5) 
        
        self.joint_1 = self.t1
        self.joint_2 = self.joint_1 @ self.t2
        self.joint_3 = self.joint_2 @ self.t3
        self.joint_4 = self.joint_3 @ self.t4
        self.joint_5 = self.joint_4 @ self.t5
        
    
    def publish_joints(self):
        
        self.broadcast_transform(self.joint_5, 5)
        
    
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
        
        self.joint_1 = self.t1
        self.joint_2 = self.joint_1 @ self.t2
        self.joint_3 = self.joint_2 @ self.t3
        self.joint_4 = self.joint_3 @ self.t4
        self.joint_5 = self.joint_4 @ self.t5

        
        # self.broadcast_transform(self.joint_1, 1)
        # self.broadcast_transform(self.joint_2, 2)
        # self.broadcast_transform(self.joint_3, 3)
        # self.broadcast_transform(self.joint_4, 4)
        self.broadcast_transform(self.joint_5, 5)
        
        
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
        
        # if(joint_num == 5):
        #     print("Rotation")
        #     print(t.transform.rotation)
        #     print()
        #     print("Position")
        #     print(t.transform.translation)

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
        
    def homo_trans(self, theta, alpha, a, d):
        # theta = np.deg2rad(theta)
        return np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                        [0, np.sin(alpha), np.cos(alpha), d],
                        [0, 0, 0, 1]])
        
    
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

        # print(error.shape)

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
        tolerance = 0.005
        
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
