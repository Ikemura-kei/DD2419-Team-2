from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Int16MultiArray, MultiArrayDimension, Float32MultiArray

import numpy as np

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_matrix, quaternion_matrix
from geometry_msgs.msg import TransformStamped

# "{layout: {dim: [{label: '', size: 0, stride: 0}], data_offset: 0}, data: [12000,12000,12000,12000,12000,12000,500,500,500,500,500,500]}"

ANGLE_LIMITS_LOW = [4200, 3100, 3200, 3000, 750, 1000]
ANGLE_LIMITS_HIGH = [12050, 21500, 20500, 23000, 17050, 22000]



ANGLE_HOMES = [12000, 12000, 12000, 12000, 12000] #keeping all but the ee
PERIOD = 0.1

ENCODER_2_DEG = 0.01 * (np.pi/180)
DEG_2_ENCODER = 100 * (180/np.pi)
D1 = 0.065
A2 = 0.101
A3 = 0.094
D5 = 0.169 # need to change this one as ee moves

'''
LX16AServo servo1(&servoBus, 1); // 0-16800 (0-168 degrees)
LX16AServo servo2(&servoBus, 2); // 0-24000 (0-240 degrees)
LX16AServo servo3(&servoBus, 3); // 0-24000 (0-240 degrees)
LX16AServo servo4(&servoBus, 4); // 0-24000 (0-240 degrees)
LX16AServo servo5(&servoBus, 5); // 0-24000 (0-240 degrees)
LX16AServo servo6(&servoBus, 6); // 0-24000 (0-240 degrees)
'''

class ForwardKinematics(Node):
    def __init__(self):
        super().__init__('forward_kinematics')
        
        self.joint_pos_sub = self.create_subscription(JointState, topic='/servo_pos_publisher', callback=self.joint_pos_cb, qos_profile=10)
        self.joint_cmd_sub = self.create_subscription(Int16MultiArray, '/multi_servo_cmd_sub', callback=self.joint_cmd_cb, qos_profile=10)
        
        self.ik_sub = self.create_subscription(Float32MultiArray, '/ik_publisher', callback=self.ik_cb, qos_profile=10)
        
        self.kinematics_pub = self.create_publisher(Int16MultiArray, '/kinematic_control', 10)
        
        self.joycon_sub = self.create_subscription(Joy, topic='/joy', callback=self.joy_cb, qos_profile=10)
        
        
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
        
        self.current_qs = [0, (np.pi/2), 0, (np.pi/2), 0] #this is our home for angles. thi smay need to change!!!!!!!!!!!!!!!!

    
    
    def joy_cb(self, msg:Joy):
        # axes: left-stick horizontal, left-stick verticle, LT, right-stick horizontal, right-stick verticle, RT, left-cross horizontal, left-cross verticle
        # buttons: A, B, X, Y, LB, RB, SACK, START
        self.joy_cmd = msg
        if self.joy_cmd.buttons[3]: # if Y is pressed. Start IK
            
            desired_pose = np.array([[-0.75475116, -0.45769641,  0.46996242,  0.13886234], #out to the left (when looking at robot)
                                     [0.40937023, -0.88839653, -0.20776821, -0.0613904],
                                     [0.51260775,  0.03557532,  0.85788559,  0.39023585],
                                     [0,          0,          0,          1 ]]).reshape((4,4))
            
            desired_pose = np.array([[-2.36612254e-01,  5.16289776e-03,  9.71590441e-01,  3.17013190e-01], #straight out
                                     [-2.18148850e-02, -9.99762027e-01, -1.23494061e-16, -2.18166311e-17],
                                     [9.71359229e-01, -2.11951338e-02,  2.36668575e-01,  2.07372523e-01],
                                     [0,          0,          0,          1 ]]).reshape((4,4))
            
            # desired_pose = np.array([[5.11022852e-01,  2.71393271e-02,  8.59138581e-01,  3.03887530e-01], #straight out, but down a lot. DOES NOT WORK!!!
            #                          [ 5.30331184e-02, -9.98592754e-01, -2.07850033e-16, -4.49749144e-17],
            #                          [8.57929562e-01,  4.55627981e-02, -5.11743000e-01, -5.34450878e-03],
            #                          [0,          0,          0,          1 ]]).reshape((4,4))
            
            
            # desired_pose = np.array([[0.25822102,  0.22619024,  0.93923366,  0.3046841], #down a lot, and to the right. DOES NOT WORK!!!
            #                          [ 0.11033404, -0.97274864,  0.20392762,  0.0661534],
            #                          [0.95976471,  0.05097104, -0.27614063,  0.07434399],
            #                          [0,          0,          0,          1 ]]).reshape((4,4))
            
        
            r_des = desired_pose[0:3, 0:3]
            position_des = [desired_pose[0,3],desired_pose[1,3], desired_pose[2,3]]
            
            print(r_des)
            print(position_des)
            
            print("Solving now...")
            
            des_qs = self.solve_ik(position_des, r_des, self.current_qs)
            
            des_qs = [round(elem,2) for elem in des_qs]
            
            print(des_qs)
            
            encoder_vals = self.angle_to_encoder(des_qs)
            
            print(encoder_vals)
            
            vals = Int16MultiArray()
            vals.data = encoder_vals
            
            print(encoder_vals)
            
            self.kinematics_pub.publish(vals)
            
    
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
        t.header.frame_id = 'base_link' # will need to change this to arm_base
        t.child_frame_id = 'joint_' + str(joint_num)

        t.transform.translation.x = trans[0,3]   
        t.transform.translation.y = trans[1,3]
        t.transform.translation.z = trans[2,3]

        q = quaternion_from_matrix(trans)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
        
    def homo_trans(self, theta, alpha, a, d):
        # theta = np.deg2rad(theta)
        return np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                        [0, np.sin(alpha), np.cos(alpha), d],
                        [0, 0, 0, 1]])
        
        
        
    
    def ik_cb(self, msg:Float32MultiArray):
        
        desired_pose = np.array(msg.data).reshape((4,4))
        
        r_des = desired_pose[0:3, 0:3]
        position_des = [desired_pose[0,3],desired_pose[1,3], desired_pose[2,3]]
        
        print(r_des)
        print(position_des)
        
        print("Solving now...")
        
        des_qs = self.solve_ik(position_des, r_des, self.current_qs)
        
        des_qs = [round(elem,2) for elem in des_qs]
        
        print(des_qs)
        
        encoder_vals = self.angle_to_encoder(des_qs)
        
        vals = Int16MultiArray()
        vals.data = encoder_vals
        
        print(encoder_vals)
        
        self.kinematics_pub.publish(vals)
        
        #then publish these.. need to put back into deg then encoder value
        
        # [-8.586421686070536e-19, 1.2806424677342814, -0.23449172978713778, 1.32853076887559, 5.99694604648262e-19] in radians
        
    
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


    def solve_ik(self, point, R, joint_positions):
        
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
        tolerance = 0.0005      
        
        while(max_error >= tolerance):
            

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

            print(max_error)
            
            
        q = np.reshape(q,(5,))
        q = q.tolist()

        #return the desired joint params
        return q











        
        
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
            if joint_times[i] < 50:
                joint_times[i] = 50
            self.joint_cmd.data.append(joint_times[i])
                
        print(self.joint_cmd)
            
        self.joint_cmd_pub.publish(self.joint_cmd)

def main():
    print('Hi from forward_kinematics.')
    
    rclpy.init()
    
    forward_kinematics = ForwardKinematics()
    
    rclpy.spin(forward_kinematics)
    
    forward_kinematics.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
