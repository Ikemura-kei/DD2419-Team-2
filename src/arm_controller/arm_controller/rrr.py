from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Int16MultiArray, MultiArrayDimension, Float32MultiArray

import numpy as np

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_matrix, quaternion_matrix
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf2_geometry_msgs

#This node is used as a test to send message via topic to kinematics node

class RRR(Node):
    def __init__(self):
        super().__init__('rrr')
        
        self.joycon_sub = self.create_subscription(Joy, topic='/joy', callback=self.joy_cb, qos_profile=10)
        self.ik_pub = self.create_publisher(PoseStamped, '/ik_publisher', 10)
        
        
        
         # Initialize the transform listener and assign it a buffer
        self.tf2Buffer = Buffer(cache_time=None)

        #transform listener fills the buffer
        self.listener = TransformListener(self.tf2Buffer, self)
        
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        
        
        
        
    
    def joy_cb(self, msg:Joy):
        # axes: left-stick horizontal, left-stick verticle, LT, right-stick horizontal, right-stick verticle, RT, left-cross horizontal, left-cross verticle
        # buttons: A, B, X, Y, LB, RB, SACK, START
        self.joy_cmd = msg
        
        if self.joy_cmd.buttons[3]: # Y button is pressed.. PUBLISH
            t = PoseStamped()
            
            t = PoseStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'

            t.pose.position.x = 0.20566888868893826
            t.pose.position.y = -0.005311581695744999
            t.pose.position.z = 0.04090986537393716

            t.pose.orientation.x = -0.966787233611128
            t.pose.orientation.y = 0.217165801036952
            t.pose.orientation.z = -0.12233614205647342
            t.pose.orientation.w = 0.05652723355392071
            
            self.ik_pub.publish(t)
            print("Published!")

            
            
            # stamp = self.get_clock().now().to_msg()
            
            # if self.tf2Buffer.can_transform('base_link', 'arm_base', stamp):
            #     transform = self.tf2Buffer.lookup_transform('base_link', 'arm_base', stamp)
                
            #     t = PoseStamped()
            #     t.header.stamp = self.get_clock().now().to_msg()
            #     t.header.frame_id = 'arm_base'

            #     t.pose.position.x = 0.1683093181842284
            #     t.pose.position.y = -0.25554552923579055
            #     t.pose.position.z = 0.0571060617789378

            #     t.pose.orientation.x = 0.6835245542145066
            #     t.pose.orientation.y = -0.433359629166147
            #     t.pose.orientation.z = 0.5355129324456646
            #     t.pose.orientation.w = 0.24128720392570538
                

            #     trans_obj = tf2_geometry_msgs.do_transform_pose(t.pose, transform)
                
            #     print(trans_obj.position)
            #     print(trans_obj.orientation)
            


def main():
    print('Hi from rrr')
    
    rclpy.init()
    
    rrr = RRR()
    
    rclpy.spin(rrr)
    
    rrr.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
