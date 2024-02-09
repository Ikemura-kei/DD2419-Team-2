from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Twist

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PointStamped

import rclpy
from rclpy.node import Node

import math

class TargetPositionController(Node):
    # Target position
    target_x = 1.0
    target_y = 1.0
    
    linear_vel = 0.2
    alpha = 0.1
    # Define a threshold for stopping distance
    stopping_distance_threshold = 0.1
    angle_threshold = 0.1

    def __init__(self):
        super().__init__('target_position_controller')
        
        # Initialize the transform listener and assign it a buffer
        self.tf2Buffer = Buffer(cache_time=None)

        #transform listener fills the buffer
        self.listener = TransformListener(self.tf2Buffer, self)

        self._pub = self.create_publisher(
            Twist, '/motor_controller/twist', 10)
        
        self._pub2 = self.create_publisher(
            PointStamped, '/clicked_point', 10)

        # Timer to publish commands every 100 milliseconds (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_twist)
        self.timer = self.create_timer(0.1, self.publish_point)
    
    def publish_point(self):
        pointStamped = PointStamped()
        pointStamped.header.stamp = self.get_clock().now().to_msg()
        pointStamped.header.frame_id = 'odom'
        pointStamped.point.x = self.target_x
        pointStamped.point.y = self.target_y
        
        self._pub2.publish(pointStamped)        
        
    
    def publish_twist(self):
        # Compute robot's position
        child_frame = 'base_link'
        parent_frame = 'odom'
        stamp = rclpy.time.Time()
        
        if self.tf2Buffer.can_transform(parent_frame, child_frame, stamp) == 0:
            # TODO: error handling?
            return
            
        transform = self.tf2Buffer.lookup_transform(parent_frame, child_frame, stamp)
        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        self.get_logger().info('Robots angle: %f x and %f y %f z and %f w' % ( transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w))
        # Compute the target orientation angle
        twist_msg = Twist()
        
        distance_x = self.target_x-robot_x
        distance_y = self.target_y-robot_y
        
        target_orientation = transform.transform.rotation.y-math.atan2(self.target_y, self.target_x)
        
        distance_to_target = math.sqrt(distance_x**2 + distance_y**2)
        self.get_logger().info('Distance to target: %f distance_to_target and %f target_orientation' % ( distance_to_target, target_orientation))
        # Compute desired angular velocity using a proportional controller
        angular_velocity = self.alpha * target_orientation
        
        # Check if the robot is close to the target
        if distance_to_target <= self.stopping_distance_threshold:
            # Set linear velocity to zero to stop the robot
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0 
        else:
            twist_msg.linear.x = self.linear_vel
            twist_msg.angular.z = angular_velocity 
            
        
        # TO TEST: Publish the DutyCycles message
        self._pub.publish(twist_msg)
        self.get_logger().info('Published twist: %f linear and %f angular' % ( twist_msg.linear.x, twist_msg.angular.z))

def main():
    rclpy.init()
    node = TargetPositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
