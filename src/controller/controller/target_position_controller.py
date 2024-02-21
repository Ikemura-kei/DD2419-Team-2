import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import math

class TargetPositionController(Node):
    # Target position
    target_x = None
    target_y = None
    
    linear_vel = 0.2
    alpha = 3.25
    # Define a threshold for stopping distance
    stopping_distance_threshold = 0.125
    angular_threshold = 0.05
    angle_threshold = 0.1

    def __init__(self):
        super().__init__('target_position_controller')
        self.stop = False
        # Initialize the transform listener and assign it a buffer
        self.tf2Buffer = Buffer(cache_time=None)

        #transform listener fills the buffer
        self.listener = TransformListener(self.tf2Buffer, self)

        self._twist_publisher = self.create_publisher(
            Twist, '/simulation/cmd_vel', 10)
        
        self._completion_publisher = self.create_publisher(String, 'task_completion', 10)
                
        self.goal_sub = self.create_subscription(Point, '/target_position', self.target_position_cb, 10)

        # Timer to publish commands every 100 milliseconds (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_twist)
        self.timer = self.create_timer(0.1, self.publish_point)

    def publish_completion(self):
        msg = String()
        msg.data = 'Task completed'
        self._completion_publisher.publish(msg)
        self.get_logger().info('Published task completion')
        
    def target_position_cb(self, msg:Point):
        self.get_logger().info('Received target position: %f, %f' % (msg.x, msg.y))
        stamp = rclpy.time.Time()

        self.goal_t = stamp
        self.target_x, self.target_y = msg.x, msg.y       
        
    def joy_command_cb(self, msg:Joy):
        if msg.buttons[1]: #set duty cycle to zero if red button pressed
            self.stop = True
        else:
            self.stop = False
    
    def publish_twist(self):
        if self.target_x is None or self.target_y is None:
            return
        # Compute robot's position
        child_frame = 'base_link'
        parent_frame = 'odom'
        stamp = rclpy.time.Time()
        
        if self.tf2Buffer.can_transform(parent_frame, child_frame, stamp) == 0:
            return
            
        transform = self.tf2Buffer.lookup_transform(parent_frame, child_frame, stamp)
        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        
        
        # print(target_pose.pose.position)
        
        # Calculate the rotation angle around the z-axis
        robot_orientation = 2 * math.atan2(transform.transform.rotation.z, transform.transform.rotation.w)
    
        # Ensure the angle is in the range [-pi, pi]
        if robot_orientation > math.pi:
            robot_orientation -= 2 * math.pi
        elif robot_orientation < -math.pi:
            robot_orientation += 2 * math.pi
        
        # Compute the target orientation angle
        twist_msg = Twist()
        
        distance_x = self.target_x-robot_x
        distance_y = self.target_y-robot_y
        
        target_orientation = math.atan2(distance_y, distance_x)-robot_orientation
        
        distance_to_target = math.sqrt(distance_x**2 + distance_y**2)
        
        self.get_logger().info('\033[36mDISTANCE TO TARGET: %f\033[0m' % (distance_to_target))
        # Compute desired angular velocity using a proportional controller
        angular_velocity = self.alpha * target_orientation
        
        # Check if the robot is close to the target
        if abs(target_orientation) <= self.angular_threshold:
            twist_msg.angular.z = 0.0 
        else:
            twist_msg.angular.z = angular_velocity 
        
        if distance_to_target <= self.stopping_distance_threshold:
            # Set linear velocity to zero to stop the robot
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.target_x = self.target_y = None
            self.publish_completion()
        else:
            twist_msg.linear.x = self.linear_vel
            
        if self.stop:
             twist_msg.linear.x = 0.0
             twist_msg.angular.z = 0.0
             
        # TO TEST: Publish the DutyCycles message
        self._twist_publisher.publish(twist_msg)

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