from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

import math

class TargetPositionController(Node):
    delta_x = 0.0
    delta_y = 0.0
    linear_vel = 1.0
    alpha = 0.05
    # Define a threshold for stopping distance
    stopping_distance_threshold = 0.1

    def __init__(self):
        super().__init__('target_position_controller')
        
        self.create_subscription(
                MarkerArray, '/object_centers', self.object_centers_callback, 10)

        self._pub = self.create_publisher(
            Twist, '/motor_controller/twist', 10)

        # Timer to publish commands every 100 milliseconds (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_twist)
        
    def object_centers_callback(self, msg: MarkerArray):
        self.delta_x = msg.position.x
        self.delta_y = msg.position.y
        self.get_logger().info('Object center: x: %f, y: %f, z: %f' % (msg.position.x, msg.position.y, msg.position.z ))
    
    def publish_twist(self):
        # Compute the target orientation angle
        twist_msg = Twist()
        
        target_orientation = math.atan2(self.delta_y, self.delta_x)
        distance_to_target = math.sqrt(self.delta_x**2 + self.delta_y**2)
        # Compute desired angular velocity using a proportional controller
        angular_velocity = self.alpha * target_orientation
        
        # Check if the robot is close to the target
        if distance_to_target <= self.stopping_distance_threshold:
            # Set linear velocity to zero to stop the robot
            twist_msg.linear.x = 0.0
        else:
            twist_msg.linear.x = self.linear_vel
            
        twist_msg.angular.z = angular_velocity 
        
        # TO TEST: Publish the DutyCycles message
        # self._pub.publish(twist_msg)
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
