from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point, PoseStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose_stamped

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from std_msgs.msg import Bool

from geometry_msgs.msg import PointStamped

import rclpy
from rclpy.node import Node

import math

class TargetPositionController(Node):

    def __init__(self):
        super().__init__('target_position_controller')
        
        # Target position
        self.target_x = None
        self.target_y = None
        
        self.linear_vel = 1.05
        self.alpha = 4.75
        # Define a threshold for stopping distance
        self.stopping_distance_threshold = 0.16
        self.angular_threshold = 0.05
        self.angle_threshold = 0.1
        
        self.stop = False
        # Initialize the transform listener and assign it a buffer
        self.tf2Buffer = Buffer(cache_time=None)

        #transform listener fills the buffer
        self.listener = TransformListener(self.tf2Buffer, self)

        self._pub = self.create_publisher(
            Twist, '/motor_controller/twist', 10)
        
        self._pub2 = self.create_publisher(
            PointStamped, '/clicked_point', 10)
        
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_command_cb, 10)
        
        self.goal_sub = self.create_subscription(PointStamped, '/plan_goal', self.goal_cb, 10)

        # Timer to publish commands every 100 milliseconds (10 Hz)
        self.timer1 = self.create_timer(0.1, self.publish_twist)
        self.timer2 = self.create_timer(0.1, self.publish_point)
        
        self.goal_reached_pub = self.create_publisher(
            Bool, '/goal_reached', 10)
        
    def goal_cb(self, msg:PointStamped): # I DO  NOT THINK THIS TRANSFORM IS NEEDED SINCE POINT IS FROM ODOM FRAME ALREADY!!!!!!!!!!
        # stamp = rclpy.time.Time()
        
        # if not self.tf2Buffer.can_transform('odom', 'base_link', stamp, rclpy.duration.Duration(seconds=0.5)):
        #     print('transform from base_link to odom not available.')
        
        # try:
        #      transform_base2odom = self.tf2Buffer.lookup_transform('odom', 'base_link', stamp, rclpy.duration.Duration(seconds=0.5))
        # except TransformException as ex:
        #     print(ex)
        #     return
        
        # transform_base2odom = self.tf2Buffer.lookup_transform('odom', 'base_link', stamp)
        # target_pose = PoseStamped()
        # target_pose.header.frame_id = 'base_link'
        # target_pose.header.stamp = stamp.to_msg()
        # target_pose.pose.position.x = msg.x
        # target_pose.pose.position.y = msg.y
        # target_pose.pose.orientation.w = 1.0
        # target_pose.pose.orientation.x = target_pose.pose.orientation.y = target_pose.pose.orientation.z = 0.0
        # target_pose = do_transform_pose_stamped(target_pose, transform_base2odom)
        
        # self.target_x = target_pose.pose.position.x
        # self.target_y = target_pose.pose.position.y
        
        # print("orig x: {}, orig y: {}".format(msg.x, msg.y))
        # print("x: {}, y: {}".format(self.target_x, self.target_y))
        self.target_frame_id = msg.header.frame_id
        self.target_stamp = msg.header.stamp
        self.target_x = msg.point.x
        self.target_y = msg.point.y
        print("x: {}, y: {}".format(self.target_x, self.target_y))
    
    def publish_point(self):
        if self.target_x is None or self.target_y is None:
            return
        pointStamped = PointStamped()
        pointStamped.header.stamp = self.get_clock().now().to_msg()
        pointStamped.header.stamp = self.target_stamp
        pointStamped.header.frame_id = 'odom' 
        pointStamped.point.x = self.target_x
        pointStamped.point.y = self.target_y
        
        self._pub2.publish(pointStamped)
        
    def joy_command_cb(self, msg:Joy):
        if msg.buttons[1]: #set duty cycle to zero if red button held
            self.stop = True
        else:
            self.stop = False
    
    # THIS ASSUMES THE TARGET IS ALWAYS IN THE ODOM FRAME
    def publish_twist(self):
        if self.target_x is None or self.target_y is None:
            return
        # Compute robot's position
        child_frame = 'base_link'
        parent_frame = 'odom'
        stamp = rclpy.time.Time()
        # stamp = self.get_clock().now().to_msg() # THIS ONE DOES NOT WORK!!!!!!!!!!!
        
        timeout = rclpy.duration.Duration(seconds=0.5)
        if self.tf2Buffer.can_transform(parent_frame, child_frame, stamp, timeout) == 0:
            print("NO TRANSFORM AVAILABLEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            return
        
        try:   
            transform = self.tf2Buffer.lookup_transform(parent_frame, child_frame, stamp, timeout)
            print("Can transform")
        except TransformException as ex:
            self.get_logger().info(f'Could not transform: {ex}')
            return
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
        
        target_orientation = robot_orientation-math.atan2(distance_y, distance_x)
        
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
            self.target_x = self.target_y = None
            reached = Bool()
            reached.data = True
            self.goal_reached_pub.publish(reached)
        else:
            twist_msg.linear.x = self.linear_vel
            
        if self.stop:
             twist_msg.linear.x = 0.0
             twist_msg.angular.z = 0.0
             
        # TO TEST: Publish the DutyCycles message
        self._pub.publish(twist_msg)

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