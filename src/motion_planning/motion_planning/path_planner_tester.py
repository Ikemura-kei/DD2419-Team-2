import rclpy
import rclpy.duration
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import numpy as np

class PathPlannerTester(Node):
    def __init__(self):
        super().__init__('path_planner_tester')
        self.get_logger().info("Path planner tester node started.")
        
        self.pub = self.create_publisher(PointStamped, '/plan_goal', 10)
        
        self.timer = self.create_timer(0.4, self.timer_callback)
        
    def timer_callback(self):
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = (self.get_clock().now() - rclpy.duration.Duration(seconds=1)).to_msg()
        msg.point.x = 1.3 + (np.random.rand() - 0.5) * 0.1
        msg.point.y = 0.4 + (np.random.rand() - 0.5) * 0.1
        msg.point.z = 0.0
        
        self.pub.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    
    path_planner_tester = PathPlannerTester()
    rclpy.spin(path_planner_tester)
    
    rclpy.shutdown()