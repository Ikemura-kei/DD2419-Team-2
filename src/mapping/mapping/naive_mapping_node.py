# -- import msgs --
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
# -- tf2 stuff --
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
# -- rclpy stuff --
import rclpy
from rclpy.node import Node
# -- other stuff --
import numpy as np

class NaiveMappingNode(Node):
    MAP_TOPIC = '/global_map'
    MAP_WIDTH = 9
    MAP_HEIGHT = 6
    MAP_ORIGIN = [-1, -1]
    MAP_RESOLUTION = 0.03
    
    def __init__(self):
        super().__init__('naive_mapping_node')
        
        self.tf2_buf = Buffer(cache_time=rclpy.duration.Duration(seconds=5))
        self.tf_listener = TransformListener(self.tf2_buf, self)
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 12)
        
        self.map = OccupancyGrid()
        self.map.info.resolution = self.MAP_RESOLUTION
        self.map.info.width = int(self.MAP_WIDTH / self.MAP_RESOLUTION)
        self.map.info.height = int(self.MAP_HEIGHT / self.MAP_RESOLUTION)
        self.map.info.origin.position.x = float(self.MAP_ORIGIN[0])
        self.map.info.origin.position.y = float(self.MAP_ORIGIN[1])
        self.map.info.map_load_time = self.get_clock().now().to_msg()
        self.map.header.frame_id = 'odom'
        self.raw_map = np.zeros((self.map.info.height, self.map.info.width), dtype=np.int8)
        self.map.data = self.raw_map.reshape((-1)).tolist()
        
        self.candidate_map = self.map
        self.raw_candidate_map = self.raw_map.copy()
        self.candidate_map.data = self.raw_candidate_map.reshape((-1)).tolist()
        
        
    def scan_cb(self, msg:LaserScan):
        # -- get current position of the robot --
        robot_loc = self.get_robot_location(msg.header.stamp)
        if robot_loc is None:
            return
        
        # print("robot location {}".format(robot_loc))
        
        # -- map all scan points to odom frame --
        
        
        # -- add points to the candidate map --
        
        # -- post-process the candidate map --
        
        # -- add valid points from the candidate map to the map --
        
        # -- remove valid points from the candidate map --
        
    
    def get_robot_location(self, stamp):
        # -- basically getting where the frame origin of base_link (thus source frame) is expressed in odom (thus target frame) --
        loc = PoseStamped()
        loc.header.stamp = stamp
        loc.header.frame_id = 'odom'

        timeout = rclpy.duration.Duration(seconds=1.0)
        can_transform = self.tf2_buf.can_transform('odom', 'base_link', stamp, timeout)
        if not can_transform:
            print('transform unavailable')
            return None
        
        try:
            transform = self.tf2_buf.lookup_transform('odom', 'base_link', stamp, timeout)
        except TransformException as ex:
            print(ex)
            return None
        
        loc.pose.position.x = transform.transform.translation.x
        loc.pose.position.y = transform.transform.translation.y
        loc.pose.orientation.x = transform.transform.rotation.x
        loc.pose.orientation.y = transform.transform.rotation.y
        loc.pose.orientation.z = transform.transform.rotation.z
        loc.pose.orientation.w = transform.transform.rotation.w
        print(loc.pose.position)
        
        return loc

def main():
    rclpy.init()
    
    node = NaiveMappingNode()
    
    rclpy.spin(node)
    
    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
