# -- import msgs --
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PointStamped
# -- tf2 stuff --
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
# -- rclpy stuff --
import rclpy
from rclpy.node import Node
# -- other stuff --
import numpy as np
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point

class NaiveMappingNode(Node):
    MAP_TOPIC = '/global_map'
    MAP_WIDTH = 12
    MAP_HEIGHT = 10.5
    MAP_ORIGIN = [-5, -5]
    MAP_RESOLUTION = 0.035
    
    def __init__(self):
        super().__init__('naive_mapping_node')
        
        self.tf2_buf = Buffer(cache_time=rclpy.duration.Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf2_buf, self)
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 12)
        self.map_pub = self.create_publisher(OccupancyGrid, self.MAP_TOPIC, 10)
        self.candidate_map_pub = self.create_publisher(OccupancyGrid, '/candidate_map', 10)
        
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
        
        self.MAX_X = int(self.MAP_WIDTH / self.MAP_RESOLUTION)
        self.MAX_Y = int(self.MAP_HEIGHT / self.MAP_RESOLUTION)
        self.UPDATE_TRHESH = 3
        self.MIN_RANGE_FILTER_THRESH = 0.2 # m
        
    def scan_cb(self, msg:LaserScan):
        # -- get current position of the robot --
        robot_loc = self.get_robot_location(msg.header.stamp)
        if robot_loc is None:
            return
        
        # print("robot location {}".format(robot_loc))
        
        # -- filter range-bearings -- 
        mask = np.isfinite(np.array(msg.ranges))
        mask = np.where((np.array(msg.ranges) > self.MIN_RANGE_FILTER_THRESH) * mask, 1, 0)
        ranges = np.array(msg.ranges)[mask == 1]
        bearings = np.arange(msg.angle_min, msg.angle_max+msg.angle_increment, msg.angle_increment)[mask == 1]
        
        # -- map all range-bearing to points --
        x = np.cos(bearings) * ranges
        y = np.sin(bearings) * ranges
        
        points = []
        for i in range(len(ranges)):
            point = PointStamped()
            point.header.stamp = msg.header.stamp
            point.header.frame_id = msg.header.frame_id
            point.point.x = x[i]
            point.point.y = y[i]
            points.append(point)

        # -- map all scan points to odom frame --
        if not self.tf2_buf.can_transform('odom', msg.header.frame_id, msg.header.stamp, rclpy.duration.Duration(seconds=0.5)):
            print('transform from {} to odom not available.'.format(msg.header.frame_id))
            
        try:
            transform = self.tf2_buf.lookup_transform('odom', msg.header.frame_id, msg.header.stamp, rclpy.duration.Duration(seconds=0.5))
        except TransformException as ex:
            print(ex)
            return
        
        points_map = np.zeros((2, len(ranges)))
        for i in range(len(ranges)):
            point = do_transform_point(points[i], transform)
            points_map[0, i] = point.point.x - self.MAP_ORIGIN[0]
            points_map[1, i] = point.point.y - self.MAP_ORIGIN[1]
            
        # -- map points to cell location --
        points_map = (points_map / self.MAP_RESOLUTION).astype(np.int32)
        mask = np.where((points_map[0] < self.MAX_X) * (points_map[1] < self.MAX_Y) * (points_map[0] >= 0) * (points_map[1] >= 0), 1, 0)
        points_map = points_map[:, mask==1]
        
        # -- add points to the candidate map --
        self.raw_candidate_map[points_map[1], points_map[0]] += 1
        
        # -- post-process the candidate map --
        update_mask = np.where(self.raw_candidate_map>self.UPDATE_TRHESH, 1, 0)
        
        # -- add valid points from the candidate map to the map --
        self.raw_map[update_mask==1] = 100
        
        # -- remove valid points from the candidate map --
        self.raw_candidate_map[update_mask==1] = 0
        
        # -- pubish map --
        # self.candidate_map.data = self.raw_candidate_map.reshape((-1)).tolist()
        # self.candidate_map_pub.publish(self.candidate_map)
        self.map.data = self.raw_map.reshape((-1)).tolist()
        self.map_pub.publish(self.map)
        
    
    def get_robot_location(self, stamp):
        # -- basically getting where the frame origin of base_link (thus source frame) is expressed in odom (thus target frame) --
        loc = PoseStamped()
        loc.header.stamp = stamp
        loc.header.frame_id = 'odom'

        timeout = rclpy.duration.Duration()
        can_transform = self.tf2_buf.can_transform('odom', 'base_link', stamp, timeout)
        if not can_transform:
            print('transform unavailable')
            return None
        
        try:
            transform = self.tf2_buf.lookup_transform('odom', 'base_link', stamp)
        except TransformException as ex:
            print(ex)
            return None
        
        loc.pose.position.x = transform.transform.translation.x
        loc.pose.position.y = transform.transform.translation.y
        loc.pose.orientation.x = transform.transform.rotation.x
        loc.pose.orientation.y = transform.transform.rotation.y
        loc.pose.orientation.z = transform.transform.rotation.z
        loc.pose.orientation.w = transform.transform.rotation.w
        # print(loc.pose.position)
        
        return loc

def main():
    rclpy.init()
    
    node = NaiveMappingNode()
    
    rclpy.spin(node)
    
    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
