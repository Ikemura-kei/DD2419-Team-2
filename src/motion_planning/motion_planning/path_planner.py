from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point, PoseStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose_stamped

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from nav_msgs.msg import OccupancyGrid

from geometry_msgs.msg import PointStamped

import rclpy
from rclpy.node import Node

import math

class PathPlanner(Node):
    target_x = None
    target_y = None
    origin_x = None
    origin_y = None
    map = None
    resolution = None
    path = None
    def __init__(self):
        super().__init__('target_position_controller')
        self.get_logger().info('Welcome to path planner')

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        
        self.timer = self.create_timer(0.1, self.dijkstra)

    def map_cb(self, msg):
        self.get_logger().info("Received map {} {} {} {}".format(msg.info.resolution, msg.info.width, msg.info.height, len(msg.data)))
        self.resolution = msg.info.resolution

        size = math.pow(len(msg.data),0.5).__int__()
        self.get_logger().info("Size: {}".format(size))
        map = [[0 for x in range(size)] for y in range(size)]
        for i in range(size):
            for j in range(size):
                map[i][j] = msg.data[i * size + j]
        self.map = map
        self.origin_x = (msg.info.width/2).__int__()
        self.origin_y = (msg.info.height/2).__int__()
        self.target_x = (msg.info.width/2+2/msg.info.resolution).__int__()
        self.target_y = (msg.info.height/2+1/msg.info.resolution).__int__()

    def dijkstra(self):
        if self.map is None or self.target_x is None or self.target_y is None or self.origin_x is None or self.origin_y is None:
            return None
        
        self.get_logger().info("Dijkstra")

        queue = []
        visited = []
        distance = []
        links = []
        for i in range(len(self.map)):
            distance.append([math.inf for x in range(len(self.map))])
            visited.append([False for x in range(len(self.map))])
            path.append([None for x in range(len(self.map))])
        distance[self.origin_x][self.origin_y] = 0
        queue.append([self.origin_x, self.origin_y])
        while len(queue) > 0:
            [x, y] = queue.pop(0)
            self.get_logger().info("Visiting {} {}".format(x, y))
            visited[x][y] = True
            if x == self.target_x and y == self.target_y:
                break
            for i in range(-1, 2):
                for j in range(-1, 2):
                    if i == 0 and j == 0:
                        continue
                    if x + i < 0 or x + i >= len(self.map) or y + j < 0 or y + j >= len(self.map):
                        continue
                    if visited[x + i][y + j] or self.map[x + i][y + j] == 100:
                        continue
                    if distance[x + i][y + j] > distance[x][y] + 1:
                        distance[x + i][y + j] = distance[x][y] + 1
                        links[x + i][y + j] = [x, y]
                        queue.append([x + i, y + j])
        path = []
        x, y = self.target_x, self.target_y
        while links[x][y] is not None:
            path.append([x, y])
            x, y = links[x][y]
        self.path = path      



def main():
    rclpy.init()
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    
if __name__ == '__main__':
    main()