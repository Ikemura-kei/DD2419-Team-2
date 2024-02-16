from geometry_msgs.msg import PoseStamped, PointStamped


from nav_msgs.msg import OccupancyGrid, Path

from tf_transformations import quaternion_from_euler

import rclpy
from rclpy.node import Node

import math
import numpy as np

class PathPlanner(Node):
    target_x = 1.5
    target_y = 3.8
    origin_x = 0.0
    origin_y = 0.0
    map = None
    resolution = None
    def __init__(self):
        super().__init__('target_position_controller')
        self.get_logger().info('Welcome to path planner')

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)

        # Initialize the path publisher
        self._path_pub = self.create_publisher(Path, 'path', 10)
        # Store the path here
        self._path = Path()   
        
    def map_cb(self, msg):
        if self.map is not None:
            return
        self.get_logger().info("Received map {} {} {} {}".format(msg.info.resolution, msg.info.width, msg.info.height, len(msg.data)))
        self.resolution = msg.info.resolution

        size = math.pow(len(msg.data),0.5).__int__()
        self.get_logger().info("Size: {}".format(size))
        map = [[0 for x in range(size)] for y in range(size)]
        for i in range(size):
            for j in range(size):
                map[i][j] = msg.data[i * size + j]
        self.map = map

        self.dijkstra()

    def dijkstra(self):
        if self.map is None or self.target_x is None or self.target_y is None or self.origin_x is None or self.origin_y is None:
            return None
        
        self.get_logger().info("Dijkstra")

        map_origin_x, map_origin_y = self.from_world_to_map(self.origin_x, self.origin_y)
        map_target_x, map_target_y = self.from_world_to_map(self.target_x, self.target_y)

        queue = []
        visited = []
        distance = []
        links = []
        for i in range(len(self.map)):
            distance.append([math.inf for x in range(len(self.map[0]))])
            visited.append([False for x in range(len(self.map[0]))])
            links.append([None for x in range(len(self.map[0]))])
        distance[map_origin_x][map_origin_y] = 0
        queue.append([map_origin_x, map_origin_y])
        while len(queue) > 0:
            [x, y] = queue.pop(0)
            self.get_logger().info("Visiting {} {}".format(x, y))
            visited[x][y] = True
            if x == map_target_x and y == map_target_y:
                break
            for i in range(-1, 2):
                for j in range(-1, 2):
                    if i == 0 and j == 0:
                        continue
                    if x + i < 0 or x + i >= len(self.map[0]) or y + j < 0 or y + j >= len(self.map):
                        continue
                    if visited[x + i][y + j] or self.map[x + i][y + j] == 100:
                        continue
                    if (i == j or i == -j):
                        if distance[x + i][y + j] > distance[x][y] + math.pow(2,0.5):
                            distance[x + i][y + j] = distance[x][y] + math.pow(2,0.5)
                            print("Distance {} {} {}".format(x + i, y + j, distance[x + i][y + j]))
                            links[x + i][y + j] = [x, y]
                            queue.append([x + i, y + j])
                        continue
                    if distance[x + i][y + j] > distance[x][y] + 1:
                        distance[x + i][y + j] = distance[x][y] + 1
                        print("Distance {} {} {}".format(x + i, y + j, distance[x + i][y + j]))
                        links[x + i][y + j] = [x, y]
                        queue.append([x + i, y + j])
        x, y = map_target_x, map_target_y
        while links[x][y] is not None:
            self.publish_path(self.get_clock().now().to_msg(), x, y, 0.0)
            x, y = links[x][y]

    def publish_path(self, stamp, x, y, yaw):
        """Takes a 2D pose appends it to the path and publishes the whole path.

        Keyword arguments:
        stamp -- timestamp of the transform
        x -- x coordinate of the 2D pose
        y -- y coordinate of the 2D pose
        yaw -- yaw of the 2D pose (in radians)
        """

        self._path.header.stamp = stamp
        self._path.header.frame_id = 'odom'

        pose = PoseStamped()
        pose.header = self._path.header
        world_x, world_y = self.from_map_to_world(x, y)
        pose.pose.position.x = world_x
        pose.pose.position.y = world_y
        pose.pose.position.z = 0.01  # 1 cm up so it will be above ground level

        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self._path.poses.append(pose)

        self._path_pub.publish(self._path)

    def from_map_to_world(self, x, y):
        """Transforms a pair of map coordinates to world coordinates.

        Keyword arguments:
        x -- x coordinate in the map
        y -- y coordinate in the map

        Returns:
        x -- x coordinate in the world
        y -- y coordinate in the world
        """

        before_x = x
        before_y = y

        x = (x - len(self.map[0])/2) * self.resolution
        y = (y - len(self.map)/2) * self.resolution

        self.get_logger().info("from_map_to_world => map x: {}, map y: {}, world x: {}, world y: {},".format(before_x, before_y, x, y))

        return x, y

    def from_world_to_map(self, x, y):
        """Transforms a pair of world coordinates to map coordinates.

        Keyword arguments:
        x -- x coordinate in the world
        y -- y coordinate in the world

        Returns:
        x -- x coordinate in the map
        y -- y coordinate in the map
        """

        before_x = x
        before_y = y

        x = int((x / self.resolution) + len(self.map[0])/2)
        y = int((y / self.resolution) + len(self.map)/2)

        self.get_logger().info("from_world_to_map => world x: {}, world y: {}, map x: {}, map y: {},".format(before_x, before_y, x, y))

        return x, y  



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