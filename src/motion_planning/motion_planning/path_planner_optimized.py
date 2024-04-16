# -- ros messages --
from geometry_msgs.msg import PoseArray, PointStamped, Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
# -- tf stuff --
import rclpy.time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_from_euler
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose_stamped
# -- rclpy --
import rclpy
from rclpy.node import Node
# -- math utilties
import math
import numpy as np
from scipy import signal
# -- misc. --
import time
from copy import deepcopy

class GRID_STATES:
    EMPTY = 0
    BORDER = -100
    OCCUPIED = 100
    
    def __init__():
        pass
    
class TfNode(Node):
    def __init__(self):
        super().__init__('tf_node')

class PathPlanner(Node):
    threshold_distance = 0.25 # at least the half of the robot's width
    target_x = None
    target_y = None
    completed = False
    origin_x = None
    origin_y = None
    map = OccupancyGrid()
    def __init__(self):
        super().__init__('path_planner')
        self.get_logger().info('Welcome to path planner')
        self.tf_node = TfNode()

        self.tfBuffer = Buffer(cache_time=rclpy.duration.Duration(seconds=11))
        self.listener = TransformListener(self.tfBuffer, self.tf_node, spin_thread=True)

        # -- create subscribers --
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.goal_sub = self.create_subscription(PointStamped, '/plan_goal', self.goal_cb, 10)

        # -- create publishers --
        self._point_pub = self.create_publisher(
            PointStamped, '/clicked_point', 10)
        self._path_pub = self.create_publisher(Path, 'path', 10)
        self.inflated_map_pub = self.create_publisher(OccupancyGrid, '/inflated_map', 10)
        
        # -- store the path here --
        self._path = Path() 

        self.last_time = self.get_clock().now() 
        self.timer = self.create_timer(0.2, self.dijkstra)
        
        self.NO_PLAN_DISTANCE_THRESHOLD = 0.25

    def publish_point(self):
        if self.target_x is None or self.target_y is None:
            return
        pointStamped = PointStamped()
        pointStamped.header.stamp = self.get_clock().now().to_msg()
        pointStamped.header.frame_id = 'odom'
        pointStamped.point.x = self.target_x
        pointStamped.point.y = self.target_y
        
        self._point_pub.publish(pointStamped) 

    def goal_cb(self, msg:PointStamped):
        can_transform = self.tfBuffer.can_transform('odom', msg.header.frame_id, msg.header.stamp, rclpy.duration.Duration(seconds=0.01))
        if not can_transform:
            return
        
        try:
            transform_base2odom = self.tfBuffer.lookup_transform('odom', msg.header.frame_id, msg.header.stamp, rclpy.duration.Duration(seconds=0.05))
        except Exception as e:
            print(e)
            return
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.header.stamp = msg.header.stamp
        target_pose.pose.position.x = msg.point.x
        target_pose.pose.position.y = msg.point.y
        target_pose.pose.orientation.w = 1.0
        target_pose.pose.orientation.x = target_pose.pose.orientation.y = target_pose.pose.orientation.z = 0.0
        target_pose_odom = do_transform_pose_stamped(target_pose, transform_base2odom)
        self.goal_t = msg.header.stamp
        self.target_x = target_pose_odom.pose.position.x
        self.target_y = target_pose_odom.pose.position.y
        self.publish_point()
        self.get_logger().info('Received goal: x: %f, y: %f' % (self.target_x, self.target_y))
        
    def map_cb(self, msg):
        self.map = msg
        
    def dijkstra(self):
        dt = (self.get_clock().now() - self.last_time).nanoseconds / 1e9
        self.last_time = self.get_clock().now()
        
        if len(self.map.data) == 0 or self.target_x is None or self.target_y is None:
            return None
        
        # Compute robot's position
        child_frame = 'base_link'
        parent_frame = 'odom'
        
        can_trans, msg = self.tfBuffer.can_transform(parent_frame, child_frame, self.goal_t, rclpy.duration.Duration(seconds=0.05), return_debug_tuple=True)
        if not can_trans:
            self.get_logger().warn("Transform unavailable. {}".format(msg))
            return
        
        
        s_t = time.time()
        
        # ====================
        # ==== PREP START ====
        # ====================
        prep_start_t = time.time()
        transform = self.tfBuffer.lookup_transform(parent_frame, child_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.05))
        self.origin_x = transform.transform.translation.x
        self.origin_y = transform.transform.translation.y

        # -- check if the goal is too close to the starting point, if yes, do not generate plan --
        if np.sqrt((self.target_x - self.origin_x)**2 + (self.target_y - self.origin_y)**2) <= self.NO_PLAN_DISTANCE_THRESHOLD:
            self.get_logger().warn("Goal is too close to the starting point")
            return
        
        map_origin_x, map_origin_y = self.from_coordinates_to_grid_index(self.origin_x, self.origin_y)
        map_target_x, map_target_y = self.from_coordinates_to_grid_index(self.target_x, self.target_y)
        self.get_logger().info("Target: %f %f, %d %d" % (self.target_x, self.target_y, map_target_x, map_target_y))
        self.get_logger().info("Origin: %f %f, %d %d" % (self.origin_x, self.origin_y, map_origin_x, map_origin_y))
                                
        matrix_map = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))
        if map_target_x < 0 or map_target_x >= len(matrix_map) or map_target_y < 0 or map_target_y >= len(matrix_map[0]):
            return None
        if map_origin_x < 0 or map_origin_x >= len(matrix_map) or map_origin_y < 0 or map_origin_y >= len(matrix_map[0]):
            return None
        
        # Good filter but conflicts with the exploration phase
        if matrix_map[map_target_x, map_target_y] == GRID_STATES.OCCUPIED:
            self.get_logger().info("Destination is an obstacle")
            return None
        elif matrix_map[map_target_x, map_target_y] == GRID_STATES.BORDER:
            self.get_logger().info("Destination is a border")
        
        # -- add obstacle inflation to enable safer planning --
        threshold_nb_cells = self.from_meters_to_square_nb(self.threshold_distance)
        self.get_logger().info("Inflation size in number of cells: %d" % threshold_nb_cells)
        prep_end_t = time.time()
        self.get_logger().info("Preparation took: {:.5f}".format(prep_end_t - prep_start_t))
        # ==================
        # ==== PREP END ====
        # ==================
        
        # =========================
        # ==== INFLATION START ====
        # =========================
        inflation_start_t = time.time()
        conv_kernel = np.ones((threshold_nb_cells, threshold_nb_cells))
        conv_result = signal.convolve2d(matrix_map, conv_kernel, mode='same', boundary='fill', fillvalue=GRID_STATES.OCCUPIED)
        matrix_map = np.where(np.abs(conv_result) > 0, GRID_STATES.OCCUPIED, matrix_map)
        x_start = map_target_x
        x_end = map_target_x
        y_start = map_target_y
        y_end = map_target_y
        if map_target_x >= 10:
            x_start -= 10
        else:
            x_start = 0
        if map_target_x < (self.map.info.width - 10):
            x_end += 10
        else:
            x_end = self.map.info.width - 1
        if map_target_y >= 10:
            y_start -= 10
        else:
            y_start = 0
        if map_target_y < (self.map.info.height - 10):
            y_end += 10
        else:
            y_end = self.map.info.height - 1
        self.get_logger().info("{}, {}, {}, {}".format(str(int(x_start)), str(int(x_end)), str(int(y_start)), str(int(y_end))))
        matrix_map[int(x_start):int(x_end), int(y_start):int(y_end)] = GRID_STATES.EMPTY
        
        inflated_map = deepcopy(self.map)
        inflated_map.data = matrix_map.flatten().tolist()
        self.inflated_map_pub.publish(inflated_map)
        
        inflation_end_t = time.time()
        self.get_logger().info("Inflation took: {:.5f}".format(inflation_end_t - inflation_start_t))
        # =======================
        # ==== INFLATION END ====
        # =======================
        
        # ========================
        # ==== DIJKSTRA START ====
        # ========================
        # -- initialize variables --
        dijkstra_start_t = time.time()
        queue = []
        visited = np.zeros_like(matrix_map).astype(bool) # (H, W)
        distance = np.ones_like(matrix_map) * np.inf # (H, W)
        links = np.ones((matrix_map.shape[0], matrix_map.shape[1], 2)).astype(int) * -1 # (H, W, 2)
        # -- start with the starting position --
        distance[map_origin_x][map_origin_y] = 0
        queue.append([map_origin_x, map_origin_y])
        
        def check_next_point(old_x, old_y, next_x, next_y, cost):
            if  next_x < 0 or next_x >= len(matrix_map) or next_y < 0 or next_y >= len(matrix_map[0]):
                return
            
            if (visited[next_x][next_y] or matrix_map[next_x][next_y] == GRID_STATES.OCCUPIED or \
                (matrix_map[next_x][next_y] == GRID_STATES.BORDER and (next_x != map_target_x or next_y != map_target_y))):
                return
            
            if distance[next_x][next_y] > distance[old_x][old_y] + cost:
                distance[next_x][next_y] = distance[old_x][old_y] + cost
                links[next_x][next_y][0] = old_x
                links[next_x][next_y][1] = old_y
                queue.append([next_x, next_y])
        
        while len(queue) > 0:
            [x, y] = queue.pop(0)
            visited[x][y] = True
            if x == map_target_x and y == map_target_y:
                self.completed = True
                break
            
            check_next_point(x, y, x, y+1, 1)
            check_next_point(x, y, x, y-1, 1+0.5)
            check_next_point(x, y, x+1, y, 1)
            check_next_point(x, y, x-1, y, 1+0.5)
            
        if not self.completed:
            self.get_logger().warn("No path found")
            return None
        
        dijkstra_end_t = time.time()
        self.get_logger().info("Dijkstra took: {:.5f}".format(dijkstra_end_t - dijkstra_start_t))
        # ======================
        # ==== DIJKSTRA END ====
        # ======================
        
        # ============================
        # ==== PREP COMMAND START ====
        # ============================
        prep_cmd_start_t = time.time()
        self.completed = False    
        x, y = map_target_x, map_target_y
        while links[x][y][0] != -1 and links[x][y][1] != -1:
            x, y = links[x][y]
            self.prep_path(self.get_clock().now().to_msg(), x, y, 0.0)

        self.prep_path(self.get_clock().now().to_msg(), map_origin_x, map_origin_y, 0.0)
        self._path_pub.publish(self._path)

        self.target_x = self.target_y = None
        self._path = Path()
        prep_cmd_end_t = time.time()
        self.get_logger().info("Preparation of command took: {:.5f}".format(prep_cmd_end_t - prep_cmd_start_t))
        # ==========================
        # ==== PREP COMMAND END ====
        # ==========================
        
        e_t = time.time()
        self.get_logger().info("Elapsed time: {:.5f}".format(e_t - s_t))
        
    def prep_path(self, stamp, x, y, yaw):
        """Takes a 2D pose appends it to the path and publishes the whole path.

        Keyword arguments:
        stamp -- timestamp of the transform
        x -- x coordinate of the 2D pose
        y -- y coordinate of the 2D pose
        yaw -- yaw of the 2D pose (in radians)
        """

        self._path.header.stamp = stamp
        self._path.header.frame_id = 'map'

        pose = PoseStamped()
        pose.header = self._path.header
        world_x, world_y = self.from_grid_index_to_coordinates(x, y)
        pose.pose.position.x = world_x
        pose.pose.position.y = world_y
        pose.pose.position.z = 0.01  # 1 cm up so it will be above ground level

        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self._path.poses.append(pose)

    def from_grid_index_to_coordinates(self, x, y):
        """Transforms a pair of map coordinates to world coordinates.

        Keyword arguments:
        x -- x coordinate in the map
        y -- y coordinate in the map

        Returns:
        x -- x coordinate in the world
        y -- y coordinate in the world
        """

        map_x, map_y = x, y

        y = map_x * self.map.info.resolution + self.map.info.origin.position.y + self.map.info.resolution
        x = map_y * self.map.info.resolution + self.map.info.origin.position.x + self.map.info.resolution

        return x, y

    def from_coordinates_to_grid_index(self, x, y):
        """Transforms a pair of world coordinates to map coordinates.

        Keyword arguments:
        x -- x coordinate in the world
        y -- y coordinate in the world

        Returns:
        x -- x coordinate in the map
        y -- y coordinate in the map
        """

        world_x, world_y = x-self.map.info.origin.position.x, y-self.map.info.origin.position.y

        y = int(world_x / self.map.info.resolution)
        x = int(world_y / self.map.info.resolution)

        return x, y  
    
    def from_meters_to_square_nb(self, meters):
        """Transforms a distance in meters to the number of squares.

        Keyword arguments:
        meters -- distance in meters

        Returns:
        squares -- distance in squares
        """

        return math.ceil(meters / self.map.info.resolution)


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