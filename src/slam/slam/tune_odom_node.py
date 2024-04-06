import numpy as np
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Pose, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from .line_extractor import *
from builtin_interfaces.msg import Duration
from copy import deepcopy
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, quaternion_multiply, quaternion_conjugate
from std_msgs.msg import String
import sys

class TuneOdomNode(Node):
    def __init__(self) -> None:
        super().__init__('tune_odom_node')
        self.L_MIN = 1.95
        self.L_MAX = 2.85
        self.MIN_RANGE = 1.251
        self.line_extractor = LineExtractor(EPSILON=0.0272, DELTA=0.0345, S_NUM=4, P_MIN=8, L_MIN=self.L_MIN, L_MAX=self.L_MAX)
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.scan_sub

        self.line_marker_pub = self.create_publisher(MarkerArray, '/extracted_lines', 10)

        self.line_marker_template = self.prep_line_marker_template()
        self.line_id = 0

    def scan_cb(self, scan:LaserScan):

        ranges = np.array(scan.ranges) # (N,)
        bearings = np.linspace(scan.angle_min, scan.angle_max, len(ranges)) # (N,)

        assert len(ranges) == len(bearings) # check if the number of ranges and bearings are the same

        # -- filter out invalid ranges --
        valid_indices = np.where(np.isfinite(ranges) * (ranges >= self.MIN_RANGE))[0]
        ranges = ranges[valid_indices]
        bearings = bearings[valid_indices]

        # -- map ranges and bearings to (x, y) coordinates --
        xs = ranges * np.cos(bearings)
        ys = ranges * np.sin(bearings)
        pnts = np.vstack([xs, ys]).T # (N, 2)

        # -- extract lines from the points --
        s, e, lines = self.line_extractor.compute(pnts, ranges, bearings)
        if len(lines) == 0:
            return

        # -- get the position of the landmarks in robot frame --
        ms = np.zeros((len(lines)))
        cs = np.zeros((len(lines)))
        line_validity = np.zeros((len(lines))).astype(np.int16)
        valid_lines = []
        for i, line in enumerate(lines):
            _, _, m, c = line
            ms[i] = m
            cs[i] = c
            line_len = get_distance_p2p(np.array([[s[i,0], s[i,1]]]), np.array([[e[i,0], e[i,1]]]))[0]
            if line_len > self.L_MIN and line_len <= self.L_MAX:
                line_validity[i] = 1
                valid_lines.append(line)
        
        lines = valid_lines
        s = s[line_validity == 1,...]
        e = e[line_validity == 1,...]
        ms = ms[line_validity == 1]
        cs = cs[line_validity == 1]        

        line_markers = MarkerArray()
        for i, line in enumerate(lines):
            self.line_id += 1

            m = ms[i]
            c = cs[i]
            
            line_markers.markers.append(self.prep_line_marker(0.2, 1.0, s[i,0], s[i,1], e[i,0], e[i,1], self.line_id))

        self.line_marker_pub.publish(line_markers)

    def prep_line_marker_template(self):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.points = []
        
        marker.scale.x = 0.05
        marker.scale.y = 0.01

        marker.lifetime = Duration(sec=0, nanosec=int(0 * 1e9))

        return marker

    def prep_line_marker(self, g, b, x1, y1, x2, y2, id):
        marker = deepcopy(self.line_marker_template)
        marker.id = id

        point = Point()
        point.x = x1
        point.y = y1
        marker.points.append(point)
        point = Point()
        point.x = x2
        point.y = y2
        marker.points.append(point)

        color = ColorRGBA()
        color.a = 1.0
        color.r = 0.1
        color.g = g
        color.b = b
        marker.colors = [color, color]

        return marker
    
def main():
    rclpy.init()
    node = TuneOdomNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()