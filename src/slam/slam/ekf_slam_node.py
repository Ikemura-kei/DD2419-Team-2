import numpy as np
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from .line_extractor import *
from builtin_interfaces.msg import Duration
from copy import deepcopy

class EKF_SLAM_Node(Node):
    def __init__(self) -> None:
        super().__init__('ekf_slam_node')
        self.line_extractor = LineExtractor(EPSILON=0.028, DELTA=0.08, S_NUM=4, P_MIN=8, L_MIN=1.5, L_MAX=5)
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.scan_sub

        self.marker_pub = self.create_publisher(MarkerArray, '/extracted_lines', 10)

        self.marker_template = self.prep_marker_template()

        self.MIN_RANGE = 1.251

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

        markers = MarkerArray()
        for i, line in enumerate(lines):
            g = (i % 2 == 0) * 1.0
            b = (i % 2 == 1) * 1.0

            _, _, m, c = line
            print(f"line {i}: y = {m}x + {c}")
            markers.markers.append(self.prep_marker(g, b, s[i,0], s[i,1], e[i,0], e[i,1], i))

        self.marker_pub.publish(markers)
        print("")

    def prep_marker_template(self):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.points = []
        
        marker.scale.x = 0.05
        marker.scale.y = 0.01

        marker.lifetime = Duration(sec=0, nanosec=int(0.25 * 1e9))

        return marker

    def prep_marker(self, g, b, x1, y1, x2, y2, id):
        marker = deepcopy(self.marker_template)
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
    node = EKF_SLAM_Node()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()