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

class EKF_SLAM_Node(Node):
    def __init__(self) -> None:
        super().__init__('ekf_slam_node')
        self.line_extractor = LineExtractor(EPSILON=0.028, DELTA=0.07, S_NUM=4, P_MIN=8, L_MIN=1.5, L_MAX=5)
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.scan_sub

        self.odometry_sub = self.create_subscription(Odometry, '/odometry', self.odometry_cb, 10)
        self.odometry_sub

        self.marker_pub = self.create_publisher(MarkerArray, '/extracted_lines', 10)

        self.path_pub = self.create_publisher(Path, '/ekf_slam_traj', 10)

        self._tf_broadcaster = TransformBroadcaster(self)

        self.marker_template = self.prep_marker_template()
        self.MIN_RANGE = 1.251
        self.last_odom_time = None
        self.MAX_NUM_LANDMARK = 10
        self.mu = np.zeros(3+2*self.MAX_NUM_LANDMARK)
        self.sigma = np.eye(3+2*self.MAX_NUM_LANDMARK)

        self.R = np.zeros((3+2*self.MAX_NUM_LANDMARK, 3+2*self.MAX_NUM_LANDMARK)) # motion noise
        self.R[:3, :3] = np.diag([0.01, 0.03, 0.01])
        self.Q = np.diag([0.05, 0.015]) # measurement noise

        self.path = Path()
        self.path.header.frame_id = 'map'

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

    def odometry_cb(self, odometry:Odometry):
        if self.last_odom_time is None:
            self.last_odom_time = odometry.header.stamp
            return
        
        dt = (odometry.header.stamp.sec + odometry.header.stamp.nanosec / 1e9) - (self.last_odom_time.sec + self.last_odom_time.nanosec / 1e9)
        self.last_odom_time = odometry.header.stamp

        # -- construct the control input --
        # print(f"dt: {dt}")
        u = np.array([odometry.twist.twist.linear.x, odometry.twist.twist.angular.z, dt])

        # -- predict the state and covariance --
        self.mu, self.sigma = self.predict(u, self.mu, self.sigma, self.R)
        self.mu[0] += np.random.normal(0, 0.01) # add noise to the x position
        self.mu[2] += np.random.normal(0, 0.007) # add noise to the heading

        # -- compute and broadcast the difference between odometry estimate and the current ekf estimate --
        q_odom = [odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w]
        q_ekf = quaternion_from_euler(0, 0, self.mu[2])
        q_diff = quaternion_multiply(q_ekf, quaternion_conjugate(q_odom))
        pos_odom = np.array([odometry.pose.pose.position.x, odometry.pose.pose.position.y, 0, 0])
        pos_odom_rotated = quaternion_multiply(q_diff, quaternion_multiply(pos_odom, quaternion_conjugate(q_diff)))
        translation = np.array([self.mu[0] - pos_odom_rotated[0], self.mu[1] - pos_odom_rotated[1], 0])
        self._tf_broadcaster.sendTransform(self.prep_transform(translation[0], translation[1], q_diff, odometry.header.stamp))

        # -- construct path for visualization --
        self.path.header.stamp = odometry.header.stamp
        self.path.poses.append(self.construct_pose(self.mu[0], self.mu[1], self.mu[2], odometry.header.stamp))
        if len(self.path.poses) > 1500:
            self.path.poses.pop(0)
        self.path_pub.publish(self.path)

    def prep_transform(self, x, y, q, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = x   
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        return t

    def construct_pose(self, x, y, theta, stamp):
        new_pose = PoseStamped()
        new_pose.header.stamp = stamp
        new_pose.pose.position.x = x
        new_pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, theta)
        new_pose.pose.orientation.x = q[0]
        new_pose.pose.orientation.y = q[1]
        new_pose.pose.orientation.z = q[2]
        new_pose.pose.orientation.w = q[3]
        return new_pose

    def predict(self, u:np.ndarray, mu:np.ndarray, sigma:np.ndarray, R:np.ndarray):
        x, y, theta = mu[:3]
        v, w, dt = u

        # -- motion model --
        disp = v * dt
        x += np.cos(theta) * disp
        y += np.sin(theta) * disp
        theta += w * dt
        theta = self.wrap_angle(theta)
        mu[:3] = np.array([x, y, theta])

        # -- motion model jacobian --
        G = self.compute_G(u, mu)

        # -- covariance update --
        sigma = G @ sigma @ G.T + R

        return mu, sigma
    
    def wrap_angle(self, angle):
        return np.mod(angle + np.pi, 2 * np.pi) - np.pi
    
    def compute_G(self, u:np.ndarray, mu:np.ndarray):
        x, y, theta = mu[:3]
        v, w, dt = u
        disp = v * dt
        
        G3 = np.eye(3)
        G3[0, 2] = -disp * np.sin(theta)
        G3[1, 2] = disp * np.cos(theta)

        two_N = 2 * self.MAX_NUM_LANDMARK
        G = np.concatenate([G3, np.zeros((3, two_N))], axis=1) # (3, 3+2N)
        G = np.concatenate([G, np.concatenate([np.zeros((two_N, 3)), np.eye(two_N)], axis=1)], axis=0) # (3+2N, 3+2N)

        return G

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