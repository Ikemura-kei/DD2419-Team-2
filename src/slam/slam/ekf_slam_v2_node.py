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

def yaw_to_rot_mat(yaw):
    c = np.cos(yaw)
    s = np.sin(yaw)
    
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def ego2global(x, y, th, pnts):
    # (N, 2)
    homo = yaw_to_rot_mat(th)
    homo[0, 2] = x
    homo[1, 2] = y
    
    N = len(pnts)
    pnts_homo = np.concatenate([pnts, np.ones((N, 1))], axis=-1) # (N, 3)
    pnts_homo = pnts_homo[...,None] # (N, 3, 1)
    new_pnts = np.squeeze(np.matmul(homo, pnts_homo), axis=-1) # (N, 3)
    
    return new_pnts[:,:2] # (N, 2)

class EKF_SLAM_Node(Node):
    def __init__(self) -> None:
        super().__init__('ekf_slam_node')
        self.START_T = 1.5
        self.L_MIN = 1.05
        self.L_MAX = 2.85
        self.line_extractor = LineExtractor(EPSILON=0.028, DELTA=0.07, S_NUM=4, P_MIN=8, L_MIN=self.L_MIN, L_MAX=self.L_MAX)
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.scan_sub

        self.odometry_sub = self.create_subscription(Odometry, '/odometry', self.odometry_cb, 10)
        self.odometry_sub

        self.line_marker_pub = self.create_publisher(MarkerArray, '/extracted_lines', 10)
        self.line_landmark_marker_pub = self.create_publisher(MarkerArray, '/landmark_lines', 10)
        self.point_marker_pub = self.create_publisher(MarkerArray, '/extracted_points', 10)

        self.path_pub = self.create_publisher(Path, '/ekf_slam_traj', 10)

        self._tf_broadcaster = TransformBroadcaster(self)

        self.line_marker_template = self.prep_line_marker_template()
        self.point_marker_template = self.prep_point_marker_template()
        self.MIN_RANGE = 0.8
        self.MAX_RANGE = 4.45
        self.last_odom_time = None
        self.MAX_NUM_LANDMARK = 6
        self.mu = np.zeros(3+2*self.MAX_NUM_LANDMARK)
        self.sigma = np.eye(3+2*self.MAX_NUM_LANDMARK)

        self.R = np.zeros((3+2*self.MAX_NUM_LANDMARK, 3+2*self.MAX_NUM_LANDMARK)) # motion noise
        self.R[:3, :3] = np.diag([0.78**2, 0.59**2, 7.35**2])
        self.Q = np.diag([0.05**2, 0.15**2]) # measurement noise

        self.path = Path()
        self.path.header.frame_id = 'world'

        self.b = np.zeros(self.MAX_NUM_LANDMARK) # for book keeping on landmark observation
        self.num_obs_landmarks = 0

        self.lock = False

        self.line_id = 0
        self.pnt_id = 500
        self.start = False

        self.is_logging = True
        if self.is_logging:
            np.set_printoptions(threshold=sys.maxsize)

            self.log_topic = self.create_publisher(String, '/ekf_slam_log', 10)

    def scan_cb(self, scan:LaserScan):
        self.get_logger().info("Scan received")
        while(self.lock):
            pass
        self.lock = True
        mu_cp = self.mu.copy()
        sigma_cp = self.sigma.copy()
        self.lock = False

        ranges = np.array(scan.ranges) # (N,)
        bearings = np.linspace(scan.angle_min, scan.angle_max, len(ranges)) # (N,)

        assert len(ranges) == len(bearings) # check if the number of ranges and bearings are the same

        # -- filter out invalid ranges --
        valid_indices = np.where(np.isfinite(ranges) * (ranges >= self.MIN_RANGE) * (ranges <= self.MAX_RANGE))[0]
        ranges = ranges[valid_indices]
        bearings = bearings[valid_indices]

        # -- map ranges and bearings to (x, y) coordinates --
        xs = ranges * np.cos(bearings)
        ys = ranges * np.sin(bearings)
        pnts = np.vstack([xs, ys]).T # (N, 2)

        # -- extract lines from the points --
        s, e, lines = self.line_extractor.compute(pnts, ranges, bearings)
        if len(lines) == 0:
            self.get_logger().info('No lines detected.')
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
        z = np.stack([ms, cs], axis=1) # (K, 2)

        line_markers = MarkerArray()
        pnt_markers = MarkerArray()
        for i, line in enumerate(lines):
            self.line_id += 1
            self.pnt_id += 1

            m = ms[i]
            c = cs[i]
            
            print(f"line {i}: y = {m}x + {c}")
            line_markers.markers.append(self.prep_line_marker(0.13, 0.87, s[i,0], s[i,1], e[i,0], e[i,1], self.line_id))
            pnt_markers.markers.append(self.prep_point_marker(0.87, 0.13, 1, z[i,1]+z[i,0], self.pnt_id))

        self.line_marker_pub.publish(line_markers)
        self.point_marker_pub.publish(pnt_markers)

        # -- update the ekf state --
        if not self.start:
            return
        
        while(self.lock):
            pass
        self.lock = True
        print(f"mu: {mu_cp}")
        print(f"b: {self.b}")
        self.mu, self.sigma, self.b = self.update(z, mu_cp, sigma_cp, self.Q, self.b)
        self.lock = False

    def odometry_cb(self, odometry:Odometry):
        self.get_logger().info("Odom received")
        if self.last_odom_time is None:
            self.last_odom_time = odometry.header.stamp
            self.start_t = odometry.header.stamp
            return
        if ((odometry.header.stamp.sec + odometry.header.stamp.nanosec / 1e9) - (self.start_t.sec + self.start_t.nanosec / 1e9)) > self.START_T:
            self.start = True
        dt = (odometry.header.stamp.sec + odometry.header.stamp.nanosec / 1e9) - (self.last_odom_time.sec + self.last_odom_time.nanosec / 1e9)
        self.last_odom_time = odometry.header.stamp

        # -- construct the control input --
        # print(f"dt: {dt}")
        u = np.array([odometry.twist.twist.linear.x, odometry.twist.twist.angular.z, dt])

        # -- predict the state and covariance --
        while(self.lock):
            pass
        self.lock = True
        self.mu, self.sigma = self.predict(u, self.mu, self.sigma, self.R)
        mu_cp = self.mu.copy()
        self.lock = False
        # self.mu[0] += np.random.normal(0, 0.01) # add noise to the x position
        # self.mu[2] += np.random.normal(0, 0.007) # add noise to the heading

        # -- compute and broadcast the difference between odometry estimate and the current ekf estimate --
        q_odom = [odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w]
        q_ekf = quaternion_from_euler(0, 0, mu_cp[2])
        q_diff = quaternion_multiply(q_ekf, quaternion_conjugate(q_odom))
        pos_odom = np.array([odometry.pose.pose.position.x, odometry.pose.pose.position.y, 0, 0])
        pos_odom_rotated = quaternion_multiply(q_diff, quaternion_multiply(pos_odom, quaternion_conjugate(q_diff)))
        translation = np.array([mu_cp[0] - pos_odom_rotated[0], mu_cp[1] - pos_odom_rotated[1], 0])
        self._tf_broadcaster.sendTransform(self.prep_transform(translation[0], translation[1], q_diff, odometry.header.stamp))

        # -- construct path and landmark for visualization --
        self.path.header.stamp = odometry.header.stamp
        self.path.poses.append(self.construct_pose(mu_cp[0], mu_cp[1], mu_cp[2], odometry.header.stamp))
        if len(self.path.poses) > 1500:
            self.path.poses.pop(0)
        self.path_pub.publish(self.path)

        line_markers = MarkerArray()
        for i in range(self.num_obs_landmarks):
            m = self.mu[3+i*2]
            c = self.mu[4+i*2]
            x1 = -9.0
            x2 = 9.0
            line = self.prep_line_marker(0.75, 0.25, x1, x1*m+c, x2, x2*m+c, i)
            line.header.frame_id = 'world'
            # line.lifetime = Duration(sec=0, nanosec=int(0.5 * 1e9))
            line_markers.markers.append(line)
        self.line_landmark_marker_pub.publish(line_markers)

    def prep_transform(self, x, y, q, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'map'
        t.transform.translation.x = x   
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        return t
    
    def pnt_on_line_closest_to(self, pnt, ms, cs):
        K = ms.shape[0]
        pnt = np.tile(pnt[None,:], (K, 1)) # (K, 2)
        a = ms
        b = np.ones((K)).astype(np.float32) * -1
        c = cs
        
        b_sqr = b ** 2
        a_sqr = a ** 2
        # print(b_sqr.shape, a_sqr.shape)
        ab = a * b
        bc = b * c
        ac = a * c
        a_sqr_plus_b_sqr = a_sqr + b_sqr
        
        new_pnts = np.zeros_like(pnt).astype(np.float32)
        # print(b_sqr.shape, pnt[:,0].shape, ab.shape, pnt[:,1].shape, ac.shape, a_sqr_plus_b_sqr.shape)
        new_pnts[:,0] = (b_sqr * pnt[:,0] - ab * pnt[:,1] - ac) / a_sqr_plus_b_sqr
        new_pnts[:,1] = (a_sqr * pnt[:,1] - ab * pnt[:,0] - bc) / a_sqr_plus_b_sqr
        
        return new_pnts

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
    
    def update(self, z:np.ndarray, mu:np.ndarray, sigma:np.ndarray, Q:np.ndarray, b:np.ndarray):
        two_N = 2 * self.MAX_NUM_LANDMARK
        z_hat = self.measurement_model(mu)

        if self.is_logging:
            log_msg = String()
            log_msg.data = f"mu: {mu}"
            self.log_topic.publish(log_msg)
            log_msg.data = f"b: {b}"
            self.log_topic.publish(log_msg)
            log_msg.data = f"z_hat: {z_hat[:self.num_obs_landmarks]}"
            self.log_topic.publish(log_msg)
            log_msg.data = f"z: {z}"
            self.log_topic.publish(log_msg)

        b, c = self.associate(mu, z_hat[:self.num_obs_landmarks], sigma, z, b)

        for i in range(len(z)):
            if self.is_logging:
                log_msg = String()
                log_msg.data = f"c[{i}]: {c[i]}"
                self.log_topic.publish(log_msg)

            if c[i] == -1:
                continue
            
            H = self.compute_H(mu, c[i]) # (2, 3+2N)
            K = mu @ H.T @ np.linalg.inv(H @ sigma @ H.T + Q)

            innovation = z[i] - z_hat[c[i]]
            # print(f"innovation: {innovation.shape}")
            mu = mu + K @ innovation
            mu[2] = self.wrap_angle(mu[2])
            sigma = (np.eye(3+two_N) - K @ H) @ sigma

        return mu, sigma, b
    
    def get_anchor_in_rob_frame(self, mu:np.ndarray):
        a_x = 1.25
        a_y = 0.5
        x, y, theta = mu[:3]

        cos_th = np.cos(theta)
        sin_th = np.sin(theta)

        anchor_x = cos_th * (a_x-x) + sin_th * (a_y-y) # (N,)
        anchor_y = -sin_th * (a_x-x) + cos_th * (a_y-y) # (N,)

        return np.array([anchor_x, anchor_y])
    
    def measurement_model(self, mu:np.ndarray):
        x, y, theta = mu[:3]

        cos_th = np.cos(theta)
        sin_th = np.sin(theta)
        y_diff = mu[4::2] - y

        m_b = (-sin_th + cos_th * mu[3::2]) / (cos_th + sin_th * mu[3::2])
        c_b = sin_th * x + cos_th * y_diff - (sin_th * cos_th * x - sin_th**2 * y_diff - cos_th**2 * mu[3::2] * x + sin_th * cos_th * mu[3::2] * y_diff) / (cos_th + sin_th * mu[3::2])

        return np.stack([m_b, c_b], axis=1) # (N,2)
    
    def associate(self, mu:np.ndarray, z_hat:np.ndarray, sigma:np.ndarray, z:np.ndarray, b:np.ndarray, outlier_threshold=2.8, new_landmark_threshold=3.25):
        c = np.ones(len(z)).astype(np.int32) * -1
        N = self.MAX_NUM_LANDMARK
        
        new = self.num_obs_landmarks < 1
        K = len(z)
        if new:
            p1_b = np.stack([np.zeros(K), z[:,1]], axis=1) # points of (0, c) in body frame
            p2_b = np.stack([np.ones(K), z[:,1] + z[:, 0]], axis=1) # points of (1, c+m) in body frame
            p1_g = ego2global(mu[0], mu[1], mu[2], p1_b)
            p2_g = ego2global(mu[0], mu[1], mu[2], p2_b)

            m_g = (p2_g[:,1] - p1_g[:,1]) / (p2_g[:,0] - p1_g[:,0] + 1e-9) # (y2 - y1) / (x2 - x1)
            c_g = p1_g[:,1] - m_g * p1_g[:,0] # y - mx
            for i in range(len(z)):
                b[self.num_obs_landmarks] = 1
                mu[3+2*self.num_obs_landmarks] = m_g[i]
                mu[4+2*self.num_obs_landmarks] = c_g[i]
                if (self.num_obs_landmarks + 1) < N:
                    self.num_obs_landmarks += 1
                c[i] = -1
            return b, c

        p_hat = np.stack([np.ones(len(z_hat)), z_hat[:,1] + z_hat[:,0]], axis=1)
        p = np.stack([np.ones(K), z[:,1] + z[:, 0]], axis=1)
        ps = np.tile(p[:,None,:], (1, len(z_hat), 1))
        th_hat = np.arctan2(z_hat[:,0] - z_hat[:,1], 1)
        th = np.arctan2(z[:,0] - z[:,1], 1)
        ths = np.tile(th[:,None], (1, len(z_hat)))
        
        REDICULOUS_THD = 10.25
        for i in range(len(z)):
            diff = p_hat - ps[i]
            # print(f"z_hat: {z_hat}")
            # print(f"zs[i]: {zs[i]}")
            # print(f"diff: {diff}")
            dists = np.sqrt(diff[:,0]**2 + diff[:,1]**2)
            th_diff = np.abs(th_hat-ths[i])

            min_idx = np.argmin(dists)
            print("min dist: {}".format(dists[min_idx]))
            print("min th diff: {}".format(th_diff[min_idx]))
            if dists[min_idx] < outlier_threshold:
                c[i] = min_idx
            elif th_diff[min_idx] < 0.15:
                c[i] = min_idx
            else:
                c[i] = -1
                if dists[min_idx] >= new_landmark_threshold and dists[min_idx] < REDICULOUS_THD:
                    b[self.num_obs_landmarks] = 1
                    p1_b = np.array([[0, z[i,1]]]) # points of (0, c) in body frame
                    p2_b = np.array([[1, z[i,1] + z[i,0]]]) # points of (1, c+m) in body frame
                    p1_g = ego2global(mu[0], mu[1], mu[2], p1_b)[0]
                    p2_g = ego2global(mu[0], mu[1], mu[2], p2_b)[0]

                    m_g = (p2_g[1] - p1_g[1]) / (p2_g[0] - p1_g[0] + 1e-9) # (y2 - y1) / (x2 - x1)
                    c_g = p1_g[1] - m_g * p1_g[0] # y - mx
                    mu[3+2*self.num_obs_landmarks] = m_g
                    mu[4+2*self.num_obs_landmarks] = c_g
                    if (self.num_obs_landmarks + 1) < N:
                        self.num_obs_landmarks += 1

        return b, c
    
    def compute_H(self, mu, c):
        H = np.zeros((2, len(mu)))

        x, y, theta = mu[:3]
        cth = np.cos(theta)
        sth = np.sin(theta)

        m_g = mu[3+2*c]
        c_g = mu[4+2*c]

        diff = c_g - y

        H[0, 2] = -1 - (-sth + cth * m_g)**2 / (cth + sth * m_g)**2
        H[0, 3+2*c] = cth / (cth + sth * m_g) - (-sth + cth * m_g) * sth / (cth + sth * m_g)**2
        H[0, 4+2*c] = 0
        H[1, 0] = sth - (sth * cth - cth**2 * m_g) / (cth + sth * m_g)
        H[1, 1] = -cth - (sth**2 - cth * sth * m_g) / (cth + sth * m_g)
        H[1, 2] = cth * x - sth * diff - ((cth**2 - sth**2) * x - 2 * cth * sth * diff + 2 * cth * sth * m_g * x + (cth**2 - sth**2) * m_g * diff)\
                 / (cth + sth * m_g) + ((-sth + m_g * cth) * (sth * cth * x - sth**2 * diff - cth**2 * m_g * x + sth * cth * m_g * diff)) / (cth + sth * m_g)**2
        H[1, 3+2*c] = - (-cth**2 * x + sth * cth * diff) / (cth + sth * m_g) +\
              sth * (sth * cth * x - sth**2 * diff - cth**2 * m_g * x + sth * cth * m_g * diff) / (cth + sth * m_g)**2
        H[1, 4+2*c] = -H[1, 1]

        return H

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

    def prep_point_marker_template(self):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.5

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
    
    def prep_point_marker(self, r, b, x:float, y:float, id):
        marker = deepcopy(self.point_marker_template)
        marker.id = id

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)

        color = ColorRGBA()
        color.a = 1.0
        color.r = r
        color.g = 0.2
        color.b = b
        marker.color = color

        return marker

def main():
    rclpy.init()
    node = EKF_SLAM_Node()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()