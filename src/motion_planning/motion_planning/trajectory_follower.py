import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import numpy as np
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')
        self.create_timer(0.01, self.timer_callback)
        self.get_logger().info('Trajectory Follower has been started')
        
        self.path_sub = self.create_subscription(Path, '/path', self.path_cb, 10)
        self.path = None
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/motor_controller/twist', 10)
        
        self.way_point_marker_pub = self.create_publisher(Marker, '/way_point_marker', 10)
        
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        
        self.WAY_POINT_THD = 0.22 # indicate the minimum distance that the way point should be from our robot (the radius of the effort)
        self.GOAL_REACH_THD = 0.035 # defines the tolerance of goal reaching
        
        self.OMEGA_P = 2.5
        self.VEL_P = 3.95
        self.ANGLE_DIFF_THD = 55 / 180 * np.pi
        self.DRIVE_FAST_ANGLE_DIFF_THD = 20 / 180 * np.pi
        
        self.x = self.y = None
        self.last_time = self.get_clock().now()
        
        self.failure_cnt = 0
        self.TF_FAILURE_CNT_THRESHOLD = 100
        self.SMALL_ANGLE_SPEED_UP_FACTOR = 1.3
        self.UPPER_ANGLE_DIFF_THD = 125 / 180 * np.pi
        self.ANGLE_TOO_LARGE_SLOW_DONW_FACTOR = 0.75
        self.DISTANCE_SHORT_THRESHOLD = self.GOAL_REACH_THD + 0.05
        self.NEAR_GOAL_BOOTS = 0.39
        
        self.poses = None

    def timer_callback(self):
        self.get_logger().info('Trajectory Follower is running')
        
        if self.path is None or self.poses is None or self.failure_cnt > self.TF_FAILURE_CNT_THRESHOLD:
            # -- if self.poses is None, then it means we have reached the destination of the current trajectory --
            twist = Twist()
            self.cmd_vel_pub.publish(twist) 
            
            self.get_logger().info("Stop.")
            if self.poses is None:
                self.get_logger().info("Trajectory Follower has reached the destination")
            if self.failure_cnt > self.TF_FAILURE_CNT_THRESHOLD:
                self.get_logger().info("Trajectory Follower has failed to get the transform too many times")
                
            return
        
        # -- check transformation availability, if not increment failure count, otherwise reset failure count --
        can_transform = self.buffer.can_transform('map', 'base_link', rclpy.time.Time(seconds=0), rclpy.duration.Duration(seconds=0.005))
        if not can_transform:
            self.failure_cnt += 1
            self.get_logger().warn("Transform unavailable from 'map' to 'base_link'")
            return
        self.failure_cnt = 0
        
        try:
            transform = self.buffer.lookup_transform('map', \
                'base_link', rclpy.time.Time(seconds=0), rclpy.duration.Duration(seconds=0.005))
            self.x = transform.transform.translation.x
            self.y = transform.transform.translation.y
            theta = euler_from_quaternion([transform.transform.rotation.x, transform.transform.rotation.y, \
                transform.transform.rotation.z, transform.transform.rotation.w])[2]
        except TransformException as e:
            self.get_logger().warn(str(e))
            return
        
        dt = (self.get_clock().now() - self.last_time).nanoseconds / 1e9
        self.last_time = self.get_clock().now()
        
        rob_pos = np.array([[self.x, self.y]])
        dists = np.linalg.norm(self.poses - rob_pos, axis=1)
        
        # -- NOTE: if there is only one pose, then we just aim at that pose --
        if len(self.poses) > 1:
            # -- retrieve the next closest valid way point, if any --
            index = np.argmax(dists >= self.WAY_POINT_THD)
            
            # -- returned index is zero either because (1) the next valid way point is actually the first point (2) no valid way point --
            if index == 0:
                # -- case (2), no valid way point --
                if np.sum(dists >= self.WAY_POINT_THD) == 0:
                    self.poses = self.poses[-1][None,:] # set the last point as the target
                    self.get_logger().info("No valid way point, just aim at the last point")
                    
                # -- case (1), the next valid way point is actually the first point --
                else:
                    self.get_logger().info("The first point is the way point")
                    pass # just aim at that way point
            # -- returned index is not zero, but is just the last index --
            elif index == (len(self.poses)-1):
                self.poses = self.poses[-1][None,:] # set the last point as the target
                self.get_logger().info("The last point is the way point")
            else:
                # -- remove any point before the way point, no need to examine them any more in the future --
                self.get_logger().info("The {}th point is the way point".format(index))
                self.poses = self.poses[index:]
        else:
            index = 0
            self.get_logger().info("Only one last pose, that is the destination, distance to goal: {}".format(dists[-1]))
            # -- judge if we are close to goal, if yes, then stop --
            if dists[-1] <= self.GOAL_REACH_THD:
                self.poses = None
                return # don't semd any command now, next loop will force stop
        
        # -- by the above processing, the first point in the pose list should just be our target way point --
        target = self.poses[0] # (x, y)
        
        # -- prep visualization --
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'way_point'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.pose.position.x = target[0]
        marker.pose.position.y = target[1]
        marker.pose.position.z = 0.0
        marker.lifetime = rclpy.time.Duration(seconds=0.5).to_msg()
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.way_point_marker_pub.publish(marker)
        
        self.get_logger().info("Remained number of trajectory points: {}".format(len(self.poses)))
        
        # -- now we compute the motion command to move toward the target way point --
        angle = np.arctan2(target[1]-self.y, target[0]-self.x) - theta
        distance = dists[index]
        
        angle = 0.0 if distance < self.DISTANCE_SHORT_THRESHOLD else angle # if the distance is too short, we just go straight
        omega = -angle * self.OMEGA_P
        vel = distance * self.VEL_P if np.abs(angle) <= self.ANGLE_DIFF_THD else 0.0 # if the angle is too large, we just rotate
        vel = (vel * self.SMALL_ANGLE_SPEED_UP_FACTOR) \
            if np.abs(angle) <= self.DRIVE_FAST_ANGLE_DIFF_THD else vel # if the angle is small, we speed up
        vel = vel + self.NEAR_GOAL_BOOTS if distance < self.DISTANCE_SHORT_THRESHOLD else vel # if the distance is too short, we speed up
            
        omega = omega * self.ANGLE_TOO_LARGE_SLOW_DONW_FACTOR if np.abs(angle) >= self.UPPER_ANGLE_DIFF_THD else omega
        twist = Twist()
        twist.angular.z = omega
        twist.linear.x = vel
        self.cmd_vel_pub.publish(twist) 
        
    def path_cb(self, msg:Path):
        self.path = msg
        self.poses = np.zeros((len(msg.poses), 2))
        
        # -- pose message seems to be reverted (i.e. goal point comes first), so we insert them backward --
        for i in range(len(msg.poses)-1, -1, -1):
            pose = msg.poses[i]
            self.poses[len(msg.poses)-1-i, 0] = pose.pose.position.x
            self.poses[len(msg.poses)-1-i, 1] = pose.pose.position.y
        
def main():
    rclpy.init()
    node = TrajectoryFollower()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
