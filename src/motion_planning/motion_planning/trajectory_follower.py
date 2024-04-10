import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import numpy as np
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion

class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')
        self.create_timer(0.018, self.timer_callback)
        self.get_logger().info('Trajectory Follower has been started')
        
        self.path_sub = self.create_subscription(Path, '/path', self.path_cb, 10)
        self.path = None
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/motor_controller/twist', 10)
        
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        
        self.WAY_POINT_THD = 0.215 # indicate the minimum distance that the way point should be from our robot (the radius of the effort)
        self.GOAL_REACH_THD = 0.105 # defines the tolerance of goal reaching
        
        self.OMEGA_P = 2.9
        self.VEL_P = 3.35
        self.ANGLE_DIFF_THD = 60 / 180 * np.pi
        self.DRIVE_FAST_ANGLE_DIFF_THD = 20 / 180 * np.pi
        
        self.x = self.y = None
        self.last_time = self.get_clock().now()

    def timer_callback(self):
        self.get_logger().info('Trajectory Follower is running')
        
        if self.path is None or self.poses is None:
            # -- if self.poses is None, then it means we have reached the destination of the current trajectory --
            twist = Twist()
            self.cmd_vel_pub.publish(twist) 
            return
        
        try:
            transform = self.buffer.lookup_transform('map', \
                'base_link', rclpy.time.Time(seconds=0))
            self.x = transform.transform.translation.x
            self.y = transform.transform.translation.y
            theta = euler_from_quaternion([transform.transform.rotation.x, transform.transform.rotation.y, \
                transform.transform.rotation.z, transform.transform.rotation.w])[2]
        except TransformException as e:
            return
        
        dt = (self.get_clock().now() - self.last_time).nanoseconds / 1e9
        self.last_time = self.get_clock().now()
        
        rob_pos = np.array([[self.x, self.y]])
        dists = np.linalg.norm(self.poses - rob_pos, axis=1)
        index = np.argmax(dists >= self.WAY_POINT_THD)
        
        if index == 0:
            if len(self.poses) == 1 and dists[0] < self.GOAL_REACH_THD:
                self.poses = None
            elif np.sum(dists >= self.WAY_POINT_THD) == 0:
                self.poses = self.poses[-1][None,:] # set the last point as the target
            return
        elif index >= (len(self.poses)-1):
            self.poses = self.poses[-1][None,:]
            return
        else:
            self.poses = self.poses[index:]
            
        target = self.poses[0] # (x, y)
        angle = np.arctan2(target[1]-self.y, target[0]-self.x) - theta
        distance = dists[index]
        
        omega = -angle * self.OMEGA_P
        vel = distance * self.VEL_P if np.abs(angle) <= self.ANGLE_DIFF_THD else 0.0 
        vel = (vel * 1.35) if np.abs(angle) <= self.DRIVE_FAST_ANGLE_DIFF_THD else vel
        twist = Twist()
        twist.angular.z = omega
        twist.linear.x = vel
        self.cmd_vel_pub.publish(twist) 
        
    def path_cb(self, msg:Path):
        self.path = msg
        self.poses = np.zeros((len(msg.poses), 2))
        
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
