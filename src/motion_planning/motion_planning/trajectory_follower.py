import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import numpy as np
from tf2_ros import Buffer, TransformListener, TransformException

class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')
        self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Trajectory Follower has been started')
        
        self.path_sub = self.create_subscription(Path, '/path', self.path_cb, 10)
        self.path = None
        
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        
        self.THD = 0.2

    def timer_callback(self):
        self.get_logger().info('Trajectory Follower is running')
        
        if self.path is None:
            return
        
        try:
            transform = self.buffer.lookup_transform('base_link', \
                'map', self.path.header.stamp)
        except TransformException as e:
            self.get_logger().error(f"Failed to lookup transform: {e}")
            return
        
        self.x = transform.transform.translation.x
        self.y = transform.transform.translation.y
        
        rob_pos = np.array([[self.x, self.y]])
        dists = np.linalg.norm(self.poses - rob_pos, axis=1)
        index = np.argmax(dists >= self.THD)
        if index == 0:
            return
        elif index >= (len(self.poses)-1):
            self.path = None
            return
        else:
            self.poses = self.poses[index:]
        
    def path_cb(self, msg:Path):
        self.path = msg
        self.poses = np.zeros((len(msg.poses), 2))
        self.get_logger().info('Received a path message')
        
def main():
    rclpy.init()
    node = TrajectoryFollower()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ is '__main__':
    main()