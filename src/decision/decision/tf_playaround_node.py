import rclpy.duration
from tf2_ros import TransformListener, Buffer, TransformException
from rclpy.node import Node
import rclpy
from tf2_geometry_msgs import do_transform_point
import time

class TfPlayaroundNode(Node):
    def __init__(self, tf_node: Node):
        super().__init__('tf_playaround_node')
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=3))
        self.tf_listener = TransformListener(self.tf_buffer, tf_node, spin_thread=True)
        
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.last_timer_cb_time = self.get_clock().now()
        
    def timer_callback(self):
        
        dt = (self.get_clock().now() - self.last_timer_cb_time).nanoseconds / 1e9
        self.last_timer_cb_time = self.get_clock().now()
        self.get_logger().info('Timer callback time, duration {}'.format(dt))
        
        # -- do some time-consuming work here --
        time.sleep(2)
        
        can_transform, msg = self.tf_buffer.can_transform('base_link', 'odom',\
            (self.get_clock().now() - rclpy.duration.Duration(seconds=1)).to_msg(), rclpy.duration.Duration(seconds=0.01), return_debug_tuple=True)
        if not can_transform:
            self.get_logger().warn('Cannot transform. {}'.format(msg))
            return
        else:
            self.get_logger().info('Can transform.')
            
class TfNode(Node):
    def __init__(self):
        super().__init__('tf_node')
    
def main():
    rclpy.init()
    
    tf_node = TfNode()
    node = TfPlayaroundNode(tf_node)
    
    rclpy.spin(node)
    rclpy.shutdown()