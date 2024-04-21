import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
from tf2_ros import TransformException
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
import rclpy
from copy import deepcopy
from std_msgs.msg import Int16

class MoveBack(TemplateBehavior):
    def __init__(self, name="move_back"):
        super().__init__(name)
        self.register_value('move_back_start_t', read=False, write=True)
        
        self.sent = False

    def initialise(self) -> None:
        self.sent = False
        return super().initialise()

    def update(self):
        
        mode = Int16()
        mode.data = 3
        self.node.traj_follower_mode_pub.publish(mode)
        
        if not self.sent:
            self.sent = True
            self.blackboard.set('move_back_start_t', self.node.get_clock().now())
            timed_twist = Twist()
            timed_twist.linear.y = 1.9
            timed_twist.linear.x = -1.6
            timed_twist.angular.z = 0.45
            self.node.timed_twist_pub.publish(timed_twist)
            
        return py_trees.common.Status.RUNNING