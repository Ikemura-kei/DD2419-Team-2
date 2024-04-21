import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
from tf2_ros import TransformException
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
import rclpy
from copy import deepcopy
from std_msgs.msg import Int16

class MoveBackDone(TemplateBehavior):
    def __init__(self, name="move_back_done"):
        super().__init__(name)
        self.register_value('move_back_start_t', read=True, write=True)
        
    def initialise(self) -> None:
        self.sent = False
        return super().initialise()

    def update(self):
        
        try:
            move_back_start_t = self.blackboard.get('move_back_start_t')
        except:
            return py_trees.common.Status.FAILURE
        
        if move_back_start_t is None:
            return py_trees.common.Status.FAILURE
        
        dt = (self.node.get_clock().now() - move_back_start_t).nanoseconds / 1e9
        if dt > 3.5:
            self.blackboard.set('move_back_start_t', None)
            mode = Int16()
            mode.data = 2
            self.node.traj_follower_mode_pub.publish(mode)
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.FAILURE