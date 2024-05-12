import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
from tf2_ros import TransformException
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
import rclpy
from copy import deepcopy
from std_msgs.msg import Int16

class PickIsReady(TemplateBehavior):
    def __init__(self, name="pick_is_ready"):
        super().__init__(name)
        self.register_value('is_pick_ready', read=True, write=True)
        
    def initialise(self) -> None:
        return super().initialise()

    def update(self):
        try:
            is_pick_ready = self.blackboard.get('is_pick_ready')
        except:
            return py_trees.common.Status.FAILURE
        
        if not is_pick_ready:
            return py_trees.common.Status.FAILURE
        else:
            self.blackboard.set('is_pick_ready', False)
            return py_trees.common.Status.SUCCESS