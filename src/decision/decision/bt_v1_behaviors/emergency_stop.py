import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior
from geometry_msgs.msg import PoseStamped, PointStamped
from tf2_ros import TransformException
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
import rclpy
from std_msgs.msg import Bool

class EmergencyStop(TemplateBehavior):
    def __init__(self, name="emergency_stop"):
        super().__init__(name)
        
        self.register_value('joy', read=True, write=False)

    def initialise(self) -> None:
        return super().initialise()

    def update(self):
        try:
            joy = self.blackboard.get('joy')
        except:
            return py_trees.common.Status.SUCCESS
        
        stop = Bool()
        stop.data = (joy.buttons[0] == 1) # if x is pressed down
        self.node.emergency_stop_pub.publish(stop)
        
        if stop.data:
            return py_trees.common.Status.FAILURE
        else: 
            return py_trees.common.Status.SUCCESS