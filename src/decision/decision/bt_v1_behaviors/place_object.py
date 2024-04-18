from py_trees.common import Status
import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Bool

class PlaceObject(TemplateBehavior):
    def __init__(self, name="place_object"):
        super().__init__(name)
        # -- the time when we start placing object, used to determine if the object is placed successfully --
        self.register_value('place_start_time', read=True, write=True)
        
        self.start_time = None
        self.START_COOLDOWN = 8.12

    def initialise(self) -> None:
        self.is_place_cmd_published = False
        self.start_time = self.node.get_clock().now()
            
        return super().initialise()

    def update(self):
        dt_since_start = (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9
        if dt_since_start <= self.START_COOLDOWN:
            return py_trees.common.Status.RUNNING
        
        if not self.is_place_cmd_published:
            self.node.drop_pub.publish(Bool(data=True))
            self.is_place_cmd_published = True
            self.blackboard.set('place_start_time', self.node.get_clock().now())
        
        return py_trees.common.Status.RUNNING