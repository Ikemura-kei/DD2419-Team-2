import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior
import rclpy

class ObjectInBox(TemplateBehavior):
    def __init__(self, name="object_in_box"):
        super().__init__(name)
        self.register_value(key="place_start_time", read=True, write=False)

        self.DONE_TIME_THRESHOLD = rclpy.duration.Duration(seconds=5.5)
        
    def initialise(self) -> None:
        return super().initialise()

    def update(self):
        try:
            place_start_time = self.blackboard.get('place_start_time')
        except:
            return py_trees.common.Status.FAILURE
        
        dt = place_start_time - self.node.get_time()
        if dt > self.DONE_TIME_THRESHOLD:
            return py_trees.common.Status.SUCCESS
        else:    
            return py_trees.common.Status.FAILURE