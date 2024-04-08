import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior
import rclpy

class ObjectInBox(TemplateBehavior):
    def __init__(self, name="object_in_box"):
        super().__init__(name)
        self.register_value(key="place_start_time", read=True, write=False)
        # -- marks if the object (identified by the key) is picked up or not --
        self.register_value(key="object_states", read=False, write=True)
        # -- the name of the object we set to pick, naming format follows that above --
        self.register_value(key="target_object", read=False, write=True)

        self.DONE_TIME_THRESHOLD = rclpy.duration.Duration(seconds=5.5)
        
    def initialise(self) -> None:
        return super().initialise()

    def update(self):
        try:
            place_start_time = self.blackboard.get('place_start_time')
            self.object_states = self.blackboard.get('object_states')
            target_object = self.blackboard.get('target_object')
        except:
            return py_trees.common.Status.FAILURE
        
        dt = self.node.get_clock().now() - place_start_time
        if dt > self.DONE_TIME_THRESHOLD:
            # -- mark the object as picked and placed --
            self.object_states[target_object] = True
            self.blackboard.set('object_states', self.object_states, overwrite=True)
            self.blackboard.unset('place_start_time')
            return py_trees.common.Status.SUCCESS
        else:    
            return py_trees.common.Status.FAILURE