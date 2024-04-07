import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior

class WonderAround(TemplateBehavior):
    def __init__(self, name="wonder_around"):
        super().__init__(name)

    def initialise(self) -> None:
        return super().initialise()

    def update(self):
        # TODO: send command to wonder around here
        
        return py_trees.common.Status.RUNNING