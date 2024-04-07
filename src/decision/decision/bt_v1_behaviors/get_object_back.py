import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior

class GetObjectBackDummy(TemplateBehavior):
    def __init__(self, name="get_object_back_dummy"):
        super().__init__(name)

    def initialise(self) -> None:
        return super().initialise()

    def update(self):
        return py_trees.common.Status.RUNNING