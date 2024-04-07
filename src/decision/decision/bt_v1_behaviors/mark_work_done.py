import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior

class MarkWorkDone(TemplateBehavior):
    def __init__(self, name="mark_work_done"):
        super().__init__(name)
        self.register_value(key="is_work_not_done", read=False, write=True)

    def initialise(self) -> None:
        return super().initialise()

    def update(self):
        self.blackboard.set('is_work_not_done', False, overwrite=True)
        return py_trees.common.Status.SUCCESS