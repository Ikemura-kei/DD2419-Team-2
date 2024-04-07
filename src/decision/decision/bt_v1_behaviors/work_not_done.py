import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior

class WorkNotDone(TemplateBehavior):
    def __init__(self, name="work_not_done"):
        super().__init__(name)
        self.register_value(key="is_work_not_done", read=True, write=True)

    def initialise(self) -> None:
        self.blackboard.set('is_work_not_done', True, overwrite=True)
        return super().initialise()

    def update(self):
        try:    
            is_work_not_done = self.blackboard.get('is_work_not_done')
        except:
            return py_trees.common.Status.FAILURE
        
        return py_trees.common.Status.SUCCESS if is_work_not_done else py_trees.common.Status.FAILURE