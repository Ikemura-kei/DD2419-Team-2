import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior

class ObjectAtHand(TemplateBehavior):
    def __init__(self, name="object_at_hand", always_true=False):
        super().__init__(name)
        # -- a boolean value indicating whether the pick action is done or not --
        self.register_value('is_pick_done', read=True, write=True)
        self.always_true = always_true

    def initialise(self) -> None:
        return super().initialise()

    def update(self):
        if self.always_true:
            return py_trees.common.Status.SUCCESS
            
        try:
            is_pick_done = self.blackboard.get('is_pick_done')
        except:
            return py_trees.common.Status.FAILURE
        
        if not is_pick_done:
            return py_trees.common.Status.FAILURE
        
        # TODO: pick done does not necessary mean the object is at hand, check if the object is really at hand here

        self.blackboard.set('is_pick_done', False)
        return py_trees.common.Status.SUCCESS