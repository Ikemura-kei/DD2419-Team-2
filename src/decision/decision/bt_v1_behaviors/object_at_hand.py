import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior

class ObjectAtHand(TemplateBehavior):
    def __init__(self, name="object_at_hand"):
        super().__init__(name)
        # -- a boolean value indicating whether the pick action is done or not --
        self.register_value('is_pick_done', read=True, write=False)

    def initialise(self) -> None:
        return super().initialise()

    def update(self):
        try:
            is_pick_done = self.blackboard.get('is_pick_done')
        except:
            return py_trees.common.Status.FAILURE
        
        if not is_pick_done:
            return py_trees.common.Status.FAILURE
        
        # TODO: pick done does not necessary mean the object is at hand, check if the object is really at hand here

        return py_trees.common.Status.SUCCESS