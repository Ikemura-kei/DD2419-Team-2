import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior

class ObjectFound(TemplateBehavior):
    def __init__(self, name="object_found"):
        super().__init__(name)
        # -- a list of strings, each string is of the form "<obj_type>_<unique_id>", for example "cube_1" --
        self.register_value(key="object_list", read=True, write=False)
        # -- the name of the object we set to pick, naming format follows that above --
        self.register_value(key="target_obj", read=False, write=True)
        # -- marks if the object (identified by the key) is picked up or not --
        self.object_states = {'dummy_0': True}

    def initialise(self) -> None:
        return super().initialise()

    def update(self):
        try:
            object_list = self.blackboard.get('object_list')
        except:
            return py_trees.common.Status.FAILURE

        # -- add any newly seen object to the state dictionary --
        for obj in object_list:
            if obj not in self.object_states:
                self.object_states[obj] = False # initilize to not being picked up yet

        # -- check if there is any object that hasn't been picked up yet, set the first one seen that is not picked as the target --
        target_obj = None
        for key, value in self.object_states.items():
            if not value: # haven't been picked yet
                target_obj = key
                break

        if target_obj is not None:
            self.blackboard.set('target_obj', target_obj, overwrite=True)
            return py_trees.common.Status.SUCCESS # we now will move-on to pick this object
        else:
            return py_trees.common.Status.FAILURE