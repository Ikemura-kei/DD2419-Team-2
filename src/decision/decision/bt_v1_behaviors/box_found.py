import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior
from decision.utils.object_utils import *

class BoxFound(TemplateBehavior):
    def __init__(self, name="box_found"):
        super().__init__(name)
        # -- list of box names, specified by "<obj_type>_<unique_id>", i.e. what type of object it expects us to put inside + an unique identifier, e.g. 'cube_1' --
        self.register_value(key="box_list", read=True, write=False)
        # -- the name of the object we set to pick, naming format follows "<obj_type>_<unique_id>", for example "cube_1" --
        self.register_value(key="target_object", read=True, write=False)
        # -- the name of the box we set to put our object in, naming format follows that mentioned above --
        self.register_value(key="target_box", read=False, write=True)

    def initialise(self) -> None:
        return super().initialise()

    def update(self):
        try:
            box_list = self.blackboard.get('box_list')
            target_object = self.blackboard.get('target_object')
        except:
            return py_trees.common.Status.FAILURE
        
        # -- check if there is any box that matches the type of the object we are supposed to put inside, use the first found matching box --
        obj_type, obj_id = get_obj_type_and_id(target_object)
        for box in box_list:
            box_type, box_id = get_obj_type_and_id(box)
            if box_type == obj_type:
                self.blackboard.set('target_box', box, overwrite=True)
                return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.FAILURE