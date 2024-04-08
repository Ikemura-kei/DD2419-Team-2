import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior
from geometry_msgs.msg import PoseStamped

class DriveToBox(TemplateBehavior):
    def __init__(self, name="drive_to_box"):
        super().__init__(name)
        # -- the name of the box we set to put our object in, naming format follows "<obj_type>_<unique_id>", for example "cube_1" --
        self.register_value('target_box', read=True, write=False)
        # -- the poses of the boxes (specified by geometry_msgs.msg.PoseStamped), naming format follows that above --
        self.register_value(key="box_poses", read=True, write=False)

    def initialise(self) -> None:
        return super().initialise()

    def update(self):
        try:
            target_box = self.blackboard.get('target_box')
            box_poses = self.blackboard.get('box_poses')
        except:
            return py_trees.common.Status.RUNNING
        
        pose:PoseStamped = box_poses[target_box]

        # TODO: send command to drive to the object here

        return py_trees.common.Status.RUNNING