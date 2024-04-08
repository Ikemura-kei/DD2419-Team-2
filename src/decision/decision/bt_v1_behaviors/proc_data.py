import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior
from dd2419_interfaces.msg import ObjectList, ObjectPoses

class ProcData(TemplateBehavior):
    def __init__(self, name="proc_data"):
        super().__init__(name)
        # -- raw data in type ObjectPoses --
        self.register_value(key="objects", read=True, write=False)
        # -- a dictionary containing object poses (as geometry_msgs.msg.PoseStamped) --
        self.register_value(key="object_poses", read=False, write=True)
        # -- raw data in type BoxPoses --
        self.register_value(key="boxes", read=True, write=False)
        # -- a dictionary containing box poses (as geometry_msgs.msg.PoseStamped) --
        self.register_value(key="box_poses", read=False, write=True)
        
        self.object_poses = {}
        self.box_poses = {}

    def initialise(self) -> None:
        return super().initialise()

    def update(self):
        try:
            objects:ObjectPoses = self.blackboard.get('objects')
            
            for i, obj in enumerate(objects.object_list.object_list):
                self.object_poses[obj] = objects.poses[i]
                
            self.blackboard.set('object_poses', self.object_poses, overwrite=True)
            
            boxes:ObjectPoses = self.blackboard.get('boxes')
            
            for i, obj in enumerate(boxes.object_list.object_list):
                self.box_poses[obj] = boxes.poses[i]
                
            self.blackboard.set('box_poses', self.box_poses, overwrite=True)
        except:
            pass 
    
        return py_trees.common.Status.SUCCESS