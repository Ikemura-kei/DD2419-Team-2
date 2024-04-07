import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior
from geometry_msgs.msg import PoseStamped, PointStamped

class PickObject(TemplateBehavior):
    def __init__(self, name="pick_object"):
        super().__init__(name)
        # -- the name of the object we set to pick, naming format follows "<obj_type>_<unique_id>", for example "cube_1" --
        self.register_value('target_object', read=True, write=False)
        # -- a dictionary containing object poses (as geometry_msgs.msg.PoseStamped), the naming format of objects follows that above --
        self.register_value('object_poses', read=True, write=False)

        self.pick_point = PointStamped()
        self.pick_point_pub = None

    def initialise(self) -> None:
        if self.pick_point_pub is None:
            self.pick_point_pub = self.node.create_publisher(PointStamped, '/pick_point', 10)
        return super().initialise()

    def update(self):
        try:
            target_object = self.blackboard.get('target_object')
            object_poses = self.blackboard.get('object_poses')
        except:
            return py_trees.common.Status.RUNNING
        
        if target_object not in object_poses:
            return py_trees.common.Status.RUNNING
        
        pose:PoseStamped = object_poses[target_object]
        self.pick_point.header = pose.header
        self.pick_point.point.x = pose.pose.position.x
        self.pick_point.point.y = pose.pose.position.y
        self.node.pick_point_pub.publish(self.pick_point)