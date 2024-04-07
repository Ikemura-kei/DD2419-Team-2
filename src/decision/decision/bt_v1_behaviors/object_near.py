import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
import rclpy
import numpy as np

class ObjectNear(TemplateBehavior):
    def __init__(self, name="object_near"):
        super().__init__(name)
        # -- the name of the object we set to pick, naming format follows "<obj_type>_<unique_id>", for example "cube_1" --
        self.register_value('target_object', read=True, write=False)
        # -- a dictionary containing object poses (as geometry_msgs.msg.PoseStamped), the naming format of objects follows that above --
        self.register_value('object_poses', read=True, write=False)

        self.TF_TIMEOUT = rclpy.duration.Duration(seconds=0)
        self.NEAR_DISTANCE_THRESHOLD = 0.15 # meters

    def initialise(self) -> None:
        return super().initialise()

    def update(self):
        try:
            target_object = self.blackboard.get('target_object')
            object_poses = self.blackboard.get('object_poses')
        except:
            print("ObjectNear: failed to get target_object or object_poses")
            return py_trees.common.Status.FAILURE
        
        pose_map:PoseStamped = object_poses[target_object]
        # -- NOTE: these pose information should be under the 'map' frame --
        try:
            transform = self.node.buffer.lookup_transform('base_link', pose_map.header.frame_id, pose_map.header.stamp, self.TF_TIMEOUT)
        except TransformException as e:
            print("TransformException: ", e)
            return py_trees.common.Status.FAILURE
        pose_base = do_transform_pose(pose_map.pose, transform)
        print("pose_base: ", pose_base)

        # -- check if the object is near enough to pick --
        distance  = np.sqrt(pose_base.position.x**2 + pose_base.position.y**2) # euclidian distance
        print("distance: ", distance)
        if distance <= self.NEAR_DISTANCE_THRESHOLD:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
