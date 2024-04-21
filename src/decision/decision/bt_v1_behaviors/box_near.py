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
from std_msgs.msg import Int16

class BoxNear(TemplateBehavior):
    def __init__(self, name="box_near"):
        super().__init__(name)
        # -- the poses of the boxes (specified by geometry_msgs.msg.PoseStamped), naming format follows "<obj_type>_<unique_id>", for example "cube_1" --
        self.register_value(key="box_poses", read=True, write=False)
        # -- the name of the box we set to put our object in, naming format follows "<obj_type>_<unique_id>", for example "cube_1" --
        self.register_value(key="target_box", read=False, write=True)

        self.TF_TIMEOUT = rclpy.duration.Duration(seconds=0.015)
        self.NEAR_DISTANCE_THRESHOLD = 0.255 # meters, has to be within the arm reach

    def initialise(self) -> None:
        return super().initialise()

    def update(self):
        # # -- for debug --
        # return py_trees.common.Status.SUCCESS
        try:
            box_poses = self.blackboard.get('box_poses')
            target_box = self.blackboard.get('target_box')
        except:
            return py_trees.common.Status.FAILURE
        
        pose_map:PoseStamped = box_poses[target_box]
        # -- NOTE: these pose information should be under the 'map' frame --
        try:
            transform = self.node.buffer.lookup_transform('base_link', \
                pose_map.header.frame_id, rclpy.time.Time(), self.TF_TIMEOUT)
        except TransformException as e:
            return py_trees.common.Status.FAILURE
        pose_base = do_transform_pose(pose_map.pose, transform)

        # -- check if the object is near enough to pick --
        distance  = np.sqrt(pose_base.position.x**2 + pose_base.position.y**2) # euclidian distance
        print(f"Distance to {target_box}: {distance}")
        if distance <= self.NEAR_DISTANCE_THRESHOLD:
            mode = Int16()
            mode.data = 2 # disable
            self.node.traj_follower_mode_pub.publish(mode)
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
