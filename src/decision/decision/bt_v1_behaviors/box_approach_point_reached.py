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

class BoxApproachPointReached(TemplateBehavior):
    def __init__(self, name="box_approach_point_reached"):
        super().__init__(name)
        
        self.register_value('box_approach_pnt', read=True, write=False)

        self.TF_TIMEOUT = rclpy.duration.Duration(seconds=0.015)
        self.NEAR_DISTANCE_THRESHOLD = 0.175 # [m]

    def initialise(self) -> None:
        return super().initialise()

    def update(self):
        try:
            box_approach_pnt = self.blackboard.get('box_approach_pnt')
        except:
            return py_trees.common.Status.FAILURE
        
        pose_map:PoseStamped = box_approach_pnt
        try:
            transform = self.node.buffer.lookup_transform('base_link', \
                pose_map.header.frame_id, rclpy.time.Time(), self.TF_TIMEOUT)
        except TransformException as e:
            return py_trees.common.Status.FAILURE
        pose_base = do_transform_pose(pose_map.pose, transform)

        # -- check if the object is near enough to pick --
        distance  = np.sqrt(pose_base.position.x**2 + pose_base.position.y**2) # euclidian distance
        # self.node.get_logger().info("distance to approach point {}".format(distance))

        if distance <= self.NEAR_DISTANCE_THRESHOLD:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
