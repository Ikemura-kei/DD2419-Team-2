import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior
import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped
import rclpy
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose
from py_trees.common import Status
from copy import deepcopy
import numpy as np
from std_msgs.msg import Int16
    
class ApproachObj(TemplateBehavior):
    def __init__(self, name="approach_obj"):
        super().__init__(name)
        # -- the name of the object we set to pick, naming format follows "<obj_type>_<unique_id>", for example "cube_1" --
        self.register_value('target_object', read=True, write=False)
        # -- a dictionary containing object poses (as geometry_msgs.msg.PoseStamped), the naming format of objects follows that above --
        self.register_value('object_poses', read=True, write=False)

        self.TF_TIMEOUT = rclpy.duration.Duration(seconds=0.05)
        self.TARGET_DIFF_THRESHOLD = 0.02 # meters
        self.last_cmd_pub_time = None
        self.CMD_PUB_PERIOD = 0.1 # seconds
        
        self.start_time = None
        self.START_COOLDOWN = 7.0
        
    def initialise(self) -> None:
        self.last_cmd_pub_time = self.node.get_clock().now()
        if self.start_time is None:
            self.start_time = self.node.get_clock().now()
        return super().initialise()
    
    def terminate(self, new_status: Status) -> None:
        return super().terminate(new_status)

    def update(self):
        try:
            target_object = self.blackboard.get('target_object')
            object_poses = self.blackboard.get('object_poses')
        except:
            return py_trees.common.Status.RUNNING
        
        dt_since_start = (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9
        if dt_since_start <= self.START_COOLDOWN:
            return py_trees.common.Status.RUNNING
        
        pose_map:PoseStamped = deepcopy(object_poses[target_object])
        
        do_send_command = False
        if self.last_cmd_pub_time is not None:
            dt = (self.node.get_clock().now() - self.last_cmd_pub_time).nanoseconds / 1e9
            if dt > self.CMD_PUB_PERIOD:
                do_send_command = True

        if not do_send_command:
            return py_trees.common.Status.RUNNING
        
        mode = Int16()
        mode.data = 1 # drive to point
        self.node.traj_follower_mode_pub.publish(mode)
        
        dist = np.sqrt(pose_map.pose.position.x**2 + pose_map.pose.position.y**2)
        
        if dist >= 0.088:
            goal_loc = PointStamped()
            goal_loc.header.frame_id = 'map'
            goal_loc.header.stamp = self.node.get_clock().now().to_msg()
            goal_loc.point.x = pose_map.pose.position.x
            goal_loc.point.y = pose_map.pose.position.y
            self.node.goal_loc_pub.publish(goal_loc)
        
        return py_trees.common.Status.RUNNING