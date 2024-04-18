from py_trees.common import Status
import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior
from geometry_msgs.msg import PoseStamped, PointStamped
from tf2_ros import TransformException
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
import rclpy
import numpy as np 
from copy import deepcopy

class DriveToObject(TemplateBehavior):
    def __init__(self, name="drive_to_object"):
        super().__init__(name)
        # -- the name of the object we set to pick, naming format follows "<obj_type>_<unique_id>", for example "cube_1" --
        self.register_value('target_object', read=True, write=False)
        # -- a dictionary containing object poses (as geometry_msgs.msg.PoseStamped), the naming format of objects follows that above --
        self.register_value('object_poses', read=True, write=False)

        self.TF_TIMEOUT = rclpy.duration.Duration(seconds=0.05)
        self.TARGET_DIFF_THRESHOLD = 0.02 # meters
        self.last_pose = None
        self.last_cmd_pub_time = None
        self.CMD_PUB_PERIOD = 0.185 # seconds
        self.goal_sent = False
        
    def initialise(self) -> None:
        self.last_pose = None
        self.last_cmd_pub_time = self.node.get_clock().now()
        self.goal_sent = False
        return super().initialise()
    
    def terminate(self, new_status: Status) -> None:
        self.last_pose = None
        return super().terminate(new_status)

    def update(self):
        try:
            target_object = self.blackboard.get('target_object')
            object_poses = self.blackboard.get('object_poses')
        except:
            return py_trees.common.Status.RUNNING
        
        pose_map:PoseStamped = deepcopy(object_poses[target_object])
        
        do_send_command = False
        if self.last_cmd_pub_time is not None:
            dt = (self.node.get_clock().now() - self.last_cmd_pub_time).nanoseconds / 1e9
            if dt > self.CMD_PUB_PERIOD:
                do_send_command = True

        orig_x = pose_map.pose.position.x
        orig_y = pose_map.pose.position.y
        pose_map.pose.position.x = pose_map.pose.position.x - 0.115
        pose_map.pose.position.y = pose_map.pose.position.y + 0.075
        
        if not do_send_command:
            return py_trees.common.Status.RUNNING
        
        try:
            transform = self.node.buffer.lookup_transform('base_link', \
                pose_map.header.frame_id, rclpy.time.Time(), self.TF_TIMEOUT)
        except TransformException as e:
            return py_trees.common.Status.RUNNING
        
        pose_base = do_transform_pose(pose_map.pose, transform)

        goal = PointStamped()
        goal.header.frame_id = 'base_link'
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.point.x = pose_base.position.x
        goal.point.y = pose_base.position.y
        self.node.goal_pub.publish(goal)
        self.last_cmd_pub_time = self.node.get_clock().now()
        
        if not self.goal_sent:
            self.goal_sent = True
            goal_loc = PointStamped()
            goal_loc.header.frame_id = 'map'
            goal_loc.header.stamp = self.node.get_clock().now().to_msg()
            goal_loc.point.x = orig_x
            goal_loc.point.y = orig_y
            self.node.goal_loc_pub.publish(goal_loc)
        
        self.last_pose = pose_map.pose

        return py_trees.common.Status.RUNNING