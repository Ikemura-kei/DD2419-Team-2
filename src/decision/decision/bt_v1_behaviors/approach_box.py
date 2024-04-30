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
    
class ApproachBox(TemplateBehavior):
    def __init__(self, name="approach_box"):
        super().__init__(name)
        # -- the name of the box we set to put our object in, naming format follows "<obj_type>_<unique_id>", for example "cube_1" --
        self.register_value('target_box', read=True, write=False)
        # -- the poses of the boxes (specified by geometry_msgs.msg.PoseStamped), naming format follows that above --
        self.register_value(key="box_poses", read=True, write=False)

        self.TF_TIMEOUT = rclpy.duration.Duration(seconds=0.05)
        self.TARGET_DIFF_THRESHOLD = 0.02 # meters
        self.last_cmd_pub_time = None
        self.CMD_PUB_PERIOD = 0.02 # seconds
        
        self.start_time = None
        self.START_COOLDOWN = 6.0
        
    def initialise(self) -> None:
        self.last_cmd_pub_time = self.node.get_clock().now()
        if self.start_time is None:
            self.start_time = self.node.get_clock().now()
        return super().initialise()
    
    def terminate(self, new_status: Status) -> None:
        return super().terminate(new_status)

    def update(self):
        try:
            target_box = self.blackboard.get('target_box')
            box_poses = self.blackboard.get('box_poses')
        except:
            return py_trees.common.Status.RUNNING
        
        dt_since_start = (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9
        if dt_since_start <= self.START_COOLDOWN:
            return py_trees.common.Status.RUNNING
        
        pose_map:PoseStamped = deepcopy(box_poses[target_box])
        
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
                
        goal_loc = PointStamped()
        goal_loc.header.frame_id = 'map'
        goal_loc.header.stamp = self.node.get_clock().now().to_msg()
        goal_loc.point.x = pose_map.pose.position.x
        goal_loc.point.y = pose_map.pose.position.y
        self.node.goal_loc_pub.publish(goal_loc)
        
        self.node.get_logger().info(str(goal_loc))
        
        return py_trees.common.Status.RUNNING