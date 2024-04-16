import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior
from geometry_msgs.msg import PoseStamped, PointStamped
from tf2_ros import TransformException
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
import rclpy
from copy import deepcopy

class DriveToBox(TemplateBehavior):
    def __init__(self, name="drive_to_box"):
        super().__init__(name)
        # -- the name of the box we set to put our object in, naming format follows "<obj_type>_<unique_id>", for example "cube_1" --
        self.register_value('target_box', read=True, write=False)
        # -- the poses of the boxes (specified by geometry_msgs.msg.PoseStamped), naming format follows that above --
        self.register_value(key="box_poses", read=True, write=False)
        
        self.TF_TIMEOUT = rclpy.duration.Duration(seconds=0.05)
        self.last_cmd_pub_time = None
        self.CMD_PUB_PERIOD = 0.205 # seconds

    def initialise(self) -> None:
        self.last_cmd_pub_time = self.node.get_clock().now()
        return super().initialise()

    def update(self):
        try:
            target_box = self.blackboard.get('target_box')
            box_poses = self.blackboard.get('box_poses')
        except:
            return py_trees.common.Status.RUNNING
        
        pose_map:PoseStamped = deepcopy(box_poses[target_box])
        pose_map.pose.position.x = pose_map.pose.position.x - 0.225
        pose_map.pose.position.y = pose_map.pose.position.y - 0.015
        
        do_send_command = False
        if self.last_cmd_pub_time is not None:
            dt = (self.node.get_clock().now() - self.last_cmd_pub_time).nanoseconds / 1e9
            if dt > self.CMD_PUB_PERIOD:
                do_send_command = True
                
        if not do_send_command:
            return py_trees.common.Status.RUNNING

        try:
            transform = self.node.buffer.lookup_transform('base_link', \
                pose_map.header.frame_id, rclpy.time.Time(), self.TF_TIMEOUT)
        except TransformException as e:
            self.node.get_logger().warn(f"Failed to transform from {pose_map.header.frame_id} to base_link: {str(e)}")
            return py_trees.common.Status.RUNNING
        pose_base = do_transform_pose(pose_map.pose, transform)

        goal = PointStamped()
        goal.header.frame_id = 'base_link'
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.point.x = pose_base.position.x
        goal.point.y = pose_base.position.y
        self.node.goal_pub.publish(goal)
        self.last_cmd_pub_time = self.node.get_clock().now()

        return py_trees.common.Status.RUNNING