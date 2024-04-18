import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior
import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped
import rclpy
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose

class WonderAround(TemplateBehavior):
    def __init__(self, name="wonder_around", cooldown=49.76):
        super().__init__(name)
        
        self.last_cmd_send_time = None
        self.last_cmd_update_time = None
        
        self.CMD_SEND_PERIOD = 0.175
        self.CMD_UPDATE_PERIOD = 15.75
        
        self.target_pos = np.array([0.0, 0.0])
        self.TARGET_GOAL_X_BOUND = [-1.7, 3.0]
        self.TARGET_GOAL_Y_BOUND = [-1.25, 1.25]
        self.TARGET_GOAL_X_RANGE = self.TARGET_GOAL_X_BOUND[1] - self.TARGET_GOAL_X_BOUND[0]
        self.TARGET_GOAL_Y_RANGE = self.TARGET_GOAL_Y_BOUND[1] - self.TARGET_GOAL_Y_BOUND[0]
        self.TF_TIMEOUT = rclpy.duration.Duration(seconds=0.01)
        self.start_time = None
        self.START_COOLDOWN = cooldown

    def initialise(self) -> None:
        self.last_cmd_send_time = self.node.get_clock().now()
        self.last_cmd_update_time = self.node.get_clock().now()
        if self.start_time is None:
            self.start_time = self.node.get_clock().now()
        self.target_pos[0] = np.random.rand() * self.TARGET_GOAL_X_RANGE + self.TARGET_GOAL_X_BOUND[0] 
        self.target_pos[1] = np.random.rand() * self.TARGET_GOAL_Y_RANGE + self.TARGET_GOAL_Y_BOUND[0] 
        # self.target_pos[0] = -2
        # self.target_pos[1] = 0
        return super().initialise()

    def update(self):
        # TODO: send command to wonder around here
        print('wondering around...')
        
        dt_since_start = (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9
        if dt_since_start <= self.START_COOLDOWN:
            return py_trees.common.Status.RUNNING
        
        if self.last_cmd_send_time is None or self.last_cmd_update_time is None:
            return py_trees.common.Status.RUNNING
        
        dt_cmd_update = (self.node.get_clock().now() - self.last_cmd_update_time).nanoseconds / 1e9
        dt_cmd_send = (self.node.get_clock().now() - self.last_cmd_send_time).nanoseconds / 1e9
        
        if dt_cmd_send >= self.CMD_SEND_PERIOD:
            pose_map = PoseStamped()
            pose_map.pose.position.x = self.target_pos[0]          
            pose_map.pose.position.y = self.target_pos[1]   
            pose_map.header.frame_id = 'map'
            pose_map.pose.orientation.w = 1.0
            pose_map.header.stamp = self.node.get_clock().now().to_msg() 
                  
            try:
                transform = self.node.buffer.lookup_transform('base_link', \
                    'map', rclpy.time.Time(), self.TF_TIMEOUT)
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
            self.last_cmd_send_time = self.node.get_clock().now()
            
            # -- debug --
            # self.pick_point = PointStamped()
            # self.pick_point.header.stamp = self.node.get_clock().now().to_msg()
            # self.pick_point.header.frame_id = "base_link"
            # self.pick_point.point.x = 0.15
            # self.pick_point.point.y = 0.01
            # self.pick_point.point.z = -0.05
            # self.node.pick_pub.publish(self.pick_point)
        
        if dt_cmd_update >= self.CMD_UPDATE_PERIOD:
            self.target_pos[0] = np.random.rand() * self.TARGET_GOAL_X_RANGE + self.TARGET_GOAL_X_BOUND[0] 
            self.target_pos[1] = np.random.rand() * self.TARGET_GOAL_Y_RANGE + self.TARGET_GOAL_Y_BOUND[0] 
            self.last_cmd_update_time = self.node.get_clock().now()
        
        return py_trees.common.Status.RUNNING