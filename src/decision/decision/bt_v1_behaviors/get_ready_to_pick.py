import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior
import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
import rclpy
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose
from py_trees.common import Status
from copy import deepcopy
import numpy as np
from std_msgs.msg import Int16, Int16MultiArray, Bool, Empty
    
class GetReadyToPick(TemplateBehavior):
    def __init__(self, name="get_ready_to_pick"):
        super().__init__(name)
        self.register_value('is_pick_ready', read=True, write=True)
        self.register_value('threshold_bbox', read=True, write=True)
        
        self.DESIRED_HEIGHT = 200
        self.THRESHOLD = 10
        self.VEL = 0.5
        
    def initialise(self) -> None:
        self.blackboard.set('is_pick_ready', False)
        en = Bool()
        en.data = True
        self.node.enable_theshold_bbox_pub.publish(en)
        self.node.pick_ready_pose_pub.publish(Empty())
        return super().initialise()
    
    def terminate(self, new_status: Status) -> None:
        en = Bool()
        en.data = False
        self.node.enable_theshold_bbox_pub.publish(en)
        return super().terminate(new_status)

    def update(self):
        try:
            threshold_bbox = self.blackboard.get('threshold_bbox')
        except:
            self.node.get_logger().info("No bbox yet")
            return py_trees.common.Status.RUNNING
        
        if len(threshold_bbox.data) == 0:
            self.node.get_logger().info("No bbox detected")
            return py_trees.common.Status.RUNNING
        
        bbox = []
        for i in range(len(threshold_bbox.data) // 4):
            bbox.append([threshold_bbox.data[i*4], threshold_bbox.data[i*4+1], threshold_bbox.data[i*4+2], threshold_bbox.data[i*4+3]])
        
        target_bbox = bbox[0]
        max_area = -1e-10
        for x, y, w, h in bbox:
            area = w * h
            if area > max_area:
                max_area = area
                target_bbox = [x, y, w, h]
        
        self.node.get_logger().info("Target bbox: {}".format(target_bbox))
        x, y, w, h = target_bbox
        target_x = x + w/2
        target_y = y + h/2
        diff = self.DESIRED_HEIGHT - target_y
        
        if target_y >= (self.DESIRED_HEIGHT - self.THRESHOLD) and target_y <= (self.DESIRED_HEIGHT + self.THRESHOLD):
            cmd = Twist()
            self.node.cmd_vel_pub.publish(cmd)
            self.blackboard.set('is_pick_ready', True)
            self.node.get_logger().info("Done.")
            return py_trees.common.Status.RUNNING
        
        cmd = Twist()
        cmd.linear.x = self.VEL if diff > 0 else -self.VEL
        self.node.cmd_vel_pub.publish(cmd)
        self.node.get_logger().info("Diff: {}".format(diff))
        
        return py_trees.common.Status.RUNNING