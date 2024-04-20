import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors.template import TemplateBehavior
from geometry_msgs.msg import PoseStamped, PointStamped
import rclpy
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose
from std_msgs.msg import Int16

class PickObject(TemplateBehavior):
    def __init__(self, name="pick_object"):
        super().__init__(name)
        # -- the name of the object we set to pick, naming format follows "<obj_type>_<unique_id>", for example "cube_1" --
        self.register_value('target_object', read=True, write=False)
        # -- a dictionary containing object poses (as geometry_msgs.msg.PoseStamped), the naming format of objects follows that above --
        self.register_value('object_poses', read=True, write=False)
        self.register_value('ik_res', read=True, write=True)
        self.register_value('is_pick_done', read=True, write=True)

        self.pick_point = PointStamped()
        self.is_first_cmd = True
        self.start_time = None
        self.START_COOLDOWN = 4.75
        self.accumulator = 0
        self.ACCUMULATOR_STEP = 0.0234
        self.TF_TIMEOUT = rclpy.duration.Duration(seconds=0.005)

    def initialise(self) -> None:
        self.is_first_cmd = True
        self.start_time = self.node.get_clock().now()
        self.accumulator = 0
        return super().initialise()

    def update(self):
        ik_res = None
        
        mode = Int16()
        mode.data = 2 # disable
        self.node.traj_follower_mode_pub.publish(mode)
            
        try:
            target_object = self.blackboard.get('target_object')
            object_poses = self.blackboard.get('object_poses')
            if not self.is_first_cmd:
                ik_res = self.blackboard.get('ik_res')
        except:
            return py_trees.common.Status.RUNNING
        
        if target_object not in object_poses:
            return py_trees.common.Status.RUNNING
        
        success = None
        if ik_res is not None:
            success = ik_res.data
            self.blackboard.set('ik_res', None)
            
        pose:PoseStamped = object_poses[target_object]
        try:
            transform = self.node.buffer.lookup_transform('base_link', \
                pose.header.frame_id, rclpy.time.Time(), self.TF_TIMEOUT)
        except TransformException as e:
            print("TransformException: ", e)
            return py_trees.common.Status.RUNNING
        pose_base = do_transform_pose(pose.pose, transform)
        added_term = (self.accumulator if (pose_base.position.y < 0) else (-self.accumulator))
        self.node.get_logger().info("pose y is {}, added term is {}, accumulator is {}".format(pose_base.position.y, added_term, self.accumulator))
        
        if (success is not None and not success) or self.is_first_cmd:
            dt_since_start = (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9
            if dt_since_start <= self.START_COOLDOWN:
                return py_trees.common.Status.RUNNING
            
            self.node.get_logger().info('arm command published')
            self.is_first_cmd = False
            
            x_target = pose_base.position.x
            y_target = pose_base.position.y
            if pose_base.position.x <= 0.1195: # will collide with body
                self.node.get_logger().warn("pick goal too close to robot body!")
                x_target = 0.1195
            if y_target < -0.05:
                y_target = -0.045
            if y_target > 0.05:
                y_target = 0.045
            
            self.pick_point.header.frame_id = "base_link"
            self.pick_point.header.stamp = self.node.get_clock().now().to_msg()
            self.pick_point.point.x = x_target if (x_target <= 0.1625) else 0.1625
            self.pick_point.point.y = y_target + added_term
            self.pick_point.point.z = -0.0495
            self.accumulator += self.ACCUMULATOR_STEP
            
            # -- for debug --
            # self.pick_point.header.stamp = self.node.get_clock().now().to_msg()
            # self.pick_point.header.frame_id = "base_link"
            # self.pick_point.point.x = 0.16
            # self.pick_point.point.y = 0.01
            # self.pick_point.point.z = -0.05
            
            self.node.pick_pub.publish(self.pick_point)
        elif (success is not None and success) :
            self.node.get_logger().warn('no need to publish arm command')
            self.blackboard.set('is_pick_done', True)
        
        return py_trees.common.Status.RUNNING