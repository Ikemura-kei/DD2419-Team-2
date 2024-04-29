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
from scipy import signal
from nav_msgs.msg import OccupancyGrid
    
class DriveToBoxApproachPoint(TemplateBehavior):
    def __init__(self, name="drive_to_box_approach_point", debug=False):
        super().__init__(name)
        self.debug = debug
        # -- the name of the box we set to put our object in, naming format follows "<obj_type>_<unique_id>", for example "cube_1" --
        self.register_value('target_box', read=True, write=False)
        # -- the poses of the boxes (specified by geometry_msgs.msg.PoseStamped), naming format follows that above --
        self.register_value(key="box_poses", read=True, write=False)
        self.register_value(key="map", read=True, write=False)
        self.register_value('box_approach_pnt', read=True, write=True)

        self.TF_TIMEOUT = rclpy.duration.Duration(seconds=0.05)
        self.TARGET_DIFF_THRESHOLD = 0.02 # meters
        self.last_cmd_pub_time = None
        self.CMD_PUB_PERIOD = 0.1 # seconds
        self.APPROACH_CIRCLE_RADIUS = 0.715 # [m]
        self.CONV_KERNEL_SIZE = 35 # [cells]
        
        np.random.seed(20010427)
        
        xs = np.arange(-self.APPROACH_CIRCLE_RADIUS, self.APPROACH_CIRCLE_RADIUS, 0.05) # (N, )
        ys = np.sqrt(-xs**2 + (self.APPROACH_CIRCLE_RADIUS+1e-8)**2) # (N, )
        
        self.xs = np.concatenate([xs, xs], axis=-1) # (2N, )
        self.ys = np.concatenate([-ys, ys], axis=-1) # (2N, )
        
        self.candidate_points = np.stack([self.xs, self.ys], axis=-1) # (2N, 2)
        
        self.goal_sent = False
        self.goal_map = None
        
    def initialise(self) -> None:
        self.last_cmd_pub_time = self.node.get_clock().now()
        self.goal_sent = False
        self.goal_map = None
        return super().initialise()
    
    def terminate(self, new_status: Status) -> None:
        return super().terminate(new_status)

    def update(self):
        if self.debug:
            return py_trees.common.Status.RUNNING
        
        try:
            target_box = self.blackboard.get('target_box')
            box_poses = self.blackboard.get('box_poses')
        except:
            return py_trees.common.Status.RUNNING
        
        pose_map:PoseStamped = deepcopy(box_poses[target_box])
        
        do_send_command = False
        if self.last_cmd_pub_time is not None:
            dt = (self.node.get_clock().now() - self.last_cmd_pub_time).nanoseconds / 1e9
            if dt > self.CMD_PUB_PERIOD:
                do_send_command = True

        if not do_send_command:
            return py_trees.common.Status.RUNNING
        
        try:
            transform = self.node.buffer.lookup_transform('base_link', \
               'map', rclpy.time.Time(), self.TF_TIMEOUT)
        except TransformException as e:
            return py_trees.common.Status.RUNNING
        
        if not self.goal_sent:
            # -- get map, for us to check if the goal selected is ok --
            try:
                map: OccupancyGrid = self.blackboard.get('map')
            except:
                self.node.get_logger().warn('Get map failed')
                return py_trees.common.Status.RUNNING
            
            # -- get submap around the object --
            map_array = np.array(map.data).reshape((map.info.height, map.info.width))
            x_cell = int((pose_map.pose.position.x - map.info.origin.position.x) / map.info.resolution)
            y_cell = int((pose_map.pose.position.y - map.info.origin.position.y) / map.info.resolution)
            r_cell = int(self.APPROACH_CIRCLE_RADIUS / map.info.resolution) + 17
            if r_cell <= 0:
                r_cell = 1
            y_low = ((y_cell-r_cell) if (y_cell-r_cell) >= 0 else 0)
            y_high = ((y_cell+r_cell) if (y_cell+r_cell) < map.info.height else map.info.height-1)
            x_low = ((x_cell-r_cell) if (x_cell-r_cell) >= 0 else 0)
            x_high = ((x_cell+r_cell) if (x_cell+r_cell) < map.info.width else map.info.width-1)
            submap = map_array[y_low:y_high,x_low:x_high]
            
            # -- run convolution to gather nearby obstacle information
            conv_kernel = np.ones((self.CONV_KERNEL_SIZE, self.CONV_KERNEL_SIZE))
            conv_result = signal.convolve2d(submap, conv_kernel, mode='same', boundary='fill', fillvalue=0)
            
            can_pnts = np.zeros_like(self.candidate_points)
            can_pnts[:,0] = self.candidate_points[:,0] + pose_map.pose.position.x
            can_pnts[:,1] = self.candidate_points[:,1] + pose_map.pose.position.y
            can_pnts_cell = np.zeros_like(self.candidate_points)
            can_pnts_cell[:,0] = ((can_pnts[:,0] - map.info.origin.position.x) / map.info.resolution - x_low).astype(np.int64) 
            can_pnts_cell[:,1] = ((can_pnts[:,1] - map.info.origin.position.y) / map.info.resolution - y_low).astype(np.int64)
            can_pnts_cell[:,0] = (np.clip(can_pnts_cell[:,0], 0, x_high-x_low-1))
            can_pnts_cell[:,1] = (np.clip(can_pnts_cell[:,1], 0, y_high-y_low-1))
            can_pnts_cell = can_pnts_cell.astype(np.int32)
            can_pnts_scores = conv_result[can_pnts_cell[:,1], can_pnts_cell[:,0]]
            min_idx = np.argmin(can_pnts_scores)
            
            # dists = (can_pnts[:,0] - transform.transform.translation.x) ** 2\
            #     + (can_pnts[:,1] - transform.transform.translation.y) ** 2
            
            # idx = np.random.randint(0, len(can_pnts))
            # idx = np.argmin(dists)
            idx = min_idx
            
            self.goal_map = deepcopy(pose_map)
            self.goal_map.pose.position.x = can_pnts[idx,0]
            self.goal_map.pose.position.y = can_pnts[idx,1]
            
            self.goal_sent = True
            
            self.blackboard.set('box_approach_pnt', self.goal_map)

        pose_base = do_transform_pose(self.goal_map.pose, transform)

        goal = PointStamped()
        goal.header.frame_id = 'base_link'
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.point.x = pose_base.position.x
        goal.point.y = pose_base.position.y
        self.node.goal_pub.publish(goal)
        self.last_cmd_pub_time = self.node.get_clock().now()
        
        mode = Int16()
        mode.data = 0 # path following
        self.node.traj_follower_mode_pub.publish(mode)

        return py_trees.common.Status.RUNNING