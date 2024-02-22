# -- rclpy stuff --
import rclpy
from rclpy.node import Node
# -- pytree ros stuff --
import py_trees
import py_trees_ros as ptr
import py_trees.console as console
# -- ros msgs --
from nav_msgs.msg import OccupancyGrid
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker, MarkerArray
from aruco_msgs.msg import MarkerArray as ArucoMarkerArray
# -- other stuff --
import os, sys

class CheckCubeDetectedBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name:str):
        super().__init__(name=name)
        self.cube_detected_currently = False
        self.not_det_cnt = 0
        self.det_cnt = 0
        self.last_check_time = None
        self.name = name
        self.blackboard = self.attach_blackboard_client(self.name)
        self.blackboard.register_key(
                key="object",
                access=py_trees.common.Access.READ
            )
        self.CHECK_PERIOD = 0.1 # seconds
        self.last_status = py_trees.common.Status.FAILURE
        self.N1 = 2 # detection count threshold
        self.N2 = 5 # miss count threshold
        self.start_time = None
        self.DEBUG = True
        
    def setup(self, **kwargs):
        self.node = kwargs.get('node')
    
    def update(self):
        time = self.node.get_clock().now()
        time = ptr.conversions.rclpy_time_to_float(time)
        
        if self.last_check_time is None: # first time coming here
            self.last_check_time = time
            self.start_time = time
            return self.last_status
        
        dt = time - self.last_check_time
        
        if self.DEBUG:
            current_detection = False # !! debug !!
            if (time-self.start_time) > 6:
                current_detection = True
        else:
            if self.blackboard.exists('object') and (self.blackboard.get('object') is not None):
                current_detection = True
            else:
                current_detection = False
        
        if (not current_detection) and dt < self.CHECK_PERIOD: # we check only once in a while, unless detection occurs
            return self.last_status
        
        self.last_check_time = time
        
        if current_detection:
            self.det_cnt += 1
            self.not_det_cnt = 0 # true detection implies object must be there, so reset not_det_cnt
        else:
            self.not_det_cnt += 1 # however, missed detection does not imply missing object, so DO NOT reset det_cnt
        
        if self.cube_detected_currently:
            # -- check for missing --
            if self.not_det_cnt >= self.N2:
                self.cube_detected_currently = False
                self.det_cnt = 0
        else:
            # -- check for dectection --
            if self.det_cnt >= self.N1:
                self.cube_detected_currently = True
                self.not_det_cnt = 0
        
        self.last_status = py_trees.common.Status.SUCCESS if self.cube_detected_currently else py_trees.common.Status.FAILURE
        return self.last_status

class RotateInPlaceBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name:str):
        super().__init__(name=name)
        self.name = name
        
    def update(self):
        # TODO: send rotate command here. We don't need to judge success or failure, we just rotate and wait to be preempted by condition node(s).
        return py_trees.common.Status.RUNNING

class CheckCubeReachedBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name:str):
        super().__init__(name=name)
        self.name = name
        self.start_time = None
        self.DEBUG = True
    
    def setup(self, **kwargs):
        self.node = kwargs.get('node')
    
    def update(self):
        time = ptr.conversions.rclpy_time_to_float(self.node.get_clock().now())
        
        if self.start_time is None:
            self.start_time = time
            return py_trees.common.Status.FAILURE
        
        if self.DEBUG:
            dt = time - self.start_time
            if dt > 5:
                return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.FAILURE

class ReachToCubeBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name:str):
        super().__init__(name=name)
        self.name = name
        
    def update(self):
        # TODO: send reach to object command here. We don't need to judge success or failure, we just rotate and wait to be preempted by condition node(s).

        return py_trees.common.Status.RUNNING

class CheckCubePickedBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name:str):
        super().__init__(name=name)
        self.name = name
        self.start_time = None
        self.DEBUG = True
        
    def setup(self, **kwargs):
        self.node: Node = kwargs.get('node')
    
    def update(self):
        time = ptr.conversions.rclpy_time_to_float(self.node.get_clock().now())
        
        if self.start_time is None:
            self.start_time = time
            return py_trees.common.Status.FAILURE
        
        if self.DEBUG:
            if (time - self.start_time) > 5:
                return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.FAILURE

class PickCubeBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name:str):
        super().__init__(name=name)
        self.name = name
        
    def update(self):
        # TODO: send pick object command here. We don't need to judge success or failure, we just rotate and wait to be preempted by condition node(s).

        return py_trees.common.Status.RUNNING

class CheckBoxDetectedBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name:str):
        super().__init__(name=name)
        self.name = name
        self.start_time = None
        self.DEBUG = True
        
    def setup(self, **kwargs):
        self.node: Node = kwargs.get('node')
        
    def update(self):
        time = ptr.conversions.rclpy_time_to_float(self.node.get_clock().now())
        
        if self.start_time is None:
            self.start_time = time
            return py_trees.common.Status.FAILURE
        
        if self.DEBUG:
            if (time - self.start_time) > 5:
                return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.FAILURE

class CheckBoxReachedBehabiour(py_trees.behaviour.Behaviour):
    def __init__(self, name:str):
        super().__init__(name=name)
        self.name = name
        self.start_time = None
        self.DEBUG = True
        
    def setup(self, **kwargs):
        self.node: Node = kwargs.get('node')
        
    def update(self):
        time = ptr.conversions.rclpy_time_to_float(self.node.get_clock().now())
        
        if self.start_time is None:
            self.start_time = time
            return py_trees.common.Status.FAILURE
        
        if self.DEBUG:
            if (time - self.start_time) > 5:
                return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.FAILURE

class ReachToBoxBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name:str):
        super().__init__(name=name)
        self.name = name
        
    def update(self):
        # TODO: send reach box command here. We don't need to judge success or failure, we just rotate and wait to be preempted by condition node(s).

        return py_trees.common.Status.RUNNING

class CheckCubePlacedBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name:str):
        super().__init__(name=name)
        self.name = name
        self.start_time = None
        self.DEBUG = True
        
    def setup(self, **kwargs):
        self.node: Node = kwargs.get('node')
        
    def update(self):
        time = ptr.conversions.rclpy_time_to_float(self.node.get_clock().now())
        
        if self.start_time is None:
            self.start_time = time
            return py_trees.common.Status.FAILURE
        
        if self.DEBUG:
            if (time - self.start_time) > 5:
                return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.FAILURE

class PlaceCubeBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name:str):
        super().__init__(name=name)
        self.name = name
        
    def update(self):
        # TODO: send place object command here. We don't need to judge success or failure, we just rotate and wait to be preempted by condition node(s).

        return py_trees.common.Status.RUNNING

def create_root():
    # what does synchronise parameter mean?
    root = py_trees.composites.Parallel(name="ms2_decision_tree", policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))
    
    # -- first level nodes --
    topics2bb = py_trees.composites.Sequence(name='topics2bb', memory=True)
    # -- map topic --
    map2bb = ptr.subscribers.ToBlackboard('map2bb', '/map', OccupancyGrid, 10, {})
    object2bb = ptr.subscribers.ToBlackboard('object2bb', '/object_centers', MarkerArray, 10, {}, blackboard_variables='object', clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE) # make sure we clear the detection so that we can perform missing check
    aruco2bb = ptr.subscribers.ToBlackboard('aruco2bb', '/aruco/markers', ArucoMarkerArray, 10, {}, blackboard_variables='aruco', clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE)
    task = py_trees.composites.Sequence(name="task", memory=True) # memory must set to True
    
    # -- second level nodes --
    detect_and_reach = py_trees.composites.Sequence(name='detect_and_reach', memory=False) # we want to make sure we always see the cube during the execution of detect and reach
    pick_and_place = py_trees.composites.Sequence(name='pick_and_place', memory=True) # we want to keep doing the current running job
    
    # -- third level nodes --
    make_sure_cube_seen = py_trees.composites.Selector(name='make_sure_cube_seen', memory=False) # we want to be able to preempt rotate in-place, which always returns running
    make_sure_cube_reached = py_trees.composites.Selector(name='make_sure_cube_reached', memory=False) # when goal reached, we want to be able to preempt reaching behaviour, which always returns running
    make_sure_cube_picked = py_trees.composites.Selector(name='make_sure_cube_picked', memory=False)
    make_sure_box_seen = py_trees.composites.Selector(name='make_sure_box_seen', memory=False)
    make_sure_box_reached = py_trees.composites.Selector(name='make_sure_box_reached', memory=False)
    make_sure_cube_placed = py_trees.composites.Selector(name='make_sure_cube_placed', memory=False)

    # -- forth level nodes --
    check_cube_det = CheckCubeDetectedBehaviour('check_cube_det')
    find_cube = RotateInPlaceBehaviour('find_cube')
    check_cube_reached = CheckCubeReachedBehaviour('check_cube_reached')
    reach_to_cube = ReachToCubeBehaviour('reach_to_cube')
    check_cube_picked = CheckCubePickedBehaviour('check_cube_picked')
    pick_cube = PickCubeBehaviour('pick_cube')
    check_box_det = CheckBoxDetectedBehaviour('check_box_det')
    find_box = RotateInPlaceBehaviour('find_box')
    check_box_reached = CheckBoxReachedBehabiour('check_box_reached')
    reach_to_box = ReachToBoxBehaviour('reach_to_box')
    check_cube_placed = CheckCubePlacedBehaviour('check_cube_placed')
    place_cube = PlaceCubeBehaviour('place_cube')
    
    # -- compose tree --
    # -- level 1 --
    root.add_children([topics2bb, task])
    # -- level 2 --
    topics2bb.add_children([map2bb])
    task.add_children([detect_and_reach, pick_and_place])
    # -- level 3 --
    detect_and_reach.add_children([make_sure_cube_seen, make_sure_cube_reached])
    pick_and_place.add_children([make_sure_cube_picked, make_sure_box_seen, make_sure_box_reached, make_sure_cube_placed])
    # -- level 4 --
    make_sure_cube_seen.add_children([check_cube_det, find_cube])
    make_sure_cube_reached.add_children([check_cube_reached, reach_to_cube])
    make_sure_cube_picked.add_children([check_cube_picked, pick_cube])
    make_sure_box_seen.add_children([check_box_det, find_box])
    make_sure_box_reached.add_children([check_box_reached, reach_to_box])
    make_sure_cube_placed.add_children([check_cube_placed, place_cube])
    
    return root

def main():
    rclpy.init()
    
    root = create_root()
    tree = ptr.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(node_name="ms2_decision_tree", timeout=15.0)
    except ptr.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()
    

if __name__ == "__main__":
    main()