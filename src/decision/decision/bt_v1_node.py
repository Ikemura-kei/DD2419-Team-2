import py_trees_ros as ptr
import py_trees
from decision.bt_v1_behaviors import *
import rclpy
import sys

def main():
    print("Hello World from bt_v1_node.py")

    rclpy.init()

    root = create_root()
    tree = ptr.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(node_name="behaviour_tree", timeout=15.0)
    except ptr.exceptions.TimedOutError as e:
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=300.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

def create_root():
    # -- LEVEL 1 --
    root = py_trees.composites.Parallel\
            (name='behaviour_tree_v1', policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))

    # -- LEVEL 2 --
    topics2bb = py_trees.composites.Parallel(name='topics2bb', policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))
    # TODO: define topics to subscribe here
    task = py_trees.composites.Sequence(name='task', memory=False)
    
    # -- LEVEL 3 --
    work_not_done = WorkNotDone()
    work = py_trees.composites.Sequence(name='work', memory=True)

    # -- LEVEL 4 --
    m_s_object_found = py_trees.composites.Selector(name='m_s_object_found', memory=False)
    collect_object = py_trees.composites.Sequence(name='collect_object', memory=True)
    place_object_ = py_trees.composites.Sequence(name='place_object_', memory=False)
    mark_work_done = MarkWorkDone()

    # -- LEVEL 5 --
    object_found = ObjectFound()
    wonder_around = WonderAround()
    m_s_object_near = py_trees.composites.Selector(name='m_s_object_near', memory=False)
    m_s_object_picked = py_trees.composites.Selector(name='m_s_object_picked', memory=False)
    m_s_object_in_hand = py_trees.composites.Selector(name='m_s_object_in_hand', memory=False)
    go_and_place = py_trees.composites.Sequence(name='go_and_place', memory=True)

    # -- LEVEL 6 --
    object_near = ObjectNear()
    drive_to_object = DriveToObject()
    object_at_hand = ObjectAtHand()
    pick_object = PickObject()
    object_at_hand_2 = ObjectAtHand()
    get_object_back = GetObjectBackDummy()
    m_s_box_found = py_trees.composites.Selector(name='m_s_box_found', memory=False)
    m_s_box_near = py_trees.composites.Selector(name='m_s_box_near', memory=False)
    m_s_object_placed = py_trees.composites.Selector(name='m_s_object_placed', memory=False)

    # -- LEVEL 7 --
    box_found = BoxFound()
    wonder_around_2 = WonderAround()
    box_near = BoxNear()
    drive_to_box = DriveToBox()
    object_in_box = ObjectInBox()
    place_object = PlaceObject()

    # -- ASSEMBLY: LEVEL 6 --
    m_s_box_found.add_children([box_found, wonder_around_2])
    m_s_box_near.add_children([box_near, drive_to_box])
    m_s_object_placed.add_children([object_in_box, place_object])

    # -- ASSEMBLY: LEVEL 5 --
    m_s_object_near.add_children([object_near, drive_to_object])
    m_s_object_picked.add_children([object_at_hand, pick_object])
    m_s_object_in_hand.add_children([object_at_hand_2, get_object_back])
    go_and_place.add_children([m_s_box_found, m_s_box_near, m_s_object_placed])

    # -- ASSEMBLY: LEVEL 4 --
    m_s_object_found.add_children([object_found, wonder_around])
    collect_object.add_children([m_s_object_near, m_s_object_picked])
    place_object_.add_children([m_s_object_in_hand, go_and_place])

    # -- ASSEMBLY: LEVEL 3 --
    work.add_children([m_s_object_found, collect_object, place_object_, mark_work_done])

    # -- ASSEMBLY: LEVEL 2 --
    task.add_children([work_not_done, work])

    # -- ASSEMBLY: LEVEL 1 --
    root.add_children([topics2bb, task])

    return root

if __name__ == "__main__":
    main()