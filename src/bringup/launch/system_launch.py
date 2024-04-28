import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bringup_pkg_dir = get_package_share_directory('bringup')
    object_detection_pkg_dir = get_package_share_directory('object_detection')
    teleop_twist_joy_pkg_dir = get_package_share_directory('teleop_twist_joy')
    
    # -- launch sensors --
    sensors_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(\
        bringup_pkg_dir, 'launch', 'sensors_launch.py')))
    
    # -- launch chassis (motor, encoder, odometry, etc.) --
    chassis_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(\
        bringup_pkg_dir, 'launch', 'chassis_launch.py')))
    
    # -- launch mapping module --
    mapper_node = Node(package='mapper', executable='mapper_node')
    
    # -- launch joy stick, for emergency control --
    joystick_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(\
        teleop_twist_joy_pkg_dir, 'launch/teleop-launch.py')), launch_arguments={'joy_config': 'xbox'}.items())
    
    # -- launch decision tree --
    decision_tree_node = Node(package='decision', executable='bt_v1_node', output='screen')
    
    # -- launch box detector (aruco-based) --
    aruco_det_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(\
        object_detection_pkg_dir, 'launch', 'aruco_detector_launch.py')))
    
    # -- launch decision tree debugger --
    decision_tree_debugger_node = Node(package='decision', executable='bt_v1_debugger')
    # dummy map->odom
    map_2_odom = Node(executable='static_transform_publisher', package='tf2_ros', arguments=['--child-frame-id', 'odom', '--frame-id', 'map'])
    
    # -- launch path planner and trajectory follower --
    path_planner_node = Node(package='motion_planning', executable='path_planner_optimized', output='screen')
    trajectory_follower_node = Node(package='motion_planning', executable='trajectory_follower')
    
    # -- launch object detection stuff --
    dl_classifier_node = Node(package='object_detection', executable='dl_object_classifier')
    dl_post_processor_node = Node(package='object_detection', executable='dl_postprocessor')
    object_tracker_node = Node(package='object_tracker', executable='object_tracker_node')
    
    # -- launch arm controller --
    robo_arm_ik_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(\
        bringup_pkg_dir, 'launch', 'robo_arm_ik_launch.py')))
    arm_node = Node(package='arm_controller', executable='inverse_kinematics')
    
    ld = LaunchDescription([sensors_launch, chassis_launch, joystick_launch, decision_tree_debugger_node, map_2_odom, \
        mapper_node, 
        path_planner_node, 
        # trajectory_follower_node, 
        # decision_tree_node, 
        dl_classifier_node,
        dl_post_processor_node,
        object_tracker_node,
        aruco_det_launch,
        robo_arm_ik_launch,
        arm_node,
        ])
    
    return ld