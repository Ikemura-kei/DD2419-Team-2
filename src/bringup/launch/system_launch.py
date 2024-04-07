import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bringup_pkg_dir = get_package_share_directory('bringup')
    teleop_twist_joy_pkg_dir = get_package_share_directory('teleop_twist_joy')
    
    # -- launch sensors --
    sensors_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(\
        bringup_pkg_dir, 'launch', 'sensors_launch.py')))
    
    # -- launch chassis (motor, encoder, odometry, etc.) --
    chassis_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(\
        bringup_pkg_dir, 'launch', 'chassis_launch.py')))
    
    # -- launch joy stick, for emergency control --
    joystick_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(\
        teleop_twist_joy_pkg_dir, 'launch/teleop-launch.py')), launch_arguments={'joy_config': 'xbox'}.items())
    
    # -- launch decision tree --
    # decision_tree_node = Node(package='decision', executable='bt_v1_node', output='screen')
    
    # -- launch decision tree debugger --
    decision_tree_debugger_node = Node(package='decision', executable='bt_v1_debugger')
    # dummy map->odom
    map_2_odom = Node(executable='static_transform_publisher', package='tf2_ros', arguments=['--child-frame-id', 'odom', '--frame-id', 'map'])
    
    ld = LaunchDescription([sensors_launch, chassis_launch, joystick_launch, decision_tree_debugger_node, map_2_odom])
    
    return ld