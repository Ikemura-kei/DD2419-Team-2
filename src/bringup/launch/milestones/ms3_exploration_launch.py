from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
    bringup_pkg = get_package_share_directory('bringup')
    mapping_pkg = get_package_share_directory('mapping')
    chassis_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup_pkg, 'launch', 'chassis_launch.py')))
    sensor_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup_pkg, 'launch', 'sensors_launch.py')))
    mapping_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(mapping_pkg, 'launch', 'naive_mapping_launch.py')))
    
    tar_ctrl_node = Node(package='controller', executable='target_position_controller')
    path_planner_node = Node(package='motion_planning', executable='path_planner')
    exp_node = Node(package='decision', executable='ms3_exploration_node')
    ld = LaunchDescription([chassis_launch, mapping_launch, tar_ctrl_node, exp_node, sensor_launch, path_planner_node])
    
    return ld