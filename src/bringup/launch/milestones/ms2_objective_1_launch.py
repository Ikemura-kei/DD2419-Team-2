from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
    bringup_pkg = get_package_share_directory('bringup')
    mapping_pkg = get_package_share_directory('mapping')
    det_pkg = get_package_share_directory('object_detection')
    chassis_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup_pkg, 'launch', 'chassis_launch.py')))
    sensor_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup_pkg, 'launch', 'sensors_launch.py')))
    map_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(mapping_pkg, 'launch', 'naive_mapping_launch.py')))
    
    tar_ctrl_node = Node(package='controller', executable='target_position_controller')

    ld = LaunchDescription([chassis_launch, map_launch, tar_ctrl_node, sensor_launch])
    
    return ld