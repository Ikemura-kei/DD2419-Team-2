from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
    bringup_pkg = get_package_share_directory('bringup')
    chassis_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup_pkg, 'launch', 'chassis_launch.py')))
    sensor_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup_pkg, 'launch', 'sensors_launch.py')))
    
    obj_det_node = Node(package='object_detection', executable='object_detection')
    tar_ctrl_node = Node(package='controller', executable='target_position_controller')
    dec_node = Node(package='decision', executable='ms1_go_to_obj_node_v2')
    ld = LaunchDescription([chassis_launch, tar_ctrl_node, dec_node, sensor_launch, obj_det_node])
    
    return ld