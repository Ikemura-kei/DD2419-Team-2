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
    ik_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup_pkg, 'launch', 'robo_arm_ik_launch.py')))
    
    obj_det_node = Node(package='object_detection', executable='object_detection')
    tar_ctrl_node = Node(package='controller', executable='target_position_controller')
    dec_node = Node(package='decision', executable='ms2_pick_and_place_node')
    ld = LaunchDescription([chassis_launch, tar_ctrl_node, dec_node, sensor_launch, obj_det_node, ik_launch])
    
    return ld