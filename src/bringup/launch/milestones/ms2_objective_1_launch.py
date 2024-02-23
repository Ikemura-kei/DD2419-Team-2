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
    aruco_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(det_pkg, 'launch', 'aruco_detector_launch.py')))

    tar_ctrl_node = Node(package='controller', executable='target_position_controller')
    obj_det_node = Node(package='object_detection', executable='object_detection')
    arm_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup_pkg, 'launch', 'robo_arm_open_launch.py')))

    ld = LaunchDescription([aruco_launch, arm_launch, obj_det_node, chassis_launch, map_launch, tar_ctrl_node, sensor_launch])
    
    return ld