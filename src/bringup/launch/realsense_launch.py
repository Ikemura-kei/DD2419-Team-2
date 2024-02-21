from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os

def generate_launch_description():
    realsense_path = get_package_share_directory('realsense2_camera')
    realsense_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(realsense_path, 'launch/rs_launch.py')), launch_arguments={'pointcloud.enable': 'true'}.items())
    
    #this static transform was given to us
    baselink_2_realsense_tf = Node(executable='static_transform_publisher', package='tf2_ros', arguments=['--child-frame-id', 'camera_link', '--frame-id', 'base_link', '--x', '0.08987', '--y', '0.0175', '--z', '0.10456'])
    
    return LaunchDescription([realsense_launch, baselink_2_realsense_tf])