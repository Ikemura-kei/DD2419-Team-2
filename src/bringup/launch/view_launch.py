#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():    
    bringup_dir = get_package_share_directory('bringup')
    rviz_config = 'rviz/basic.rviz'
    lidar_launch = 'launch/lidar_launch.py'
    realsense_launch = 'launch/realsense_launch.py'
    lidar_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup_dir, lidar_launch)))
    realsense_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup_dir, realsense_launch)))
    
    rviz_config = os.path.join(bringup_dir, rviz_config)
    rviz_node = Node(executable='rviz2', package='rviz2', arguments=['-d', rviz_config])

    return LaunchDescription([
        lidar_launch,
        realsense_launch,
        rviz_node
    ])