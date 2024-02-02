#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
rviz_path = "/home/team2/dd2419_ws/src/bringup/rviz/dd2419_basic.rviz"
def generate_launch_description():    
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=['-d', str(rviz_path)],
            output='screen'),
    ])