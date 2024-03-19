#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():    
    bringup_dir = get_package_share_directory('bringup')
    rviz_config = 'rviz/basic.rviz'
    rviz_config = os.path.join(bringup_dir, rviz_config)
    rviz_node = Node(executable='rviz2', package='rviz2', arguments=['-d', rviz_config])

    return LaunchDescription([
        rviz_node
    ])