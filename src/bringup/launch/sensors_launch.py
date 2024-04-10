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
    
    lidar_launch = 'launch/lidar_launch.py'
    realsense_launch = 'launch/realsense_launch.py'
    usb_camera_launch = 'launch/arm_cam_launch.py'
    view_launch = 'launch/view_launch.py'
    
    lidar_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup_dir, lidar_launch)))
    realsense_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup_dir, realsense_launch)))
    usb_camera_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup_dir, usb_camera_launch)))
    view_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup_dir, view_launch)))

    return LaunchDescription([
        usb_camera_launch,
        lidar_launch,
        realsense_launch,
        # view_launch
    ])