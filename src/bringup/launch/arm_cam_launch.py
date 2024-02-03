import argparse
import os
from pathlib import Path  # noqa: E402
import sys

# Hack to get relative import of .camera_config file working
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)

from pathlib import Path
from typing import List, Optional

from ament_index_python.packages import get_package_share_directory

USB_CAM_DIR = get_package_share_directory('bringup')

from launch import LaunchDescription  # noqa: E402
from launch.actions import GroupAction  # noqa: E402
from launch_ros.actions import Node  # noqa: E402


def generate_launch_description():
    ld = LaunchDescription()

    parser = argparse.ArgumentParser(description='usb_cam demo')
    parser.add_argument('-n', '--node-name', dest='node_name', type=str,
                        help='name for device', default='usb_cam')
    name='arm_cam'
    param_file = Path(USB_CAM_DIR, 'configs', 'arm_cam_ros_params.yaml')
    
    return LaunchDescription([Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name=name,
            namespace='',
            parameters=[param_file],)])