from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    det_dir = get_package_share_directory("object_detection")
    det_launch = os.path.join(det_dir, 'launch', 'aruco_detector_launch.py')
    det_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(det_launch))
    
    ld = LaunchDescription([det_launch])
    
    return ld