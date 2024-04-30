from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    
    pkg_robp_phigets_motors = get_package_share_directory("robp_phidgets_motors")
    pkg_robp_phidgets_encoders = get_package_share_directory("robp_phidgets_encoders")
    
    motors_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_robp_phidgets_encoders, 'launch', 'encoders_launch.py')))
    encoders_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_robp_phigets_motors, 'launch', 'motors_launch.py')))
    
    # controller_node = Node(package='controller', executable='cartesian_controller')
    controller_node = Node(package='controller', executable='open_loop_controller')
    odometry_node = Node(package='odometry', executable='odometry', output='screen')
    
    ld = LaunchDescription([motors_launch, encoders_launch, controller_node, odometry_node])
    
    return ld