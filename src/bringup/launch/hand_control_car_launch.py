from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    bringup_pkg = get_package_share_directory('bringup')
    pkg_robp_phigets_motors = get_package_share_directory("robp_phidgets_motors")
    pkg_robp_phidgets_encoders = get_package_share_directory("robp_phidgets_encoders")
    teleop_twist_joy_dir = get_package_share_directory('teleop_twist_joy')
    
    motors_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_robp_phidgets_encoders, 'launch', 'encoders_launch.py')))
    encoders_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_robp_phigets_motors, 'launch', 'motors_launch.py')))
    sensor_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup_pkg, 'launch', 'sensors_launch.py')))
    joystick_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(teleop_twist_joy_dir, 'launch/teleop-launch.py')), launch_arguments={'joy_config': 'xbox'}.items())

    controller_node = Node(package='controller', executable='open_loop_controller')
    
    ld = LaunchDescription([motors_launch, sensor_launch, encoders_launch, joystick_launch, controller_node])
    
    return ld