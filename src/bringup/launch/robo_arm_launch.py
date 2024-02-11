from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    teleop_twist_joy_dir = get_package_share_directory('teleop_twist_joy')
    
    micro_ros_agent_node = Node(executable='micro_ros_agent', package='micro_ros_agent', arguments=['serial', '--dev', '/dev/ttyUSB0', '-v6'])
    
    # joystick_ctrl_node = Node(executable='joystick_controller', package='arm_controller')
    
    joystick_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(teleop_twist_joy_dir, 'launch/teleop-launch.py')), launch_arguments={'joy_config': 'xbox'}.items())
    
    # return LaunchDescription([joystick_launch, micro_ros_agent_node, open_loop_ctrl_node])
    return LaunchDescription([joystick_launch, micro_ros_agent_node])