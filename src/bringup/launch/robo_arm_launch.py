from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    micro_ros_agent_node = Node(executable='micro_ros_agent', package='micro_ros_agent', arguments=['serial', '--dev', '/dev/ttyUSB0', '-v6'])
    
    return LaunchDescription([micro_ros_agent_node])