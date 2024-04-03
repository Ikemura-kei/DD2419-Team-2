import launch
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    
    naive_mapping_node = Node(package='mapping', executable='naive_mapping_node')
    
    ld = LaunchDescription([naive_mapping_node])
    return ld