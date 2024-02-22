import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bag_file = os.path.join(get_package_share_directory('bringup'), '../../../../bags', 'rosbag2_2024_02_21-20_59_14')
    bag_node = launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_file],
            output='screen'
        )
    
    map_launch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('mapping'), 'launch', 'naive_mapping_launch.py'))
    map_launch = IncludeLaunchDescription(map_launch)
    
    bringup_dir = get_package_share_directory('bringup')
    rviz_config = 'rviz/basic.rviz'
    rviz_config = os.path.join(bringup_dir, rviz_config)
    rviz_node = Node(executable='rviz2', package='rviz2', arguments=['-d', rviz_config])
    
    ld = LaunchDescription([bag_node, rviz_node, map_launch])
    
    return ld