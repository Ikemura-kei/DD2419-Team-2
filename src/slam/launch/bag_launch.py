import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node 
import os

def generate_launch_description():
    
    # -- locate the bag file --
    bag_file = os.path.join(get_package_share_directory('bringup'), '../../../../bags', 'scene_1_traj_1')

    # -- launch the bag file --
    bag_node = launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_file],
            output='screen'
        )
    
    # -- launch the odometry node --
    odometry_node = Node(package='odometry', executable='odometry', output='screen')
    
    # -- launch the rviz node --
    this_dir = get_package_share_directory('slam')
    rviz_config = 'rviz/mapping.rviz'
    rviz_config = os.path.join(this_dir, rviz_config)
    rviz_node = Node(executable='rviz2', package='rviz2', arguments=['-d', rviz_config])
    
    ld = LaunchDescription([bag_node, odometry_node, rviz_node])
    
    return ld