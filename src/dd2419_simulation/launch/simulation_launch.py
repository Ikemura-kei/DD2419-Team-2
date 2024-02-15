from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_simulation = get_package_share_directory("dd2419_simulation")
    rviz_file = os.path.join(pkg_simulation, 'rviz', 'simulation.rviz')

    world = os.path.join(pkg_simulation, 'world', 'simulation.world')
    
    # Start Gazebo server
    gz_server = os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_server),
        launch_arguments={'world': world}.items())

    # Start Gazebo client    
    gz_clinet = os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_clinet))
    
    baselink_2_lidar_tf = Node(executable='static_transform_publisher', package='tf2_ros', arguments=['--child-frame-id', 'laser', '--frame-id', 'base_link', '--x', '0.065', '--y', '0', '--z', '0.3215'])

    rviz = Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_file])
    
    sim_node = Node(package='dd2419_simulation', executable='simulation_node')
    
    ld = LaunchDescription([baselink_2_lidar_tf, start_gazebo_server_cmd, start_gazebo_client_cmd, rviz, sim_node])
    
    return ld
    