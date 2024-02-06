from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_ros',
            executable='marker_publisher',
            name='aruco',
            remappings=[
                ("/camera_info", "/camera_info"),
                ("/image", "/image_raw"),
            ],
            parameters=[{'image_is_rectified':True,
                         'marker_size': 0.157,
                         'reference_frame': "base_link",
                         'camera_frame': "arm_camera"}]
        )
    ])