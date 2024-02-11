from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    marker_node = Node(package='object_detection', executable='display_markers')
    
    return LaunchDescription([
        marker_node,
        Node(
            package='aruco_ros',
            executable='marker_publisher',
            name='aruco',
            remappings=[
                ("/camera_info", "/camera/color/camera_info"),
                ("/image", "/camera/color/image_raw"),
            ],
            parameters=[{'image_is_rectified':True,
                         'marker_size': 0.05,
                         'reference_frame': "",
                         'camera_frame': "camera_color_optical_frame"}]
        )
    ])