#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs
# from tf_transformations import quaternion_about_axis, quaternion_multiply

# import math


from geometry_msgs.msg import TransformStamped


# ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id camera_color_optical_frame


class DisplayMarkers(Node):

    def __init__(self):
        super().__init__('display_markers')

        # Initialize the transform listener and assign it a buffer
        self.tf2Buffer = Buffer(cache_time=None)

        #transform listener fills the buffer
        self.listener = TransformListener(self.tf2Buffer, self)
        
        # Initialize the transform broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)


        # Subscribe to aruco marker topic and call callback function on each received message
        self.create_subscription(
            MarkerArray, '/aruco/markers', self.aruco_callback, 10)
    

    def aruco_callback(self, msg: MarkerArray) :

        # Broadcast/publish the transform between the map frame and the detected aruco marker


        for marker in msg.markers:

            marker_stamp = marker.header.stamp
            camera_frame = marker.header.frame_id
            
            map_frame = "base_link"

            #TODO: SHOULD ADD TIMEOUT!!!
            if self.tf2Buffer.can_transform(map_frame, camera_frame, marker_stamp):
                transform = self.tf2Buffer.lookup_transform(map_frame, camera_frame, marker_stamp)
                aruco_pose = marker.pose.pose
                
                aruco_map_pose = tf2_geometry_msgs.do_transform_pose(aruco_pose, transform)

                t = TransformStamped()
                t.header.stamp = marker_stamp
                t.header.frame_id = map_frame
                t.child_frame_id = '/aruco/detected' + str(marker.id)

                t.transform.translation.x = aruco_map_pose.position.x
                t.transform.translation.y = aruco_map_pose.position.y
                t.transform.translation.z = aruco_map_pose.position.z
                t.transform.rotation.x = aruco_map_pose.orientation.x
                t.transform.rotation.y = aruco_map_pose.orientation.y
                t.transform.rotation.z = aruco_map_pose.orientation.z
                t.transform.rotation.w = aruco_map_pose.orientation.w

                # #Need to rotate
                
                # rotate_x = quaternion_about_axis(3*math.pi/2, (1,0,0))
                # rotate_z = quaternion_about_axis(3*math.pi/2, (0,0,1))

                # interim = quaternion_multiply((aruco_map_pose.orientation.x,aruco_map_pose.orientation.y,aruco_map_pose.orientation.z,aruco_map_pose.orientation.w), rotate_x)
                # aruco_rotated = quaternion_multiply(interim, rotate_z)

                # t.transform.rotation.x = aruco_rotated[0]
                # t.transform.rotation.y = aruco_rotated[1]
                # t.transform.rotation.z = aruco_rotated[2]
                # t.transform.rotation.w = aruco_rotated[3]

                self._tf_broadcaster.sendTransform(t)
        

def main():
    rclpy.init()
    node = DisplayMarkers()
    try :
        rclpy.spin(node)
    except KeyboardInterrupt :
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()