#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray as ArucoMarkerArray
from visualization_msgs.msg import MarkerArray, Marker

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped

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
            ArucoMarkerArray, '/aruco/markers', self.aruco_callback, 10)
        
        self.box_marker_pub = self.create_publisher(MarkerArray, '/box_markers', 10)

    def aruco_callback(self, msg: ArucoMarkerArray) :
        vis_markers = MarkerArray()
        for marker in msg.markers:
            box = Marker()
            box.header = marker.header
            box.pose = marker.pose.pose
            box.type = Marker.CUBE
            box.scale.x = 0.18
            box.scale.y = 0.25
            box.scale.z = 0.12
            box.id = marker.id
            box.color.a = 1.0
            box.color.r = box.color.g = box.color.b = 0.75
            box.lifetime = rclpy.time.Duration(seconds=0.5).to_msg()
            
            vis_markers.markers.append(box)
            
        self.box_marker_pub.publish(vis_markers)

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