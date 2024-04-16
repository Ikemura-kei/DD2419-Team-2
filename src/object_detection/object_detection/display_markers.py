#!/usr/bin/env python

import rclpy
import rclpy.duration
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray as ArucoMarkerArray
from visualization_msgs.msg import MarkerArray, Marker
from dd2419_interfaces.msg import ObjectList, ObjectPoses

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from tf2_geometry_msgs import do_transform_pose_stamped
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose
import numpy as np

from copy import deepcopy
from PyKDL import Rotation, Vector

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
        self.BOX_LEN_X = 0.24
        self.BOX_LEN_Y = 0.1
        self.BOX_LEN_Z = 0.16
        self.create_subscription(
            ArucoMarkerArray, '/aruco/markers', self.aruco_callback, 10)
        
        self.box_marker_pub = self.create_publisher(MarkerArray, '/box_markers', 10)
        self.box_list_pub = self.create_publisher(ObjectPoses, '/box_list_real', 10)
        
        self.marker_template = self.prep_marker_template()
        self.box_list = ObjectPoses()
        
        self.timer = self.create_timer(0.1, self.run)
        
    def run(self):
        self.box_list_pub.publish(self.box_list)

    def aruco_callback(self, msg: ArucoMarkerArray) :
        vis_markers = MarkerArray()
        
        can_transform, debug_msg = self.tf2Buffer.can_transform('map', msg.header.frame_id, msg.header.stamp, return_debug_tuple=True)
        if not can_transform:
            self.get_logger().warn(f"Cannot transform from {msg.header.frame_id} to map: {debug_msg}")
            return
        
        try:
            transform_to_map = self.tf2Buffer.lookup_transform('map', msg.header.frame_id, msg.header.stamp, rclpy.duration.Duration(seconds=0.05))
        except TransformException as e:
            self.get_logger().warn(f"Failed to transform from {msg.header.frame_id} to map: {e}")
            return
        

        for marker in msg.markers:
            T = np.zeros((4, 4))
            T[:, 3] = [marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z, 1] # the translation part
            rot_mat = Rotation.Quaternion(marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z, marker.pose.pose.orientation.w)
            for i in range(3):
                for j in range(3):
                    T[i, j] = rot_mat[i, j]
            
            inv_T = np.linalg.inv(T)
            inv_T[2, 3] += (self.BOX_LEN_Z / 2.0)
            T = np.linalg.inv(inv_T)
            
            marker.pose.pose.position.x = T[0, 3]
            marker.pose.pose.position.y = T[1, 3]
            marker.pose.pose.position.z = T[2, 3]
            transform = self.prep_transform(marker.header, marker.pose.pose, marker.id)
            self._tf_broadcaster.sendTransform(transform)
            
            vis_markers.markers.append(self.prep_marker(marker.header, marker.id))
            
            # -- TODO: change this to be adaptive, now just for testing --
            for name in ['Hugo_Hugo1', 'Oakie_Oakie1', 'Muddles_Muddles1', 'Blue_ball_Blue_ball1', \
                'Green_ball_Green_ball1', 'Binky_Binky1', 'Slush_Slush1']:
                pose_map: PoseStamped = do_transform_pose_stamped(marker.pose, transform_to_map)
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = marker.header.stamp
                pose.pose = pose_map.pose
                if name not in self.box_list.object_list.object_list:
                    self.box_list.object_list.object_list.append(name)
                    self.box_list.poses.append(pose)
                else:
                    index = self.box_list.object_list.object_list.index(name)
                    self.box_list.poses[index] = pose
            
        self.box_marker_pub.publish(vis_markers)
        
    def prep_transform(self, header, pose, id):
        transform_to_this_box = TransformStamped()
        transform_to_this_box.header.stamp = header.stamp
        transform_to_this_box.header.frame_id = header.frame_id 
        transform_to_this_box.child_frame_id = "box_" + str(id)
        transform_to_this_box.transform.translation.x = pose.position.x
        transform_to_this_box.transform.translation.y = pose.position.y
        transform_to_this_box.transform.translation.z = pose.position.z
        transform_to_this_box.transform.rotation.x = pose.orientation.x
        transform_to_this_box.transform.rotation.y = pose.orientation.y
        transform_to_this_box.transform.rotation.z = pose.orientation.z
        transform_to_this_box.transform.rotation.w = pose.orientation.w
        
        return transform_to_this_box
    
    def prep_marker(self, header, id):
        box = deepcopy(self.marker_template)
        box.header.stamp = header.stamp
        box.id = id
        box.header.frame_id = "box_" + str(id)
        
        return box
        
    def prep_marker_template(self):
        box = Marker()
        box.type = Marker.CUBE
        box.scale.x = self.BOX_LEN_X
        box.scale.y = self.BOX_LEN_Y
        box.scale.z = self.BOX_LEN_Z
        box.color.a = 1.0
        box.color.r = box.color.g = box.color.b = 0.75
        box.pose.position.x = 0.0
        box.pose.position.y = 0.0
        box.pose.position.z = 0.0
        box.pose.orientation.w = 1.0
        box.lifetime = rclpy.time.Duration(seconds=0.5).to_msg()
        
        return box
        

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