#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

from sklearn.cluster import KMeans
from scipy import stats

import sensor_msgs_py.point_cloud2 as pc2
from open3d import open3d as o3d

import ctypes
import struct
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import sys
from sensor_msgs.msg import PointField  



class Detection(Node):

    def __init__(self):
        super().__init__('detection')

        # Initialize the publisher
        self._pub = self.create_publisher(
            PointCloud2, '/camera/depth/color/ds_points', 100)

        # Subscribe to point cloud topic and call callback function on each recieved message
        self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.cloud_callback, 10)
        
        # Create a publisher for center points
        self._center_pub = self.create_publisher(
            MarkerArray, '/object_centers', 10)
        
        self.tf_buffer = Buffer(rclpy.duration.Duration(seconds=100.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        
        self.marker_id_counter = 0 
        
        self.RED_OBJECT_COLOR = (150, 40, 20)
        self.GREEN_OBJECT_COLOR = (0, 70, 60)
        self.BLUE_OBJECT_COLOR = (0, 90, 130)
        self.RED_TOLERANCE = 45
        self.GREEN_TOLERANCE = 40
        self.BLUE_TOLERANCE = 50
        self.DIST_THRESHOLD = 2.0 


    def cloud_callback(self, msg: PointCloud2):

        # Convert ROS -> NumPy
        gen = pc2.read_points_numpy(msg, skip_nans=True)
        xyz = gen[:,:3]
        rgb = np.empty(xyz.shape, dtype=np.uint32)

        for idx, x in enumerate(gen):
            c = x[3]
            s = struct.pack('>f' , c)
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value
            rgb[idx, 0] = np.asarray((pack >> 16) & 255, dtype=np.uint8) 
            rgb[idx, 1] = np.asarray((pack >> 8) & 255, dtype=np.uint8) 
            rgb[idx, 2] = np.asarray(pack & 255, dtype=np.uint8)

        rgb = rgb.astype(np.float32) / 255

        # Convert NumPy -> Open3D
        o3d_point_cloud = o3d.geometry.PointCloud()    
        o3d_point_cloud.points = o3d.utility.Vector3dVector(xyz)
        o3d_point_cloud.colors = o3d.utility.Vector3dVector(rgb)

        # Downsample the point cloud to 5 cm
        ds_o3d_point_cloud = o3d_point_cloud.voxel_down_sample(voxel_size=0.005)

        # Convert Open3D -> NumPy
        points = np.asarray(ds_o3d_point_cloud.points)
        raw_colors = np.asarray(ds_o3d_point_cloud.colors)
        colors = (raw_colors * 255).astype(np.uint8)
 
        red_color_differences =np.abs( np.linalg.norm(colors - self.RED_OBJECT_COLOR, axis=1))
        green_color_differences =np.abs( np.linalg.norm(colors - self.GREEN_OBJECT_COLOR, axis=1))
        blue_color_differences =np.abs( np.linalg.norm(colors - self.BLUE_OBJECT_COLOR, axis=1))
        
        GREEN = '\033[92m'
        RED = '\033[91m'
        BLUE = '\033[36m'#'\033[34m'
        RESET = '\033[0m'

        filtered_points = []

        filtered_points_dictionary = {
            'red': [],
            'green': [],
            'blue': [],
        }

        for i in range(len(colors)):
           

            dist = np.sqrt(points[i][0]**2 + points[i][1]**2 + points[i][2]**2)


            if red_color_differences[i] < self.RED_TOLERANCE and dist < self.DIST_THRESHOLD:
                self.get_logger().info(RED +f"_Red Object detected"+ RESET)
                filtered_points.append([points[i][0], points[i][1], points[i][2], colors[i][0], colors[i][1], colors[i][2]])
                filtered_points_dictionary['red'].append([points[i][0], points[i][1], points[i][2], colors[i][0], colors[i][1], colors[i][2]])

            if green_color_differences[i] < self.GREEN_TOLERANCE and dist < self.DIST_THRESHOLD:
                self.get_logger().info(GREEN+f"_Green Object detected"+ RESET)
                filtered_points.append([points[i][0], points[i][1], points[i][2], colors[i][0], colors[i][1], colors[i][2]])
                filtered_points_dictionary['green'].append([points[i][0], points[i][1], points[i][2], colors[i][0], colors[i][1], colors[i][2]])


            if blue_color_differences[i] < self.BLUE_TOLERANCE and dist < self.DIST_THRESHOLD: 
                self.get_logger().info(BLUE+f"_Blue Object detected"+ RESET)
                filtered_points.append([points[i][0], points[i][1], points[i][2], colors[i][0], colors[i][1], colors[i][2]])
                filtered_points_dictionary['blue'].append([points[i][0], points[i][1], points[i][2], colors[i][0], colors[i][1], colors[i][2]])

        if filtered_points:
            # Create a new PointCloud2 message with filtered data
            filtered_pc2_msg = PointCloud2()
            filtered_pc2_msg.header = msg.header
            filtered_pc2_msg.height = msg.height
            filtered_pc2_msg.width = len(filtered_points)
            filtered_pc2_msg.fields = msg.fields

            filtered_pc2_msg.is_bigendian = False
            filtered_pc2_msg.point_step = msg.point_step  # Length of a point in bytes (4 floats)
            filtered_pc2_msg.row_step = filtered_pc2_msg.point_step * filtered_pc2_msg.width
            filtered_pc2_msg.data = bytes()  # Initialize the data as empty
            filtered_pc2_msg.is_dense = True  # Set to True if there are no invalid points

            # Create an array to hold the binary data for all points
            filtered_data = bytearray()

            for point in filtered_points:
                x, y, z, r, g, b = point

                # Convert x, y, z, and rgb values to bytes and append them to filtered_data
                point_data = bytearray(struct.pack('<ffffI', x, y, z, 0.0, 0))  # Insert four zeros at offset 12
                rgb_value = ((int(r) << 16) | (int(g) << 8) | int(b))
                struct.pack_into('<I', point_data, 16, rgb_value)  # Update the rgb value at offset 16
                filtered_data.extend(point_data)
            
            filtered_pc2_msg.data = bytes(filtered_data)

            # Publish the filtered PointCloud2 message
            self._pub.publish(filtered_pc2_msg)

            self.culc_publish_object_centers(filtered_points_dictionary, msg)

    def culc_publish_object_centers(self, filtered_points_dictionary, msg):
            
            object_centers = []

            for color, points_list in filtered_points_dictionary.items():

                points_array = np.array([point[:3] for point in points_list])

                filtered_points_array = self.remove_outliers_z_score(points_array)
                
                if len(filtered_points_array) > 0:  # Ensure there are points to process
                    kmeans = KMeans(n_clusters=1, random_state=0, n_init=10).fit(filtered_points_array)
                    centroid = kmeans.cluster_centers_[0]  # Extract the centroid coordinates


                    marker = Marker()
                    marker.header = msg.header
                    marker.header.frame_id = 'odom'
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    center_x = centroid[0]
                    center_y = centroid[1]
                    center_z = centroid[2]

                    try:
                        t = self.tf_buffer.lookup_transform(
                            'odom', 
                            msg.header.frame_id,
                            rclpy.time.Time())
                        
                    except TransformException as ex:
                            self.get_logger().info(
                                f'Could not transform: {ex}')
                            return
                    
                    marker.pose.position.x = t.transform.translation.x + center_z
                    marker.pose.position.y = t.transform.translation.y - center_x 
                    marker.pose.position.z = t.transform.translation.z - center_y 
                    marker.lifetime = Duration(sec=1, nanosec=0)
                    marker.scale.x = 0.01  # Adjust the scale as needed
                    marker.scale.y = 0.01
                    marker.scale.z = 0.01
                    marker.color.r = 1.0  
                    marker.color.g = 1.0
                    marker.color.b = 1.0
                    marker.color.a = 1.0  
                    marker.ns = color+"_cube_centroid"  # Set a unique namespace for object with each color
                    marker.id = self.marker_id_counter  
                    self.marker_id_counter += 1  

                    object_centers.append(marker)

                marker_array_msg = MarkerArray(markers=object_centers)
                self._center_pub.publish(marker_array_msg)


    def remove_outliers_z_score(self, points_array, threshold=1.5):
            # Ensure points_array is at least 2-dimensional
        if points_array.ndim == 1:
            # Handle the case where points_array might be 1-dimensional
            points_array = points_array.reshape(-1, 1)
        elif points_array.size == 0:
            # If points_array is empty, directly return it without further processing
            return points_array

        # Proceed with calculating Z-scores and filtering
        z_scores = np.abs(stats.zscore(points_array, axis=0))
        
        # Check if z_scores calculation returned an empty or invalid result
        if z_scores.size == 0 or z_scores.ndim < 2:
            return np.array([]).reshape(-1, points_array.shape[1])

        filtered_points = points_array[(z_scores < threshold).all(axis=1)]
        return filtered_points



def main():
    rclpy.init()
    node = Detection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()