#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from dl_perception_interfaces.msg import  BoundingBoxArray, ObjectInstanceArray, ObjectInstance
import rclpy.time
import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import time
from tf2_geometry_msgs import  PointStamped
from geometry_msgs.msg import TransformStamped, Point, Quaternion, Vector3
from visualization_msgs.msg import Marker, MarkerArray
import message_filters 
from scipy.cluster.hierarchy import linkage, fcluster
from scipy.spatial.distance import pdist
import numpy as np
from collections import Counter
import math
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
from std_msgs.msg import String
from builtin_interfaces.msg import Time
from datetime import timedelta
from copy import deepcopy
from typing import List

class Instance():
    postion = None
    category_name = ""
    stamp = None
    bbox = [0, 0, 0, 0] # (x, y, width, height)
    image = None
    
    def __init__(self, category_name, position, stamp, bbox, image) -> None:
        self.category_name = category_name
        self.position = position
        self.stamp = stamp
        self.bbox = bbox
        self.image = image
        
class TfNode(Node):
    def __init__(self):
        super().__init__('tf_node')

class Object_postprocessor(Node):
    def __init__(self):
        """ Put the node name here, and description of the node"""
        super().__init__('dl_object_postprocessor')

        self.tf_node = TfNode()
       
        # Tf 
        self.tfBuffer = Buffer(cache_time=rclpy.duration.Duration(seconds=5))
        self.listener = TransformListener(self.tfBuffer, self.tf_node, spin_thread=True)

        
        # Parameters
        self.objects_dict = {}
        self.frame_id = "camera_color_optical_frame"
        self.temp_dict = {}

        self.directory = "/home/team2/dd2419_ws/src/object_detection/object_detection/saved_instances"
        self.bridge = CvBridge()
        self.NUM_OBSERVATION_THRESHOLD = 4
        self.id = 0
        
        self.declare_parameter('reduce_categories', True)  # Default value is True
        self.reduce_categories = self.get_parameter('reduce_categories').value
        self.mapping_animals = ["Binky", "Hugo", "Slush", "Muddles", "Kiki", "Oakie"]
        self.mapping_cubes = ["Red_cube", "Green_cube", "Blue_cube", "Wooden_cube"]
        self.mapping_spheres = ["Red_ball", "Green_ball", "Blue_ball"]

        # Define rate
        self.update_rate = 1 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = self.create_rate(self.update_rate)

        self.make_marker_template()
        # Subscribers 
        self.sub_bounding_boxes = self.create_subscription(
            BoundingBoxArray,
            "detection/bounding_boxes",
            self.bounding_boxes_callback,
            10)

        self.sub_image = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.image_callback,
            10)
        
        self.sub_remove_instance = self.create_subscription(
            String, 
            "/detection/remove_instance", 
            self.remove_instance_callback, 
            10)
        
        # Create a list to store incoming messages 
        self.bounding_boxes_buffer = []
        self.image_buffer = []
    
        
        # Publisher
        self.instances_pub = self.create_publisher(
            ObjectInstanceArray,
             "/detection/object_instances", 
            10)
        
        self.markers_pub = self.create_publisher(MarkerArray, '/detection/marker_array', 10)
        
        self.speaker_pub = self.create_publisher(
            String, 
            "/speaker/speech", 
            10)
        
        time.sleep(1)
        
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.LOOK_BACK_DURATION = 2.35 # [s]
        self.OBJECT_NEAR_THRESHOLD = 0.305 # [m]
        self.OBJ_HEIGHT_THRESHOLD = 0.0555 # [m]
        self.DET_CNT_THRESHOLD = 30
        
    def filter(self, batch, time):
        nb_msgs = len(batch)
        if nb_msgs == 0:
            return
        
        # -- re-organize data into separate lists --
        points = []
        bb_list = []
        for curr_msg in batch:
            for bb in curr_msg.bounding_boxes:
                bb_list.append(bb)
                points.append([bb.bb_center.x, bb.bb_center.y, bb.bb_center.z])
        
        # -- compute pair-wise distances between all bboxes --
        pair_distances = pdist(points, 'euclidean')
        
        if len(pair_distances) == 0:
            return
        
        # -- cluster bboxes --
        linkage_matrix = linkage(pair_distances, method='single', metric='euclidean')
        clusters = fcluster(linkage_matrix, t=0.05, criterion='distance')

        # -- keep object clusters with more than NUM_OBSERVATION_THRESHOLD bbox detected --
        bbs_by_cluster = []
        for this_cluster_index in np.unique(clusters):
            bb_cluster = []
            matched_indexes = [j for j, that_cluster_index in enumerate(clusters) if that_cluster_index == this_cluster_index ]
            for index in matched_indexes:
                bb_cluster.append(bb_list[index])
            
            num_observations = len(bb_cluster)
            if num_observations > self.NUM_OBSERVATION_THRESHOLD:
                bbs_by_cluster.append(bb_cluster)

        # -- make conclusion on object position as well as object lable given multiple observations --
        instances_to_save = []
        for j, cluster in enumerate(bbs_by_cluster):
            
            category_names = [o.category_name for o in cluster]
            x = [o.bb_center.x for o in cluster]
            y = [o.bb_center.y for o in cluster]
            z = [o.bb_center.z for o in cluster]

            # -- define category name as the maximum vote --
            occurence_count = Counter(category_names)
            category_name = occurence_count.most_common(1)[0][0]
            
            # -- define object position as the mean of all observations --
            x = np.mean(x)
            y = np.mean(y)
            z = np.mean(z)

            if y < self.OBJ_HEIGHT_THRESHOLD:
                continue
            
            # -- just pick a random bounding box as the bounding box of this object --
            bb = cluster[int(self.NUM_OBSERVATION_THRESHOLD/2)]
            
            # -- just pick a random time stamp as the time stamp of this object --
            bbox_stamp = bb.stamp
            
            # -- pick the image that is closest in time to the chosen bounding box time as our evidence --
            closest_stamp_diff = float('inf')
            closest_image = None

            for image in self.image_buffer:
                stamp = image.header.stamp        
                stamp_diff = abs((bbox_stamp.sec * 1e9 + bbox_stamp.nanosec) - (stamp.sec * 1e9 + stamp.nanosec))
                if stamp_diff < closest_stamp_diff:
                    closest_stamp_diff = stamp_diff
                    closest_image = image
                    
            image = closest_image
            
            # -- finally, we construct an object instance out of all of the observations --
            instances_to_save.append(Instance(category_name, [x, y, z], bbox_stamp, [bb.x, bb.y, bb.width, bb.height], image))

        self.save_instances(instances_to_save)

    # instances_to_save.append([(category_name, x, y, z), time, (bb.x, bb.y, bb.width, bb.height, image)])

    def save_instances(self, list_instances: List[Instance]):
        new_instance_key = 0

        self.get_logger().info("# of instances detected in batch: {}".format(len(list_instances)))

        for instance in list_instances:
            
            if self.reduce_categories:
                new_instance_name = self.reduce_category(instance.category_name)
                new_instance = (instance.category_name, instance.position[0], instance.position[1], instance.position[2])
                self.get_logger().info("Reduced category: {}".format(instance.category_name))

            time = instance.stamp
            bb_info = (*instance.bbox, instance.image)

            # -- convert object position from local frame to map frame --
            point_map: PointStamped = self.instance_to_point_map(new_instance[1], new_instance[2], new_instance[3], time)
            
            if point_map is None: # i.e. transformation failed
                return

            # -- retrieve all objects in our database that have the same label as the currently examined observation --
            instances_matched_by_category = [item for item in self.objects_dict if instance.category_name == self.objects_dict[item][0]]
            nb_instances_matched_by_category = len(instances_matched_by_category)
            self.get_logger().info("Number of matched items {}".format(nb_instances_matched_by_category))
            # -- if no object of the same label is found in the database --
            if nb_instances_matched_by_category == 0:
                
                # Is there an object instance closer than 5cm to the new instance ?
                found_close, old_instance_key = self.found_close(list(self.objects_dict.keys()), \
                                                            point_map, self.OBJECT_NEAR_THRESHOLD, self.objects_dict)
                new_instance_key = instance.category_name+str(1)

                # -- this is a new one, add it to database -- 
                if not found_close:
                    self.objects_dict[new_instance_key] = (instance.category_name, point_map.point.x, point_map.point.y, point_map.point.z, 1, self.id)  
                    self.id += 1 

                    self.get_logger().info("Save inst. New object detected: {}. Position in map: {}".format(new_instance_key, point_map.point))
                    
                    message = String()
                    message.data = "New object detected: " + str(new_instance_key)
                    
                    self.speaker_pub.publish(message)
                    # self.save_instance_image(new_instance_key, bb_info)
                    
                # -- an object in database has been found to be close to the new one (but with different category) --
                else:
                    # Goal: keep only one in the long term memory. Keep the one with the largest number of detections
                    
                    # -- NOTE: tempt_dict stores all new observations that had no matched item in database using (1) category name (2) distance to objects --
                    
                    # -- below, we check if the new observation is a repeated observation of any of the temporary instances --
                    
                    # -- fist, filter all tempopary instances based on category name --
                    temp_instances = [item for item in self.temp_dict if instance.category_name == self.temp_dict[item][0]]
                    
                    # -- there is at least 1 matched item in the temporary instance list --
                    if len(temp_instances) > 0:
                        # -- second, check if there is a close instance in the temp memory with the same category name --
                        found_close, tmp_old_instance_key = self.found_close(temp_instances, point_map, self.OBJECT_NEAR_THRESHOLD, self.temp_dict)

                        # -- indeed this new observation is a repeated observation of a temporary instance, so we update it --
                        if found_close:
                            instance_temp = self.temp_dict[tmp_old_instance_key]
                            
                            # -- NOTE: we update both the position AND the detection counter --
                            # -- [category_name, x, y, z, nb_detections] --
                            self.temp_dict[tmp_old_instance_key] = (instance_temp[0], (point_map.point.x +float(instance_temp[1]))/2,\
                                (point_map.point.y +float(instance_temp[2]))/2, (point_map.point.z +float(instance_temp[3]))/2, int(instance_temp[4])+1) 
                            new_instance_key = tmp_old_instance_key
                    # -- there is at no matched item in the temporary instance list, so initiate a new temporary instance --
                    else:
                        self.temp_dict[new_instance_key] = (instance.category_name, point_map.point.x, point_map.point.y, point_map.point.z, 1)

                    # -- update the matched object by distance in database if the new observation has more detections than the old one --
                    # -- NOTE: self.objects_dict[old_instance_key] is the closest object in the database to the new observation (though different label) --
                    if self.objects_dict[old_instance_key][4] <= self.temp_dict[new_instance_key][4]:
                        old_id = self.objects_dict[old_instance_key][5]
                        del self.objects_dict[old_instance_key]
                        
                        # -- old object is updated here --
                        self.objects_dict[new_instance_key] = (self.temp_dict[new_instance_key][0], self.temp_dict[new_instance_key][1],self.temp_dict[new_instance_key][2], self.temp_dict[new_instance_key][3], self.temp_dict[new_instance_key][4], old_id)
                        del self.temp_dict[new_instance_key]
                        
                        self.get_logger().info("New object detected: {}. Position in map: {}".format(new_instance_key, point_map.point))

                        message = String()
                        message.data = "New object detected: " + str(new_instance_key)
                        
                        self.speaker_pub.publish(message)

                        old_instance_path = self.directory+"/"+old_instance_key+".jpg"
                        if os.path.exists(old_instance_path):
                            os.remove(old_instance_path)
            else:
                # -- associate the new observation to whatever matched one that is closest and distance smaller than threshold --
                found_close, old_instance_key = self.found_close(instances_matched_by_category, point_map, 0.475, self.objects_dict)
                # -- NOTE: we typically use a larger threshold since the same label gives us more confidence --
            
                if found_close: 
                    # -- update the matched object in database --
                    instance = self.objects_dict[old_instance_key]
                    point_map.point.x = (point_map.point.x +float(instance[1]))/2
                    point_map.point.y = (point_map.point.y +float(instance[2]))/2
                    point_map.point.z = (point_map.point.z +float(instance[3]))/2
                    self.objects_dict[old_instance_key] = (new_instance[0], point_map.point.x, \
                                point_map.point.y, point_map.point.z, int(instance[4])+1, instance[5])  
                else:
                    # -- add new instance to dict --
                    instance_key = new_instance[0]+str(nb_instances_matched_by_category+1)
                    self.objects_dict[instance_key] = (new_instance[0], point_map.point.x, point_map.point.y, point_map.point.z, 1, self.id)  
                    self.id += 1
                    
                    self.get_logger().info("New object detected: {}. Position in map: {}".format(new_instance_key, point_map.point))

                    message = String()
                    message.data = "New object detected: " + str(new_instance_key)
                    
                    self.speaker_pub.publish(message)
                    # self.save_instance_image(instance_key, bb_info)


    def found_close(self, instance_keys, point_map, threshold, dictionnary):
        # -- use the closest instance to the new one that is also smaller than the threshold as the match --
        found_close = 0
        instance_key = None
        min_dist = 1e10
        for key in instance_keys:
            instance = dictionnary[key]
            
            dist = math.sqrt((point_map.point.x - float(instance[1]))**2 + (point_map.point.y - float(instance[2]))**2 + (point_map.point.z - float(instance[3]))**2 )
            if dist < threshold: 
                found_close = 1
                
                if dist < min_dist:
                    min_dist = dist
                    instance_key = key
                
        return found_close, instance_key


    def save_instance_image(self, instance_key, bb_info):
        image = bb_info[4]
        x_min = bb_info[0]
        y_min = bb_info[1]
        width = bb_info[2]
        height = bb_info[3]
        
        start_point = (int(x_min), int(y_min))
        end_point = (int(x_min+width), int(y_min+height))
        color = (0, 0, 255)
        thickness = 2

        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            cv_image = cv2.rectangle(cv_image, start_point, end_point, color, thickness)
            cv_image = cv2.putText(cv_image, instance_key, (start_point[0]-10, start_point[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 1, color, thickness, cv2.LINE_AA)
            path = self.directory+"/"+instance_key+".jpg"
            cv2.imwrite(path, cv_image)
        except CvBridgeError as e:
            print(e)


    def publish_instances(self):

        # -- publish list of current instances on topic /detection/object_instances --
        instances_list_msg = ObjectInstanceArray()
        stamp = self.get_clock().now().to_msg()
        instances_list_msg.header.stamp = stamp
        instances_list_msg.header.frame_id = "map"
        for instance_key in self.objects_dict:
            instance = self.objects_dict[instance_key]
            if instance[4] <= self.DET_CNT_THRESHOLD:
                continue
            instance_msg = ObjectInstance()
            instance_msg.category_name = instance[0]
            instance_msg.instance_name = instance_key
            point = Point()
            point.x = float(instance[1])
            point.y = float(instance[2])
            point.z = float(instance[3])
            instance_msg.object_position = point
            instance_msg.latest_stamp = stamp
            
            instance_msg.nb_detections = min(max(int(instance[4]), 0), 255)
            
            instance_msg.id = instance[5]
            instances_list_msg.instances.append(instance_msg)

        self.instances_pub.publish(instances_list_msg)


    def instance_to_point_map(self, x, y, z, time_):

        point_map = PointStamped()
        point_map.header.frame_id = self.frame_id
        point_map.header.stamp = time_ 
        
        point_map.point = Point(x=x, y=y, z=z)

        can_transform = self.tfBuffer.can_transform("map", point_map.header.frame_id, time_)
        if not can_transform:
            self.get_logger().warn("Cannot transform from {} to map".format(point_map.header.frame_id))
            return None
        
        s_t = time.time()
        try:
            self.get_logger().info("Point before transform {}".format(point_map))
            point_map = self.tfBuffer.transform(point_map, "map", rclpy.duration.Duration(seconds=0.001))
            e_t = time.time()
            self.get_logger().info("Transform suceeded and took {} seconds".format(e_t - s_t))
            self.get_logger().info("Point after transform {}".format(point_map))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(str(e))
            e_t = time.time()
            self.get_logger().info("Transform failed and took {} seconds".format(e_t - s_t))
            return None
        
        return point_map
    
    def publish_tf(self, instance_key, point_map): 
        t = TransformStamped()
        t.header.frame_id = "map"
        t.child_frame_id = "object/detected/"+instance_key

        t.header.stamp = point_map.header.stamp 
        
        t.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        t.transform.translation = Vector3(x=point_map.point.x, y=point_map.point.y, z=point_map.point.z)
        
        self.tf_broadcaster.sendTransform(t)
        
    def make_marker_template(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = marker.scale.y = marker.scale.z = 0.085
        marker.lifetime = rclpy.time.Duration(nanoseconds=0).to_msg()
        
        marker.action = Marker.ADD
        marker.type = Marker.CUBE
        
        marker.color.a = 1.0
        marker.color.g = marker.color.r = 0.5
        marker.color.b = 1.0
        
        self.marker_template = marker
        
    def make_marker(self, point_map, stamp, id, is_new=True):
        marker = deepcopy(self.marker_template)
        
        marker.header.stamp = stamp
        
        marker.pose.position.x = point_map.point.x
        marker.pose.position.y = point_map.point.y
        marker.pose.position.z = 0.0
        marker.id = id+1
        
        marker.color.r += (np.random.rand()-0.5) * 0.2
        marker.color.g += (np.random.rand()-0.5) * 0.1
        
        return marker
        
    def bounding_boxes_callback(self, msg):
        self.bounding_boxes_buffer.append(msg)

    def image_callback(self, msg):
        self.image_buffer.append(msg)

    def remove_instance_callback(self, msg):
        instance_key = msg.data
        # delete instance from dict
        try:
            del self.objects_dict[instance_key]
        except KeyError as e:
            self.get_logger().warn(e)

    def reduce_category(self, new_instance):
        if new_instance in self.mapping_animals:
            new_instance = "animal"
        elif new_instance in self.mapping_cubes:
            new_instance = "cube"
        elif new_instance in self.mapping_spheres:
            new_instance = "sphere"
        return new_instance
    
    def main_loop(self):

        current_time = self.get_clock().now()
        time_in_sec = current_time.nanoseconds / 1e9
        start_time = time_in_sec - self.LOOK_BACK_DURATION # using msg received a while ago , change this to the time you want to use!

        latest_msgs = []
        for msg in self.bounding_boxes_buffer:
            # -- check recent data only, for efficiency --
            if msg.header.stamp.sec >= start_time:
                latest_msgs.append(msg)
        
        batch = latest_msgs
        self.get_logger().info("Length of batch to process: {}".format(len(batch)))
        
        self.bounding_boxes_buffer = latest_msgs
        
        self.publish_instances()
        self.get_logger().info(str(self.objects_dict))
        markers = MarkerArray()
        for keys in self.objects_dict:
            instance = self.objects_dict[keys]
            if instance[4] <= self.DET_CNT_THRESHOLD:
                continue
            point_map = PointStamped()
            point_map.header.frame_id = 'map'
            point_map.header.stamp = self.get_clock().now().to_msg()
            point_map.point.x = instance[1]
            point_map.point.y = instance[2]
            point_map.point.z = instance[3]
            self.publish_tf(keys, point_map)
            markers.markers.append(self.make_marker(point_map, current_time.to_msg(), instance[5]))
            
        self.markers_pub.publish(markers)
        
        if len(batch) == 0:
            return
        
        s_t = time.time()
        self.filter(batch, current_time - rclpy.duration.Duration(seconds=1))
        e_t = time.time()
        self.get_logger().info("Filter function took {:.5f} seconds to execute".format(e_t - s_t))
        

def main():
    rclpy.init()
    node = Object_postprocessor()
    try:
        while rclpy.ok():
            node.main_loop()
            rclpy.spin_once(node, timeout_sec=0.05)

    except KeyboardInterrupt:
        pass

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
