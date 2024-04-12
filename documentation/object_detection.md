# Package: object_detection

## 1. Features
This is a package provides Deep learning-based and point cloud-based functionalities to detect, classify and manage various categories of object instances (balls, cubes, dolls, etc.) appearing in the DD2419 course project task,

## 2. Nodes

### 2.1 `display_markers`
This node receives the detected ArUco markers and publish a visualization marker for each detected ArUco.

###### Published Topics
- `/box_markers`: `<visualization_msgs/msg/MarkerArray>`, the visualization markers of the detected ArUco's mounted on boxes. The color of the markers is gray.

###### Subscribed Topics
- `/aruco/markers`: `<aruco_msgs/msg/MarkerArray>`, the detected ArUcos, published by the aruco detection package.

### 2.2 `dl_object_classifier` 

This node implements objects classification in RGB images and publishing bounding box information.

###### Published Topics

- `/detection/bounding_boxes`:`<dl_perception_interfaces/msg/BoundingBoxArray>` Detected bounding boxes.
- `/detection/image_with_bounding_boxes`: `<sensor_msgs/msg/Image>` RGB image with bounding boxes visualized.

###### Subscribed Topics

- `/camera/color/image_raw`:`<sensor_msgs/msg/Image>` RGB image topic.
- `/camera/depth/image_rect_raw`:`<sensor_msgs/msg/Image>` Depth image topic.
- `/camera/color/camera_info`:`<sensor_msgs/msg/CameraInfo>` Camera information topic.

###### Notes

- This node uses a pre-trained deep learning model for object detection.
- Bounding boxes are visualized on the RGB image and published as a separate topic.

### 2.3 `dl_object_postprocessor`

This node processes the generated bounding boxes (already containing 3D information) by classifier, performs clustering, and publish a list of detected object instances. It also respensible for managing seen object instances and broadcast instance TF under the map frame.

#### Published Topics

- `/detection/object_instances`: `<dl_perception_interfaces/msg/ObjectInstanceArray>` List of detected object instances.

#### Subscribed Topics

- `detection/bounding_boxes`: `<dl_perception_interfaces/msg/BoundingBoxArray>` Bounding box information of detected objects.
- `/camera/color/image_raw`: `<sensor_msgs/msg/Image>` RGB image from the camera.
- `/detection/remove_instance`: `<std_msgs/msg/String>` Request to remove a specific object instance.

#### Notes

- This node clusters detected instance's position in camera frame, and utilizes the averaged position to derive their 3D positions using map-camera_link transform.
- Object instances are managed based on their categories and positions in the environment.
- Instances are associated with unique keys for identification and tracking.
- Auto saving snapshots of all seen instances.



### 2.4 `dl_detection`
It is a baseline detector model inspired by the "You Only Look Once" (YOLO) approach for real-time object detection, using a MobileNetV2 architecture as a backbone for feature extraction and a 1x1 Conv head for bounding box prediction.

###### Published and Subscribed Topics
- `None`

###### Notes

- It includes image augmentation techniques such as Gaussian noise, motion blur, and affine transformations.
- The model predicts bounding boxes in relative coordinates and confidence scores.
- Non-maximum suppression (NMS) is applied to filter overlapping bounding boxes.



### 2.5 `object_detection` （nearly deprecated）
This node is the  point cloud-based object detector which only performs well for cubes.
It filter the object's point clouds by color, culculate the the centriod of detected object and then publish it. 

###### Published Topics
- `/object_centers`: `<visualization_msgs/msg/MarkerArray>`, the markers of the detected objects convey their position infomation. 
- `/camera/depth/color/ds_points`: `<sensor_msgs/msg/PointCloud2>`, the filtered point clouds which is used to visualize objects in RViz.

###### Subscribed Topics
- `/camera/depth/color/points`: `<sensor_msgs/msg/PointCloud2>`, the original point clouds detected by RGBD camera.


## 3. Launch Files
#### 3.1 `aruco_detector_launch.py`
This script launches the ArUco marker detection (on the boxes) using Realsense D435-i, together with the `display_markers` node to publish markers of the detected ArUco's.

#### 3.2 `aruco_detector_arm_camera_launch.py`
This script launches the ArUco marker detection (on the boxes) using the USB camera mounted on the robotic arm.

## 4.Examples
Ex1. Run deep learning based object detector: Two nodes inloved only
```bash
ros2 run object_detection dl_object_classifier
ros2 run object_detection dl_object_postprocessor
```
Ex2. Run point cloud based object detector: One nodes inloved only
```bash
ros2 run object_detection object_detection
```
Ex2. Run display_markers: 

```bash
ros2 run object_detection display_markers
```
