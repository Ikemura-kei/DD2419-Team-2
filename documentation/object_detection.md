# Package: object_detection

## 1. Features
This is a package provides functionalities to detect various objects appearing in the DD2419 course project task, including boxes, cubes, dolls, etc.

## 2. Nodes

#### 2.1 `display_markers`
This node receives the detected ArUco markers and publish a visualization marker for each detected ArUco.

###### Published Topics
- `/box_markers`: `<visualization_msgs/msg/MarkerArray>`, the visualization markers of the detected ArUco's mounted on boxes. The color of the markers is gray.

###### Subscribed Topics
- `/aruco/markers`: `<aruco_msgs/msg/MarkerArray>`, the detected ArUcos, published by the aruco detection package.

#### 2.2 `object_detection`
This node filter the object's point clouds, culculate the the centriod of detected object and publish it.

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
```bash
ros2 run object_detection display_markers
```