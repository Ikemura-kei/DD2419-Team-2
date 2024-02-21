# Package: mapping

## 1. Features
This is a package that provides the mapping functionalities.

## 2. Nodes
For now, we only implement a naive mapping node that maps LiDAR readings directly to the map (with some basic outlier rejection).

#### 2.1 `naive_mapping_node`
This node performs mapping in a most straigh-forward way, that is, map LiDAR scans to `odom` frame and put that point into the corresponding cell in the map.

###### Published Topics
- `/candidate_map`: `<nav_msgs/msg/OccupancyGrid>`, the intermediate map used to filter outliers, published for debugging purposes.
- `/global_map`: `<nav_msgs/msg/OccupancyGrid>`, the map.

###### Subscribed Topics
- `/scan`: `<sensor_msgs/msg/LaserScan>`, the laser scan from the LiDAR.

## 3. Launch Files
#### 3.1 `naive_mapping_launch`
This script launches the `naive_mapping_node`, nothing else, provided just for convinience.

## 4.Examples
```bash
ros2 launch mapping naive_mapping_node
```