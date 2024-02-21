# Package: motion_planning

## 1. Features
This package contains all the motion planning nodes.

## 2. Nodes

#### 2.1 `path_planner`

This path planner receives the target point and calculates the shortest path from the robot's position to this point, navigating around obstacles. It then publish this computed path.

###### Published Topics

- `/clicked_point`: `<geometry_msgs/msg/PoseStamped>`, target point for vizualisation.
- `/planned_poses`: `<geometry_msgs/msg/PoseArray>`, planned poses (filtered computed path, containing only points where de direction change).
- `/path`: `<nav_msgs/msg/Path>`, computed path for vizualisation.

###### Subscribed Topics

- `/map`: `<nav_msgs/msg/OccupancyGrid>`, the current map.
- `/plan_goal`: `<geometry_msgs/msg/Point>`, the target position.

## 3. Launch Files
#### 3.1


## 4.Examples
```bash
ros2 run motion_planning path_planner
```