# Package: bringup

## 1. Features
This is a package responsible for all of the instantiation of various modules that collectively serve the integrated system.

## 2. Nodes
This package does not implement any specific node.

## 3. Launch Files
#### 2.1 `detection_launch.py`
This script brings up all of the nodes relevant to object detection, including box detection, cube detection, doll detection, etc.  
#### 2.2 `sensors_launch.py`
This script brings up all sensors. Now we include the following sensors:
* Realsense D435-i
* SLAMTECH LiDAR A1
* An USB camera mounted on the arm

#### 2.3 `chassis_launch.py`
This script launches the chassis controller as well as the odometry nodes.

#### 2.4 `robo_arm_launch.py`
This script launches the nodes needed to control the arm, including the joy node that receives command from the game pad.

## 4.Examples
One should only use the launch files inside this package. As it is pretty trivial to run launch files, and the purpose of each launch file is documented above, no examples will be written here.