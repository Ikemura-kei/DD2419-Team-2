# Package: controller

## 1. Features
This package provides controllers for controlling the motion of a robotic system. It includes functionalities for cartesian control, target position control, and open-loop control.

## 2. Nodes
This package contains the following nodes:

#### 2.1 `cartesian_controller`
This node implements a controller for cartesian motion control. It takes desired linear and angular velocity as input and generates control commands to achieve them.

#### 2.2 `target_position_controller`
This node implements a controller for target position control. It takes a desired target point as input and generates linear and angular velocity to send to the cartesian controller to move the robot to this point.

###### Published Topics

- `/simulation/cmd_vel`: `<geometry_msgs/msg/Twist>`, equivalent of /motor_controller/twist for the simulator.
- `/task_completion`: `<String>`, send 'Task completed' when the robots has reached the target position.

###### Subscribed Topics

- `/target_position`: `<geometry_msgs/msg/Point>`, the target position.

#### 2.3 `open_loop_controller`
This node implements an open-loop controller. It applies control commands from the controller to the robot without feedback from sensors.

## 4. Examples
```bash
ros2 run controller cartesian_controller
ros2 run controller target_position_controller
```
You should ensure that your entire system is running to allow the target_position_controller to receive a point from the state machine.