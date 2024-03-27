# Package: arm_controller

## 1. Features
This package contains several ways of controlling robotic arm.

## 2. Nodes

#### 2.1 `joystick_controller`
This node controls the arm using the joystick. The RB must be held down in order to start controlling the arm.

###### Published Topics
- `/multi_servo_cmd_sub`: `<std_msgs/msg/Int16MultiArray>`, enocder position of the joints along with the duration to achieve desired position.

###### Subscribed Topics
- `/servo_pos_publisher`: `<sensor_msgs/msg/JointState>`, encoder readings of joint positions.

- `/joy`: `<sensor_msgs/msg/Joy>`, buttons pressed from joystick.

#### 2.2 `open_loop_controller`
This node picks and places object using a fixed predefined position. Joystick must be used to start sequences. A to initiate pickup. X to initiate place. B to reset.

###### Published Topics
- `/multi_servo_cmd_sub`: `<std_msgs/msg/Int16MultiArray>`, enocder position of the joints along with the duration to achieve desired position.

###### Subscribed Topics
- `/servo_pos_publisher`: `<sensor_msgs/msg/JointState>`, encoder readings of joint positions.

- `/joy`: `<sensor_msgs/msg/Joy>`, buttons pressed from joystick.

#### 2.3 `inverse_kinematics`
This node picks object from requested point and places/drops object. Quasi inverse kinematics (iterative inverse kinematic approximation) used to find joint position to reach desired point. Topics are currently used to interface with the node. Pressing B on joystick resets the arm.

###### Published Topics
- `/ik_res`: `<std_msgs/msg/Bool>`, result from pick-up request. True if node believes it has picked up object. Otherwise False

- `/drop_obj_res`: `<std_msgs/msg/Bool>`, result from drop request. True if drop completes. Otherwise False

- `/multi_servo_cmd_sub`: `<std_msgs/msg/Int16MultiArray>`, enocder position of the joints along with the duration to achieve desired position.

###### Subscribed Topics
- `/pick_ik`: `<geometry_msgs/msg/PointStamped>`, request point stamped to pick. Need to specify the frame in the PointStamped along with message stamp. 

- `/drop_obj`: `<std_msgs/msg/Bool>`, request to drop object

- `/servo_pos_publisher`: `<sensor_msgs/msg/JointState>`, encoder readings of joint positions.

- `/multi_servo_cmd_sub`: `<std_msgs/msg/Int16MultiArray>`, enocder position of the joints along with the duration to achieve desired position.

- `/joy`: `<sensor_msgs/msg/Joy>`, buttons pressed from joystick.

## 3. Launch Files
#### 3.1 `robo_arm_ik_launch.py`
This script launches the micro ros agent to move the arm and creates a static transform between `arm_base` and `base_link`.

###### Note
Additional nodes and launch files must be run for the `inverse_kinemaitcs` node to run correctly (i.e. Joystick node, realsense camera). This should be handled in brain node/launch. 


## 4.Examples
```bash
ros2 run arm_controller open_loop_controller
```