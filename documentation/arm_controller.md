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

## 3. Launch Files
#### 3.1


## 4.Examples
```bash
ros2 run arm_controller open_loop_controller
```