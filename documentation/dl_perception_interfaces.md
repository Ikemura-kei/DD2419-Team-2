# Package: dl_perception_interface

## 1. Features
This is a CMake packages where defines the interfaces of deep learning-based detector needed in perception part.

## 2. Custom Message Structures
There are total four custom message types define for dl detector.

---------------------------------------------------------------------------------------------------------------
#### 2.1 `<dl_perception_interfaces/msg/ObjectInstanceArray>`
This is the message provided to other modules in the system by dl detector.
It conveys list of ObjectInstance message.
```
std_msgs/Header header
dl_perception_interfaces/ObjectInstance[] instances
```

#### 2.2 `<dl_perception_interfaces/msg/ObjectInstance>`
The message type conveys info about single detected object instance.
```
string instance_name
string category_name
geometry_msgs/Point object_position
builtin_interfaces/Time latest_stamp
uint8 nb_detections
uint8 id
```
--------------------------------------------------------------------------------------------------------------
#### 2.3 `<dl_perception_interfaces/msg/BoundingBoxArray>`
This message type used inside of dl detector provides list of bounding boxs messages.
```
std_msgs/Header header
dl_perception_interfaces/BoundingBox[] bounding_boxes
```

#### 2.4 `<dl_perception_interfaces/msg/BoundingBox>`
This message type used inside of dl detector to convey info about single bounding box.
```
builtin_interfaces/Time stamp
float32 x
float32 y
float32 height
float32 width
uint8 category_id
string category_name
geometry_msgs/Point bb_center
```

