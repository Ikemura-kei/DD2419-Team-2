# Package: package_name

## 1. Features
This is a package contains the nodes controlling the behavior and logic flow that allows accomplishing subset of or all tasks.

## 2. Nodes

#### 2.1 `bt_v1_node`
This node contains the implementation of version 1 of the final bevavior tree design. By design this behavior tree should allow us to finish the whole task required.

###### Published Topics
- `/pick_point`: `<geometry_msgs.msg.PointStamped>`, is the point of picking in the `base_link` frame, intended to be sent to the IK node to activate a pick action.
- `/drop_obj`: `<std_msgs.msg.Bool>`, indicating if we want to place the object currently at hand or not. `True` to activate place action and `False` otherwise.

###### Subscribed Topics
- `/object_list`: `<dd2419_interfaces.msg.ObjectList>`, is the list of unique objects detected, should be coming from the perception module.
- `/box_list`: `<dd2419_interfaces.msg.ObjectList>`, is the list of unique boxes detected, should be coming from the perception module.
- `/is_pick_done`: `<std_msgs.msg.Bool>`, indicating if the current pick action requested has been successfully executed or not yet, should be coming from the IK node.

<!-- #### 2.2 `node-2`
This node ...

###### Published Topics
- `topic_name`: `<type>`, description.

###### Subscribed Topics
- `topic_name`: `<type>`, description. -->

## 3. Launch Files
<!-- #### 3.1 `launch-1`
This script ...
#### 3.2 `launch-2`
This script ... -->

## 4.Examples
```bash
ros2 run decision bt_v1_node
```