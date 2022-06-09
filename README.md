# First Assignment of the Research Track 2 course (Robotics Engineering / JEMARO, Unige)
Roberta Reho - 5075214

The ROS2 package contains the **state_machine** and **position_service** cpp nodes for controlling a mobile robot in the Gazebo simulation environment. To run the whole simulation it is necessary to use the **ros1_bridge** to be able to communicate with the ROS nodes **go_to_point** and **user_interface**.

## Package content
The two ROS2 nodes **state_machine** and **position_service** are developed using classes, so that they can be used as components, as the bridge only supports composable nodes.

The file: **my_mapping_rules.yaml** contains the name of the package in ROS and ROS2 that need to be bridged.

The launch file ```/launch/launch.py``` runs the cpp nodes as a container.

## Running
To run the whole simulation is necessary to operate in both ROS and ROS2 workspaces.
Please follow this step below to run the code both in ROS and ROS2:
The ROS package used is available in the ```main``` branch.

1. Open a terminal and **source ros**
```
#!/bin/bash
source /root/<ros_workspace>/devel/setup.bash
```
2. In that terminal build the workspace and run the lounch file:
```
roslaunch rt2_assignment1 sim_bridge.launch
```
3. Open one more terminal and **source ros12** 
```
#!/bin/bash
source /root/my_ros/devel/setup.bash
source /root/my_ros2/install/setup.bash
```
4. In that terminal build the bridge:
```
ros2 run ros1_bridge dynamic_bridge 
```
5. Open one more terminal and **source ros2**
```
#!/bin/bash
source /root/my_ros2/install/setup.bash
```
6. In this last terminal launch the ROS2 container:
```
ros2 run rt2_assignment1 launch.py
```

## Documentation
Further documentation is available in the **docs** folder.
