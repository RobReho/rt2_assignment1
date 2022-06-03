# First Assignment of the Research Track 2 course (Robotics Engineering / JEMARO, Unige)
Roberta Reho - 5075214

The package contains the nodes and the simulation environment for controlling a mobile robot in the Gazebo simulation environment or, arternatively, a pioneer robot in Coppeliasim.
The mobile robot behaves as follows:
1. a random goal [x,y,theta] is published upon request from the UI
2. the robot orients itself towards the [x,y] destination;
3. it then drives straight to that position adjusting the orientation;
4. having reached the [x,y] goal position the robot turns in place in order to match the goal _theta_;
5. once the goal is reached the cycles repets itself from step 1, unless the there has been a canceling request from the UI;

The goal is implemented as an action, so it can be preempted at any time through the UI. The robot may restart when new goal is issued.

## Nodes Explanation

Two nodes are implemented as python scripts
- **go_to_point.py**: the action server managing the robot speed control depending on the goal received.
- **user_interface.py**:  the simple command line user interface, which sends the requests to start/stop the go_to_point behaviour.

Other two are C++ scripts
- **position_service.cpp**: the server generating a random pose [x,y,theta] as a response to a request.
- **state_machine.cpp**:  the FSM managing the request of a new goal pose when needed, sending it as a goal to 'go_to_point' action server.


## Running 
The package is provided with multiple launch files:
- **sim.launch**: as in the original package, the file will launch all the nodes and the Gazebo environment.

```bash
 roslaunch rt2_assignment1 sim.launch
```

- **sim_coppelia.launch**: the file will launch all the ROS nodes but not the Gazebo environment, as this file is ment to start the  simulation in a Coppelia environment. The Coppelia environment provided by the file ``/rt2_assignment1/coppelia_scene.ttt`` shall be open and run aside. The version of the used software is Coppelia EDU, available at the link http://www.coppeliarobotics.com/downloads.html

```bash
 roslaunch rt2_assignment1 sim_coppelia.launch
```
- **sim_bridge.launch**: the file will only launch the python nodes and the Gazebo environment, as the cpp nodes will be run and bridged in ROS2.

```bash
 roslaunch rt2_assignment1 sim_bridge.launch
```
For further indication on how to run this last configuration, look at the documentation on the ``ros2`` branch.