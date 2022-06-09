# First Assignment of the Research Track 2 course (Robotics Engineering / JEMARO, Unige)
Roberta Reho - 5075214

The package contains the nodes and the simulation environment for controlling a mobile robot in the Gazebo simulation environment.
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

The code can be run through the provided launch file **sim.launch** that will launch all the nodes and the Gazebo environment.

```bash
 roslaunch rt2_assignment1 sim.launch
```
 https://robreho.github.io/rt2_assignment1/

