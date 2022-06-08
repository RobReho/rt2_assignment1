/**
* \file state_machine.cpp
* \brief User interface server and FSM
* \author Carmine Recchiuto, Roberta Reho
* \version 1.0
* \date 06/04/2022
*
* \details
*
* ServiceServer:<BR>
*   /user_interface (rt2_assignment1::Command)
*
* ServiceClient:<BR>
*   /position_server (rt2_assignment1::RandomPosition)
*
* ActionClient:<BR>
*   /reaching_goal (rt2_assignment1::PLanningAction)
*
* Description:
*
* This node communicates with the user interface.
* If the "start" command is issued it calls for a random 
* position to send to the /reaching_goal node. 
* In the same way it can cancel a running goal
* if a request for "stop" is received from the
* user_interface.

*/
#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/Position.h"
#include "rt2_assignment1/RandomPosition.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rt2_assignment1/PlanningAction.h>

bool start = false;         
bool cancel = false;
bool goalRunning = false;
bool result = false;


/**
* Service callback setting the start/stop
* robot state
*
* \param req (rt2_assignment1::Command::Request &):
*   Service request, containing the command (string).
* \param res (rt2_assignment1::Command::Response &):
*   Service response, the value of the 'start' state (bool).
*
*/ 
bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else if (req.command == "stop"){
        cancel = true;
    }
    else {
    	start = false;

    }
    return true;
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   actionlib::SimpleActionClient<rt2_assignment1::PlanningAction> ac("/reaching_goal", true);
   
   /// Limits for the random position request
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   rt2_assignment1::PlanningGoal goal;

   
   while(ros::ok()){
       	ros::spinOnce();
       	if (start){     /// if the commandto start has been received from the UI
       	    start = false;
       		client_rp.call(rp);     /// call for a random position
       		goal.target_pose.header.frame_id = "base_link";
      		goal.target_pose.header.stamp = ros::Time::now();
       		goal.target_pose.pose.position.x = rp.response.x;
       		goal.target_pose.pose.position.y = rp.response.y;
       		goal.target_pose.pose.orientation.w  = rp.response.theta;
       		std::cout << "\nGoing to the position: x= " << goal.target_pose.pose.position.x << " y= " << goal.target_pose.pose.position.y << " theta = \n" << goal.target_pose.pose.orientation.w << std::endl;
       		ac.sendGoal(goal);      /// set the goal of the action
       		goalRunning = true;     /// raise the goal running flag
       	}
       	else{
       	    if(goalRunning){    /// if the goal is running call for the result message 
           	    rt2_assignment1::PlanningResultConstPtr Result;
          		Result = ac.getResult();
          		result = (bool) Result->result;
          		if (result) {   /// if the returned value is true then the goal has been reached
          		    std::cout << "Position reached\n" << std::endl;
          		    start = true;
          		}
       	    }
       	    
      		if (cancel){        /// if the cancel flag is up then the goal needs to be canceled
      		    ac.cancelGoal();
      		    std::cout << "Goal has been canceled\n" << std::endl;
      		    cancel = false;
      		}
       	}
   	}
   
   return 0;
}
