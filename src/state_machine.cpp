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
   //ros::ServiceClient client_p = n.serviceClient<rt2_assignment1::Position>("/go_to_point");
   actionlib::SimpleActionClient<rt2_assignment1::PlanningAction> ac("/reaching_goal", true);
   
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   rt2_assignment1::PlanningGoal goal;

   
   while(ros::ok()){
       	ros::spinOnce();
       	if (start){
       	    start = false;
       		client_rp.call(rp);
       		goal.target_pose.header.frame_id = "base_link";
      		goal.target_pose.header.stamp = ros::Time::now();
       		goal.target_pose.pose.position.x = rp.response.x;
       		goal.target_pose.pose.position.y = rp.response.y;
       		goal.target_pose.pose.orientation.w  = rp.response.theta;
       		std::cout << "\nGoing to the position: x= " << goal.target_pose.pose.position.x << " y= " << goal.target_pose.pose.position.y << " theta = " << goal.target_pose.pose.orientation.w << std::endl;
       		ac.sendGoal(goal);
       		goalRunning = true;
       	}
       	else{
       	    if(goalRunning){
           	    rt2_assignment1::PlanningResultConstPtr Result;
          		Result = ac.getResult();
          		result = (bool) Result->result;
          		if (result) {
          		    std::cout << "Position reached" << std::endl;
          		    start = true;
          		}
       	    }
       	    
      		if (cancel){
      		    ac.cancelGoal();
      		    std::cout << "Goal has been canceled" << std::endl;
      		    cancel = false;
      		}
       	}
   	}
   
   return 0;
}
