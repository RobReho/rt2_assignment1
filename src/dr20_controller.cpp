#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"


ros::Publisher pub_left;
ros::Publisher pub_right;

float wheelDiameter=0.195;
float interWheelDistance=0.331;


void publishVelocity(const geometry_msgs::Twist::ConstPtr& msg)
{

	std_msgs::Float32 vel_l;
	std_msgs::Float32 vel_r;
	
	vel_r.data = ( ((2 * msg->linear.x - msg->angular.z * interWheelDistance) / (wheelDiameter)));
    vel_l.data = ( ((2 * msg->linear.x + msg->angular.z * interWheelDistance) / (wheelDiameter)));
    
    //vel_r.data = (8 * msg->linear.x - msg->angular.z * 0.8) ;
    //vel_l.data = (8 * msg->linear.x + msg->angular.z * 0.8) ;

	pub_left.publish(vel_l);
	pub_right.publish(vel_r);
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "dr20_controller");
   ros::NodeHandle n;
   
   pub_left = n.advertise<std_msgs::Float32>("/leftwheel_vel", 1000);
   pub_right = n.advertise<std_msgs::Float32>("/rightwheel_vel", 1000);
   ros::Subscriber sub_vel = n.subscribe("/cmd_vel", 1000, publishVelocity);
   
   ros::spin();
   
   return 0;
}
