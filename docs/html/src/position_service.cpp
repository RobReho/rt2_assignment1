/**
* \file position_service.cpp
* \brief Node that generates 
*	a random position (x, y, \theta).
* \author Carmine Recchiuto, Roberta Reho
* \version 1.0
* \date 06/04/2022
*
* \details
*
* ServiceServer:<BR>
*   /position_server (rt2_assignment1::RandomPosition)
*
* Description:
*
* This node replies to a request for a random
* pose (x, y, \theta) with a random pose within
* the limits passed in the request.

*/
#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

/**
 *  Generates a random number between M and N
 *  \param M (double): Lower bound;
 *  \param N (double): Upper bound;
 *  
 *  \return The random number
 */
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


/**
 * Service callback generating a random
 * (x,y,theta) pose
 *
 * \param req (rt2_assignment1::RandomPosition::Request &):
 *   Service request, containing the (x,y) ranges.
 * \param res (rt2_assignment1::RandomPosition::Response):
 *   Service response, containing (x,y,theta).
 *
 */
bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);   /// Server generating a random position
   ros::spin();

   return 0;
}
