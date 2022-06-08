#! /usr/bin/env python
"""
.. module:: go_to_point
    :platform: Unix
    :synopsis: Node implementing the go_to_point behavior
.. moduleauthor:: Carmine Recchiuto <carmine.recchiuto@dibris.unige.it>, Roberta Reho <robertareho@gmail.com>

Publishes to:
    /cmd_vel (geometry_msgs.msg.Twist)

ServiceServer:
    /odom (nav_msgs.msg.Odom)

ActionServer:
    /go_to_point (rt2_assignment1.msg.PoseAction)

Description:

 This node controls the go_to_point behavior of
 the robot using an action server.
 Whenever a goal is received, a state machine
 handles the behaviour: <BR>

   1. align with the goal position <BR>
   2. go straight to the goal position <BR>
   3. align with the goal orientation <BR>
   4. goal pose reached <BR>

"""
import rospy
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from tf import transformations
from rt2_assignment1.srv import Position
import math
import actionlib
import actionlib.msg
import rt2_assignment1.msg

# robot state variables
position_ = Point()
pose_ = Pose()
yaw_ = 0
position_ = 0
state_ = 0
pub_ = None # publisher
state_ = 0
desired_position_ = Point()

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

# action_server
act_s = None

def clbk_odom(msg):
    """
    Odometry callback

    Retrieves the position and orientation quaternion 
    from the Odom message.

    Args:
      msg (Odometry): odometry message.
    """
    global position_
    global yaw_
    global pose_
    # position
    position_ = msg.pose.pose.position
    pose_ = msg.pose.pose
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
    """
    Updates the current global state

    Args:
      state (int):  new state
    """
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    """
    Normalizes an angle berween [-pi, pi]

    Args:
      angle (float):  input angle
      
    Returns:
      angle (float):  normalized angle
    """
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    """
    Orients the robot towards the desired orientation

    The function is used to orient
    the robot toward the goal (x,y) position. 
    It changes to a new state, depending on the current
    one.

    Args:
      des_yaw (float):  desired yaw
      next_state (int): next state to set
    """
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):
    """
    Goes straight towards the goal

    Set the linear and angular speed
    according to the distance to the goal pose.

    Args:
      des_pos (Point):  desired (x, y) position
    """
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub_.publish(twist_msg)
    else: # state change conditions
        #print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

def fix_final_yaw(des_yaw):
    """
    Orients the robot towards the desired orientation

    The function is used to orient
    the robot to achieve the goal orientation. 
    It changes to a new state, depending on the current
    one.

    Args:
      des_yaw (float):  desired yaw
      next_state (int): next state to set
    """
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)
        
def done():
    """
    Stops the robot

    Sets the robot linear and angular 
    velocity to 0.0
    """
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)
    
def planning(goal):
    """
    State machine 

    Sets an appropriate behaviour depending
    on the current robot state, in order
    to reach the goal.
    The state machine keeps running until
    the goal is reached or the action is
    preempted (the goal gets cancelled).

    Args:
      goal (PoseActionGoal): (x,y,theta) goal pose
    """
    global state_, desired_position_
    global act_s

    desired_position_.x = goal.target_pose.pose.position.x
    desired_position_.y = goal.target_pose.pose.position.y

    state_ = 0
    rate = rospy.Rate(20)
    feedback = rt2_assignment1.msg.PlanningFeedback()
    result = rt2_assignment1.msg.PlanningResult()
    
    while not rospy.is_shutdown():
        if act_s.is_preempt_requested():
            rospy.loginfo('Goal was preempted')
            act_s.set_preempted()
            done()
            success = False
            break
        elif state_ == 0:
            feedback.stat = "Fixing the yaw"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            fix_yaw(desired_position_)
        elif state_ == 1:
            feedback.stat = "Angle aligned"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            go_straight_ahead(desired_position_)

        elif state_ == 2:
            feedback.stat = "Target reached!"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            done()
            result.result = True
            success = True
            break
        else:
            rospy.logerr('Unknown state!')

        rate.sleep()
    if success:
       rospy.loginfo('Goal: Succeeded!')
       act_s.set_succeeded(result)


def main():
    global pub_, act_s
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    #service = rospy.Service('/go_to_point', Position, go_to_point)
    #rospy.spin()
    act_s = actionlib.SimpleActionServer(
        '/reaching_goal', rt2_assignment1.msg.PlanningAction, planning, auto_start=False)
    act_s.start()

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
