#! /usr/bin/env python

## @package rt2_assignment1
# \file user_interface.py
# \brief Node implementing the user interface
# \author Carmine Recchiuto
# \version 0.1
# \date 06/04/2022
#
# \details
#
# ServiceClient:<BR>
#   /user_interface (rt2_assignment1.srv.Command)
#
# Description:
#
# This node receives inputs from the terminal.
# If the input received is "1" it sends the Command 
# "start" to the server /user_interface.
# Otherwise, if "0" is received the Command "stop"
# is sent the same way.
#
##

import rospy
import time
from rt2_assignment1.srv import Command

def main():
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(10)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot\n "))
    while not rospy.is_shutdown():
        if (x == 1):
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot\n "))
        else:
            print("Cancelling goal...\n")
            ui_client("stop")
            x = int(input("\nPress 1 to start the robot\n "))
            
if __name__ == '__main__':
    main()
