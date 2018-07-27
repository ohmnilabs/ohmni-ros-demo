#!/usr/bin/env python

#
# Simple teleop controller based on the
# turtlebot_teleop from ROS
# https://github.com/turtlebot/turtlebot/blob/kinetic/turtlebot_teleop/scripts/turtlebot_teleop_key
#
# Hoang
#

# Importing necessary libraries
import rospy
import sys, select, termios, tty

# User prompt message
msg = """
Control Your Turtlebot!
---------------------------
Moving around:
        w    
   a    s    d
r/f : look up and down
CTRL-C to quit
"""

# List of valid inputs
validInput = ['a', 'w', 'd', 's', 'r', 'f'];

# Getting stream of user input from the keyboard
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Main function here
if __name__ == "__main__":

    # not even sure what is the point of this...
    settings = termios.tcgetattr(sys.stdin)

    # Declare node name and topic to publish to
    rospy.init_node('ohmni_teleop')
    #pub = rospy.Publisher('some_topic', std_msgs.msg.String, queue_size=5)
    
    # Prompting user input
    print(msg)
    
    # Main loop here
    while(1):
        key = getKey()
        if (key == '\x03'):
            break
        elif (key in validInput):
            print("You input key is", key)
        elif (key == ''):
            pass
        else:
            print("Invalid input")

# Some mysteries remain here... I guess it has something to do with the stream input
termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)










