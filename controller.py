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
from std_msgs.msg import String

# User prompt message
msg = """
Control Ohmni!
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

    # Some sort of necessary declaration for input stream
    settings = termios.tcgetattr(sys.stdin)

    # Declare node name and topic to publish to
    rospy.init_node('ohmni_teleop')
    pub = rospy.Publisher('ohmni_teleop_controller', String, queue_size=5)
    
    # Prompting user input
    print(msg)
    
    # Main loop here
    while(1):
        key = getKey()
        if (key == '\x03'):
            break
        elif (key in validInput):
            #print("You input key is", key)
            pub.publish(key)
        elif (key == ''):
            pass
        #else:
        #    print("Invalid input")

# Again, probably a wrapper of the settings above
termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)










