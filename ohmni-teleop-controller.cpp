// 
// Basic ROS node to take user input and
// publish to teleop node
// 
// Hoang
// 

// Include some basic ROS libraries here
#include "ros/ros.h"
#include "std_msgs/Char.h"

// Main function, note here the argc and argv are not 
// hard coded so make sure the environment variables are
// setup correctly else the node will fail
int main (int argc, char **argv) {

    // First thing first, declare the node and node handle
    ros::init(argc, argv, "ohmni-teleop/controller");
    ros::NodeHandle n;

    // Now we want to publish our messages to our robot
    ros::Publisher input_pub = n.advertise<std_msgs::Char>("ohmni-bot", 1000);
}