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

    // User input prompt
    std::cout << """
        Control Ohmni!
        ---------------------------
        Moving around:
                w    
           a    s    d
        r and f : looking up and down with neck
        CTRL-C to quit
    """
    // Setting up publishing topic to Ohmni
    // ros::Publisher input_pub = n.advertise<std_msgs::Char>("ohmni-bot", 1000);

    // Storage space for user input key
    std::char userKey;

    while(1) {
        // Getting user input
        std::getline(std::cin, userKey);
        std::cout << "Your input key is " << userKey << " \n";
    }



    // No errors
    return 0;
}