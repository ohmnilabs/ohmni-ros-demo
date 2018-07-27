
//
// Basic subscriber node that will get data
// from controller topic, perform conversion
// and parse over to telebot socket
//
// Hoang
//

// Import some basic libraries
#include "ros/ros.h"
#include "std_msgs/String.h"

// Callback function on subscription
void keyCB(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Recieved: %s", msg->data.c_str());
}


// Main function
int main(int argc, char **argv) {

    // Tell master who we are, this requires correct
    // environment variables setup
    ros::init(argc, argv, "ohmni_bot");
    ros::NodeHandle nh;
    
    // Subscibe to topic here
    ros::Subscriber teleop_key = nh.subscribe("ohmni_teleop_controller", 1000, keyCB);
    
    // Enter a loop and pumping callbacks
    ros::spin();
    
    // Returns without any errors, I hope
    return 0;
}

