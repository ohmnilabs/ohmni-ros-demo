
//
// Basic subscriber node that will get data
// from controller topic, perform conversion
// and parse over to telebot socket
//
// Hoang
//

// Import necessary headers to setup the UNIX domain socket
// really hope catkin_make would compile these...
#include <iostream> // for exit(1)
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h> // for socket in general

// Unix domain socket name, hardcoded for now
#define NAME "/opt/sockets/node.sock"

// Macro for consistent buffer size
#define BUFFER_SIZE 8192

// Import some basic ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"

// Global struct so we can store socket information and such
struct Socket {
    int sock; // pointer to the socket
    char buf[BUFFER_SIZE]; // message buffer side
    struct sockaddr_un server; // server configuration
};
struct Socket _sock = {0}; // init struct to zeros now

// Unix domain client side initialize
int ud_client_init() {

     // Welp, create a socket first
    _sock.sock = socket(AF_UNIX, SOCK_STREAM, 0);
    if (_sock.sock < 0) {
        ROS_ERROR("UD: opening stream socket error");
        return -1;
    }

    // Configure the local file name so we can request the server
    _sock.server.sun_family = AF_UNIX;
    strcpy(_sock.server.sun_path, NAME);

    // Sanity check for connection error
    if (connect(_sock.sock, (struct sockaddr *) &(_sock.server), sizeof(struct sockaddr_un)) < 0) {
        close(_sock.sock);
        ROS_ERROR("UD: connecting stream socket error");
        return -1;
    }   
}

// Callback function on subscription
void keyCB(const std_msgs::String::ConstPtr& msg) {

    // Declare for debug purpose
    ROS_INFO("Recieved: %s", msg->data.c_str());
    
    // Store the message into our buffer and write to socket
    strcpy(_sock.buf, msg->data.c_str());
    if (write(_sock.sock, _sock.buf, sizeof(_sock.buf)) < 0) {
        ROS_ERROR("UD: writing on stream socket error");
    }
}

// Main function
int main(int argc, char **argv) {

    // Tell master who we are, this requires correct
    // environment variables setup
    ros::init(argc, argv, "ohmni_bot");
    
    /* UNIX domain client side setup here with sanity check */
    if (ud_client_init() < 0) {
        ROS_ERROR("UD: failed to establish Unix domain client");
        return -1;
    }
    // Subscibe to topic here
    ros::NodeHandle nh;
    ros::Subscriber teleop_key = nh.subscribe("ohmni_teleop_controller", 1000, keyCB);
    
    // Enter a loop and pumping callbacks
    ros::spin();
    
    // Returns without any errors, I hope
    return 0;
}

