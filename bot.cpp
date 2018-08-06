
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
#include <vector>
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

// Input message process
void input_process(const char* input) {

    // JSON msg generated here 
    char msg[] = "{\"cmd\":\"move\",\"data\":{\"lspeed\":500,\"rspeed\":500,\"time\":10}}";

    // Generate the data length and message type
    uint16_t type = 1; //type = 1 for json now
    uint16_t msg_len = sizeof(msg); // length of the data msg only (the json)

    // Message protocol is like this: 2 bytes for type (type=1 for json) + 2 bytes for data len + the actual data
    char buffer[4 + sizeof(msg)];

    // The first 4 bytes of the buffer will store 16-bit ints
    // for msg type and json length
    buffer[0] = (type) & 0xff;
    buffer[1] = (type >> 8) & 0xff;
    buffer[2] = (msg_len) & 0xff;
    buffer[3] = (msg_len >> 8) & 0xff;

    // Now concatenate the json msg itself into the buffer
    strcat(buffer, msg);

    // Write out the message now
    if (write(_sock.sock, &buffer, sizeof(buffer)) < 0) {
        ROS_ERROR("UD: writing on stream socket error");
    }
}

// Callback function on subscription
void keyCB(const std_msgs::String::ConstPtr& msg) {

    // Declare for debug purpose
    ROS_INFO("Recieved: %s", msg->data.c_str());
    
    // Parse the input to process before sending it out to our socket
    input_process(msg->data.c_str());
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

