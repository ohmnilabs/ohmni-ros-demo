# tb-ros bridge
Ohmni-ROS communication bridge with simple UI for basic Ohmni tele-op control.


## Files description
### RapidJson
Fast and lightweight JSON parser/generator for C++. Please see their GitHub repo for more information https://github.com/Tencent/rapidjson

### controller.py
Simple UI controller for teleop based on Turtlebot2's controller UI from ROS https://github.com/turtlebot/turtlebot/blob/kinetic/turtlebot_teleop/scripts/turtlebot_teleop_key

### bot.cpp
Ohmni's ROS node to get user input from controller.py, generate JSON messages based on inputs (following Ohmni's local_api JSON message protocol) and parse over to tb-node via UNIX domain socket.

## Getting started
Please follow this documentation [blah] for detailed getting started guide.
