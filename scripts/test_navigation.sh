#!/bin/sh
xterm  -e  "roslaunch add_markers home_service_navigation.launch" & 
sleep 5
xterm  -e  "rosrun add_markers add_markers "&
sleep 5
xterm  -e  "roslaunch turtlebot_teleop keyboard_teleop.launch "