#!/bin/sh
xterm  -e  "roslaunch add_markers world.launch" & 
sleep 5
xterm  -e  "roslaunch turtlebot_teleop keyboard_teleop.launch "