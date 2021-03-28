#!/bin/sh
xterm  -e  "roslaunch turtlebot_gazebo world.launch" & 
sleep 5
xterm  -e  "roslaunch turtlebot_teleop keyboard_teleop.launch "