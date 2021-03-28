#!/bin/sh
xterm  -e  "roslaunch add_markers world_navigation.launch" & 
sleep 5
xterm  -e  "rosrun add_markers show_markers; read -p Press_a_Key " 