#!/bin/sh
xterm  -e  "roslaunch add_markers world_navigation.launch" & 
sleep 5
xterm  -e  "rosrun pick_objects pick_objects; read -p Press_a_Key " 