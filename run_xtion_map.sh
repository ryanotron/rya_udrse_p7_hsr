#!/bin/sh

xterm -e "roslaunch hsr turtlebot_world.launch" &
sleep 5
xterm -e "roslaunch hsr gmapping.launch" &
sleep 5
xterm -e "roslaunch hsr view_navigation.launch" &
sleep 5
xterm -e "roslaunch hsr keyboard_teleop.launch" &