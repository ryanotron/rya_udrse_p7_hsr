#!/bin/sh

xterm -e "roslaunch hsr turtlebot_world.launch" &
sleep 5
xterm -e "roslaunch hsr amcl.launch" &
sleep 5
xterm -e "roslaunch hsr view_navigation.launch" &
sleep 5
xterm -e "roslaunch pick_objects pick_objects.launch"  &