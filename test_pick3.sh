#!/bin/sh

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 3
xterm -e "roslaunch turtlebot_gazebo amcl_demo_rya.launch" &
sleep 3
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 3
xterm -e "roslaunch pick_objects pick_objects.launch"  &