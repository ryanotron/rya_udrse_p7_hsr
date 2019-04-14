#!/bin/sh

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/robond/catkin_ws/src/turtlebot_simulator/turtlebot_gazebo/worlds/playground.world" &
sleep 3
xterm -e "roslaunch turtlebot_gazebo amcl_demo_rya.launch" &
sleep 3
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
