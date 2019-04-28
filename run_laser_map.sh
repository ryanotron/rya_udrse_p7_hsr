#!/bin/sh

xterm -e "roslaunch ryabot world.launch" &
sleep 5
xterm -e "roslaunch ryabot gmapping.launch" &
sleep 5
xterm -e "roslaunch hsr view_navigation.launch" &
sleep 5
xterm -e "rosrun teleop_twist_keyboard teleop_twist_keyboard.py" &