# Home Service Robot
Does these things:
- map an area (a home, to match the title, for a generous definition of home)
- pick things up from pickup locations in the area
- drop them off in a dropoff location

## Requirements
- ROS Kinetic, with packages
    - turtlebot_gazebo
    - turtlebot_navigation
    - their attendant dependencies
    - teleop_twist_keyboard package from [here](https://github.com/ros-teleop/teleop_twist_keyboard) 

### Setup
Clone this repository to your catkin workspace, then build. Install any missing dependencies.

## Usage
Usage is done in two stages, mapping and pickup. Some pre-made maps are provided, to go directly to pickup tasks, go to the Pickup section.

### Mapping
Two ways are provided to map: with Turtlebot and Xtion (translated into `laser'), and with a custom robot with lidar. The former has narrower FOV, but the Pickup part is only tested with Turtlebot. If you would like to use the same sensor for mapping and localisation, use Xtion. You might need several tries to get the map right. Stay in wider corridors in your mapping run. If you want a more precise map instead, use lidar.

#### Xtion
Steps:
1. launch world with `roslaunch hsr turtlebot_world.launch`
1. launch gmapping with `roslaunch hsr gmapping.launch`
1. launch rviz with `roslaunch hsr view_navigation.launch`
1. launch teleop with `roslaunch hsr keyboard_teleop.launch`
1. drive the robot with teleop until you have a map with good coverage
1. save the map with `rosrun map_server map_saver -f mymap_xtion`
1. move on to Pickup section

An example map is provided in hsr/map/map_xtion.yaml

#### Lidar
Steps:
1. launch world with `roslaunch ryabot world.launch`
1. launch gmapping with `roslaunch ryabot gmapping.launch`
1. launch rviz with `roslaunch hsr view_navigation.launch`
1. launch teleop with `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`
1. drive the robot with teleop until you have a map with good coverage
1. save the map with `rosrun map_server map_saver -f mymap_lidar`
1. move on to Pickup section

An example map is provided in hsr/map/map_urg.yaml

### Pickup
The pickup task is only tested with Turtlebot/Xtion, so only instruction for that is provided here. 

Pickup is performed with the `pick_objects` node. It asks ros parameter server for the pickup locations and the dropoff location, picks up objects in order, and drops them off at the dropoff point. Visualisation of the objects to be picked up are provided by the add_markers node. Upon pickup, the objects are moved to the robot's frame, so it appears to be attached to the robot. They are still *illustrations*, so their dimension has no bearing on collision and path planning.

If the robot is unable to find a path to a pickup location, it will be skipped. If the dropoff point is unreachable, the robot will end its run at the last reachable pickup location. In this version, it is not possible to change the dropoff point mid-run. Stubs for adding pickup locations are in the source, but it isn't tested, so leave it be for now.

Example pickup points and dropoff point are defined in `pick_objects/param/coordinates.yaml`. Make sure that those points are defined in the map's coordinate system (not Gazebo!).

Steps:
1. launch world with `roslaunch hsr turtlebot_world.launch`
1. launch localiser and planner with `roslaunch hsr amcl.launch`. Input your map into this launch file
1. launch rviz with `roslaunch hsr view_navigation.launch`
1. launch object picker and visualiser with `roslaunch pick_objects pick_objects.launch`. Make sure your pickup/dropoff params are set. Working example in pick_objects/param/coordinates.yaml
1. watch the robot does its work

## Batteries Included

These things are included: laser robot (package ryabot), example maps (hsr/map), example world (hsr/world), convenience scripts (sh files in root)

ryabot is adapted from a previous [Udacity project](https://github.com/ryanotron/rya_udrse_p4_wai). Laser FOV was increased to 270 [deg], and some cosmetic adjustments were made.

Example maps include ground truth map generated from Gazebo world (map_geo2_alt), map generated with Turtlebot/Xtion (map_xtion), and map generated with ryabot/lidar (map_urg). The latter two have the same coordinate system, different from the first one. The example pickup parameters in coordinates.yaml assumes map_xtion or map_urg.

Example world is also taken from a [previous project](https://github.com/ryanotron/rya_udrse_p4_wai).

Convenience scripts:
- `run_xtion_map.sh` launch the relevant parts of section Mapping/Xtion
- `run_laser_map.sh` launch the relevant parts of section Mappinig/Laser
- `run_pickup.sh` launch the relevant parts of section Pickup

The mapping scripts do not save the resultant map. Remember to manually run `map_saver`.