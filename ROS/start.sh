#!/bin/bash

cd /ROS
# Build everything in the workspace
source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash
# Start the nodes
roslaunch rover_core rover.launch

exec "$@"