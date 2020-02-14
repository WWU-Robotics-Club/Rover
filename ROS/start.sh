#!/bin/bash

# always run from same directory as this script. https://stackoverflow.com/questions/59895
cd $( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )
# Build everything in the workspace
source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash
# Start the nodes
roslaunch rover_core rover.launch

exec "$@"