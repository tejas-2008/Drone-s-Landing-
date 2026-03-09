#!/bin/bash

# Setup ROS environment
source "/opt/ros/noetic/setup.bash"

# Source the catkin workspace if it exists
if [ -f "${CATKIN_WS}/devel/setup.bash" ]; then
    source "${CATKIN_WS}/devel/setup.bash"
elif [ -f "${CATKIN_WS}/install/setup.bash" ]; then
    source "${CATKIN_WS}/install/setup.bash"
fi

# Execute the command passed to the container
cd "$HOME/dev_ws"
export GAZEBO_PLUGIN_PATH=/home/tejas/dev_ws/devel/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/noetic/lib
exec "$@"
