#!/bin/bash
set -e
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f /opt/gpd_ros_ws/install/setup.bash ]; then
    source /opt/gpd_ros_ws/install/setup.bash
fi
exec "$@"