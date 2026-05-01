#!/bin/bash
set -e
source /opt/ros/${ROS_DISTRO}/setup.bash
INSTALL_SETUP="${WORKSPACE}/install/setup.bash"
if [ -f "${INSTALL_SETUP}" ]; then
    source "${INSTALL_SETUP}"
fi
exec "$@"