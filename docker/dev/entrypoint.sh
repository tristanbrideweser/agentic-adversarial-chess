#!/bin/bash
# Entrypoint for the chess-robots dev container.
# Sources ROS 2 Jazzy and the colcon workspace overlay before handing off.

set -e

# Always source the base ROS installation
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source the workspace overlay if it has been built
INSTALL_SETUP="${WORKSPACE}/install/setup.bash"
if [ -f "${INSTALL_SETUP}" ]; then
    source "${INSTALL_SETUP}"
    echo "[entrypoint] Sourced workspace overlay: ${INSTALL_SETUP}"
else
    echo "[entrypoint] Workspace not built yet — run 'colcon build' inside the container"
fi

# Execute the command passed to the container (default: bash)
exec "$@"