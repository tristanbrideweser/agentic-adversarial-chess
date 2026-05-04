#!/bin/bash
set -e

# 1. ROS 2 Jazzy base
source /opt/ros/${ROS_DISTRO}/setup.bash

# 2. Franka overlay (franka_description + franka_msgs)
if [ -f /opt/franka_ws/install/setup.bash ]; then
    source /opt/franka_ws/install/setup.bash
fi

# 3. Chess workspace overlay
if [ -f ${WORKSPACE}/install/setup.bash ]; then
    source ${WORKSPACE}/install/setup.bash
    echo "[entrypoint] Sourced chess_ws overlay"
else
    echo "[entrypoint] chess_ws not built yet"
    echo "            Run: colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
fi

# 4. Environment
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>lo</NetworkInterfaceAddress></General></Domain></CycloneDDS>"
export PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.12/site-packages:${PYTHONPATH:-}
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/${ROS_DISTRO}/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}
export GZ_SIM_RESOURCE_PATH=${WORKSPACE}/src:${GZ_SIM_RESOURCE_PATH:-}

# 5. Stockfish symlink fallback
if [ ! -f /usr/bin/stockfish ] && [ -f /usr/games/stockfish ]; then
    ln -sf /usr/games/stockfish /usr/bin/stockfish 2>/dev/null || true
fi

export GZ_SIM_RESOURCE_PATH=/opt/franka_ws/install/franka_description/share:${GZ_SIM_RESOURCE_PATH}

exec "$@"