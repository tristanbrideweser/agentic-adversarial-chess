#!/bin/bash
set -e

# 1. Source the base ROS 2 Jazzy installation
source /opt/ros/${ROS_DISTRO}/setup.bash

# 2. Source the Franka drivers overlay
if [ -f /opt/franka_ws/install/setup.bash ] && grep -q "franka_msgs" /opt/franka_ws/install/setup.bash 2>/dev/null; then
    source /opt/franka_ws/install/setup.bash
fi

# 3. CycloneDDS scoped to localhost
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>lo</NetworkInterfaceAddress></General></Domain></CycloneDDS>"

# 4. Source the chess workspace overlay
if [ -f ${WORKSPACE}/install/setup.bash ]; then
    source ${WORKSPACE}/install/setup.bash
    echo "[entrypoint] Sourced chess_ws overlay"
else
    echo "[entrypoint] chess_ws not built yet"
    echo "            Run: colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
fi


# Gazebo plugin and resource paths
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/${ROS_DISTRO}/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}
export GZ_SIM_RESOURCE_PATH=${WORKSPACE}/src:${GZ_SIM_RESOURCE_PATH:-}

# Stockfish binary location (Ubuntu installs to /usr/games/)
if [ ! -f /usr/bin/stockfish ] && [ -f /usr/games/stockfish ]; then
    ln -sf /usr/games/stockfish /usr/bin/stockfish 2>/dev/null || true
fi
# moveit_py overlay
if [ -f /opt/moveit_py_ws/install/setup.bash ]; then
    source /opt/moveit_py_ws/install/setup.bash
fi

exec "$@"
# moveit_py Python path (installed but not auto-added to PYTHONPATH in Jazzy)
export PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.12/site-packages:${PYTHONPATH:-}

# Stockfish binary (Ubuntu installs to /usr/games/, node expects /usr/bin/)
[ ! -f /usr/bin/stockfish ] && [ -f /usr/games/stockfish ] && \
    ln -sf /usr/games/stockfish /usr/bin/stockfish 2>/dev/null || true

# moveit_py Python path
export PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.12/site-packages:${PYTHONPATH:-}

# Gazebo paths  
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/${ROS_DISTRO}/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}
export GZ_SIM_RESOURCE_PATH=${WORKSPACE}/src:${GZ_SIM_RESOURCE_PATH:-}
