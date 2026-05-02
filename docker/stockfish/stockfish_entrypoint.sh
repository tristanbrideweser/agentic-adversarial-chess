#!/bin/bash
set -e

# 1. Source the base ROS 2 Jazzy installation
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# 2. Source the workspace overlay
INSTALL_SETUP="${WORKSPACE}/install/setup.bash"
if [ -f "${INSTALL_SETUP}" ]; then
    source "${INSTALL_SETUP}"
    echo "[Stockfish Entrypoint] Sourced workspace overlay."
fi

# 3. CycloneDDS scoped to localhost
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>lo</NetworkInterfaceAddress></General></Domain></CycloneDDS>"

exec "$@"