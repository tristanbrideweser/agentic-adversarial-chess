#!/usr/bin/env bash
# =============================================================================
# chess-robots/docker/run.bash
#
# Launches the chess-robots dev container with:
#   - X11 forwarding (Gazebo, RViz GUI)
#   - NVIDIA GPU passthrough if available
#   - ROS 2 DDS networking (host network + IPC)
#   - Workspace bind-mounted from host
#   - CycloneDDS scoped to localhost
#
# Usage:
#   ./run.bash                        # interactive shell
#   ./run.bash "ros2 launch ..."      # run a specific command
#   ./run.bash -v /extra:/extra       # add extra volume
#   ./run.bash -e MY_VAR=value        # add extra env var
# =============================================================================

set -e

# ---------------------------------------------------------------------------
# Resolve paths
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
REPO_DIR="$(dirname "${SCRIPT_DIR}")"          # chess-robots/
WS_SRC_DIR="${REPO_DIR}"                        # mounted as ~/chess_ws/src

IMAGE="chess-robots/dev:latest"
CONTAINER_NAME="chess_dev"
DOCKER_WS="/root/chess_ws"

# ---------------------------------------------------------------------------
# Parse optional flags: -v VOLUME, -e ENV
# ---------------------------------------------------------------------------
EXTRA_VOLUMES=()
EXTRA_ENVS=()
while getopts ":v:e:" opt; do
    case "${opt}" in
        v) EXTRA_VOLUMES+=("${OPTARG}") ;;
        e) EXTRA_ENVS+=("${OPTARG}") ;;
        *)
            echo >&2 "Usage: ${0} [-v VOLUME] [-e ENV] [CMD]"
            exit 2
            ;;
    esac
done
shift "$((OPTIND - 1))"
CMD="${*}"

# ---------------------------------------------------------------------------
# GPU detection (NVIDIA only — falls back gracefully)
# ---------------------------------------------------------------------------
GPU_OPTS=()
GPU_ENVS=()
if [[ "$(uname)" == "Linux" ]]; then
    HW_VENDOR=$(lshw -C display 2>/dev/null | grep vendor || true)
    if [[ "${HW_VENDOR^^}" =~ NVIDIA ]]; then
        if docker run --rm --gpus all nvidia/cuda:11.0.3-base-ubuntu20.04 \
               nvidia-smi &>/dev/null 2>&1; then
            GPU_OPTS=("--gpus" "all")
            GPU_ENVS=(
                "NVIDIA_VISIBLE_DEVICES=all"
                "NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics"
            )
            echo -e "\033[0;32m[chess-robots] NVIDIA GPU passthrough enabled\033[0m"
        else
            echo -e "\033[0;33m[chess-robots] NVIDIA GPU detected but nvidia-container-toolkit not available — running without GPU\033[0m"
        fi
    fi
fi

# ---------------------------------------------------------------------------
# X11 / GUI setup
# ---------------------------------------------------------------------------
XAUTH=/tmp/.chess_docker.xauth
if [ ! -f "${XAUTH}" ]; then
    touch "${XAUTH}"
    chmod a+r "${XAUTH}"
    XAUTH_LIST=$(xauth nlist "${DISPLAY}" 2>/dev/null || true)
    if [ -n "${XAUTH_LIST}" ]; then
        echo "${XAUTH_LIST}" | sed 's/^..../ffff/' | xauth -f "${XAUTH}" nmerge -
    fi
fi

GUI_VOLUMES=(
    "${XAUTH}:${XAUTH}"
    "/tmp/.X11-unix:/tmp/.X11-unix"
)
GUI_ENVS=(
    "XAUTHORITY=${XAUTH}"
    "DISPLAY=${DISPLAY}"
    "QT_X11_NO_MITSHM=1"
)

# ---------------------------------------------------------------------------
# CycloneDDS — scope to localhost so ROS 2 DDS doesn't flood the LAN
# ---------------------------------------------------------------------------
CYCLONE_XML=$(mktemp /tmp/cyclone_XXXXXX.xml)
cat > "${CYCLONE_XML}" <<'EOF'
<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>lo</NetworkInterfaceAddress>
    </General>
  </Domain>
</CycloneDDS>
EOF

# ---------------------------------------------------------------------------
# Assemble docker run command
# ---------------------------------------------------------------------------
DOCKER_CMD=(
    docker run
    --interactive
    --tty
    --rm
    --name "${CONTAINER_NAME}"
    --network host
    --ipc host
    --privileged
    --security-opt "seccomp=unconfined"

    # Workspace: entire repo mounted as the colcon src tree
    --volume "${WS_SRC_DIR}/ws/src:${DOCKER_WS}/src:rw"

    # Persist build/install/log across container restarts
    --volume "chess_build_cache:${DOCKER_WS}/build"
    --volume "chess_install_cache:${DOCKER_WS}/install"
    --volume "chess_log_cache:${DOCKER_WS}/log"

    # Timezone
    --volume "/etc/localtime:/etc/localtime:ro"

    # CycloneDDS config
    --volume "${CYCLONE_XML}:${CYCLONE_XML}:ro"
    --env "CYCLONEDDS_URI=file://${CYCLONE_XML}"
)

# GUI
for vol in "${GUI_VOLUMES[@]}"; do
    DOCKER_CMD+=("--volume" "${vol}")
done
for env in "${GUI_ENVS[@]}"; do
    DOCKER_CMD+=("--env" "${env}")
done

# GPU
if [ ${#GPU_OPTS[@]} -gt 0 ]; then
    DOCKER_CMD+=("${GPU_OPTS[@]}")
fi
for env in "${GPU_ENVS[@]:-}"; do
    [ -n "${env}" ] && DOCKER_CMD+=("--env" "${env}")
done

# ROS environment
DOCKER_CMD+=(
    --env "ROS_DISTRO=jazzy"
    --env "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}"
    --env "GZ_SIM_RESOURCE_PATH=${DOCKER_WS}/src/chess_robot_description/models"
)

# Extra volumes / envs from -v / -e flags
for vol in "${EXTRA_VOLUMES[@]:-}"; do
    [ -n "${vol}" ] && DOCKER_CMD+=("--volume" "${vol}")
done
for env in "${EXTRA_ENVS[@]:-}"; do
    [ -n "${env}" ] && DOCKER_CMD+=("--env" "${env}")
done

# Image + optional command
DOCKER_CMD+=("${IMAGE}")
[ -n "${CMD}" ] && DOCKER_CMD+=("${CMD}")

# ---------------------------------------------------------------------------
# Print and run
# ---------------------------------------------------------------------------
echo -e "\033[1;30m${DOCKER_CMD[*]}\033[0m"
exec "${DOCKER_CMD[@]}"