# =============================================================================
# chess-robots: dev container
# Base: ROS 2 Jazzy on Ubuntu 24.04
# =============================================================================

FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# -----------------------------------------------------------------------------
# 1. System & ROS Dependencies
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget curl \
    python3-pip python3-colcon-common-extensions \
    python3-rosdep python3-vcstool \
    # Use the ROS-specific package names
    ros-${ROS_DISTRO}-py-trees-ros \
    ros-${ROS_DISTRO}-py-trees-ros-interfaces \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-ros-planning-interface \
    # Fixed the double-hyphen below
    ros-${ROS_DISTRO}-moveit-servo \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-gz-ros2-control \
    libpoco-dev libeigen3-dev \
    python3-numpy python3-scipy python3-psutil \
    iputils-ping net-tools vim tmux \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# 2. franka_description + franka_msgs
#    franka_description: URDF/xacro macros for the FR3 arm
#    franka_msgs:        gripper action types used by arm_controller
#    No libfranka needed — gz_ros2_control handles sim hardware, not franka_hardware
# -----------------------------------------------------------------------------
RUN mkdir -p /opt/franka_ws/src && \
    cd /opt/franka_ws/src && \
    git clone --depth 1 \
        https://github.com/frankaemika/franka_description.git && \
    git clone --depth 1 -b jazzy \
        https://github.com/frankarobotics/franka_ros2.git && \
    cd /opt/franka_ws && \
    bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
        colcon build \
            --packages-select \
                franka_description \
                franka_msgs \
            --cmake-args \
                -DCMAKE_BUILD_TYPE=Release \
                -DBUILD_TESTING=OFF" && \
    rm -rf /opt/franka_ws/build /opt/franka_ws/log /opt/franka_ws/src
# -----------------------------------------------------------------------------
# 3. Python & Binaries
# -----------------------------------------------------------------------------
RUN wget -q https://github.com/official-stockfish/Stockfish/releases/download/sf_16/stockfish-ubuntu-x86-64-avx2.tar \
        -O /tmp/stockfish.tar && \
    tar -xf /tmp/stockfish.tar -C /tmp/ && \
    mv /tmp/stockfish/stockfish-ubuntu-x86-64-avx2 /usr/bin/stockfish && \
    chmod +x /usr/bin/stockfish && \
    rm -rf /tmp/stockfish*

RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed \
    chess stockfish open3d transforms3d py-trees \
    "setuptools==68.2.2"


# -----------------------------------------------------------------------------
# 4. Environment & Workspace
# -----------------------------------------------------------------------------
ARG WORKSPACE=/root/chess_ws
ENV WORKSPACE=${WORKSPACE}
RUN mkdir -p ${WORKSPACE}/src
WORKDIR ${WORKSPACE}

ENV ROS_DOMAIN_ID=42
ENV NVIDIA_DRIVER_CAPABILITIES=all

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]