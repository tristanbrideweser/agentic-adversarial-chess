# =============================================================================
# chess-robots: dev container
# Base: ROS 2 Jazzy on Ubuntu 24.04
# Includes: MoveIt 2, Franka ROS 2, Gazebo Harmonic, gz-ros2-bridge,
#           python-chess, pcl_ros, cv_bridge, py_trees
# =============================================================================

FROM osrf/ros:jazzy-desktop-full

# -----------------------------------------------------------------------------
# System dependencies
# -----------------------------------------------------------------------------
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential \
    cmake \
    git \
    wget \
    curl \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # ROS tools
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-image \
    # MoveIt 2
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-ros-planning-interface \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    # Franka
    ros-${ROS_DISTRO}-franka-ros2 \
    ros-${ROS_DISTRO}-franka-description \
    # Perception
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-depth-image-proc \
    # Controllers
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-gripper-controllers \
    # Behavior trees
    ros-${ROS_DISTRO}-py-trees \
    ros-${ROS_DISTRO}-py-trees-ros \
    # TF
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    # Misc utilities
    ros-${ROS_DISTRO}-topic-tools \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rviz2 \
    # Python libs
    python3-numpy \
    python3-scipy \
    python3-pytest \
    python3-yaml \
    # Networking / debug
    iputils-ping \
    net-tools \
    vim \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Python packages (no ROS equivalent)
# -----------------------------------------------------------------------------
RUN pip3 install --no-cache-dir \
    chess \
    stockfish \
    py-trees==2.2.3 \
    open3d \
    transforms3d

# -----------------------------------------------------------------------------
# Workspace setup
# -----------------------------------------------------------------------------
ARG WORKSPACE=/root/chess_ws
ENV WORKSPACE=${WORKSPACE}
WORKDIR ${WORKSPACE}

# Copy source into the image at build time.
# In development, this layer is overridden by the bind-mount in compose.
COPY src/ src/ 2>/dev/null || true

# rosdep install (skips missing packages gracefully in case src is empty)
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true

# Build workspace if src is present
RUN bash -c "\
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    if [ -d src ] && [ \"$(ls -A src)\" ]; then \
        colcon build --symlink-install \
            --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo; \
    else \
        echo 'src/ is empty — skipping build (will build at runtime via volume mount)'; \
    fi"

# -----------------------------------------------------------------------------
# Environment
# -----------------------------------------------------------------------------
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=42

# Entrypoint sources ROS and the workspace overlay
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]