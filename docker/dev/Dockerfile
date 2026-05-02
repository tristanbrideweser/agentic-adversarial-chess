# =============================================================================
# chess-robots: dev container
# Base: ROS 2 Jazzy on Ubuntu 24.04
# =============================================================================

FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# -----------------------------------------------------------------------------
# System dependencies
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-image \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-ros-planning-interface \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-depth-image-proc \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-gripper-controllers \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-py-trees \
    ros-${ROS_DISTRO}-py-trees-ros \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    # franka_ros2 build deps (not in apt — source build below)
    libpoco-dev \
    libeigen3-dev \
    python3-numpy \
    python3-scipy \
    python3-pytest \
    python3-psutil \
    iputils-ping \
    net-tools \
    vim \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# franka_ros2 — source build
# Step 1: build libfranka from source (required by franka_ros2)
# -----------------------------------------------------------------------------
# Install pinocchio from robotpkg (required by libfranka >= 0.14.0)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libpoco-dev libspdlog-dev lsb-release curl \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /etc/apt/keyrings && \
    curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
        | tee /etc/apt/keyrings/robotpkg.asc && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
        | tee /etc/apt/sources.list.d/robotpkg.list && \
    apt-get update && apt-get install -y robotpkg-pinocchio && \
    rm -rf /var/lib/apt/lists/*

RUN git clone --recurse-submodules --depth 1 --branch 0.15.0 \
        https://github.com/frankaemika/libfranka.git /opt/libfranka && \
    mkdir -p /opt/libfranka/build && \
    cd /opt/libfranka/build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTS=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DCMAKE_PREFIX_PATH=/opt/openrobots && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# Step 2: build franka_ros2 — skip optional/unavailable deps
RUN rosdep update && \
    mkdir -p /opt/franka_ws/src && \
    cd /opt/franka_ws/src && \
    git clone --depth 1 \
        https://github.com/frankarobotics/franka_ros2.git && \
    cd /opt/franka_ws && \
    bash -c "\
        source /opt/ros/${ROS_DISTRO}/setup.bash && \
        rosdep install --from-paths src --ignore-src -r -y \
            --skip-keys=\"libfranka franka_description pinocchio \
                realsense2_camera realsense2_description \
                sick_safetyscanners2 olv_module_descriptions \
                ament_cmake_clang_format ament_cmake_clang_tidy \
                hardware_interface_testing joint_state_publisher \
                joint_state_publisher_gui gz_ros2_control\" || true && \
        colcon build \
            --packages-select franka_msgs \
            --cmake-args -DCMAKE_BUILD_TYPE=Release \
                -DCMAKE_PREFIX_PATH=/usr/local \
            --symlink-install" && \
    rm -rf /opt/franka_ws/build /opt/franka_ws/log

# -----------------------------------------------------------------------------
# Python packages
# Ubuntu 24.04 requires --break-system-packages for pip installs
# -----------------------------------------------------------------------------
# Stockfish binary (not in Ubuntu 24.04 apt repos — download directly)
RUN wget -q https://github.com/official-stockfish/Stockfish/releases/download/sf_16/stockfish-ubuntu-x86-64-avx2.tar \
        -O /tmp/stockfish.tar && \
    tar -xf /tmp/stockfish.tar -C /tmp/ && \
    mv /tmp/stockfish/stockfish-ubuntu-x86-64-avx2 /usr/bin/stockfish && \
    chmod +x /usr/bin/stockfish && \
    rm -rf /tmp/stockfish.tar /tmp/stockfish

RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed \
    chess \
    stockfish \
    open3d \
    transforms3d

# -----------------------------------------------------------------------------
# Workspace setup
# src is bind-mounted at runtime — no bake-time copy
# -----------------------------------------------------------------------------
ARG WORKSPACE=/root/chess_ws
ENV WORKSPACE=${WORKSPACE}
RUN mkdir -p ${WORKSPACE}/src
WORKDIR ${WORKSPACE}

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=42

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
# moveit_py — Python bindings for MoveIt 2 (not available as apt package in Jazzy)
RUN mkdir -p /opt/moveit_py_ws/src && \
    cd /opt/moveit_py_ws/src && \
    git clone --depth 1 --branch main \
        https://github.com/moveit/moveit2.git && \
    cd /opt/moveit_py_ws && \
    bash -c "\
        source /opt/ros/${ROS_DISTRO}/setup.bash && \
        colcon build \
            --packages-select moveit_py \
            --cmake-args -DCMAKE_BUILD_TYPE=Release \
            --symlink-install" && \
    rm -rf /opt/moveit_py_ws/build /opt/moveit_py_ws/log
