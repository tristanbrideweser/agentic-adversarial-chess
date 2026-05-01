# =============================================================================
# chess-robots: GPD (Grasp Pose Detection) container
#
# Builds GPD from source against ROS 2 Jazzy + PCL 1.14 (Ubuntu 24.04).
# Exposes a ROS 2 action server on /grasp_planner/gpd_action that accepts
# a cropped PointCloud2 and returns ranked GraspConfig candidates.
#
# Build time: ~10–15 min (PCL + GPD compile).
# Use the pre-built image tag in CI to avoid rebuilding every time.
# =============================================================================

FROM osrf/ros:jazzy-desktop-full AS gpd-builder

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# -----------------------------------------------------------------------------
# Stage 1: build dependencies
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    # PCL
    libpcl-dev \
    pcl-tools \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    # OpenCV (used by GPD for grasp visualisation)
    libopencv-dev \
    python3-opencv \
    # Eigen
    libeigen3-dev \
    # Python
    python3-pip \
    python3-numpy \
    # ROS action / message deps
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Stage 1a: build GPD C++ library from source
# Using the ROS-compatible fork that works with PCL 1.14
# -----------------------------------------------------------------------------
WORKDIR /opt/gpd_ws

RUN git clone --depth 1 https://github.com/atenpas/gpd.git && \
    cd gpd && \
    mkdir build && cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -Duse_cuda=OFF && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# -----------------------------------------------------------------------------
# Stage 1b: build gpd_ros ROS 2 package
# -----------------------------------------------------------------------------
WORKDIR /opt/gpd_ros_ws/src

RUN git clone --depth 1 -b ros2 https://github.com/atenpas/gpd_ros.git || \
    git clone --depth 1 https://github.com/atenpas/gpd_ros.git

WORKDIR /opt/gpd_ros_ws

RUN bash -c "\
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release"

# =============================================================================
# Stage 2: runtime image (slimmer — drop build tools)
# =============================================================================
FROM ros:jazzy-ros-base AS gpd-runtime

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
    libpcl-dev \
    libopencv-dev \
    libeigen3-dev \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    python3-pip \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Copy GPD library and gpd_ros workspace from builder
COPY --from=gpd-builder /usr/local /usr/local
COPY --from=gpd-builder /opt/gpd_ros_ws/install /opt/gpd_ros_ws/install

RUN ldconfig

# Python deps for the action server node
RUN pip3 install --no-cache-dir --break-system-packages \
    numpy \
    open3d

# gpd_server.py lives alongside this Dockerfile
COPY gpd_server.py /gpd_server.py

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=42
# GPD model weights — shipped with the library
ENV GPD_MODELS_DIR=/usr/local/share/gpd/models

COPY gpd_entrypoint.sh /gpd_entrypoint.sh
RUN chmod +x /gpd_entrypoint.sh
ENTRYPOINT ["/gpd_entrypoint.sh"]
CMD ["python3", "/gpd_server.py"]