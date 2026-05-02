# =============================================================================
# chess-robots: GPD (Grasp Pose Detection) container
# Builds GPD from source against ROS 2 Jazzy + PCL 1.14 (Ubuntu 24.04).
# =============================================================================

FROM osrf/ros:jazzy-desktop-full AS gpd-builder

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    libpcl-dev \
    pcl-tools \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    libopencv-dev \
    python3-opencv \
    libeigen3-dev \
    python3-pip \
    python3-numpy \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    && rm -rf /var/lib/apt/lists/*

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

# =============================================================================
# Stage 2: runtime image
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

COPY --from=gpd-builder /usr/local /usr/local
RUN ldconfig

# Ubuntu 24.04 requires --break-system-packages
RUN pip3 install --no-cache-dir --break-system-packages \
    numpy \
    open3d

COPY gpd_server.py /gpd_server.py

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=42
ENV GPD_MODELS_DIR=/usr/local/share/gpd/models

COPY gpd_entrypoint.sh /gpd_entrypoint.sh
RUN chmod +x /gpd_entrypoint.sh
ENTRYPOINT ["/gpd_entrypoint.sh"]
CMD ["python3", "/gpd_server.py"]