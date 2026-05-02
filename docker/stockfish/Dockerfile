# =============================================================================
# chess-robots: Stockfish container
# =============================================================================

FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
    stockfish \
    python3-pip \
    python3-numpy \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Ubuntu 24.04 requires --break-system-packages
RUN pip3 install --no-cache-dir --break-system-packages \
    chess \
    stockfish

ENV WORKSPACE=/root/chess_ws
WORKDIR ${WORKSPACE}

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY stockfish_entrypoint.sh /stockfish_entrypoint.sh
RUN chmod +x /stockfish_entrypoint.sh
ENTRYPOINT ["/stockfish_entrypoint.sh"]
CMD ["ros2", "run", "chess_engine", "stockfish_node"]