# =============================================================================
# chess-robots: Stockfish container
# Runs the Stockfish ROS 2 node (stockfish_node.py from chess_engine package).
# One instance is launched per player (white/black) via the compose env file.
# =============================================================================

FROM osrf/ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# -----------------------------------------------------------------------------
# Stockfish binary + Python chess deps
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    stockfish \
    python3-pip \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    chess \
    stockfish

# Verify stockfish is callable
RUN which stockfish && stockfish <<< "quit"

# -----------------------------------------------------------------------------
# ROS 2 message deps (std_msgs only — chess_engine uses String topics)
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Workspace — the chess_engine package is bind-mounted from the host at runtime.
# At build time we just create the workspace skeleton so the entrypoint works.
# -----------------------------------------------------------------------------
ENV WORKSPACE=/root/chess_ws
WORKDIR ${WORKSPACE}

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY stockfish_entrypoint.sh /stockfish_entrypoint.sh
RUN chmod +x /stockfish_entrypoint.sh
ENTRYPOINT ["/stockfish_entrypoint.sh"]

# Default CMD is overridden per-service in docker-compose.yaml
# to set the correct --ros-args player color and params file.
CMD ["ros2", "run", "chess_engine", "stockfish_node"]