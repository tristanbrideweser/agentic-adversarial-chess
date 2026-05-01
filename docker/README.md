# chess-robots Docker Setup

## Services

| Service | Image | Purpose |
|---|---|---|
| `dev` | `chess-robots/dev` | ROS 2 Jazzy + MoveIt 2 + Gazebo Harmonic dev shell |
| `stockfish_white` | `chess-robots/stockfish` | Stockfish engine, White player |
| `stockfish_black` | `chess-robots/stockfish` | Stockfish engine, Black player (ELO-limited) |
| `gpd` | `chess-robots/gpd` | GPD grasp pose detection action server |

All services use `network_mode: host` and `ROS_DOMAIN_ID=42`.

---

## First-time setup

```bash
# 1. Allow the container to connect to your X server (for Gazebo/RViz GUI)
xhost +local:docker

# 2. Build all images (GPD takes ~10–15 min the first time)
cd docker/
docker compose build

# 3. Start all services
docker compose up -d

# 4. Open a dev shell
docker compose exec dev bash
```

Inside the dev shell, build your workspace once:

```bash
cd ~/chess_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

Subsequent runs reuse the `chess_build_cache` volume — no full rebuild needed.

---

## Common workflows

### Run the full simulation
```bash
# Inside the dev container
ros2 launch chess_robot_bringup sim.launch.py
```

### Rebuild only the GPD image (after GPD source changes)
```bash
docker compose build --no-cache gpd
```

### Restart Stockfish engines with different ELO
Edit `stockfish/stockfish.env` or the per-service `environment` block in
`docker-compose.yaml`, then:
```bash
docker compose restart stockfish_white stockfish_black
```

### Check that all nodes are visible from the dev container
```bash
# Inside dev container
ros2 node list
# Expected output includes:
#   /stockfish_white
#   /stockfish_black
#   /gpd_server
```

### Run unit tests (no ROS required)
```bash
# On the host, no Docker needed
cd chess_robots_ws
pip install chess pytest
pytest src/move_translator/test/ src/grasp_planner/test/ -v
```

---

## GPU support (optional, for CUDA-accelerated GPD)

1. Install [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html).
2. In `docker-compose.yaml`, uncomment the `deploy.resources` block under the `gpd` service.
3. In `gpd/Dockerfile`, change `-Duse_cuda=OFF` to `-Duse_cuda=ON`.
4. Rebuild: `docker compose build --no-cache gpd`

---

## Ports and networking

All containers share `network_mode: host`, so no port mapping is needed.
ROS 2 DDS discovery is scoped to localhost via `CYCLONEDDS_URI`.

If you need to run nodes on a separate machine on the same LAN, remove the
`NetworkInterfaceAddress` restriction from `CYCLONEDDS_URI` in `docker-compose.yaml`
and ensure all machines share the same `ROS_DOMAIN_ID=42`.

---

## GPD fallback mode

If the `gpd` container fails to build (e.g., PCL version mismatch), the
`grasp_planner` node automatically falls back to its precomputed lookup table
— no point cloud required. This is the recommended mode for Gazebo simulation.

Set `mode: "lookup"` in `src/grasp_planner/config/gpd_params.yaml` to
permanently disable GPD and skip starting the `gpd` service entirely:

```bash
docker compose up dev stockfish_white stockfish_black
```