# Agentic Adversarial Chess 

## System Overview 
Two robot arms play chess against each other. Each arm has its own Stockfish engine for move generation, uses GPD for grasp planning on chess pieces, and a FEN-to-world-coordinate mapping layer to translate board state into physical pick-and-place targets.

## Core Modules

1. Game Engine (chess_engine/)
Manages the chess game state and interfaces with Stockfish. Maintains the FEN string, validates moves, detects checkmate/stalemate, and alternates turns between the two arms. Each player gets its own Stockfish process so you can set different ELO/depth if you want asymmetric play. Publishes the current FEN and the next move (e.g., e2e4) as ROS 2 topics.

2. Board Localization (board_localization/)
Maps the 8×8 grid to world-frame coordinates — similar to what you built for your chess TF broadcaster. Takes a known board origin + orientation (either from a calibration routine or a fixed mount) and computes the (x, y, z) centroid of each square. Exposes a service: given a square name (e.g., e4), returns a geometry_msgs/PoseStamped in the world frame. Also handles the "graveyard" zones where captured pieces go (one per player, off-board).

3. Move Translator (move_translator/)
Bridges the symbolic and physical layers. Takes a UCI move string (e.g., e2e4) and decomposes it into a sequence of pick-and-place tasks. Handles special cases: captures (remove opponent piece first → graveyard), castling (two-piece move), en passant (non-intuitive capture square), and promotion (piece swap). Outputs an ordered task queue of (pick_pose, place_pose) pairs.

4. Grasp Planner (grasp_planner/)
Wraps GPD (Grasp Pose Detection). Takes a target piece location, crops the point cloud around that region, runs GPD to generate candidate grasps, filters/ranks them (top-down grasps preferred for chess pieces — tall kings vs. flat pawns will have different grasp profiles). Publishes the selected geometry_msgs/PoseStamped grasp pose. You'll likely want piece-type-aware filtering since a pawn vs. a rook has very different geometry.

5. Arm Controller (arm_controller/)
One instance per arm. Receives pick-and-place commands, plans and executes trajectories via MoveIt 2. Handles the full sequence: approach → pre-grasp → grasp → lift → transit → pre-place → place → release → retract. Manages gripper open/close. Needs a turn-based mutex so the arms don't collide — only one arm moves at a time (or you get fancy with workspace partitioning).

6. Perception (perception/)
Point cloud processing from a depth camera (e.g., RealSense). Segments the board and pieces, feeds cropped clouds to GPD. Optional but valuable: board state verification — after a move executes, use vision to confirm the physical board matches the expected FEN. Detects knocked-over pieces or failed grasps.

7. Coordination (game_coordinator/)
Top-level state machine (could use py_trees or a simple FSM). Orchestrates the full loop: request move from Stockfish → translate to physical tasks → plan grasps → execute arm motion → verify → switch turns. Handles error recovery (failed grasp → retry, knocked piece → reposition).

## Repo Structure
```
chess_robots_ws/
├── src/
│   ├── chess_engine/            # Stockfish interface, FEN management
│   │   ├── chess_engine/
│   │   │   ├── stockfish_node.py
│   │   │   └── game_state.py
│   │   ├── config/
│   │   │   └── stockfish_params.yaml   # ELO, depth, time per move
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── board_localization/      # Square → world frame mapping
│   │   ├── board_localization/
│   │   │   ├── tf_broadcaster.py       # Publishes frames for all 64 squares
│   │   │   └── square_lookup_service.py
│   │   ├── config/
│   │   │   └── board_params.yaml       # origin, orientation, square size
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── move_translator/         # UCI move → pick/place task queue
│   │   ├── move_translator/
│   │   │   ├── move_decomposer.py
│   │   │   └── special_moves.py        # castling, en passant, promotion
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── grasp_planner/           # GPD integration
│   │   ├── grasp_planner/
│   │   │   ├── gpd_client.py
│   │   │   └── grasp_filter.py         # piece-type-aware ranking
│   │   ├── config/
│   │   │   └── gpd_params.yaml
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── arm_controller/          # MoveIt 2 pick-and-place execution
│   │   ├── arm_controller/
│   │   │   ├── pick_place_server.py
│   │   │   └── gripper_interface.py
│   │   ├── config/
│   │   │   ├── arm_white.yaml
│   │   │   └── arm_black.yaml
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── perception/              # Point cloud + board verification
│   │   ├── perception/
│   │   │   ├── pointcloud_segmenter.py
│   │   │   ├── board_verifier.py
│   │   │   └── piece_detector.py
│   │   ├── config/
│   │   │   └── camera_params.yaml
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── game_coordinator/        # Top-level state machine
│   │   ├── game_coordinator/
│   │   │   ├── coordinator_node.py
│   │   │   └── behavior_tree.xml       # or FSM definition
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── chess_robot_description/  # URDF/xacro, Gazebo world
│   │   ├── urdf/
│   │   ├── meshes/               # piece STLs, board mesh
│   │   ├── worlds/
│   │   │   └── chess_table.sdf
│   │   └── launch/
│   │       └── spawn_robots.launch.py
│   │
│   └── chess_robot_bringup/     # Top-level launch + config
│       ├── launch/
│       │   ├── sim.launch.py
│       │   └── real.launch.py
│       └── config/
│           └── robot_positions.yaml    # where each arm sits relative to board
│
├── docker/                      # Stockfish + GPD containerized
├── docs/
│   └── architecture.md
└── README.md
```