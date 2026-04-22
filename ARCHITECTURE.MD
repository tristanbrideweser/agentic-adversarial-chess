# Chess Robot Architecture

Two Franka Panda arms play a full game of chess against each other in Gazebo Harmonic.
Each arm uses Stockfish for move generation, GPD for grasp planning, and a FEN-to-world-coordinate
translation layer to convert symbolic game state into physical pick-and-place operations.

---

## 1. System Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              GAZEBO HARMONIC                                │
│                                                                             │
│    ┌──────────┐          ┌───────────────┐          ┌──────────┐           │
│    │  Panda   │          │  Chess Board  │          │  Panda   │           │
│    │  White   │   pick   │   + 32 Pieces │   pick   │  Black   │           │
│    │ y=-0.45  │◄────────►│   (0, 0, 0)   │◄────────►│ y=+0.45  │           │
│    └────┬─────┘          └───────┬───────┘          └────┬─────┘           │
│         │                        │                        │                 │
│         │              ┌─────────┴─────────┐              │                 │
│         │              │  RGBD Camera      │              │                 │
│         │              │  (0, 0, 1.8)      │              │                 │
│         │              └───────────────────┘              │                 │
└─────────┼──────────────────────┼──────────────────────────┼─────────────────┘
          │  gz-ros2 bridge      │  /overhead_camera/*      │
          ▼                      ▼                          ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                               ROS 2 JAZZY                                   │
│                                                                             │
│  ┌──────────────┐    ┌──────────────┐    ┌────────────────┐                │
│  │  Stockfish   │    │  Stockfish   │    │  Board State   │                │
│  │  (White)     │    │  (Black)     │    │  Node          │                │
│  │  /stockfish/ │    │  /stockfish/ │    │  /board_state  │                │
│  │  white/      │    │  black/      │    │  /apply_move   │                │
│  │  get_move    │    │  get_move    │    │  /reset_board  │                │
│  └──────┬───────┘    └──────┬───────┘    └───────┬────────┘                │
│         │                   │                     │                         │
│         │     UCI move      │                     │  FEN string             │
│         └─────────┬─────────┘                     │                         │
│                   ▼                               ▼                         │
│         ┌─────────────────────────────────────────────────┐                 │
│         │              MOVE TRANSLATOR                     │                 │
│         │                                                  │                 │
│         │  Input:  FEN string + UCI move (e.g. "e2e4")    │                 │
│         │  Output: Ordered pick/place task queue           │                 │
│         │                                                  │                 │
│         │  ┌─────────────┐  ┌──────────────┐              │                 │
│         │  │ Board       │  │ Move         │              │                 │
│         │  │ Geometry    │  │ Decomposer   │              │                 │
│         │  │             │  │              │              │                 │
│         │  │ square→XYZ  │  │ UCI→tasks    │              │                 │
│         │  │ piece heights│  │ special moves│              │                 │
│         │  └─────────────┘  └──────────────┘              │                 │
│         └──────────────────────┬──────────────────────────┘                 │
│                                │  task queue                                │
│                                ▼                                            │
│         ┌──────────────────────────────────────────────┐                    │
│         │              GRASP PLANNER                    │                    │
│         │  Input:  target pose + piece type             │                    │
│         │  Output: gripper pose (PoseStamped)           │                    │
│         │                                               │                    │
│         │  GPD or precomputed lookup table              │                    │
│         └──────────────────────┬────────────────────────┘                    │
│                                │  grasp pose                                │
│                                ▼                                            │
│         ┌──────────────────────────────────────────────┐                    │
│         │              ARM CONTROLLER                   │                    │
│         │  Input:  pick pose + place pose + grasp pose  │                    │
│         │  Output: joint trajectories via MoveIt 2      │                    │
│         │                                               │                    │
│         │  Sequence: approach → grasp → lift → transit  │                    │
│         │            → lower → release → retract        │                    │
│         └──────────────────────┬────────────────────────┘                    │
│                                │                                            │
│                                ▼                                            │
│         ┌──────────────────────────────────────────────┐                    │
│         │              PERCEPTION                       │                    │
│         │  Board state verification after each move     │                    │
│         │  Point cloud segmentation for GPD             │                    │
│         └──────────────────────────────────────────────┘                    │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────┐            │
│  │                    GAME COORDINATOR                          │            │
│  │  Behavior tree / FSM that orchestrates the full game loop    │            │
│  │                                                              │            │
│  │  Loop:                                                       │            │
│  │    1. Read /board_state → determine whose turn               │            │
│  │    2. Call /stockfish/{color}/get_move → get UCI move         │            │
│  │    3. Publish to /apply_move → board_state validates & updates│            │
│  │    4. Move translator produces task queue                     │            │
│  │    5. Grasp planner generates gripper poses                   │            │
│  │    6. Arm controller executes pick-and-place                  │            │
│  │    7. Perception verifies physical board matches FEN          │            │
│  │    8. Repeat until /game_over                                 │            │
│  └─────────────────────────────────────────────────────────────┘            │
│                                                                             │
│  TF Tree:                                                                   │
│    world ──► chess_board ──► square_a1 ... square_h8  (64 frames)          │
│         │                ──► graveyard_white_0 ... graveyard_white_15       │
│         │                ──► graveyard_black_0 ... graveyard_black_15       │
│         ├──► white_panda/panda_link0                                        │
│         └──► black_panda/panda_link0                                        │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Modules

### 2.1 Board State Node

**Owner**: Teammate
**Package**: `chess_engine`

Maintains the authoritative game state using `python-chess`. All other nodes treat
the FEN published on `/board_state` as the single source of truth.

| Interface        | Type          | Direction | Format                        |
|------------------|---------------|-----------|-------------------------------|
| `/board_state`   | `String` pub  | out       | FEN string, 1 Hz + on change |
| `/apply_move`    | `String` sub  | in        | UCI string, e.g. `"e2e4"`    |
| `/reset_board`   | `String` sub  | in        | FEN string                    |
| `/game_over`     | `String` pub  | out       | termination reason            |

Key behaviors:
- Validates moves via `chess.Board.is_legal()` before applying
- Detects all game-over conditions: checkmate, stalemate, insufficient material,
  threefold repetition, fifty-move rule (via `board.is_game_over()`)
- Publishes updated FEN immediately after a successful move (not just on timer)

### 2.2 Stockfish Node

**Package**: `chess_engine`

One instance per player. Wraps `chess.engine.SimpleEngine` to interface with
Stockfish over UCI protocol.

| Interface                          | Type    | Direction | Format                     |
|------------------------------------|---------|-----------|----------------------------|
| `/stockfish/{color}/get_move`      | Service | in/out    | Request: FEN → Response: UCI |
| `/board_state`                     | Sub     | in        | FEN string                  |

Configuration (per-player `stockfish_params.yaml`):
- `depth`: search depth (default: 15)
- `time_limit`: seconds per move (default: 2.0)
- `elo`: optional ELO limit for asymmetric play
- `threads`: CPU threads allocated
- `hash_mb`: transposition table size

For best play quality, the node should send `position startpos moves e2e4 e7e5 ...`
(full move history) rather than `position fen ...` (snapshot only). This gives Stockfish
access to its transposition table across the full game.

### 2.3 Move Translator

**Owner**: Tristan
**Package**: `move_translator`

Bridges the symbolic chess game and the physical robot workspace. Subscribes to
the FEN and the UCI move, decomposes the move into an ordered list of pick-and-place
tasks with world-frame coordinates.

| Interface           | Type          | Direction | Format                              |
|---------------------|---------------|-----------|-------------------------------------|
| `/board_state`      | `String` sub  | in        | FEN string                          |
| `/apply_move`       | `String` sub  | in        | UCI string                          |
| `/pick_place_tasks` | `String` pub  | out       | JSON-encoded task queue (see below) |
| TF2 lookups         | tf_buffer     | in        | `square_XX` → `world` transforms   |

#### 2.3.1 Files

```
move_translator/
├── move_translator/
│   ├── __init__.py
│   ├── board_geometry.py         # square_to_world(), piece heights, graveyard
│   ├── move_decomposer.py       # FEN + UCI → ordered task list
│   ├── special_moves.py         # castling, en passant, promotion
│   └── move_translator_node.py  # ROS 2 node (subs, pubs, TF)
├── test/
│   ├── test_board_geometry.py
│   ├── test_decomposer.py
│   └── test_special_moves.py
├── config/
│   └── board_params.yaml
├── package.xml
└── setup.py
```

#### 2.3.2 Task Queue Format

Each task in the queue is a JSON object:

```json
{
  "task_type": "pick_and_place",
  "piece_char": "P",
  "piece_name": "white pawn",
  "pick_square": "e2",
  "place_square": "e4",
  "pick_pose": {"x": -0.075, "y": -0.125, "z": 0.789, "yaw": 0.0},
  "place_pose": {"x": -0.075, "y": -0.025, "z": 0.789, "yaw": 0.0},
  "grasp_height": 0.027
}
```

The task queue is an ordered JSON array. The arm controller executes tasks
sequentially. For a simple move, the queue has one task. For captures, castling,
and en passant, it has multiple.

#### 2.3.3 Move Decomposition Logic

**Standard move** (e.g. `e2e4`, pawn advance):

```
FEN lookup: e2 has 'P', e4 is empty
Queue:
  1. pick(e2) → place(e4)    [move the pawn]
```

**Capture** (e.g. `d1h5`, queen captures on h5):

```
FEN lookup: d1 has 'Q', h5 has 'p' (black pawn)
Queue:
  1. pick(h5) → place(graveyard)   [remove captured piece first]
  2. pick(d1) → place(h5)          [move attacking piece]
```

The captured piece must be removed before the attacking piece is placed.
The graveyard allocator tracks the next free slot.

**Kingside castling** (e.g. `e1g1` for white):

```
FEN lookup: e1 has 'K', h1 has 'R'
python-chess flags: move.is_castling() == True
Queue:
  1. pick(e1) → place(g1)    [move king]
  2. pick(h1) → place(f1)    [move rook]
```

**Queenside castling** (e.g. `e1c1` for white):

```
Queue:
  1. pick(e1) → place(c1)    [move king]
  2. pick(a1) → place(d1)    [move rook]
```

**En passant** (e.g. `d5c6` when black pawn on c5 just double-advanced):

```
FEN lookup: d5 has 'P', c6 is empty, c5 has 'p'
python-chess flags: move.is_en_passant() == True
Queue:
  1. pick(c5) → place(graveyard)   [remove captured pawn — NOT on c6]
  2. pick(d5) → place(c6)          [move attacking pawn]
```

The captured pawn is on c5 (same rank as attacker), not c6 (the target square).
This is the only move in chess where the captured piece isn't on the target square.

**Promotion** (e.g. `e7e8q`, pawn promotes to queen):

```
FEN lookup: e7 has 'P'
python-chess flags: move.promotion == chess.QUEEN
Queue:
  1. pick(e7) → place(graveyard)          [remove the pawn]
  2. pick(reserve_queen) → place(e8)      [place the promoted piece]
```

Promotion requires a reserve piece. Options:
- Keep a set of reserve pieces off-board at known positions
- Use a previously captured piece (if available)
- Skip the physical swap if all pieces are cylinders (just move the pawn)

**Promotion with capture** (e.g. `d7c8q`, pawn captures and promotes):

```
Queue:
  1. pick(c8) → place(graveyard)          [remove captured piece]
  2. pick(d7) → place(graveyard)          [remove the promoting pawn]
  3. pick(reserve_queen) → place(c8)      [place the promoted piece]
```

#### 2.3.4 Determining the Active Arm

The FEN string encodes whose turn it is (`w` or `b` after the position).
However, the move translator receives the move *after* it has been applied
to the board, so the turn indicator has already flipped. Two approaches:

**Option A**: Read the FEN *before* the move is applied (cache the previous FEN).
If the previous FEN says `w`, White just moved.

**Option B**: Infer from the piece character. If the piece on the source square
(in the pre-move FEN) is uppercase, it's White's arm; lowercase is Black's arm.

Option B is simpler and doesn't require synchronization between the move
subscriber and the board state subscriber. The move translator must subscribe
to `/board_state` and cache the FEN before subscribing to `/apply_move`.

### 2.4 Board Localization

**Owner**: Tristan (shared with world scaffold)
**Package**: `chess_robot_world` (TF broadcaster) + `move_translator` (lookup logic)

#### 2.4.1 Coordinate Convention

All coordinates are in the world frame (meters).

```
                    +Y  (Black's side)
                     │
          h8 ────────┼──────── a8
          │  rank 8  │         │
          │          │         │
          │     (0,0,0.762)    │      ← board center
          │          │         │
          │  rank 1  │         │
          h1 ────────┼──────── a1
                     │
                    -Y  (White's side)

               -X ←──┼──→ +X
```

| Reference point     | World coordinates          |
|---------------------|----------------------------|
| Board center        | (0, 0, 0.762)              |
| a1 square center    | (-0.175, -0.175, 0.762)    |
| h8 square center    | (+0.175, +0.175, 0.762)    |
| White Panda base    | (0, -0.45, 0.75)           |
| Black Panda base    | (0, +0.45, 0.75)           |
| Overhead camera     | (0, 0, 1.80)               |

Square size: 0.05 m (5 cm)

Conversion formula:

```
file_idx = ord(square[0]) - ord('a')    # 0 (a-file) to 7 (h-file)
rank_idx = int(square[1]) - 1           # 0 (rank 1) to 7 (rank 8)

x = -0.175 + file_idx * 0.05
y = -0.175 + rank_idx * 0.05
z = 0.762  (board surface)
```

#### 2.4.2 TF Frames

The `board_tf_broadcaster` node publishes 96 static frames:

- 64 board squares: `square_a1` through `square_h8`, relative to `chess_board`
- 32 graveyard slots: `graveyard_white_0..15` and `graveyard_black_0..15`

The `chess_board` frame is published relative to `world` at (0, 0, 0.762)
by the launch file's static transform publisher.

To get a square's world pose from any node:

```python
transform = tf_buffer.lookup_transform("world", "square_e4", rclpy.time.Time())
```

#### 2.4.3 Piece Heights

Grasp Z = board surface + piece height × 0.6 (grasp at 60% height,
above center of mass, below decorative top).

| Piece  | FEN char | Height (m) | Grasp Z (m) | Collision radius (m) |
|--------|----------|------------|-------------|----------------------|
| Pawn   | P / p    | 0.045      | 0.789       | 0.012                |
| Rook   | R / r    | 0.055      | 0.795       | 0.014                |
| Knight | N / n    | 0.060      | 0.798       | 0.015                |
| Bishop | B / b    | 0.065      | 0.801       | 0.013                |
| Queen  | Q / q    | 0.080      | 0.810       | 0.015                |
| King   | K / k    | 0.095      | 0.819       | 0.015                |

### 2.5 Grasp Planner

**Package**: `grasp_planner`

| Interface                 | Type    | Direction | Format                            |
|---------------------------|---------|-----------|-----------------------------------|
| `/grasp_planner/plan`     | Action  | in/out    | target pose + piece type → grasp  |
| `/overhead_camera/points` | Sub     | in        | PointCloud2 (for GPD)             |

Two modes:

**GPD mode**: Crops the point cloud around the target square (±3 cm box),
runs GPD to generate grasp candidates, filters for top-down approach angle
(±15° from vertical), ranks by antipodal quality score.

**Lookup mode** (recommended for controlled environments): Precomputed
top-down grasp per piece type. The gripper approaches vertically, centered
on the square, with finger separation matched to piece diameter. This is
faster, more reliable, and doesn't require a depth camera.

### 2.6 Arm Controller

**Package**: `arm_controller`

One instance per arm, namespaced as `white_panda` and `black_panda`.

| Interface                             | Type    | Direction | Format                  |
|---------------------------------------|---------|-----------|-------------------------|
| `/{ns}/pick_place`                    | Action  | in/out    | pick pose + place pose  |
| `/{ns}/arm_controller`               | JTC     | out       | joint trajectories      |
| `/{ns}/hand_controller`              | Gripper | out       | open/close              |

Pick-and-place sequence (7 waypoints):

```
1. APPROACH     → move above pick pose, +10 cm Z offset
2. DESCEND      → lower to grasp height
3. GRASP        → close gripper
4. LIFT         → raise +10 cm
5. TRANSIT      → move above place pose, +10 cm Z offset
6. LOWER        → descend to place height
7. RELEASE      → open gripper
8. RETRACT      → raise +10 cm, return to home pose
```

Collision avoidance: both arms exist in a shared MoveIt planning scene.
Each arm's joint states are published as collision objects for the other arm.
Strict turn-taking (only one arm moves at a time) avoids the need for
concurrent motion planning.

### 2.7 Perception

**Package**: `perception`

| Interface                     | Type    | Direction | Format           |
|-------------------------------|---------|-----------|------------------|
| `/overhead_camera/image`      | Sub     | in        | RGB image        |
| `/overhead_camera/depth`      | Sub     | in        | depth image      |
| `/overhead_camera/points`     | Sub     | in        | PointCloud2      |
| `/perception/verify_board`    | Service | in/out    | FEN → bool match |

Responsibilities:
- Point cloud segmentation: isolate board from table, segment individual pieces
- Board state verification: after each physical move, compare vision-detected
  board state against expected FEN
- Failure detection: knocked pieces, failed grasps, pieces out of position
- Feed cropped point clouds to GPD for grasp planning

### 2.8 Game Coordinator

**Package**: `game_coordinator`

Top-level orchestrator implemented as a behavior tree (`py_trees`) or FSM.

```
Root (Sequence)
├── WaitForGameStart
└── GameLoop (RepeatUntilFailure)
    ├── ReadBoardState           → cache FEN
    ├── CheckGameOver            → success if game continues
    ├── DetermineActivePlayer    → set white/black context
    ├── RequestMove              → call Stockfish service
    ├── ApplyMove                → publish to /apply_move
    ├── WaitForBoardUpdate       → confirm FEN changed
    ├── TranslateMove            → wait for /pick_place_tasks
    ├── ExecuteTasks (Sequence)
    │   ├── PlanGrasp            → call grasp planner
    │   ├── ExecutePickPlace     → call arm controller action
    │   └── VerifyBoard          → call perception verification
    └── HandleErrors (Fallback)
        ├── RetryGrasp           → re-plan and re-execute
        ├── ManualIntervention   → pause and alert
        └── AbortGame            → publish game over
```

---

## 3. ROS 2 Topic / Service Map

```
/board_state              String          board_state_node → all nodes
/apply_move               String          coordinator → board_state_node
/reset_board              String          external → board_state_node
/game_over                String          board_state_node → coordinator

/stockfish/white/get_move Service         coordinator → stockfish_white
/stockfish/black/get_move Service         coordinator → stockfish_black

/pick_place_tasks         String (JSON)   move_translator → coordinator
/white_panda/pick_place   Action          coordinator → arm_controller_white
/black_panda/pick_place   Action          coordinator → arm_controller_black

/grasp_planner/plan       Action          coordinator → grasp_planner

/overhead_camera/image    Image           gz_bridge → perception
/overhead_camera/depth    Image           gz_bridge → perception
/overhead_camera/points   PointCloud2     gz_bridge → perception, grasp_planner
/overhead_camera/info     CameraInfo      gz_bridge → perception

/perception/verify_board  Service         coordinator → perception

/clock                    Clock           gz_bridge → all nodes (sim time)
```

---

## 4. Data Flow for One Turn

```
Step  Source               Action                        Destination
─────────────────────────────────────────────────────────────────────
 1    coordinator          reads /board_state            FEN cached
 2    coordinator          calls /stockfish/white        gets UCI "e2e4"
 3    coordinator          publishes "e2e4"              /apply_move
 4    board_state_node     validates, pushes move        /board_state updated
 5    move_translator      reads pre-move FEN + "e2e4"  decomposes
 6    move_translator      publishes task queue          /pick_place_tasks
 7    coordinator          reads task queue              iterates tasks
 8    grasp_planner        plans grasp for e2 pawn       returns grasp pose
 9    arm_controller       pick(e2) → place(e4)         arm executes
10    perception           verifies board matches FEN    returns OK
11    coordinator          turn complete                 loop to step 1
```

For a **capture** (e.g. `d1h5` taking a pawn), step 6 produces two tasks
and steps 8-9 execute twice: first removing the captured piece to the
graveyard, then moving the attacking piece.

---

## 5. Package Dependency Graph

```
chess_robot_world          (Gazebo world, models, TF broadcaster)
    │
    ├── chess_engine        (board_state_node, stockfish_node)
    │       │
    │       └── python-chess, stockfish
    │
    ├── move_translator     (Tristan: FEN + UCI → pick/place tasks)
    │       │
    │       ├── python-chess
    │       ├── tf2_ros
    │       └── chess_robot_world  (board geometry constants)
    │
    ├── grasp_planner       (GPD or lookup table)
    │       │
    │       └── pcl_ros, gpd
    │
    ├── arm_controller      (MoveIt 2 pick-and-place)
    │       │
    │       ├── moveit2
    │       └── franka_ros2
    │
    ├── perception          (point cloud + board verification)
    │       │
    │       └── pcl_ros, cv_bridge
    │
    └── game_coordinator    (behavior tree orchestrator)
            │
            └── py_trees
```

---

## 6. Coordinate Reference

### 6.1 Board Layout (top-down view, +Y up on page)

```
     a    b    c    d    e    f    g    h
   ┌────┬────┬────┬────┬────┬────┬────┬────┐
8  │ r  │ n  │ b  │ q  │ k  │ b  │ n  │ r  │  ← Black (Panda at y=+0.45)
   ├────┼────┼────┼────┼────┼────┼────┼────┤
7  │ p  │ p  │ p  │ p  │ p  │ p  │ p  │ p  │
   ├────┼────┼────┼────┼────┼────┼────┼────┤
6  │    │    │    │    │    │    │    │    │
   ├────┼────┼────┼────┼────┼────┼────┼────┤
5  │    │    │    │    │    │    │    │    │
   ├────┼────┼────┼────┼────┼────┼────┼────┤
4  │    │    │    │    │    │    │    │    │
   ├────┼────┼────┼────┼────┼────┼────┼────┤
3  │    │    │    │    │    │    │    │    │
   ├────┼────┼────┼────┼────┼────┼────┼────┤
2  │ P  │ P  │ P  │ P  │ P  │ P  │ P  │ P  │
   ├────┼────┼────┼────┼────┼────┼────┼────┤
1  │ R  │ N  │ B  │ Q  │ K  │ B  │ N  │ R  │  ← White (Panda at y=-0.45)
   └────┴────┴────┴────┴────┴────┴────┴────┘

        -X ←─── world ───→ +X
```

### 6.2 Reach Analysis

```
Panda arm max reach: 0.855 m

White Panda base: (0, -0.45, 0.75)
  Distance to a1: sqrt(0.175² + 0.275²) = 0.326 m  ✓
  Distance to h8: sqrt(0.175² + 0.625²) = 0.649 m  ✓
  Distance to a8: sqrt(0.175² + 0.625²) = 0.649 m  ✓
  Distance to h1: sqrt(0.175² + 0.275²) = 0.326 m  ✓
  Worst case (corner): 0.649 m < 0.855 m  ✓

Black Panda base: (0, +0.45, 0.75)
  Symmetric — same reach analysis.

Both arms can reach all 64 squares with margin.
Overlap zone: ranks 3–6 (both arms can reach these squares).
```

### 6.3 World-Frame Coordinate Table (selected squares)

| Square | File idx | Rank idx | World X  | World Y  | World Z |
|--------|----------|----------|----------|----------|---------|
| a1     | 0        | 0        | -0.175   | -0.175   | 0.762   |
| e1     | 4        | 0        | +0.025   | -0.175   | 0.762   |
| h1     | 7        | 0        | +0.175   | -0.175   | 0.762   |
| d4     | 3        | 3        | -0.025   | -0.025   | 0.762   |
| e4     | 4        | 3        | +0.025   | -0.025   | 0.762   |
| a8     | 0        | 7        | -0.175   | +0.175   | 0.762   |
| e8     | 4        | 7        | +0.025   | +0.175   | 0.762   |
| h8     | 7        | 7        | +0.175   | +0.175   | 0.762   |

---

## 7. Failure Modes and Recovery

| Failure                    | Detection                          | Recovery                                  |
|----------------------------|------------------------------------|-------------------------------------------|
| Grasp misses piece         | Force/torque sensor or vision      | Re-plan grasp, retry (max 3 attempts)     |
| Piece knocked over         | Perception verification fails      | Vision-guided pickup from fallen pose     |
| Piece dropped in transit   | Gripper force feedback             | Search nearby area, re-grasp              |
| Collision during motion    | MoveIt planning failure            | Re-plan with updated collision scene      |
| FEN mismatch after move    | Perception vs expected FEN         | Pause, alert operator, manual correction  |
| Stockfish timeout          | Service call timeout               | Retry with increased time limit           |
| Arm unreachable pose       | MoveIt IK failure                  | Use other arm if square is in overlap zone|

---

## 8. Configuration Files

| File                              | Contents                                    |
|-----------------------------------|---------------------------------------------|
| `config/robot_positions.yaml`     | Arm base poses, camera position, board pose |
| `config/stockfish_params.yaml`    | Per-player engine depth, ELO, time limits   |
| `models/chess_pieces/configs/piece_params.yaml` | Piece heights, collision shapes, mass, inertia |
| `config/board_params.yaml`        | Square size, a1 offset, surface Z (re-exported from piece_params) |

---

## 9. Development Workflow

### 9.1 Testing Without Hardware

```bash
# 1. Launch Gazebo world with both arms and pieces
ros2 launch chess_robot_world chess_sim.launch.py

# 2. Manually trigger a move to test the pipeline
ros2 topic pub --once /apply_move std_msgs/String "data: 'e2e4'"

# 3. Verify TF frames
ros2 run tf2_tools view_frames    # generates frames.pdf

# 4. Check board state
ros2 topic echo /board_state
```

### 9.2 Unit Testing the Move Translator (no ROS required)

```python
import chess
from move_translator.move_decomposer import decompose_move
from move_translator.board_geometry import GraveyardAllocator

board = chess.Board()
move = chess.Move.from_uci("e2e4")
tasks = decompose_move(board, move, GraveyardAllocator())

assert len(tasks) == 1
assert tasks[0].pick_square == "e2"
assert tasks[0].place_square == "e4"
assert tasks[0].piece_char == "P"
```

### 9.3 Testing Special Moves

```python
# En passant
board = chess.Board("rnbqkbnr/pppp1ppp/8/4pP2/8/8/PPPPP1PP/RNBQKBNR w KQkq e6 0 3")
move = chess.Move.from_uci("f5e6")
tasks = decompose_move(board, move, GraveyardAllocator())
assert len(tasks) == 2
assert tasks[0].pick_square == "e5"        # captured pawn (not e6!)
assert tasks[0].task_type == "remove_piece"
assert tasks[1].pick_square == "f5"
assert tasks[1].place_square == "e6"

# Kingside castling
board = chess.Board("r3k2r/pppppppp/8/8/8/8/PPPPPPPP/R3K2R w KQkq - 0 1")
move = chess.Move.from_uci("e1g1")
tasks = decompose_move(board, move, GraveyardAllocator())
assert len(tasks) == 2
assert tasks[0].pick_square == "e1"   # king
assert tasks[0].place_square == "g1"
assert tasks[1].pick_square == "h1"   # rook
assert tasks[1].place_square == "f1"
```