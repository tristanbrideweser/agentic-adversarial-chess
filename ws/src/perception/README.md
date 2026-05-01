# perception

Board state and piece perception for the chess robots.

Currently implements `board_verifier` — publishes an 8×8 occupancy grid of the
board derived from the overhead depth camera.

## Run

The node depends on the world from `chess_robot_description` and the
Gazebo↔ROS bridge. The bringup launch starts everything together:

```bash
ros2 launch chess_robot_bringup sim.launch.py
```

To run the node alone against an already-running sim:

```bash
ros2 run perception board_verifier
```

Wait for `[board_verifier] 64 ROIs precomputed …` in the log — that's the
ready signal.

The world spawns empty, so `/chess/occupancy` publishes all zeros until
something is on the board. Populate the standard starting position with:

```bash
ros2 run chess_robot_description spawn_pieces.py
```

Then grab a single occupancy snapshot:

```bash
ros2 topic echo /chess/occupancy --once
```

The reply is a `std_msgs/UInt8MultiArray` whose `data` field is a 64-byte
array of `0`/`1` values — `data[rank_idx * 8 + file_idx]` is `1` when the
square is occupied. For the standard starting position, you'll see ones on
ranks 1, 2, 7, 8 and zeros on ranks 3-6. Downstream nodes consume the same
topic and index the array directly (see [Topics](#topics) for the full
layout convention).

## Topics

| Direction | Topic | Type | Rate |
|---|---|---|---|
| sub | `/overhead_camera/depth` | `sensor_msgs/Image` (32FC1) | 30 Hz |
| pub | `/chess/occupancy` | `std_msgs/UInt8MultiArray` | 5 Hz |

`data[rank_idx * 8 + file_idx]` is `1` if the square is occupied, `0` otherwise.
`rank_idx = 0` is rank 1; `file_idx = 0` is file a. So `data[0] = a1`,
`data[7] = h1`, `data[56] = a8`, `data[63] = h8`. The message's `layout.dim`
is set to `[(rank, 8, 64), (file, 8, 8)]` for consumers that prefer indexing
through that.

## How it works

On startup, projects each of the 64 squares' world centers to depth-image
pixels using a pinhole model derived from the camera's hfov, then stores the
ROIs (shrunk by `roi_shrink` to avoid edge bleed). The empty-board depth is
the constant `reference_depth = camera_z − board_top_z` (≈ 0.42 m for the
default world). Per 5 Hz tick: a square is declared *occupied* if more than
`min_occupied_fraction` of its ROI pixels read a depth less than
`reference_depth − empty_depth_threshold` — i.e. something is sticking up at
least `empty_depth_threshold` above the board surface. This is robust to small
pieces — pawns only cover ~40% of a square from above.

## Configuration

- `config/camera_params.yaml` — camera pose, hfov, image dims, depth/occupancy
  topic names, publish rate, and the three detection knobs
  (`empty_depth_threshold`, `roi_shrink`, `min_occupied_fraction`).
- `../chess_robot_description/config/board_params.yaml` — board center, size,
  top z, square size.

The camera pose and hfov in `camera_params.yaml` must match the SDF in
`chess_robot_description/worlds/chess_table.sdf` — they're consumed
independently, not wired through TF.

## Quick debug

Pretty-print the current board:

```bash
ros2 topic echo /chess/occupancy --once --field data | python3 -c '
import sys, re
d = re.findall(r"[01]", sys.stdin.read())
print("    a b c d e f g h")
for r in range(7, -1, -1):
    print(f"{r+1}   " + " ".join("X" if d[r*8+f] == "1" else "." for f in range(8)))
'
```
