"""
spawn_chess_pieces.launch.py
Spawns all 32 chess pieces into Gazebo at their standard starting positions.

Board params (from board_params.yaml):
  center_xy: [0.0, 0.0]
  square_size: 0.05625
  board top_z: 0.762  (from get_square_pose service)
  piece half-height: 0.02  (cylinder length=0.04)
  spawn_z: 0.762 + 0.02 = 0.782

Square coordinate convention (matching board_localization):
  Files a-h map to x axis, ranks 1-8 map to y axis.
  a1 is at the white-side corner.

  x = center_x + (file_index - 3.5) * square_size
  y = center_y + (rank_index - 3.5) * square_size
  where file a=0, b=1, ..., h=7
        rank 1=0, 2=1, ..., 8=7
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.substitutions import FindPackageShare
import os


# Board geometry
CENTER_X = 0.0
CENTER_Y = 0.0
SQUARE = 0.05625
SPAWN_Z = 0.782  # board top (0.762) + piece half-height (0.02)


def sq(file_char, rank_int):
    """Return (x, y) world coordinates for a square e.g. sq('e', 2)."""
    file_idx = ord(file_char.lower()) - ord('a')  # a=0 .. h=7
    rank_idx = rank_int - 1                         # 1=0 .. 8=7
    x = CENTER_X + (file_idx - 3.5) * SQUARE
    y = CENTER_Y + (rank_idx - 3.5) * SQUARE
    return x, y


def spawn(name, model, file_char, rank_int, delay):
    """Return a TimerAction that spawns a named piece at a square."""
    x, y = sq(file_char, rank_int)
    pose = f"{x} {y} {SPAWN_Z} 0 0 0"
    return TimerAction(period=float(delay), actions=[
        ExecuteProcess(
            cmd=[
                "gz", "service",
                "-s", "/world/chess_world/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "5000",
                "--req",
                f'sdf_filename: "{model}", '
                f'name: "{name}", '
                f'pose: {{position: {{x: {x}, y: {y}, z: {SPAWN_Z}}}, '
                f'orientation: {{w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}}',
            ],
            output="screen",
        )
    ])


def generate_launch_description():
    pkg = FindPackageShare("chess_robot_description").find("chess_robot_description")
    models_dir = os.path.join(pkg, "models")

    def model_path(model_name):
        return os.path.join(models_dir, model_name, "model.sdf")

    # Build all 32 spawn actions with small staggered delays to avoid
    # overwhelming the Gazebo service endpoint.
    actions = []
    t = 2.0  # start delay — give Gazebo time to load the world
    dt = 0.3  # delay between spawns

    # ------------------------------------------------------------------
    # White pieces (rank 1 = back rank, rank 2 = pawns)
    # ------------------------------------------------------------------
    back_rank_white = [
        ('a', 'rook_white'),
        ('b', 'knight_white'),
        ('c', 'bishop_white'),
        ('d', 'queen_white'),
        ('e', 'king_white'),
        ('f', 'bishop_white'),
        ('g', 'knight_white'),
        ('h', 'rook_white'),
    ]
    for file_char, model in back_rank_white:
        name = f"{model}_{file_char}1"
        actions.append(spawn(name, model_path(model), file_char, 1, t))
        t += dt

    for file_char in 'abcdefgh':
        name = f"pawn_white_{file_char}2"
        actions.append(spawn(name, model_path("pawn_white"), file_char, 2, t))
        t += dt

    # ------------------------------------------------------------------
    # Black pieces (rank 8 = back rank, rank 7 = pawns)
    # ------------------------------------------------------------------
    back_rank_black = [
        ('a', 'rook_black'),
        ('b', 'knight_black'),
        ('c', 'bishop_black'),
        ('d', 'queen_black'),
        ('e', 'king_black'),
        ('f', 'bishop_black'),
        ('g', 'knight_black'),
        ('h', 'rook_black'),
    ]
    for file_char, model in back_rank_black:
        name = f"{model}_{file_char}8"
        actions.append(spawn(name, model_path(model), file_char, 8, t))
        t += dt

    for file_char in 'abcdefgh':
        name = f"pawn_black_{file_char}7"
        actions.append(spawn(name, model_path("pawn_black"), file_char, 7, t))
        t += dt

    return LaunchDescription(actions)