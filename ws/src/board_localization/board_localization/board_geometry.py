"""
board_geometry.py
=================
Single source of truth for board coordinate math in the board_localization
package.

All constants here match the architecture specification exactly:
  - Board surface Z   : 0.762 m
  - a1 world centre   : (-0.175, -0.175, 0.762)
  - h8 world centre   : (+0.175, +0.175, 0.762)
  - Square size        : 0.05 m
  - +Y → Black's side, -Y → White's side

This module is intentionally ROS-free so every function can be unit-tested
without a running ROS context.

The board_localization package owns these canonical constants.  The
move_translator and grasp_planner packages carry their own copies only for
import-independence; any divergence should be fixed here first.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterator, List, Tuple


# ---------------------------------------------------------------------------
# Primary constants (from architecture §6.1 and §2.4)
# ---------------------------------------------------------------------------

BOARD_SURFACE_Z: float = 0.762      # world Z of top face of the board (m)
SQUARE_SIZE: float     = 0.05       # edge length of one square (m)
A1_X: float            = -0.175     # world X of a1 square centre (m)
A1_Y: float            = -0.175     # world Y of a1 square centre (m)
BOARD_HALF: float      = 0.175      # half-width of the board (4 × 0.05 - 0.025 from centre)

# Frame name strings used throughout the TF tree
WORLD_FRAME   = "world"
BOARD_FRAME   = "chess_board"       # static offset = (0, 0, BOARD_SURFACE_Z) from world

# Arm base positions (world frame, metres)
WHITE_ARM_BASE: Tuple[float, float, float] = (0.0, -0.45, 0.75)
BLACK_ARM_BASE: Tuple[float, float, float] = (0.0, +0.45, 0.75)

# Overhead camera position (world frame, metres)
CAMERA_POSITION: Tuple[float, float, float] = (0.0, 0.0, 1.80)

# Graveyard layout
GRAVEYARD_WHITE_X: float = -0.275   # left of a-file (captures of white pieces)
GRAVEYARD_BLACK_X: float = +0.275   # right of h-file (captures of black pieces)
GRAVEYARD_START_Y: float = -0.175   # aligned with rank 1
GRAVEYARD_SPACING: float = 0.050    # same as SQUARE_SIZE

# Reserve piece shelf (for pawn promotion)
RESERVE_WHITE_X: float       = +0.275
RESERVE_WHITE_Y_START: float = +0.175
RESERVE_BLACK_X: float       = -0.275
RESERVE_BLACK_Y_START: float = -0.175


# ---------------------------------------------------------------------------
# Pose dataclass
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class BoardPose:
    """
    6-DOF pose in the world frame (metres + quaternion).

    For board squares the orientation is always identity (flat on the board).
    The quaternion is stored as (x, y, z, w) — ROS convention.
    """
    x: float
    y: float
    z: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0

    def position(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def orientation(self) -> Tuple[float, float, float, float]:
        return (self.qx, self.qy, self.qz, self.qw)

    def to_dict(self) -> dict:
        return {
            "position": {"x": self.x, "y": self.y, "z": self.z},
            "orientation": {"x": self.qx, "y": self.qy,
                            "z": self.qz, "w": self.qw},
        }


# ---------------------------------------------------------------------------
# Square ↔ coordinate conversions
# ---------------------------------------------------------------------------

def square_to_board_pose(square: str) -> BoardPose:
    """
    Convert an algebraic square name to a world-frame BoardPose.

    The pose sits at the centre of the square on the board surface.
    Orientation is identity (no rotation — gripper approaches from above).

    Parameters
    ----------
    square : two-character algebraic notation, e.g. "e4", "a1", "h8".
             Case-insensitive.

    Returns
    -------
    BoardPose in the world frame.

    Raises
    ------
    ValueError for malformed input.
    """
    square = square.lower()
    _validate_square(square)

    file_idx = ord(square[0]) - ord("a")   # 0 (a) … 7 (h)
    rank_idx = int(square[1]) - 1          # 0 (rank 1) … 7 (rank 8)

    x = round(A1_X + file_idx * SQUARE_SIZE, 6)
    y = round(A1_Y + rank_idx * SQUARE_SIZE, 6)

    return BoardPose(x=x, y=y, z=BOARD_SURFACE_Z)


def board_pose_to_square(x: float, y: float, tolerance_m: float = 0.02) -> str:
    """
    Reverse map: world (x, y) → nearest algebraic square.

    Parameters
    ----------
    x, y       : world frame coordinates (metres)
    tolerance_m: maximum allowed distance from a square centre to be
                 considered a match (default 2 cm = 40 % of square size)

    Returns
    -------
    Algebraic square name, e.g. "e4".

    Raises
    ------
    ValueError if the point is outside the board + tolerance.
    """
    file_float = (x - A1_X) / SQUARE_SIZE
    rank_float = (y - A1_Y) / SQUARE_SIZE
    file_idx = round(file_float)
    rank_idx = round(rank_float)

    if not (0 <= file_idx <= 7 and 0 <= rank_idx <= 7):
        raise ValueError(
            f"World position ({x:.4f}, {y:.4f}) is outside the board."
        )

    sq = f"{chr(ord('a') + file_idx)}{rank_idx + 1}"
    expected = square_to_board_pose(sq)
    dist = math.hypot(x - expected.x, y - expected.y)
    if dist > tolerance_m:
        raise ValueError(
            f"Position ({x:.4f}, {y:.4f}) deviates {dist*1000:.1f} mm from nearest "
            f"square {sq} (tolerance {tolerance_m*1000:.0f} mm)."
        )
    return sq


def square_to_tf_frame(square: str) -> str:
    """Return the TF frame name for a given square (e.g. 'e4' → 'square_e4')."""
    _validate_square(square.lower())
    return f"square_{square.lower()}"


def tf_frame_to_square(frame: str) -> str:
    """
    Parse a TF frame name back to an algebraic square.
    'square_e4' → 'e4'.

    Raises ValueError for non-square frame names.
    """
    prefix = "square_"
    if not frame.startswith(prefix):
        raise ValueError(f"'{frame}' is not a square frame (expected 'square_XX')")
    sq = frame[len(prefix):]
    _validate_square(sq)
    return sq


def all_squares() -> Iterator[str]:
    """Yield all 64 algebraic square names in file-major order (a1, a2, … h8)."""
    for file in "abcdefgh":
        for rank in "12345678":
            yield f"{file}{rank}"


def all_square_poses() -> Dict[str, BoardPose]:
    """Return a dict mapping every square name to its world-frame BoardPose."""
    return {sq: square_to_board_pose(sq) for sq in all_squares()}


# ---------------------------------------------------------------------------
# Graveyard helpers
# ---------------------------------------------------------------------------

def graveyard_slot_pose(color: str, slot_index: int) -> BoardPose:
    """
    World pose for the nth graveyard slot for captured pieces of the given colour.

    Parameters
    ----------
    color : "white" (white pieces captured by Black go here)
            "black" (black pieces captured by White go here)
    slot_index : 0-based slot number (0..15)

    Returns
    -------
    BoardPose at the graveyard slot position.
    """
    if color == "white":
        x = GRAVEYARD_WHITE_X
    elif color == "black":
        x = GRAVEYARD_BLACK_X
    else:
        raise ValueError(f"color must be 'white' or 'black', got '{color}'")

    if not (0 <= slot_index <= 15):
        raise ValueError(f"slot_index must be 0–15, got {slot_index}")

    y = round(GRAVEYARD_START_Y + slot_index * GRAVEYARD_SPACING, 6)
    return BoardPose(x=x, y=y, z=BOARD_SURFACE_Z)


def graveyard_tf_frame(color: str, slot_index: int) -> str:
    """Return the TF frame name for a graveyard slot, e.g. 'graveyard_white_3'."""
    return f"graveyard_{color}_{slot_index}"


def all_graveyard_poses() -> Dict[str, BoardPose]:
    """Return all 32 graveyard slot poses keyed by TF frame name."""
    poses: Dict[str, BoardPose] = {}
    for color in ("white", "black"):
        for i in range(16):
            frame = graveyard_tf_frame(color, i)
            poses[frame] = graveyard_slot_pose(color, i)
    return poses


# ---------------------------------------------------------------------------
# Reach analysis
# ---------------------------------------------------------------------------

def arm_reach_to_square(
    arm_base: Tuple[float, float, float],
    square: str,
) -> float:
    """
    Return the Euclidean 3D distance from an arm base to a square centre.

    Parameters
    ----------
    arm_base : (x, y, z) world coordinates of the arm's panda_link0
    square   : algebraic square name

    Returns
    -------
    Distance in metres.
    """
    p = square_to_board_pose(square)
    dx = p.x - arm_base[0]
    dy = p.y - arm_base[1]
    dz = p.z - arm_base[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def worst_case_reach(arm_base: Tuple[float, float, float]) -> Tuple[str, float]:
    """
    Find the hardest-to-reach square for an arm base position.

    Returns (square, distance_m) for the furthest square.
    """
    worst_sq = "a1"
    worst_d = 0.0
    for sq in all_squares():
        d = arm_reach_to_square(arm_base, sq)
        if d > worst_d:
            worst_d = d
            worst_sq = sq
    return worst_sq, worst_d


PANDA_MAX_REACH: float = 0.855   # metres (from architecture §6.2)


def validate_arm_reach() -> List[str]:
    """
    Verify both arms can reach all 64 squares.

    Returns a list of warning strings for any square that violates the
    reach limit (should be empty for the current geometry).
    """
    warnings: List[str] = []
    for arm_name, base in [("white", WHITE_ARM_BASE), ("black", BLACK_ARM_BASE)]:
        for sq in all_squares():
            d = arm_reach_to_square(base, sq)
            if d > PANDA_MAX_REACH:
                warnings.append(
                    f"{arm_name} arm cannot reach {sq}: "
                    f"{d:.4f} m > {PANDA_MAX_REACH} m limit"
                )
    return warnings


# ---------------------------------------------------------------------------
# Calibration math helpers
# ---------------------------------------------------------------------------

def estimate_board_origin(
    a1_measured: Tuple[float, float, float],
    h8_measured: Tuple[float, float, float],
) -> Tuple[float, float, float, float]:
    """
    Estimate the board's world-frame origin and yaw from two measured corner
    positions (e.g. from ArUco marker detection).

    Parameters
    ----------
    a1_measured : measured world (x, y, z) of the a1 corner
    h8_measured : measured world (x, y, z) of the h8 corner

    Returns
    -------
    (origin_x, origin_y, origin_z, yaw_rad) where the origin is the board
    centre and yaw is the rotation around world Z relative to nominal.
    """
    # Board centre = midpoint of a1..h8 diagonal
    cx = (a1_measured[0] + h8_measured[0]) / 2.0
    cy = (a1_measured[1] + h8_measured[1]) / 2.0
    cz = (a1_measured[2] + h8_measured[2]) / 2.0

    # Nominal a1→h8 direction: (+1, +1, 0) normalised = (cos45°, sin45°, 0)
    dx = h8_measured[0] - a1_measured[0]
    dy = h8_measured[1] - a1_measured[1]
    measured_angle = math.atan2(dy, dx)
    nominal_angle  = math.atan2(1.0, 1.0)   # 45°
    yaw = measured_angle - nominal_angle

    return (round(cx, 6), round(cy, 6), round(cz, 6), round(yaw, 6))


def apply_board_transform(
    square: str,
    board_origin: Tuple[float, float, float],
    board_yaw: float,
) -> BoardPose:
    """
    Compute the world pose of a square given a calibrated (non-nominal) board
    origin and yaw.

    Use this when the board has been placed at an arbitrary position/orientation
    (e.g., physical deployment with AR marker calibration).  In Gazebo, the
    board is fixed at the nominal pose so this is not needed.

    Parameters
    ----------
    square       : algebraic square name
    board_origin : (x, y, z) world position of the board centre
    board_yaw    : rotation of the board around world Z (radians)

    Returns
    -------
    BoardPose of the square in the world frame.
    """
    _validate_square(square.lower())
    # Nominal offset of the square from board centre
    nominal = square_to_board_pose(square)
    # Nominal board centre
    nominal_cx = (A1_X + (A1_X + 7 * SQUARE_SIZE)) / 2.0   # 0.0
    nominal_cy = (A1_Y + (A1_Y + 7 * SQUARE_SIZE)) / 2.0   # 0.0
    dx = nominal.x - nominal_cx
    dy = nominal.y - nominal_cy

    # Rotate offset by board_yaw
    cos_y = math.cos(board_yaw)
    sin_y = math.sin(board_yaw)
    rx = dx * cos_y - dy * sin_y
    ry = dx * sin_y + dy * cos_y

    # Build quaternion for yaw rotation
    sz = math.sin(board_yaw / 2)
    cz = math.cos(board_yaw / 2)

    return BoardPose(
        x=round(board_origin[0] + rx, 6),
        y=round(board_origin[1] + ry, 6),
        z=round(board_origin[2], 6),
        qx=0.0, qy=0.0, qz=round(sz, 8), qw=round(cz, 8),
    )


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _validate_square(square: str) -> None:
    """Raise ValueError if the square string is not valid algebraic notation."""
    if len(square) != 2:
        raise ValueError(f"Square must be two characters, got '{square!r}'")
    if square[0] not in "abcdefgh":
        raise ValueError(f"Invalid file '{square[0]}' in square '{square}'")
    if square[1] not in "12345678":
        raise ValueError(f"Invalid rank '{square[1]}' in square '{square}'")