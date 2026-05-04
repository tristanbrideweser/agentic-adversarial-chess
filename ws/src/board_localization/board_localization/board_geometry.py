"""
board_geometry.py
=================
Single source of truth for board coordinate math in the board_localization
package.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterator, List, Tuple


BOARD_SURFACE_Z: float = 0.762
SQUARE_SIZE: float     = 0.05625
A1_X: float            = -0.196875
A1_Y: float            = -0.196875
BOARD_HALF: float      = 0.225
WORLD_FRAME            = "world"
BOARD_FRAME            = "chess_board"
WHITE_ARM_BASE: Tuple[float, float, float] = (0.0, -0.45, 0.75)
BLACK_ARM_BASE: Tuple[float, float, float] = (0.0, +0.45, 0.75)
CAMERA_POSITION: Tuple[float, float, float] = (0.0, 0.0, 1.80)
GRAVEYARD_WHITE_X: float = -0.275
GRAVEYARD_BLACK_X: float = +0.275
GRAVEYARD_START_Y: float = -0.175
GRAVEYARD_SPACING: float = 0.050
RESERVE_WHITE_X: float       = +0.275
RESERVE_WHITE_Y_START: float = +0.175
RESERVE_BLACK_X: float       = -0.275
RESERVE_BLACK_Y_START: float = -0.175
PANDA_MAX_REACH: float = 0.855


@dataclass(frozen=True)
class BoardPose:
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
            "orientation": {"x": self.qx, "y": self.qy, "z": self.qz, "w": self.qw},
        }


def _validate_square(square: str) -> None:
    if len(square) != 2:
        raise ValueError(f"Square must be two characters, got {square!r}")
    if square[0] not in "abcdefgh":
        raise ValueError(f"Invalid file '{square[0]}' in square '{square}'")
    if square[1] not in "12345678":
        raise ValueError(f"Invalid rank '{square[1]}' in square '{square}'")


def square_to_board_pose(square: str) -> BoardPose:
    square = square.lower()
    _validate_square(square)
    file_idx = ord(square[0]) - ord("a")
    rank_idx = int(square[1]) - 1
    x = round(A1_X + rank_idx * SQUARE_SIZE, 6)
    y = round(A1_Y + file_idx * SQUARE_SIZE, 6)
    return BoardPose(x=x, y=y, z=BOARD_SURFACE_Z)


def board_pose_to_square(x: float, y: float, tolerance_m: float = 0.02) -> str:
    file_float = (x - A1_X) / SQUARE_SIZE
    rank_float = (y - A1_Y) / SQUARE_SIZE
    file_idx = round(file_float)
    rank_idx = round(rank_float)
    if not (0 <= file_idx <= 7 and 0 <= rank_idx <= 7):
        raise ValueError(f"World position ({x:.4f}, {y:.4f}) is outside the board.")
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
    _validate_square(square.lower())
    return f"square_{square.lower()}"


def tf_frame_to_square(frame: str) -> str:
    prefix = "square_"
    if not frame.startswith(prefix):
        raise ValueError(f"'{frame}' is not a square frame (expected 'square_XX')")
    sq = frame[len(prefix):]
    _validate_square(sq)
    return sq


def all_squares() -> Iterator[str]:
    for file in "abcdefgh":
        for rank in "12345678":
            yield f"{file}{rank}"


def all_square_poses() -> Dict[str, BoardPose]:
    return {sq: square_to_board_pose(sq) for sq in all_squares()}


def graveyard_slot_pose(color: str, slot_index: int) -> BoardPose:
    if color == "white":
        x = GRAVEYARD_WHITE_X
    elif color == "black":
        x = GRAVEYARD_BLACK_X
    else:
        raise ValueError(f"color must be 'white' or 'black', got '{color}'")
    if not (0 <= slot_index <= 15):
        raise ValueError(f"slot_index must be 0-15, got {slot_index}")
    y = round(GRAVEYARD_START_Y + slot_index * GRAVEYARD_SPACING, 6)
    return BoardPose(x=x, y=y, z=BOARD_SURFACE_Z)


def graveyard_tf_frame(color: str, slot_index: int) -> str:
    return f"graveyard_{color}_{slot_index}"


def all_graveyard_poses() -> Dict[str, BoardPose]:
    poses: Dict[str, BoardPose] = {}
    for color in ("white", "black"):
        for i in range(16):
            frame = graveyard_tf_frame(color, i)
            poses[frame] = graveyard_slot_pose(color, i)
    return poses


def arm_reach_to_square(arm_base: Tuple[float, float, float], square: str) -> float:
    p = square_to_board_pose(square)
    dx = p.x - arm_base[0]
    dy = p.y - arm_base[1]
    dz = p.z - arm_base[2]
    return math.sqrt(dx*dx + dy*dy + dz*dz)


def worst_case_reach(arm_base: Tuple[float, float, float]) -> Tuple[str, float]:
    worst_sq = "a1"
    worst_d = 0.0
    for sq in all_squares():
        d = arm_reach_to_square(arm_base, sq)
        if d > worst_d:
            worst_d = d
            worst_sq = sq
    return worst_sq, worst_d


def validate_arm_reach() -> List[str]:
    warnings: List[str] = []
    for arm_name, base in [("white", WHITE_ARM_BASE), ("black", BLACK_ARM_BASE)]:
        for sq in all_squares():
            d = arm_reach_to_square(base, sq)
            if d > PANDA_MAX_REACH:
                warnings.append(f"{arm_name} arm cannot reach {sq}: {d:.4f}m")
    return warnings


def estimate_board_origin(
    a1_measured: Tuple[float, float, float],
    h8_measured: Tuple[float, float, float],
) -> Tuple[float, float, float, float]:
    cx = (a1_measured[0] + h8_measured[0]) / 2.0
    cy = (a1_measured[1] + h8_measured[1]) / 2.0
    cz = (a1_measured[2] + h8_measured[2]) / 2.0
    dx = h8_measured[0] - a1_measured[0]
    dy = h8_measured[1] - a1_measured[1]
    measured_angle = math.atan2(dy, dx)
    nominal_angle  = math.atan2(1.0, 1.0)
    yaw = measured_angle - nominal_angle
    return (round(cx, 6), round(cy, 6), round(cz, 6), round(yaw, 6))


def apply_board_transform(
    square: str,
    board_origin: Tuple[float, float, float],
    board_yaw: float,
) -> BoardPose:
    _validate_square(square.lower())
    nominal = square_to_board_pose(square)
    nominal_cx = 0.0
    nominal_cy = 0.0
    dx = nominal.x - nominal_cx
    dy = nominal.y - nominal_cy
    cos_y = math.cos(board_yaw)
    sin_y = math.sin(board_yaw)
    rx = dx * cos_y - dy * sin_y
    ry = dx * sin_y + dy * cos_y
    sz = math.sin(board_yaw / 2)
    cz = math.cos(board_yaw / 2)
    return BoardPose(
        x=round(board_origin[0] + rx, 6),
        y=round(board_origin[1] + ry, 6),
        z=round(board_origin[2], 6),
        qx=0.0, qy=0.0, qz=round(sz, 8), qw=round(cz, 8),
    )
