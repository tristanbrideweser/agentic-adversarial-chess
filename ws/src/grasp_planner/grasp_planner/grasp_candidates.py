"""
grasp_candidates.py
===================
Data models for grasp poses and ranked candidate sets.

A GraspCandidate holds a 6-DOF gripper pose (position + quaternion),
a quality score, and metadata used by the filter/ranker to select
the best grasp for a given piece type.

These are pure dataclasses — no ROS, no GPD dependency — so they can
be constructed and tested without a running system.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Tuple


# ---------------------------------------------------------------------------
# Piece type enumeration (mirrors architecture piece table)
# ---------------------------------------------------------------------------

class PieceType(Enum):
    PAWN   = auto()
    ROOK   = auto()
    KNIGHT = auto()
    BISHOP = auto()
    QUEEN  = auto()
    KING   = auto()

    @classmethod
    def from_fen_char(cls, fen_char: str) -> "PieceType":
        mapping = {
            "P": cls.PAWN,   "p": cls.PAWN,
            "R": cls.ROOK,   "r": cls.ROOK,
            "N": cls.KNIGHT, "n": cls.KNIGHT,
            "B": cls.BISHOP, "b": cls.BISHOP,
            "Q": cls.QUEEN,  "q": cls.QUEEN,
            "K": cls.KING,   "k": cls.KING,
        }
        if fen_char not in mapping:
            raise ValueError(f"Unknown FEN character: '{fen_char}'")
        return mapping[fen_char]


# ---------------------------------------------------------------------------
# Piece geometry constants (duplicated from move_translator for independence)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class PieceProfile:
    """Physical profile used for grasp planning."""
    piece_type: PieceType
    height_m: float           # total piece height
    grasp_z: float            # absolute world Z for jaw centre (60% height)
    collision_radius_m: float # approximate XY radius
    finger_separation_m: float  # recommended gripper jaw gap at contact


# Finger separation = 2 * collision_radius + 4 mm clearance
PIECE_PROFILES: dict[PieceType, PieceProfile] = {
    PieceType.PAWN:   PieceProfile(PieceType.PAWN,   0.045, 0.789, 0.012, 0.028),
    PieceType.ROOK:   PieceProfile(PieceType.ROOK,   0.055, 0.795, 0.014, 0.032),
    PieceType.KNIGHT: PieceProfile(PieceType.KNIGHT, 0.060, 0.798, 0.015, 0.034),
    PieceType.BISHOP: PieceProfile(PieceType.BISHOP, 0.065, 0.801, 0.013, 0.030),
    PieceType.QUEEN:  PieceProfile(PieceType.QUEEN,  0.080, 0.810, 0.015, 0.034),
    PieceType.KING:   PieceProfile(PieceType.KING,   0.095, 0.819, 0.015, 0.034),
}


def get_profile(piece_type: PieceType) -> PieceProfile:
    return PIECE_PROFILES[piece_type]


def get_profile_from_fen(fen_char: str) -> PieceProfile:
    return PIECE_PROFILES[PieceType.from_fen_char(fen_char)]


# ---------------------------------------------------------------------------
# Quaternion helpers (no scipy/tf2 dependency)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0

    def norm(self) -> float:
        return math.sqrt(self.x**2 + self.y**2 + self.z**2 + self.w**2)

    def normalised(self) -> "Quaternion":
        n = self.norm()
        if n < 1e-9:
            return Quaternion(0, 0, 0, 1)
        return Quaternion(self.x / n, self.y / n, self.z / n, self.w / n)

    def to_tuple(self) -> Tuple[float, float, float, float]:
        return (self.x, self.y, self.z, self.w)

    @classmethod
    def from_axis_angle(cls, ax: float, ay: float, az: float, angle_rad: float) -> "Quaternion":
        """Construct from a unit axis and angle (radians)."""
        s = math.sin(angle_rad / 2)
        return cls(ax * s, ay * s, az * s, math.cos(angle_rad / 2)).normalised()

    @classmethod
    def top_down(cls, yaw_rad: float = 0.0) -> "Quaternion":
        """
        Gripper pointing straight down (+Z world → -Z gripper), rotated
        by yaw_rad around the world Z axis.

        In ROS convention: a gripper facing -Z means the approach vector
        points down.  We represent this as a 180° rotation about X
        (flipping Z), then a yaw around Z.

        rot_x_180: (x=1, y=0, z=0, w=0)
        then yaw:  (x=0, y=0, z=sin(yaw/2), w=cos(yaw/2))
        combined via quaternion product.
        """
        # Quaternion for 180° rotation about X: (1,0,0,0)
        # Quaternion for yaw about Z
        sy = math.sin(yaw_rad / 2)
        cy = math.cos(yaw_rad / 2)
        # product: q_z * q_x
        # q_x = (1,0,0,0), q_z = (0,0,sz,cz)
        # result: (cx*qx.x + sx*qx.w,  ...) — expand manually
        # q = q_yaw * q_flip_x
        # q_flip_x = (x=1, y=0, z=0, w=0) → axes: x,y,z,w
        qx_x, qx_y, qx_z, qx_w = 1.0, 0.0, 0.0, 0.0
        qz_x, qz_y, qz_z, qz_w = 0.0, 0.0, sy, cy
        # Hamilton product q_z * q_x
        rx = qz_w * qx_x + qz_x * qx_w + qz_y * qx_z - qz_z * qx_y
        ry = qz_w * qx_y - qz_x * qx_z + qz_y * qx_w + qz_z * qx_x
        rz = qz_w * qx_z + qz_x * qx_y - qz_y * qx_x + qz_z * qx_w
        rw = qz_w * qx_w - qz_x * qx_x - qz_y * qx_y - qz_z * qx_z
        return cls(rx, ry, rz, rw).normalised()


# ---------------------------------------------------------------------------
# Grasp candidate
# ---------------------------------------------------------------------------

@dataclass
class GraspCandidate:
    """
    A single gripper pose candidate produced by GPD or the lookup table.

    position:          (x, y, z) world frame, metres
    orientation:       unit quaternion (x, y, z, w)
    score:             GPD antipodal quality score, or 1.0 for lookup-table grasps
    approach_angle_deg: angle of the approach vector from vertical (0 = top-down)
    finger_separation: jaw gap at contact (metres)
    source:            "gpd" | "lookup"
    """
    position: Tuple[float, float, float]
    orientation: Quaternion
    score: float = 1.0
    approach_angle_deg: float = 0.0
    finger_separation: float = 0.030
    source: str = "lookup"

    # Populated by the filter after ranking
    rank: Optional[int] = None
    rejection_reason: Optional[str] = None

    def is_top_down(self, tolerance_deg: float = 15.0) -> bool:
        """Return True if the approach vector is within tolerance of vertical."""
        return self.approach_angle_deg <= tolerance_deg

    def to_dict(self) -> dict:
        return {
            "position": list(self.position),
            "orientation": list(self.orientation.to_tuple()),
            "score": self.score,
            "approach_angle_deg": self.approach_angle_deg,
            "finger_separation": self.finger_separation,
            "source": self.source,
            "rank": self.rank,
        }


@dataclass
class GraspResult:
    """
    The output of the grasp planner for one pick target.

    selected:        The best GraspCandidate (None if planning failed)
    all_candidates:  All candidates before filtering (for debugging)
    piece_type:      The piece being grasped
    target_square:   Algebraic square name (e.g. "e4")
    success:         Whether a valid grasp was found
    failure_reason:  Human-readable reason if success is False
    """
    piece_type: PieceType
    target_square: str
    selected: Optional[GraspCandidate] = None
    all_candidates: List[GraspCandidate] = field(default_factory=list)
    success: bool = False
    failure_reason: str = ""

    def to_dict(self) -> dict:
        return {
            "piece_type": self.piece_type.name,
            "target_square": self.target_square,
            "success": self.success,
            "failure_reason": self.failure_reason,
            "selected": self.selected.to_dict() if self.selected else None,
            "num_candidates": len(self.all_candidates),
        }