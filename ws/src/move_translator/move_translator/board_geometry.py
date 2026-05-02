"""
board_geometry.py
=================
Converts algebraic chess notation into world-frame coordinates and manages
graveyard slot allocation.

Coordinate convention (world frame, meters):
    - Board center at (0, 0, 0.762)
    - a1 at (-0.175, -0.175, 0.762)
    - h8 at (+0.175, +0.175, 0.762)
    - Square size: 0.05 m
    - +Y toward Black's side, -Y toward White's side
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Tuple

# ---------------------------------------------------------------------------
# Board geometry constants
# ---------------------------------------------------------------------------

BOARD_SURFACE_Z: float = 0.762      # m — top face of the board in world frame
SQUARE_SIZE: float = 0.05           # m — one square edge
A1_X: float            = -0.196875                # world X of the a1 square centre
A1_Y: float            = -0.196875                # world Y of the a1 square centre

# Graveyard layout: two columns off the side of the board, 8 rows each
# White captures go to the left (-X side), black captures to the right (+X side)
GRAVEYARD_WHITE_X: float = -0.275   # 1 square left of the a-file
GRAVEYARD_BLACK_X: float = +0.275   # 1 square right of the h-file
GRAVEYARD_START_Y: float = -0.175   # aligned with rank 1
GRAVEYARD_SPACING: float = 0.05     # same as square size


# ---------------------------------------------------------------------------
# Piece data
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Block piece geometry
# ---------------------------------------------------------------------------
# Current pieces are uniform blocks with letters on top.
# All pieces share the same height and collision radius, so grasp Z is
# identical for every piece type.
#
# To switch to real (height-differentiated) pieces:
#   1. Set UNIFORM_BLOCK_PIECES = False
#   2. Fill in the per-type overrides in PIECE_GEOMETRY_OVERRIDES below
#   3. No other code changes needed — get_piece_geometry() handles both modes
# ---------------------------------------------------------------------------

UNIFORM_BLOCK_PIECES: bool = True

# Physical dimensions of the uniform block
BLOCK_HEIGHT_M: float        = 0.040   # total block height (m)
BLOCK_GRASP_RATIO: float     = 0.60    # grasp at 60% of height
BLOCK_GRASP_Z: float         = round(BOARD_SURFACE_Z + BLOCK_HEIGHT_M * BLOCK_GRASP_RATIO, 6)
BLOCK_COLLISION_RADIUS_M: float = 0.018  # half the block XY footprint + clearance

# When UNIFORM_BLOCK_PIECES = False, these override the uniform values
# per piece type.  Fill these in when you have real piece dimensions.
PIECE_GEOMETRY_OVERRIDES: Dict[str, dict] = {
    # "P": {"height_m": 0.045, "grasp_z": 0.789, "collision_radius_m": 0.012},
    # "R": {"height_m": 0.055, "grasp_z": 0.795, "collision_radius_m": 0.014},
    # "N": {"height_m": 0.060, "grasp_z": 0.798, "collision_radius_m": 0.015},
    # "B": {"height_m": 0.065, "grasp_z": 0.801, "collision_radius_m": 0.013},
    # "Q": {"height_m": 0.080, "grasp_z": 0.810, "collision_radius_m": 0.015},
    # "K": {"height_m": 0.095, "grasp_z": 0.819, "collision_radius_m": 0.015},
}


@dataclass(frozen=True)
class PieceGeometry:
    """Physical dimensions for a single piece type."""
    fen_chars: Tuple[str, str]   # (white_char, black_char)
    name: str
    height_m: float              # total height in metres
    grasp_z: float               # absolute world Z for gripper jaw centre
    collision_radius_m: float    # approximate XY radius for collision checks


def _build_piece_geometry() -> Dict[str, "PieceGeometry"]:
    """
    Build the PIECE_GEOMETRY table.

    In UNIFORM_BLOCK_PIECES mode all types share BLOCK_* constants.
    When disabled, PIECE_GEOMETRY_OVERRIDES supplies per-type values;
    any type not listed there falls back to the block defaults.
    """
    pieces = {
        "P": ("P", "p", "pawn"),
        "R": ("R", "r", "rook"),
        "N": ("N", "n", "knight"),
        "B": ("B", "b", "bishop"),
        "Q": ("Q", "q", "queen"),
        "K": ("K", "k", "king"),
    }
    table: Dict[str, PieceGeometry] = {}
    for key, (wc, bc, name) in pieces.items():
        if not UNIFORM_BLOCK_PIECES and key in PIECE_GEOMETRY_OVERRIDES:
            ov = PIECE_GEOMETRY_OVERRIDES[key]
            h  = ov["height_m"]
            gz = ov.get("grasp_z", round(BOARD_SURFACE_Z + h * BLOCK_GRASP_RATIO, 6))
            cr = ov.get("collision_radius_m", BLOCK_COLLISION_RADIUS_M)
        else:
            h  = BLOCK_HEIGHT_M
            gz = BLOCK_GRASP_Z
            cr = BLOCK_COLLISION_RADIUS_M
        table[key] = PieceGeometry((wc, bc), name, h, gz, cr)
    return table


# Keyed by uppercase FEN character
PIECE_GEOMETRY: Dict[str, PieceGeometry] = _build_piece_geometry()


def get_piece_geometry(fen_char: str) -> PieceGeometry:
    """Return PieceGeometry for a FEN character (upper or lower case)."""
    key = fen_char.upper()
    if key not in PIECE_GEOMETRY:
        raise ValueError(f"Unknown FEN piece character: '{fen_char}'")
    return PIECE_GEOMETRY[key]


# ---------------------------------------------------------------------------
# Square → world coordinate
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class WorldPose:
    """A 4-DOF pose (x, y, z in metres; yaw in radians, default top-down)."""
    x: float
    y: float
    z: float
    yaw: float = 0.0

    def to_dict(self) -> dict:
        return {"x": self.x, "y": self.y, "z": self.z, "yaw": self.yaw}


def square_to_world(square: str, z_override: float | None = None) -> WorldPose:
    """
    Convert a two-character algebraic square name to a world-frame pose.

    Parameters
    ----------
    square:
        Algebraic notation, e.g. "e4", "a1", "h8".
    z_override:
        If provided, use this Z value instead of BOARD_SURFACE_Z.

    Returns
    -------
    WorldPose with yaw=0 (top-down approach by default).

    Raises
    ------
    ValueError if the square string is malformed.
    """
    if len(square) != 2:
        raise ValueError(f"Square must be two characters, got '{square}'")

    file_char = square[0].lower()
    rank_char = square[1]

    if file_char not in "abcdefgh":
        raise ValueError(f"Invalid file '{file_char}' in square '{square}'")
    if rank_char not in "12345678":
        raise ValueError(f"Invalid rank '{rank_char}' in square '{square}'")

    file_idx = ord(file_char) - ord("a")   # 0..7
    rank_idx = int(rank_char) - 1          # 0..7

    x = A1_X + file_idx * SQUARE_SIZE
    y = A1_Y + rank_idx * SQUARE_SIZE
    z = z_override if z_override is not None else BOARD_SURFACE_Z

    return WorldPose(x=round(x, 6), y=round(y, 6), z=round(z, 6))


def square_to_grasp_pose(square: str, fen_char: str) -> WorldPose:
    """
    Return the world pose at which the gripper should close around the piece.

    The Z is the piece-type-specific grasp height (60 % of piece height above
    board surface, as specified in the architecture document).
    """
    geom = get_piece_geometry(fen_char)
    base = square_to_world(square)
    return WorldPose(x=base.x, y=base.y, z=geom.grasp_z)


def distance_2d(square_a: str, square_b: str) -> float:
    """Euclidean XY distance (m) between two square centres."""
    pa = square_to_world(square_a)
    pb = square_to_world(square_b)
    return math.sqrt((pa.x - pb.x) ** 2 + (pa.y - pb.y) ** 2)


# ---------------------------------------------------------------------------
# Graveyard allocator
# ---------------------------------------------------------------------------

class GraveyardAllocator:
    """
    Tracks free graveyard slots for captured pieces.

    White captures (uppercase pieces removed by Black) are placed on the
    White graveyard row (left side); Black captures go right.

    Slots are allocated in order 0..15 (16 pieces per side max).  The
    allocator is stateful — create one per game and pass it through every
    call to decompose_move.
    """

    def __init__(self) -> None:
        self._white_count: int = 0   # slots used for white-piece captures
        self._black_count: int = 0   # slots used for black-piece captures

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def next_slot(self, captured_fen_char: str) -> WorldPose:
        """
        Reserve the next free graveyard slot for the given captured piece and
        return its world pose.

        Parameters
        ----------
        captured_fen_char:
            FEN character of the piece being captured (upper = white piece,
            lower = black piece).

        Returns
        -------
        WorldPose for the graveyard slot.
        """
        if captured_fen_char.isupper():
            # White piece captured — goes to white graveyard
            slot_index = self._white_count
            self._white_count += 1
            return self._slot_pose(GRAVEYARD_WHITE_X, slot_index)
        else:
            # Black piece captured — goes to black graveyard
            slot_index = self._black_count
            self._black_count += 1
            return self._slot_pose(GRAVEYARD_BLACK_X, slot_index)

    def peek_slot(self, captured_fen_char: str) -> WorldPose:
        """
        Return the next graveyard slot without advancing the counter.
        Useful for pre-flight checks or dry-run decomposition.
        """
        if captured_fen_char.isupper():
            return self._slot_pose(GRAVEYARD_WHITE_X, self._white_count)
        else:
            return self._slot_pose(GRAVEYARD_BLACK_X, self._black_count)

    def reset(self) -> None:
        """Reset counters for a new game."""
        self._white_count = 0
        self._black_count = 0

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _slot_pose(x: float, slot_index: int) -> WorldPose:
        y = GRAVEYARD_START_Y + slot_index * GRAVEYARD_SPACING
        return WorldPose(x=round(x, 6), y=round(y, 6), z=BOARD_SURFACE_Z)


# ---------------------------------------------------------------------------
# Reserve piece registry (for pawn promotion)
# ---------------------------------------------------------------------------

# Reserve pieces are parked off the back of the board at known positions.
# These positions assume a small shelf behind each player's back rank.
# Adjust in board_params.yaml if the physical layout differs.

RESERVE_WHITE_X: float = +0.275
RESERVE_WHITE_Y_START: float = +0.175   # behind rank 8
RESERVE_BLACK_X: float = -0.275
RESERVE_BLACK_Y_START: float = -0.175  # behind rank 1

# One reserve queen per colour is sufficient for almost all games.
# Extend if you want spare rooks/bishops/knights.
RESERVE_SLOTS: Dict[str, WorldPose] = {
    "white_queen_0": WorldPose(RESERVE_WHITE_X, RESERVE_WHITE_Y_START,        BOARD_SURFACE_Z),
    "white_rook_0":  WorldPose(RESERVE_WHITE_X, RESERVE_WHITE_Y_START + 0.05, BOARD_SURFACE_Z),
    "black_queen_0": WorldPose(RESERVE_BLACK_X, RESERVE_BLACK_Y_START,        BOARD_SURFACE_Z),
    "black_rook_0":  WorldPose(RESERVE_BLACK_X, RESERVE_BLACK_Y_START - 0.05, BOARD_SURFACE_Z),
}


class ReserveRegistry:
    """
    Tracks which reserve pieces are still available (not yet used for
    promotion) and returns their world poses.
    """

    def __init__(self) -> None:
        self._available: Dict[str, WorldPose] = dict(RESERVE_SLOTS)

    def get_reserve(self, color: str, piece_type: str) -> WorldPose:
        """
        Reserve and return the world pose of a spare piece.

        Parameters
        ----------
        color : "white" or "black"
        piece_type : "queen", "rook", "bishop", "knight"

        Raises
        ------
        KeyError if no reserve piece of that type is available.
        """
        key = f"{color}_{piece_type}_0"
        if key not in self._available:
            raise KeyError(
                f"No reserve {color} {piece_type} available. "
                "Consider using a previously captured piece."
            )
        pose = self._available.pop(key)
        return pose

    def return_piece(self, color: str, piece_type: str) -> None:
        """Return a piece to the reserve (e.g. after a game reset)."""
        key = f"{color}_{piece_type}_0"
        self._available[key] = RESERVE_SLOTS[key]

    def reset(self) -> None:
        self._available = dict(RESERVE_SLOTS)