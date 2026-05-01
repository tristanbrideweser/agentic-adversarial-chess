"""
board_geometry.py
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
A1_X: float = -0.175                # world X of the a1 square centre
A1_Y: float = -0.175                # world Y of the a1 square centre

# Graveyard layout: two columns off the side of the board, 8 rows each
# White captures go to the left (-X side), black captures to the right (+X side)
GRAVEYARD_WHITE_X: float = -0.275   # 1 square left of the a-file
GRAVEYARD_BLACK_X: float = +0.275   # 1 square right of the h-file
GRAVEYARD_START_Y: float = -0.175   # aligned with rank 1
GRAVEYARD_SPACING: float = 0.05     # same as square size


# ---------------------------------------------------------------------------
# Piece data
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class PieceGeometry:
    """Physical dimensions for a single piece type."""
    fen_chars: Tuple[str, str]   # (white_char, black_char)
    name: str
    height_m: float              # total height in metres
    grasp_z: float               # absolute world Z for gripper jaw centre
    collision_radius_m: float    # approximate XY radius for collision checks


# Keyed by uppercase FEN character
PIECE_GEOMETRY: Dict[str, PieceGeometry] = {
    "P": PieceGeometry(("P", "p"), "pawn",   0.045, 0.789, 0.012),
    "R": PieceGeometry(("R", "r"), "rook",   0.055, 0.795, 0.014),
    "N": PieceGeometry(("N", "n"), "knight", 0.060, 0.798, 0.015),
    "B": PieceGeometry(("B", "b"), "bishop", 0.065, 0.801, 0.013),
    "Q": PieceGeometry(("Q", "q"), "queen",  0.080, 0.810, 0.015),
    "K": PieceGeometry(("K", "k"), "king",   0.095, 0.819, 0.015),
}


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
import dataclasses
import yaml
import math

# ====================================================
# constants
# ====================================================

# piece heights (m)
PIECE_HEIGHTS: dict[str, float] = {
    'p': 0.05,
    'r': ...,
    'n': ...,
    'b': ...,
    'q': ...,
    'k': ...
}

# piece names shorthand -> english
PIECE_NAMES: dict[str, str] = {
    'p': "pawn",
    'r': "rook",
    'n': "knight",
    'b': "bishop",
    'q': "queen",
    'k': "king"
}

# ====================================================
# board onfig
# ====================================================

@dataclasses
class BoardConfig:
    """
    physical dims + placement of chess board in world frame

    all values in meters
    defaults match chess_world.sdf and piece_params.yaml
    """
    # TODO: specify values
    square_size: float = ...
    surface_z: float
    a1_center_x: float 
    a1_center_y: float

    # grasp tuning
    grasp_height_fraction: float = 0.6  # grasp at % of piece height
    place_clearance: float = 0.005      # drop clearance above surface

    # graveyard layout
    graveyard_white_x: float            # captured-by-white pieces
    graveyard_black_x: float            # captured-by-black pieces
    graveyard_start_y: float            # alignment with a-file
    graveyard_slot_spacing: float       # spacing between adj slots

    @classmethod
    def from_yaml(cls, path: str) -> BoardConfig:
        """
        load board config from YAML file
        """
        with open(path) as f:
            raw = yaml.safe_load(f)

        board = raw.get("board", {})

        a1 = board.get("a1_center", [cls.a1_center_x, cls.a1_center_y])

        return cls(
            square_size=board.get("square_size", cls.square_size),
            surface_z=board.get("surface_z", cls.surface_z),
            a1_center_x=a1[0],
            a1_center_y=a1[1]
        )

DEFAULT_CONFIG = BoardConfig()

# ====================================================
# pose
# ====================================================

@dataclasses
class Pose3D:
    """
    world-frame pose for pick + place target

    top-down approach by gripper, reduce orientation to single yaw value
    """
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
    
    @classmethod
    def as_dict(cls) -> dict:
        """
        serialize for JSON task queue output
        """
        return {
            'x': round(cls.x, 6),
            'y': round(cls.y, 6),
            'z': round(cls.z, 6),
            'yaw': round(cls.yaw, 4)
        }

# ====================================================
# square coordinate helpers
# ====================================================

def square_to_indices(square: str) -> tuple[int,int]:
    """
    convert algebraic notation to zero-based

    params: square      # algebraic notation
    returns: tuple      # (file_idx, rank_idx)
    raises: ValueError  # malformed name
    """
    
    if len(square) != 2:
        raise ValueError(f"square name must be 2 characters, got '{square}'")
    
    file_char = square[0].lower()
    rank_char = square[1]

    if file_char < 'a' or file_char > 'h': raise ValueError(f"Invalid file '{file_char}' in square '{square}'")
    if rank_char < '1' or rank_char > '8': raise ValueError(f"Invalid rank '{rank_char}' in square '{square}'")

    file_idx = ord(file_char) - ord('a')
    rank_idx = int(rank_char) - 1

    return file_idx, rank_idx

def square_to_world(square: str, config: BoardConfig) -> tuple[float,float,float]:
    """
    convert algebraic square name to world-frame

    params: str, config # square name
    returns: tuple  # center at board surface height
    """
    
    fi, ri = square_to_indices(square=square)
    x = config.a1_center_x + fi * config.square_size
    y = config.a1_center_y + ri * config.square_size
    z = config.surface_z

    return (x, y, z)

# ====================================================
# piece height and z helpers 
# ====================================================

def get_piece_height(piece_char: str) -> float:
    """
    lookup physcial height of piece by FEN char

    params: 
    returns: 
    """
    return PIECE_HEIGHTS.get(piece_char.lower(), 0.05)

def piece_grasp_z(piece_char: str, config: BoardConfig) -> float:
    """
    world-frame z for grasping piece

    params: 
    returns:
    """
    
    height = get_piece_height(piece_char)
    return config.surface_z + height * config.grasp_height_fraction

def piece_place_z(piece_char: str, config: BoardConfig) -> float:
    """
    world-frame z for placing piece
    """
    
    height = get_piece_height(piece_char)
    return config.surface_z + height * 0.5 + config.place_clearance

# ====================================================
# piece color helper 
# ====================================================

def get_piece_color(piece_char: str) -> str:
    """
    determine piece color from FEN char

    params:
    returns:
    """
    return "white" if piece_char.isupper() else "black"

def get_piece_name(piece_char: str) -> str:
    """
    human-readable piece name and color

    params:
    returns:
    """
    color = get_piece_color(piece_char)
    name = PIECE_NAMES.get(piece_char.lower(), "unknown")
    return f"{color} {name}"


# ====================================================
# pose builders 
# ====================================================

def make_pick_pose(square: str, piece_char: str, config: BoardConfig) -> Pose3D:
    """
    build a grasp pose for piece given square

    params:
    returns:
    """
    x, y, _ = square_to_world(square, config)
    z = piece_grasp_z(piece_char, config)
    yaw = 0.0 if piece_char.isupper() else math.pi

    return Pose3D(x=x, y=y, z=z, yaw=yaw)

def make_place_pose(square: str, piece_char: str, config: BoardConfig) -> Pose3D:
    """
    build a place pose for piece given square
    """
    x, y, _ = square_to_world(square, config)
    z = piece_grasp_z(piece_char, config)
    yaw = 0.0 if piece_char.isupper() else math.pi
# ====================================================
# graveyard 
# ====================================================

class GraveyardAllocator:
    """
    tracks and allocates graveyard sots for captured pieces
    2-column grid of slots on each side of board

    slot layout:
        col0    col1
        ----    ----
        slot0   slot1   # closest a-file
        ...   
        slot14  slot15  # closest h-file
    """

    def __init__(self, config) -> None:
        self._config = config
        self._white_idx: int = 0    # next slot for pieces captured by white
        self._black_idx: int = 0    # next slot for pieces captured by black

    @property
    def white_count(self) -> int:
        return self._white_idx
    
    @property
    def black_count(self) -> int:
        return self._black_idx
    
    def allocate(self, captured_by: str) -> tuple[float, float, float]:
        """
        get world-frame pose for next available graveyard slot

        params:
        returns:
        """

        if captured_by not in ("white", "black"): 
            raise ValueError(f"captured_by must be 'white' or 'black', got {captured_by}")
        
        cfg = self._config

        if captured_by == "white":
            idx = self._white_idx
            if idx >= 16:
                raise RuntimeError("white graveyard is full (16 slots)")
            self._white_idx += 1

            base_x = cfg.graveyard_black_x
            col_dir = -1.0
        
        else:
            idx = self._black_idx
            if idx >= 16:
                raise RuntimeError("black graveyard is full (16 slots)")
            self._black_idx += 1

            base_x = cfg.graveyard_white_x
            col_dir = 1.0

        row = idx // 2
        col = idx % 2

        x = base_x + col * cfg.graveyard_slot_spacing * col_dir
        y = cfg.graveyard_start_y + row * cfg.graveyard_slot_spacing
        z = cfg.surface_z + 0.01 

        return Pose3D(x=x, y=y, z=z, yaw=0.0)
    
    def reset(self) -> None:
        """
        reset graveyard counters
        """
        self._white_idx = 0
        self._black_idx = 0









