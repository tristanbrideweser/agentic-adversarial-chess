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









