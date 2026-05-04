"""
move_translator
===============
ROS 2 package: bridges the symbolic chess game state (FEN / UCI) and the
physical Franka Panda workspace (world-frame pick-and-place task queues).

Public API
----------
board_geometry  : square_to_world(), GraveyardAllocator, ReserveRegistry, PieceGeometry
move_decomposer : decompose_move(), PickPlaceTask, task_queue_to_json()
special_moves   : classify_move(), get_castling_info(), get_en_passant_info(),
                  get_promotion_info(), determine_active_arm()
"""

from .board_geometry import (
    PIECE_GEOMETRY,
    BOARD_SURFACE_Z,
    SQUARE_SIZE,
    GraveyardAllocator,
    PieceGeometry,
    ReserveRegistry,
    WorldPose,
    get_piece_geometry,
    square_to_grasp_pose,
    square_to_world,
    distance_2d,
)

from .move_decomposer import (
    PickPlaceTask,
    decompose_move,
    task_queue_to_json,
    task_queue_from_json,
)

from .special_moves import (
    MoveClass,
    CastlingInfo,
    EnPassantInfo,
    PromotionInfo,
    classify_move,
    determine_active_arm,
    get_castling_info,
    get_en_passant_info,
    get_promotion_info,
    arm_can_reach,
    suggest_fallback_arm,
)

__all__ = [
    # board_geometry
    "PIECE_GEOMETRY",
    "BOARD_SURFACE_Z",
    "SQUARE_SIZE",
    "GraveyardAllocator",
    "PieceGeometry",
    "ReserveRegistry",
    "WorldPose",
    "get_piece_geometry",
    "square_to_grasp_pose",
    "square_to_world",
    "distance_2d",
    # move_decomposer
    "PickPlaceTask",
    "decompose_move",
    "task_queue_to_json",
    "task_queue_from_json",
    # special_moves
    "MoveClass",
    "CastlingInfo",
    "EnPassantInfo",
    "PromotionInfo",
    "classify_move",
    "determine_active_arm",
    "get_castling_info",
    "get_en_passant_info",
    "get_promotion_info",
    "arm_can_reach",
    "suggest_fallback_arm",
]