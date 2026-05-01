"""
board_localization
==================
ROS 2 package: board coordinate system, TF tree, and square lookup service.

Owns the canonical coordinate constants for the chess robot system.
All other packages (move_translator, grasp_planner, arm_controller) should
treat these values as the single source of truth.

Public API
----------
board_geometry    : BoardPose, square_to_board_pose(), board_pose_to_square(),
                    square_to_tf_frame(), all_squares(), all_square_poses(),
                    graveyard_slot_pose(), all_graveyard_poses(),
                    apply_board_transform(), validate_arm_reach(),
                    estimate_board_origin()

calibration       : CalibrationResult, calibrate_two_corners(),
                    calibrate_four_corners(), calibrate_nominal(),
                    validate_calibration()

ROS 2 nodes
-----------
board_tf_broadcaster   : publishes 97 static TF frames at startup
square_lookup_service  : /board_localization/get_square_pose service
"""

from .board_geometry import (
    # Constants
    BOARD_SURFACE_Z,
    BOARD_FRAME,
    WORLD_FRAME,
    SQUARE_SIZE,
    A1_X,
    A1_Y,
    WHITE_ARM_BASE,
    BLACK_ARM_BASE,
    CAMERA_POSITION,
    PANDA_MAX_REACH,
    # Dataclass
    BoardPose,
    # Square conversions
    square_to_board_pose,
    board_pose_to_square,
    square_to_tf_frame,
    tf_frame_to_square,
    all_squares,
    all_square_poses,
    # Graveyard
    graveyard_slot_pose,
    graveyard_tf_frame,
    all_graveyard_poses,
    # Reach analysis
    arm_reach_to_square,
    worst_case_reach,
    validate_arm_reach,
    # Calibration helpers
    estimate_board_origin,
    apply_board_transform,
)

from .calibration import (
    CalibrationResult,
    calibrate_two_corners,
    calibrate_four_corners,
    calibrate_nominal,
    validate_calibration,
)

__all__ = [
    # board_geometry constants
    "BOARD_SURFACE_Z", "BOARD_FRAME", "WORLD_FRAME", "SQUARE_SIZE",
    "A1_X", "A1_Y", "WHITE_ARM_BASE", "BLACK_ARM_BASE",
    "CAMERA_POSITION", "PANDA_MAX_REACH",
    # board_geometry types & functions
    "BoardPose",
    "square_to_board_pose", "board_pose_to_square",
    "square_to_tf_frame", "tf_frame_to_square",
    "all_squares", "all_square_poses",
    "graveyard_slot_pose", "graveyard_tf_frame", "all_graveyard_poses",
    "arm_reach_to_square", "worst_case_reach", "validate_arm_reach",
    "estimate_board_origin", "apply_board_transform",
    # calibration
    "CalibrationResult",
    "calibrate_two_corners", "calibrate_four_corners",
    "calibrate_nominal", "validate_calibration",
]