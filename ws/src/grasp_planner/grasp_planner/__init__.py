"""
grasp_planner
=============
ROS 2 package: piece-type-aware grasp planning for chess robot arms.

Operates in two modes:
  - Lookup mode  : precomputed top-down grasps per piece type (fast, no cloud)
  - GPD mode     : live Grasp Pose Detection from overhead PointCloud2

Public API
----------
grasp_candidates : GraspCandidate, GraspResult, PieceType, PieceProfile,
                   Quaternion, PIECE_PROFILES
lookup_grasps    : LookupGraspPlanner
gpd_client       : GPDClient, compute_crop_box, parse_gpd_grasp,
                   approach_angle_from_quaternion
grasp_filter     : GraspFilter, FilterConfig, filter_gpd_candidates
"""

from .grasp_candidates import (
    GraspCandidate,
    GraspResult,
    PieceProfile,
    PieceType,
    Quaternion,
    PIECE_PROFILES,
    get_profile,
    get_profile_from_fen,
)

from .lookup_grasps import LookupGraspPlanner

from .gpd_client import (
    GPDClient,
    approach_angle_from_quaternion,
    compute_crop_box,
    parse_gpd_grasp,
)

from .grasp_filter import (
    FilterConfig,
    GraspFilter,
    filter_gpd_candidates,
    make_config,
)

__all__ = [
    # grasp_candidates
    "GraspCandidate",
    "GraspResult",
    "PieceProfile",
    "PieceType",
    "Quaternion",
    "PIECE_PROFILES",
    "get_profile",
    "get_profile_from_fen",
    # lookup_grasps
    "LookupGraspPlanner",
    # gpd_client
    "GPDClient",
    "approach_angle_from_quaternion",
    "compute_crop_box",
    "parse_gpd_grasp",
    # grasp_filter
    "FilterConfig",
    "GraspFilter",
    "filter_gpd_candidates",
    "make_config",
]