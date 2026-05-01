"""
lookup_grasps.py
================
Precomputed top-down grasp lookup table.

For a controlled Gazebo / lab environment with known piece positions and
uniform cylindrical-ish piece shapes, top-down grasps are faster, more
reliable, and need no point cloud.  This module provides a grasp for any
(square, piece_type) pair without running GPD.

The lookup table encodes:
  - Gripper approach direction: straight down (approach_angle = 0°)
  - Jaw separation: matched to the piece's collision diameter + 4 mm clearance
  - Grasp Z:  piece-type-specific (60 % of height above board surface)
  - Yaw:  default 0 (fingers parallel to X axis).  Overrideable per square
          if a neighbouring piece would cause a collision.

Usage
-----
    from grasp_planner.lookup_grasps import LookupGraspPlanner

    planner = LookupGraspPlanner()
    result = planner.plan("e4", "P")   # white pawn on e4
    if result.success:
        pose = result.selected

The planner also accepts an optional yaw hint (radians) to rotate the
gripper around Z — useful when a neighbouring piece on the same rank/file
would be clipped by the open jaw.
"""

from __future__ import annotations

import math
from typing import Optional

from .grasp_candidates import (
    GraspCandidate,
    GraspResult,
    PieceType,
    Quaternion,
    get_profile,
    get_profile_from_fen,
)

# Board geometry constants (kept local to avoid circular import)
BOARD_SURFACE_Z = 0.762
A1_X = -0.175
A1_Y = -0.175
SQUARE_SIZE = 0.05


def _square_xy(square: str) -> tuple[float, float]:
    """Return (x, y) world centre of a square."""
    file_idx = ord(square[0].lower()) - ord("a")
    rank_idx = int(square[1]) - 1
    return (
        round(A1_X + file_idx * SQUARE_SIZE, 6),
        round(A1_Y + rank_idx * SQUARE_SIZE, 6),
    )


class LookupGraspPlanner:
    """
    Fast, deterministic top-down grasp planner using a precomputed table.

    Does not require a point cloud or GPD.  Always produces exactly one
    GraspCandidate with source="lookup" and score=1.0.

    Parameters
    ----------
    default_yaw_rad : float
        Default gripper yaw (rotation around world Z).  0 = fingers parallel
        to the X axis (pointing along file direction).
    top_down_tolerance_deg : float
        Maximum allowed approach-angle deviation from vertical.  Lookup
        grasps are always exactly 0°, so this only matters if you later
        blend lookup with GPD candidates.
    """

    def __init__(
        self,
        default_yaw_rad: float = 0.0,
        top_down_tolerance_deg: float = 15.0,
    ) -> None:
        self._default_yaw = default_yaw_rad
        self._tolerance = top_down_tolerance_deg

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def plan(
        self,
        square: str,
        fen_char: str,
        yaw_override_rad: Optional[float] = None,
    ) -> GraspResult:
        """
        Generate a top-down grasp for the piece on the given square.

        Parameters
        ----------
        square : algebraic notation, e.g. "e4"
        fen_char : FEN character of the piece (e.g. "P", "q", "K")
        yaw_override_rad : optional rotation around world Z axis (radians).
            Use this when a neighbour would collide with the open jaw.

        Returns
        -------
        GraspResult with success=True and exactly one candidate.
        """
        try:
            piece_type = PieceType.from_fen_char(fen_char)
        except ValueError as e:
            return GraspResult(
                piece_type=PieceType.PAWN,
                target_square=square,
                success=False,
                failure_reason=str(e),
            )

        profile = get_profile(piece_type)
        x, y = _square_xy(square)
        yaw = yaw_override_rad if yaw_override_rad is not None else self._default_yaw

        candidate = GraspCandidate(
            position=(x, y, profile.grasp_z),
            orientation=Quaternion.top_down(yaw),
            score=1.0,
            approach_angle_deg=0.0,
            finger_separation=profile.finger_separation_m,
            source="lookup",
            rank=0,
        )

        return GraspResult(
            piece_type=piece_type,
            target_square=square,
            selected=candidate,
            all_candidates=[candidate],
            success=True,
        )

    def plan_approach_waypoints(
        self,
        square: str,
        fen_char: str,
        approach_z_offset: float = 0.10,
        yaw_override_rad: Optional[float] = None,
    ) -> dict:
        """
        Return the full set of waypoint poses for one pick operation.

        Waypoints (all world-frame, metres):
            approach  : above the piece at grasp_z + approach_z_offset
            grasp     : at grasp_z (jaw closes here)
            lift      : above the piece again after grasping

        Returns a dict with keys "approach", "grasp", "lift", each being
        a dict with "position" and "orientation" sub-keys.
        """
        result = self.plan(square, fen_char, yaw_override_rad)
        if not result.success or result.selected is None:
            return {}

        g = result.selected
        x, y, z = g.position
        q = g.orientation.to_tuple()

        return {
            "approach": {
                "position": (x, y, z + approach_z_offset),
                "orientation": q,
            },
            "grasp": {
                "position": (x, y, z),
                "orientation": q,
                "finger_separation": g.finger_separation,
            },
            "lift": {
                "position": (x, y, z + approach_z_offset),
                "orientation": q,
            },
        }

    def yaw_for_clearance(self, square: str, neighbour_squares: list[str]) -> float:
        """
        Suggest a yaw angle (radians) that minimises jaw–neighbour overlap.

        Strategy: rotate the jaw axis 90° away from the direction of the
        nearest neighbour.  Falls back to the default yaw if all orientations
        are roughly equivalent.

        Parameters
        ----------
        square : the square being grasped
        neighbour_squares : list of occupied adjacent squares

        Returns
        -------
        Suggested yaw in radians.
        """
        if not neighbour_squares:
            return self._default_yaw

        sx, sy = _square_xy(square)
        # Find the closest neighbour
        closest = min(
            neighbour_squares,
            key=lambda sq: math.hypot(
                _square_xy(sq)[0] - sx,
                _square_xy(sq)[1] - sy,
            ),
        )
        nx, ny = _square_xy(closest)
        dx, dy = nx - sx, ny - sy
        angle_to_neighbour = math.atan2(dy, dx)

        # Rotate jaw 90° away from the neighbour direction
        return angle_to_neighbour + math.pi / 2