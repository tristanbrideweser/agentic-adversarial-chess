"""
grasp_filter.py
===============
Piece-type-aware grasp candidate filtering and ranking.

The filter applies a cascade of checks in order:
  1. Top-down angle check  — reject grasps whose approach vector deviates
                             more than `top_down_tolerance_deg` from vertical
  2. Height check          — gripper jaw centre must be within ±height_tol_m
                             of the piece's design grasp Z
  3. Score threshold       — reject grasps below `min_score`
  4. Finger separation     — jaw gap must be between piece diameter and
                             max_finger_separation_m
  5. Ranking               — surviving candidates ranked by composite score:
                             score * angle_weight + (1 - normalised_angle) * (1 - angle_weight)

After filtering, the ranker assigns .rank (0 = best) to each survivor.

All rejected candidates have .rejection_reason set for diagnostics.

Usage
-----
    from grasp_planner.grasp_filter import GraspFilter
    from grasp_planner.grasp_candidates import PieceType

    f = GraspFilter()
    ranked = f.filter_and_rank(candidates, piece_type=PieceType.QUEEN, target_z=0.810)
    best = ranked[0] if ranked else None
"""

from __future__ import annotations

import math
from typing import List, Optional

from .grasp_candidates import GraspCandidate, PieceType, get_profile


# ---------------------------------------------------------------------------
# Filter configuration
# ---------------------------------------------------------------------------

class FilterConfig:
    """
    Per-piece-type filter parameters.  Constructed from defaults then
    optionally overridden by gpd_params.yaml.
    """

    def __init__(
        self,
        top_down_tolerance_deg: float = 15.0,
        height_tolerance_m: float = 0.010,
        min_score: float = 0.0,
        max_finger_separation_m: float = 0.080,
        angle_weight: float = 0.4,
    ) -> None:
        self.top_down_tolerance_deg = top_down_tolerance_deg
        self.height_tolerance_m = height_tolerance_m
        self.min_score = min_score
        self.max_finger_separation_m = max_finger_separation_m
        self.angle_weight = angle_weight   # weight for angle score in composite


# Per-piece-type filter overrides.
#
# In UNIFORM_BLOCK_PIECES mode all pieces are the same height and shape,
# so a single uniform tolerance applies to every type.  The overrides dict
# is intentionally empty — no piece needs special treatment.
#
# When switching to real pieces (UNIFORM_BLOCK_PIECES = False), uncomment
# the entries below to restore per-type tuning:
PIECE_FILTER_OVERRIDES: dict[PieceType, dict] = {
    # PieceType.PAWN:   {"height_tolerance_m": 0.006},
    # PieceType.ROOK:   {"height_tolerance_m": 0.008},
    # PieceType.KNIGHT: {"height_tolerance_m": 0.008, "top_down_tolerance_deg": 20.0},
    # PieceType.BISHOP: {"height_tolerance_m": 0.008},
    # PieceType.QUEEN:  {"height_tolerance_m": 0.012, "top_down_tolerance_deg": 20.0},
    # PieceType.KING:   {"height_tolerance_m": 0.015, "top_down_tolerance_deg": 20.0},
}


def make_config(piece_type: PieceType, **overrides) -> FilterConfig:
    """Build a FilterConfig with per-piece defaults plus any extra overrides."""
    cfg = FilterConfig()
    piece_defaults = PIECE_FILTER_OVERRIDES.get(piece_type, {})
    for k, v in {**piece_defaults, **overrides}.items():
        if hasattr(cfg, k):
            setattr(cfg, k, v)
    return cfg


# ---------------------------------------------------------------------------
# Filter
# ---------------------------------------------------------------------------

class GraspFilter:
    """
    Stateless filter/ranker for GraspCandidate lists.

    Parameters
    ----------
    top_down_tolerance_deg : default angle tolerance (overridden per piece type)
    min_score : default minimum GPD score
    """

    def __init__(
        self,
        top_down_tolerance_deg: float = 15.0,
        min_score: float = 0.0,
    ) -> None:
        self._default_tol = top_down_tolerance_deg
        self._default_min_score = min_score

    def filter_and_rank(
        self,
        candidates: List[GraspCandidate],
        piece_type: PieceType,
        target_z: Optional[float] = None,
        **config_overrides,
    ) -> List[GraspCandidate]:
        """
        Filter and rank a list of candidates for the given piece type.

        Parameters
        ----------
        candidates : raw candidates from GPD (will be mutated — .rank and
                     .rejection_reason set in place)
        piece_type : determines per-piece filter thresholds
        target_z : expected world Z for the jaw centre; if None, uses the
                   piece profile's design grasp_z
        **config_overrides : keyword overrides forwarded to FilterConfig

        Returns
        -------
        Filtered, ranked list (best first).  Rejected candidates are NOT
        included but have .rejection_reason set.
        """
        cfg = make_config(
            piece_type,
            top_down_tolerance_deg=self._default_tol,
            min_score=self._default_min_score,
            **config_overrides,
        )
        # Per-piece overrides may set a *larger* tolerance than the node default.
        # Re-apply the piece override if it is more permissive.
        piece_override_tol = PIECE_FILTER_OVERRIDES.get(piece_type, {}).get(
            "top_down_tolerance_deg"
        )
        if piece_override_tol is not None and piece_override_tol > cfg.top_down_tolerance_deg:
            cfg.top_down_tolerance_deg = piece_override_tol

        profile = get_profile(piece_type)
        z_target = target_z if target_z is not None else profile.grasp_z
        min_finger_sep = profile.collision_radius_m * 2 + 0.002  # 2mm clearance each side

        survivors: List[GraspCandidate] = []

        for c in candidates:
            reason = self._check(c, cfg, z_target, min_finger_sep)
            if reason:
                c.rejection_reason = reason
            else:
                survivors.append(c)

        # Rank survivors by composite score
        ranked = self._rank(survivors, cfg)
        for i, c in enumerate(ranked):
            c.rank = i

        return ranked

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _check(
        c: GraspCandidate,
        cfg: FilterConfig,
        z_target: float,
        min_finger_sep: float,
    ) -> Optional[str]:
        """Return a rejection reason string, or None if the candidate passes."""

        # 1. Approach angle
        if c.approach_angle_deg > cfg.top_down_tolerance_deg:
            return (
                f"approach_angle {c.approach_angle_deg:.1f}° > "
                f"tolerance {cfg.top_down_tolerance_deg:.1f}°"
            )

        # 2. Grasp height
        dz = abs(c.position[2] - z_target)
        if dz > cfg.height_tolerance_m:
            return (
                f"grasp Z {c.position[2]:.4f}m deviates {dz*1000:.1f}mm "
                f"from target {z_target:.4f}m (tol={cfg.height_tolerance_m*1000:.1f}mm)"
            )

        # 3. Score threshold
        if c.score < cfg.min_score:
            return f"score {c.score:.3f} < min_score {cfg.min_score:.3f}"

        # 4. Finger separation bounds
        if c.finger_separation < min_finger_sep:
            return (
                f"finger_separation {c.finger_separation*1000:.1f}mm < "
                f"min {min_finger_sep*1000:.1f}mm"
            )
        if c.finger_separation > cfg.max_finger_separation_m:
            return (
                f"finger_separation {c.finger_separation*1000:.1f}mm > "
                f"max {cfg.max_finger_separation_m*1000:.1f}mm"
            )

        return None

    @staticmethod
    def _rank(
        candidates: List[GraspCandidate],
        cfg: FilterConfig,
    ) -> List[GraspCandidate]:
        """
        Rank candidates by a composite score:
            composite = w * quality_score + (1-w) * angle_score

        quality_score : normalised GPD score in [0, 1]
        angle_score   : 1.0 for perfectly top-down, decreasing with angle
        w             : cfg.angle_weight
        """
        if not candidates:
            return []

        # Normalise quality scores
        max_score = max(c.score for c in candidates) or 1.0

        def composite(c: GraspCandidate) -> float:
            q = c.score / max_score
            a = 1.0 - c.approach_angle_deg / max(cfg.top_down_tolerance_deg, 1.0)
            a = max(0.0, min(1.0, a))
            w = cfg.angle_weight
            return (1.0 - w) * q + w * a

        return sorted(candidates, key=composite, reverse=True)


# ---------------------------------------------------------------------------
# Convenience: filter a single GPD result list outside the ROS context
# ---------------------------------------------------------------------------

def filter_gpd_candidates(
    candidates: List[GraspCandidate],
    fen_char: str,
    target_z: Optional[float] = None,
) -> List[GraspCandidate]:
    """
    One-shot helper for filtering GPD candidates in tests or scripts.

    Parameters
    ----------
    candidates : list of GraspCandidate from GPD
    fen_char : FEN character of the target piece
    target_z : optional override for the grasp height

    Returns
    -------
    Filtered, ranked candidate list.
    """
    from .grasp_candidates import PieceType  # avoid circular at module level
    piece_type = PieceType.from_fen_char(fen_char)
    return GraspFilter().filter_and_rank(candidates, piece_type, target_z)