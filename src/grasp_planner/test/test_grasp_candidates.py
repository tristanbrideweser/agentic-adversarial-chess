"""
test_grasp_candidates.py
========================
Unit tests for grasp_candidates.py — data models, quaternion math,
piece profiles.  No ROS required.

Run with:
    pytest grasp_planner/test/test_grasp_candidates.py -v
"""

import math
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from src.grasp_planner.grasp_planner.grasp_candidates import (
    GraspCandidate,
    GraspResult,
    PieceProfile,
    PieceType,
    Quaternion,
    PIECE_PROFILES,
    BLOCK_GRASP_Z,
    BLOCK_HEIGHT_M,
    BLOCK_FINGER_SEP,
    UNIFORM_BLOCK_PIECES,
    get_profile,
    get_profile_from_fen,
)


# ---------------------------------------------------------------------------
# PieceType
# ---------------------------------------------------------------------------

class TestPieceType:
    @pytest.mark.parametrize("char,expected", [
        ("P", PieceType.PAWN),   ("p", PieceType.PAWN),
        ("R", PieceType.ROOK),   ("r", PieceType.ROOK),
        ("N", PieceType.KNIGHT), ("n", PieceType.KNIGHT),
        ("B", PieceType.BISHOP), ("b", PieceType.BISHOP),
        ("Q", PieceType.QUEEN),  ("q", PieceType.QUEEN),
        ("K", PieceType.KING),   ("k", PieceType.KING),
    ])
    def test_from_fen_char(self, char, expected):
        assert PieceType.from_fen_char(char) == expected

    def test_invalid_char_raises(self):
        with pytest.raises(ValueError):
            PieceType.from_fen_char("X")


# ---------------------------------------------------------------------------
# Piece profiles
# ---------------------------------------------------------------------------

class TestPieceProfiles:
    def test_all_six_types_present(self):
        for pt in PieceType:
            assert pt in PIECE_PROFILES

    def test_all_heights_uniform(self):
        """In block mode all pieces share BLOCK_HEIGHT_M."""
        for pt in PieceType:
            assert get_profile(pt).height_m == pytest.approx(BLOCK_HEIGHT_M, abs=1e-6), pt

    def test_all_grasp_z_uniform(self):
        """In block mode all pieces share BLOCK_GRASP_Z."""
        for pt in PieceType:
            assert get_profile(pt).grasp_z == pytest.approx(BLOCK_GRASP_Z, abs=1e-3), pt

    def test_all_finger_sep_uniform(self):
        """In block mode all pieces share BLOCK_FINGER_SEP."""
        for pt in PieceType:
            assert get_profile(pt).finger_separation_m == pytest.approx(BLOCK_FINGER_SEP, abs=1e-3), pt

    def test_finger_separation_larger_than_diameter(self):
        """Finger gap should be at least 2 × collision_radius."""
        for pt, prof in PIECE_PROFILES.items():
            assert prof.finger_separation_m >= prof.collision_radius_m * 2, (
                f"{pt}: finger_sep {prof.finger_separation_m} < 2×radius {prof.collision_radius_m*2}"
            )

    def test_get_profile_from_fen_uppercase(self):
        assert get_profile_from_fen("Q").piece_type == PieceType.QUEEN

    def test_get_profile_from_fen_lowercase(self):
        assert get_profile_from_fen("k").piece_type == PieceType.KING


# ---------------------------------------------------------------------------
# Quaternion
# ---------------------------------------------------------------------------

class TestQuaternion:
    def test_default_is_identity(self):
        q = Quaternion()
        assert q.w == pytest.approx(1.0)
        assert q.x == pytest.approx(0.0)

    def test_norm_of_identity(self):
        assert Quaternion().norm() == pytest.approx(1.0)

    def test_normalised_of_identity(self):
        q = Quaternion().normalised()
        assert q.w == pytest.approx(1.0)

    def test_normalised_non_unit(self):
        q = Quaternion(1.0, 0.0, 0.0, 0.0)
        n = q.normalised()
        assert abs(n.x**2 + n.y**2 + n.z**2 + n.w**2 - 1.0) < 1e-9

    def test_from_axis_angle_90_about_z(self):
        q = Quaternion.from_axis_angle(0, 0, 1, math.pi / 2)
        # 90° about Z: (0, 0, sin(π/4), cos(π/4))
        assert q.z == pytest.approx(math.sin(math.pi / 4), abs=1e-6)
        assert q.w == pytest.approx(math.cos(math.pi / 4), abs=1e-6)

    def test_top_down_zero_yaw_is_unit(self):
        q = Quaternion.top_down(0.0)
        assert abs(q.x**2 + q.y**2 + q.z**2 + q.w**2 - 1.0) < 1e-9

    def test_top_down_yaw_pi_is_unit(self):
        q = Quaternion.top_down(math.pi)
        assert abs(q.x**2 + q.y**2 + q.z**2 + q.w**2 - 1.0) < 1e-9

    def test_top_down_points_down(self):
        """
        A top-down quaternion represents a 180° rotation about X.
        Verify that world +Z maps to world -Z (the board surface normal
        points into the gripper after flipping).
        """
        q = Quaternion.top_down(0.0)
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        # Rotate (0, 0, 1) by q: optimised sandwich product for v=(0,0,1)
        # v' = q * v * q*
        tx = 2.0 * (qx * qz + qy * qw)
        ty = 2.0 * (qy * qz - qx * qw)
        tz = 1.0 - 2.0 * (qx * qx + qy * qy)
        # 180° about X: (0,0,1) → (0,0,-1)
        assert tz < 0, f"Expected rotated +Z to map to -Z, got tz={tz}"

    def test_to_tuple_length(self):
        t = Quaternion(0.1, 0.2, 0.3, 0.9).to_tuple()
        assert len(t) == 4

    def test_zero_norm_returns_identity(self):
        q = Quaternion(0, 0, 0, 0).normalised()
        assert q.w == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# GraspCandidate
# ---------------------------------------------------------------------------

class TestGraspCandidate:
    def _make(self, angle=0.0, score=0.8, finger_sep=0.030):
        return GraspCandidate(
            position=(0.0, 0.0, 0.789),
            orientation=Quaternion.top_down(0.0),
            score=score,
            approach_angle_deg=angle,
            finger_separation=finger_sep,
            source="lookup",
        )

    def test_is_top_down_at_zero(self):
        assert self._make(angle=0.0).is_top_down() is True

    def test_is_top_down_within_tolerance(self):
        assert self._make(angle=14.9).is_top_down(15.0) is True

    def test_is_not_top_down_beyond_tolerance(self):
        assert self._make(angle=15.1).is_top_down(15.0) is False

    def test_to_dict_keys(self):
        d = self._make().to_dict()
        for k in ("position", "orientation", "score", "approach_angle_deg",
                  "finger_separation", "source", "rank"):
            assert k in d

    def test_rank_default_none(self):
        assert self._make().rank is None


# ---------------------------------------------------------------------------
# GraspResult
# ---------------------------------------------------------------------------

class TestGraspResult:
    def test_default_not_success(self):
        r = GraspResult(piece_type=PieceType.PAWN, target_square="e4")
        assert r.success is False

    def test_to_dict_success(self):
        c = GraspCandidate(
            position=(0.0, 0.0, 0.789),
            orientation=Quaternion.top_down(),
            score=1.0,
        )
        r = GraspResult(
            piece_type=PieceType.PAWN,
            target_square="e4",
            selected=c,
            success=True,
        )
        d = r.to_dict()
        assert d["success"] is True
        assert d["selected"] is not None

    def test_to_dict_failure(self):
        r = GraspResult(
            piece_type=PieceType.QUEEN,
            target_square="d1",
            success=False,
            failure_reason="no candidates",
        )
        d = r.to_dict()
        assert d["selected"] is None
        assert d["failure_reason"] == "no candidates"