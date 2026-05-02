"""
test_lookup_grasps.py
=====================
Unit tests for lookup_grasps.py.  No ROS required.

Run with:
    pytest grasp_planner/test/test_lookup_grasps.py -v
"""

import math
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from grasp_planner.grasp_candidates import (
    PieceType, get_profile, PIECE_PROFILES,
    BLOCK_GRASP_Z, BLOCK_FINGER_SEP, UNIFORM_BLOCK_PIECES,
)
from grasp_planner.lookup_grasps import LookupGraspPlanner, _square_xy


# ---------------------------------------------------------------------------
# _square_xy helpers
# ---------------------------------------------------------------------------

class TestSquareXY:
    def test_a1(self):
        x, y = _square_xy("a1")
        assert x == pytest.approx(-0.175, abs=1e-6)
        assert y == pytest.approx(-0.175, abs=1e-6)

    def test_h8(self):
        x, y = _square_xy("h8")
        assert x == pytest.approx(0.175, abs=1e-6)
        assert y == pytest.approx(0.175, abs=1e-6)

    def test_e4(self):
        x, y = _square_xy("e4")
        assert x == pytest.approx(0.025, abs=1e-6)
        assert y == pytest.approx(-0.025, abs=1e-6)


# ---------------------------------------------------------------------------
# LookupGraspPlanner
# ---------------------------------------------------------------------------

class TestLookupGraspPlanner:

    def test_plan_returns_success(self):
        p = LookupGraspPlanner()
        result = p.plan("e4", "P")
        assert result.success is True

    def test_plan_correct_piece_type(self):
        p = LookupGraspPlanner()
        result = p.plan("d1", "Q")
        assert result.piece_type == PieceType.QUEEN

    def test_plan_single_candidate(self):
        p = LookupGraspPlanner()
        result = p.plan("e4", "P")
        assert len(result.all_candidates) == 1

    def test_plan_source_is_lookup(self):
        p = LookupGraspPlanner()
        result = p.plan("e4", "P")
        assert result.selected.source == "lookup"

    def test_plan_score_is_one(self):
        p = LookupGraspPlanner()
        result = p.plan("e4", "K")
        assert result.selected.score == pytest.approx(1.0)

    def test_plan_approach_angle_zero(self):
        p = LookupGraspPlanner()
        result = p.plan("e4", "R")
        assert result.selected.approach_angle_deg == pytest.approx(0.0)

    def test_plan_position_xy_matches_square(self):
        p = LookupGraspPlanner()
        sq = "g6"
        result = p.plan(sq, "b")
        x, y = _square_xy(sq)
        assert result.selected.position[0] == pytest.approx(x, abs=1e-6)
        assert result.selected.position[1] == pytest.approx(y, abs=1e-6)

    def test_plan_grasp_z_uniform(self):
        """All pieces return the same grasp Z in uniform block mode."""
        p = LookupGraspPlanner()
        for fen_char in "PRNBQKprnbqk":
            result = p.plan("a1", fen_char)
            assert result.selected.position[2] == pytest.approx(BLOCK_GRASP_Z, abs=1e-3), (
                f"Piece '{fen_char}': expected uniform BLOCK_GRASP_Z={BLOCK_GRASP_Z}, "
                f"got {result.selected.position[2]}"
            )

    def test_plan_grasp_z_matches_profile(self):
        """Grasp Z reported by plan() matches the PieceProfile."""
        p = LookupGraspPlanner()
        for fen_char in "PRNBQKprnbqk":
            result = p.plan("a1", fen_char)
            profile = get_profile(PieceType.from_fen_char(fen_char))
            assert result.selected.position[2] == pytest.approx(profile.grasp_z, abs=1e-3)

    def test_plan_finger_separation_uniform(self):
        """All pieces return the same finger separation in uniform block mode."""
        p = LookupGraspPlanner()
        for fen_char in "PRNBQKprnbqk":
            result = p.plan("e4", fen_char)
            assert result.selected.finger_separation == pytest.approx(BLOCK_FINGER_SEP, abs=1e-3), (
                f"Piece '{fen_char}': expected BLOCK_FINGER_SEP={BLOCK_FINGER_SEP}"
            )

    def test_plan_finger_separation_matches_profile(self):
        """Finger separation from plan() matches PieceProfile."""
        p = LookupGraspPlanner()
        for fen_char in "PRNBQKprnbqk":
            result = p.plan("e4", fen_char)
            profile = get_profile(PieceType.from_fen_char(fen_char))
            assert result.selected.finger_separation == pytest.approx(
                profile.finger_separation_m, abs=1e-3
            )

    def test_plan_all_64_squares_succeed(self):
        p = LookupGraspPlanner()
        for file in "abcdefgh":
            for rank in "12345678":
                sq = f"{file}{rank}"
                result = p.plan(sq, "P")
                assert result.success, f"Failed for square {sq}"

    def test_plan_all_piece_types_succeed(self):
        p = LookupGraspPlanner()
        for fen_char in "PRNBQKprnbqk":
            result = p.plan("e4", fen_char)
            assert result.success, f"Failed for piece '{fen_char}'"

    def test_plan_invalid_fen_returns_failure(self):
        p = LookupGraspPlanner()
        result = p.plan("e4", "X")
        assert result.success is False
        assert result.failure_reason

    def test_plan_yaw_override(self):
        p = LookupGraspPlanner()
        result = p.plan("e4", "P", yaw_override_rad=math.pi / 2)
        # The orientation should differ from the default
        default = p.plan("e4", "P")
        assert result.selected.orientation != default.selected.orientation

    def test_plan_default_yaw_respected(self):
        p = LookupGraspPlanner(default_yaw_rad=math.pi / 4)
        r1 = p.plan("e4", "P")
        p2 = LookupGraspPlanner(default_yaw_rad=0.0)
        r2 = p2.plan("e4", "P")
        assert r1.selected.orientation != r2.selected.orientation

    def test_orientation_is_unit_quaternion(self):
        p = LookupGraspPlanner()
        result = p.plan("d4", "Q")
        q = result.selected.orientation
        norm = (q.x**2 + q.y**2 + q.z**2 + q.w**2) ** 0.5
        assert norm == pytest.approx(1.0, abs=1e-6)

    def test_rank_is_zero(self):
        p = LookupGraspPlanner()
        result = p.plan("e4", "P")
        assert result.selected.rank == 0


# ---------------------------------------------------------------------------
# Approach waypoints
# ---------------------------------------------------------------------------

class TestApproachWaypoints:
    def test_returns_three_waypoints(self):
        p = LookupGraspPlanner()
        wp = p.plan_approach_waypoints("e4", "P")
        assert set(wp.keys()) == {"approach", "grasp", "lift"}

    def test_approach_above_grasp(self):
        p = LookupGraspPlanner()
        wp = p.plan_approach_waypoints("e4", "P", approach_z_offset=0.10)
        grasp_z = wp["grasp"]["position"][2]
        approach_z = wp["approach"]["position"][2]
        assert approach_z == pytest.approx(grasp_z + 0.10, abs=1e-6)

    def test_lift_equals_approach(self):
        p = LookupGraspPlanner()
        wp = p.plan_approach_waypoints("e4", "P")
        assert wp["lift"]["position"] == wp["approach"]["position"]

    def test_grasp_has_finger_separation(self):
        p = LookupGraspPlanner()
        wp = p.plan_approach_waypoints("e4", "P")
        assert "finger_separation" in wp["grasp"]

    def test_invalid_piece_returns_empty(self):
        p = LookupGraspPlanner()
        wp = p.plan_approach_waypoints("e4", "X")
        assert wp == {}


# ---------------------------------------------------------------------------
# Yaw clearance helper
# ---------------------------------------------------------------------------

class TestYawForClearance:
    def test_no_neighbours_returns_default(self):
        p = LookupGraspPlanner(default_yaw_rad=0.5)
        assert p.yaw_for_clearance("e4", []) == pytest.approx(0.5)

    def test_neighbour_to_right_gives_perpendicular_yaw(self):
        p = LookupGraspPlanner()
        # f4 is directly to the right of e4 (+X direction)
        # Expected yaw = atan2(0, 0.05) + π/2 = 0 + π/2 = π/2
        yaw = p.yaw_for_clearance("e4", ["f4"])
        assert yaw == pytest.approx(math.pi / 2, abs=0.05)

    def test_multiple_neighbours_uses_closest(self):
        p = LookupGraspPlanner()
        # f4 is adjacent; a1 is far away — should use f4
        yaw_single = p.yaw_for_clearance("e4", ["f4"])
        yaw_multi = p.yaw_for_clearance("e4", ["f4", "a1"])
        assert yaw_single == pytest.approx(yaw_multi, abs=1e-6)