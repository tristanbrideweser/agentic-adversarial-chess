"""
test_board_geometry.py
======================
Unit tests for board_geometry.py.  No ROS required.

Run with:
    pytest move_translator/test/test_board_geometry.py -v
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import math
import pytest
from src.move_translator.move_translator.board_geometry import (
    BOARD_SURFACE_Z,
    BLOCK_GRASP_Z,
    BLOCK_HEIGHT_M,
    BLOCK_COLLISION_RADIUS_M,
    SQUARE_SIZE,
    GraveyardAllocator,
    ReserveRegistry,
    WorldPose,
    distance_2d,
    get_piece_geometry,
    square_to_grasp_pose,
    square_to_world,
)


# ---------------------------------------------------------------------------
# square_to_world
# ---------------------------------------------------------------------------

class TestSquareToWorld:
    def test_a1_corner(self):
        p = square_to_world("a1")
        assert p.x == pytest.approx(-0.175, abs=1e-6)
        assert p.y == pytest.approx(-0.175, abs=1e-6)
        assert p.z == pytest.approx(BOARD_SURFACE_Z, abs=1e-6)

    def test_h8_corner(self):
        p = square_to_world("h8")
        assert p.x == pytest.approx(0.175, abs=1e-6)
        assert p.y == pytest.approx(0.175, abs=1e-6)

    def test_e1(self):
        p = square_to_world("e1")
        assert p.x == pytest.approx(0.025, abs=1e-6)
        assert p.y == pytest.approx(-0.175, abs=1e-6)

    def test_e4(self):
        p = square_to_world("e4")
        assert p.x == pytest.approx(0.025, abs=1e-6)
        assert p.y == pytest.approx(-0.025, abs=1e-6)

    def test_d4(self):
        p = square_to_world("d4")
        assert p.x == pytest.approx(-0.025, abs=1e-6)
        assert p.y == pytest.approx(-0.025, abs=1e-6)

    def test_h1(self):
        p = square_to_world("h1")
        assert p.x == pytest.approx(0.175, abs=1e-6)
        assert p.y == pytest.approx(-0.175, abs=1e-6)

    def test_a8(self):
        p = square_to_world("a8")
        assert p.x == pytest.approx(-0.175, abs=1e-6)
        assert p.y == pytest.approx(0.175, abs=1e-6)

    def test_e8(self):
        p = square_to_world("e8")
        assert p.x == pytest.approx(0.025, abs=1e-6)
        assert p.y == pytest.approx(0.175, abs=1e-6)

    def test_z_override(self):
        p = square_to_world("d4", z_override=1.234)
        assert p.z == pytest.approx(1.234, abs=1e-6)

    def test_default_yaw_is_zero(self):
        assert square_to_world("e4").yaw == 0.0

    def test_uppercase_square(self):
        # Should normalise to lowercase
        p = square_to_world("E4")
        assert p.x == pytest.approx(0.025, abs=1e-6)

    def test_invalid_file(self):
        with pytest.raises(ValueError, match="Invalid file"):
            square_to_world("z4")

    def test_invalid_rank(self):
        with pytest.raises(ValueError, match="Invalid rank"):
            square_to_world("a9")

    def test_wrong_length(self):
        with pytest.raises(ValueError, match="two characters"):
            square_to_world("e44")

    def test_all_64_squares_unique(self):
        """Every square should have a unique (x, y) pair."""
        poses = {}
        for file in "abcdefgh":
            for rank in "12345678":
                sq = f"{file}{rank}"
                p = square_to_world(sq)
                key = (round(p.x, 6), round(p.y, 6))
                assert key not in poses, f"Duplicate pose for {sq}: {key}"
                poses[key] = sq

    def test_square_spacing_is_5cm(self):
        """Adjacent squares should be exactly SQUARE_SIZE apart."""
        p_a1 = square_to_world("a1")
        p_b1 = square_to_world("b1")
        p_a2 = square_to_world("a2")
        assert abs(p_b1.x - p_a1.x) == pytest.approx(SQUARE_SIZE, abs=1e-6)
        assert abs(p_a2.y - p_a1.y) == pytest.approx(SQUARE_SIZE, abs=1e-6)


# ---------------------------------------------------------------------------
# get_piece_geometry
# ---------------------------------------------------------------------------

class TestPieceGeometry:
    def test_all_heights_uniform(self):
        """All pieces share BLOCK_HEIGHT_M in uniform block mode."""
        for char in "PRNBQKprnbqk":
            g = get_piece_geometry(char)
            assert g.height_m == pytest.approx(BLOCK_HEIGHT_M, abs=1e-6), char

    def test_all_grasp_z_uniform(self):
        """All pieces share BLOCK_GRASP_Z in uniform block mode."""
        for char in "PRNBQKprnbqk":
            g = get_piece_geometry(char)
            assert g.grasp_z == pytest.approx(BLOCK_GRASP_Z, abs=1e-3), char

    def test_grasp_z_above_board_surface(self):
        g = get_piece_geometry("P")
        assert g.grasp_z > BOARD_SURFACE_Z

    def test_invalid_char(self):
        with pytest.raises(ValueError):
            get_piece_geometry("X")

    def test_upper_lower_same(self):
        assert get_piece_geometry("Q").height_m == get_piece_geometry("q").height_m


# ---------------------------------------------------------------------------
# square_to_grasp_pose
# ---------------------------------------------------------------------------

class TestGraspPose:
    def test_all_pieces_same_grasp_z(self):
        """All piece types return BLOCK_GRASP_Z in uniform block mode."""
        for char in "PRNBQKprnbqk":
            pose = square_to_grasp_pose("e2", char)
            assert pose.z == pytest.approx(BLOCK_GRASP_Z, abs=1e-3), char

    def test_xy_matches_square_to_world(self):
        sq = "g5"
        grasp = square_to_grasp_pose(sq, "R")
        base = square_to_world(sq)
        assert grasp.x == pytest.approx(base.x, abs=1e-6)
        assert grasp.y == pytest.approx(base.y, abs=1e-6)


# ---------------------------------------------------------------------------
# distance_2d
# ---------------------------------------------------------------------------

class TestDistance2D:
    def test_same_square_is_zero(self):
        assert distance_2d("e4", "e4") == pytest.approx(0.0, abs=1e-6)

    def test_adjacent_file(self):
        d = distance_2d("a1", "b1")
        assert d == pytest.approx(SQUARE_SIZE, abs=1e-6)

    def test_adjacent_rank(self):
        d = distance_2d("a1", "a2")
        assert d == pytest.approx(SQUARE_SIZE, abs=1e-6)

    def test_diagonal(self):
        # a1 to b2: sqrt(0.05^2 + 0.05^2)
        d = distance_2d("a1", "b2")
        assert d == pytest.approx(math.sqrt(2) * SQUARE_SIZE, abs=1e-6)

    def test_full_diagonal_a1_h8(self):
        # 7 squares diagonally
        d = distance_2d("a1", "h8")
        assert d == pytest.approx(7 * math.sqrt(2) * SQUARE_SIZE, abs=1e-4)


# ---------------------------------------------------------------------------
# GraveyardAllocator
# ---------------------------------------------------------------------------

class TestGraveyardAllocator:
    def test_white_piece_goes_left(self):
        g = GraveyardAllocator()
        pose = g.next_slot("P")   # white pawn
        assert pose.x < 0, "White captures should go to negative-X side"

    def test_black_piece_goes_right(self):
        g = GraveyardAllocator()
        pose = g.next_slot("p")   # black pawn
        assert pose.x > 0, "Black captures should go to positive-X side"

    def test_slots_are_sequential(self):
        g = GraveyardAllocator()
        poses = [g.next_slot("p") for _ in range(4)]
        ys = [p.y for p in poses]
        for i in range(1, len(ys)):
            assert ys[i] > ys[i - 1], "Graveyard slots should advance in Y"

    def test_white_and_black_slots_independent(self):
        g = GraveyardAllocator()
        w0 = g.next_slot("R")
        b0 = g.next_slot("r")
        w1 = g.next_slot("N")
        b1 = g.next_slot("n")
        # White slots should advance independently of black slots
        assert w0.y == w1.y - 0.05
        assert b0.y == b1.y - 0.05

    def test_peek_does_not_advance(self):
        g = GraveyardAllocator()
        p1 = g.peek_slot("p")
        p2 = g.peek_slot("p")
        assert p1 == p2

    def test_reset_clears_state(self):
        g = GraveyardAllocator()
        pose_before = g.next_slot("q")
        g.reset()
        pose_after = g.next_slot("q")
        assert pose_before == pose_after, "After reset, first slot should be same"

    def test_sixteen_captures_max(self):
        g = GraveyardAllocator()
        # 16 white pieces can be captured
        poses = [g.next_slot("P") for _ in range(16)]
        assert len(set(p.y for p in poses)) == 16, "Should have 16 unique Y positions"


# ---------------------------------------------------------------------------
# ReserveRegistry
# ---------------------------------------------------------------------------

class TestReserveRegistry:
    def test_get_white_queen(self):
        r = ReserveRegistry()
        pose = r.get_reserve("white", "queen")
        assert isinstance(pose, WorldPose)

    def test_get_black_queen(self):
        r = ReserveRegistry()
        pose = r.get_reserve("black", "queen")
        assert isinstance(pose, WorldPose)

    def test_used_reserve_raises(self):
        r = ReserveRegistry()
        r.get_reserve("white", "queen")
        with pytest.raises(KeyError, match="No reserve"):
            r.get_reserve("white", "queen")

    def test_return_piece_restores(self):
        r = ReserveRegistry()
        original = r.get_reserve("white", "queen")
        r.return_piece("white", "queen")
        restored = r.get_reserve("white", "queen")
        assert original == restored

    def test_reset_restores_all(self):
        r = ReserveRegistry()
        r.get_reserve("white", "queen")
        r.get_reserve("black", "queen")
        r.reset()
        # Should not raise
        r.get_reserve("white", "queen")
        r.get_reserve("black", "queen")