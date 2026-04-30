"""
test_board_geometry.py
Unit tests for board_geometry.py.  No ROS required.

Run with:
    pytest move_translator/test/test_board_geometry.py -v
"""

import math
import sys
import os

# Allow running from repo root without installing the package
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from move_translator.board_geometry import (
    BOARD_SURFACE_Z,
    SQUARE_SIZE,
    GraveyardAllocator,
    ReserveRegistry,
    WorldPose,
    distance_2d,
    get_piece_geometry,
    square_to_grasp_pose,
Unit tests for board_geometry module.

Run with:  pytest test_board_geometry.py -v
"""

import math
import pytest

from move_translator.board_geometry import (
    BoardConfig,
    DEFAULT_CONFIG,
    GraveyardAllocator,
    PIECE_HEIGHTS,
    PIECE_NAMES,
    Pose3D,
    get_piece_color,
    get_piece_full_name,
    get_piece_height,
    make_pick_pose,
    make_place_pose,
    piece_grasp_z,
    piece_place_z,
    square_to_indices,
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
    @pytest.mark.parametrize("char,expected_height", [
        ("P", 0.045), ("p", 0.045),
        ("R", 0.055), ("r", 0.055),
        ("N", 0.060), ("n", 0.060),
        ("B", 0.065), ("b", 0.065),
        ("Q", 0.080), ("q", 0.080),
        ("K", 0.095), ("k", 0.095),
    ])
    def test_heights(self, char, expected_height):
        g = get_piece_geometry(char)
        assert g.height_m == pytest.approx(expected_height, abs=1e-6)

    @pytest.mark.parametrize("char,expected_grasp_z", [
        ("P", 0.789), ("R", 0.795), ("N", 0.798),
        ("B", 0.801), ("Q", 0.810), ("K", 0.819),
    ])
    def test_grasp_z(self, char, expected_grasp_z):
        g = get_piece_geometry(char)
        assert g.grasp_z == pytest.approx(expected_grasp_z, abs=1e-3)

    def test_invalid_char(self):
        with pytest.raises(ValueError):
            get_piece_geometry("X")

    def test_upper_lower_same(self):
        assert get_piece_geometry("Q").height_m == get_piece_geometry("q").height_m


# ---------------------------------------------------------------------------
# square_to_grasp_pose
# ---------------------------------------------------------------------------

class TestGraspPose:
    def test_pawn_grasp_z(self):
        pose = square_to_grasp_pose("e2", "P")
        assert pose.z == pytest.approx(0.789, abs=1e-3)

    def test_queen_grasp_z(self):
        pose = square_to_grasp_pose("d1", "Q")
        assert pose.z == pytest.approx(0.810, abs=1e-3)

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
# ============================================================
# BoardConfig
# ============================================================

class TestBoardConfig:
    def test_defaults_match_world_sdf(self):
        cfg = BoardConfig()
        assert cfg.square_size == 0.05
        assert cfg.surface_z == 0.762
        assert cfg.a1_center_x == -0.175
        assert cfg.a1_center_y == -0.175

    def test_from_yaml(self, tmp_path):
        yaml_content = """
board:
  square_size: 0.06
  surface_z: 0.800
  a1_center: [-0.200, -0.200]
"""
        p = tmp_path / "test_params.yaml"
        p.write_text(yaml_content)
        cfg = BoardConfig.from_yaml(str(p))
        assert cfg.square_size == 0.06
        assert cfg.surface_z == 0.800
        assert cfg.a1_center_x == -0.200
        assert cfg.a1_center_y == -0.200

    def test_from_yaml_missing_fields_use_defaults(self, tmp_path):
        yaml_content = """
board:
  square_size: 0.04
"""
        p = tmp_path / "partial.yaml"
        p.write_text(yaml_content)
        cfg = BoardConfig.from_yaml(str(p))
        assert cfg.square_size == 0.04
        assert cfg.surface_z == BoardConfig.surface_z  # default


# ============================================================
# Pose3D
# ============================================================

class TestPose3D:
    def test_as_dict_rounding(self):
        p = Pose3D(x=0.123456789, y=-0.987654321, z=0.762, yaw=3.14159)
        d = p.as_dict()
        assert d["x"] == 0.123457
        assert d["y"] == -0.987654
        assert d["yaw"] == 3.1416

    def test_default_yaw_is_zero(self):
        p = Pose3D(x=0, y=0, z=0)
        assert p.yaw == 0.0


# ============================================================
# square_to_indices
# ============================================================

class TestSquareToIndices:
    def test_a1(self):
        assert square_to_indices("a1") == (0, 0)

    def test_h8(self):
        assert square_to_indices("h8") == (7, 7)

    def test_e4(self):
        assert square_to_indices("e4") == (4, 3)

    def test_d7(self):
        assert square_to_indices("d7") == (3, 6)

    def test_uppercase_file_accepted(self):
        assert square_to_indices("E4") == (4, 3)

    def test_invalid_file_raises(self):
        with pytest.raises(ValueError, match="Invalid file"):
            square_to_indices("z1")

    def test_invalid_rank_raises(self):
        with pytest.raises(ValueError, match="Invalid rank"):
            square_to_indices("a0")

    def test_too_short_raises(self):
        with pytest.raises(ValueError, match="2 characters"):
            square_to_indices("a")

    def test_too_long_raises(self):
        with pytest.raises(ValueError, match="2 characters"):
            square_to_indices("a10")

    def test_rank_9_raises(self):
        with pytest.raises(ValueError, match="Invalid rank"):
            square_to_indices("a9")


# ============================================================
# square_to_world
# ============================================================

class TestSquareToWorld:
    def test_a1_coordinates(self):
        x, y, z = square_to_world("a1")
        assert x == pytest.approx(-0.175)
        assert y == pytest.approx(-0.175)
        assert z == pytest.approx(0.762)

    def test_h8_coordinates(self):
        x, y, z = square_to_world("h8")
        assert x == pytest.approx(0.175)
        assert y == pytest.approx(0.175)
        assert z == pytest.approx(0.762)

    def test_e4_coordinates(self):
        x, y, z = square_to_world("e4")
        # file e = index 4 → -0.175 + 4*0.05 = 0.025
        # rank 4 = index 3 → -0.175 + 3*0.05 = -0.025
        assert x == pytest.approx(0.025)
        assert y == pytest.approx(-0.025)

    def test_e1_coordinates(self):
        x, y, z = square_to_world("e1")
        assert x == pytest.approx(0.025)
        assert y == pytest.approx(-0.175)

    def test_d8_coordinates(self):
        x, y, z = square_to_world("d8")
        # file d = index 3 → -0.175 + 3*0.05 = -0.025
        # rank 8 = index 7 → -0.175 + 7*0.05 = 0.175
        assert x == pytest.approx(-0.025)
        assert y == pytest.approx(0.175)

    def test_all_z_values_equal_surface(self):
        for f in "abcdefgh":
            for r in "12345678":
                _, _, z = square_to_world(f"{f}{r}")
                assert z == pytest.approx(0.762)

    def test_custom_config(self):
        cfg = BoardConfig(square_size=0.10, a1_center_x=0.0, a1_center_y=0.0, surface_z=1.0)
        x, y, z = square_to_world("b2", cfg)
        assert x == pytest.approx(0.10)
        assert y == pytest.approx(0.10)
        assert z == pytest.approx(1.0)

    def test_adjacent_squares_differ_by_square_size(self):
        x1, y1, _ = square_to_world("d4")
        x2, y2, _ = square_to_world("e4")
        assert (x2 - x1) == pytest.approx(0.05)
        assert y2 == pytest.approx(y1)

        x3, y3, _ = square_to_world("d5")
        assert x3 == pytest.approx(x1)
        assert (y3 - y1) == pytest.approx(0.05)


# ============================================================
# Piece height and Z helpers
# ============================================================

class TestPieceHeights:
    def test_known_pieces(self):
        assert get_piece_height("p") == 0.045
        assert get_piece_height("P") == 0.045
        assert get_piece_height("K") == 0.095
        assert get_piece_height("q") == 0.080

    def test_unknown_piece_returns_default(self):
        assert get_piece_height("x") == 0.05

    def test_grasp_z_pawn(self):
        z = piece_grasp_z("P")
        expected = 0.762 + 0.045 * 0.6
        assert z == pytest.approx(expected)

    def test_grasp_z_king(self):
        z = piece_grasp_z("k")
        expected = 0.762 + 0.095 * 0.6
        assert z == pytest.approx(expected)

    def test_place_z_above_grasp_z(self):
        # Place Z should be close to but not exactly equal to grasp Z
        for char in "prnbqk":
            gz = piece_grasp_z(char)
            pz = piece_place_z(char)
            # Both should be above the board surface
            assert gz > 0.762
            assert pz > 0.762

    def test_place_z_includes_clearance(self):
        cfg = BoardConfig(place_clearance=0.010)
        z = piece_place_z("p", cfg)
        expected = 0.762 + 0.045 * 0.5 + 0.010
        assert z == pytest.approx(expected)


# ============================================================
# Piece color and name helpers
# ============================================================

class TestPieceColorAndName:
    def test_white_pieces(self):
        for char in "PRNBQK":
            assert get_piece_color(char) == "white"

    def test_black_pieces(self):
        for char in "prnbqk":
            assert get_piece_color(char) == "black"

    def test_full_name_white_pawn(self):
        assert get_piece_full_name("P") == "white pawn"

    def test_full_name_black_queen(self):
        assert get_piece_full_name("q") == "black queen"

    def test_full_name_unknown(self):
        assert get_piece_full_name("X") == "white unknown"


# ============================================================
# Pose convenience builders
# ============================================================

class TestMakePose:
    def test_pick_pose_white_pawn_e2(self):
        pose = make_pick_pose("e2", "P")
        assert pose.x == pytest.approx(0.025)
        assert pose.y == pytest.approx(-0.125)
        assert pose.z == pytest.approx(piece_grasp_z("P"))
        assert pose.yaw == pytest.approx(0.0)

    def test_pick_pose_black_rook_a8(self):
        pose = make_pick_pose("a8", "r")
        assert pose.x == pytest.approx(-0.175)
        assert pose.y == pytest.approx(0.175)
        assert pose.yaw == pytest.approx(math.pi)

    def test_place_pose_uses_place_z(self):
        pick = make_pick_pose("d4", "Q")
        place = make_place_pose("d4", "Q")
        # Same XY, different Z
        assert pick.x == pytest.approx(place.x)
        assert pick.y == pytest.approx(place.y)
        assert pick.z != place.z  # grasp vs place height differ

    def test_place_pose_white_yaw(self):
        pose = make_place_pose("e4", "N")
        assert pose.yaw == pytest.approx(0.0)

    def test_place_pose_black_yaw(self):
        pose = make_place_pose("e5", "n")
        assert pose.yaw == pytest.approx(math.pi)


# ============================================================
# GraveyardAllocator
# ============================================================

class TestGraveyardAllocator:
    def test_first_white_capture(self):
        ga = GraveyardAllocator()
        pose = ga.allocate("white")
        # Captured by white → black piece → goes to black's side (-X)
        assert pose.x == pytest.approx(-0.26)
        assert pose.y == pytest.approx(-0.175)
        assert pose.z > 0.762
        assert ga.white_count == 1
        assert ga.black_count == 0

    def test_first_black_capture(self):
        ga = GraveyardAllocator()
        pose = ga.allocate("black")
        # Captured by black → white piece → goes to white's side (+X)
        assert pose.x == pytest.approx(0.26)
        assert pose.y == pytest.approx(-0.175)
        assert ga.black_count == 1

    def test_sequential_allocation_fills_grid(self):
        ga = GraveyardAllocator()
        poses = [ga.allocate("white") for _ in range(4)]

        # Slot 0: row 0 col 0
        # Slot 1: row 0 col 1
        # Slot 2: row 1 col 0
        # Slot 3: row 1 col 1
        assert poses[0].y == pytest.approx(poses[1].y)  # same row
        assert poses[0].x != pytest.approx(poses[1].x)  # different col
        assert poses[2].y == pytest.approx(poses[3].y)  # same row
        assert poses[2].y > poses[0].y                   # next row

    def test_white_and_black_independent(self):
        ga = GraveyardAllocator()
        w1 = ga.allocate("white")
        b1 = ga.allocate("black")
        w2 = ga.allocate("white")

        assert ga.white_count == 2
        assert ga.black_count == 1
        # They should be on opposite sides of the board
        assert w1.x < 0  # black's side
        assert b1.x > 0  # white's side

    def test_reset_clears_counters(self):
        ga = GraveyardAllocator()
        ga.allocate("white")
        ga.allocate("black")
        ga.reset()
        assert ga.white_count == 0
        assert ga.black_count == 0

    def test_overflow_raises(self):
        ga = GraveyardAllocator()
        for _ in range(16):
            ga.allocate("white")
        with pytest.raises(RuntimeError, match="full"):
            ga.allocate("white")

    def test_invalid_color_raises(self):
        ga = GraveyardAllocator()
        with pytest.raises(ValueError, match="captured_by"):
            ga.allocate("red")

    def test_all_16_slots_unique(self):
        ga = GraveyardAllocator()
        poses = [ga.allocate("white") for _ in range(16)]
        coords = [(p.x, p.y) for p in poses]
        assert len(set(coords)) == 16  # all unique positions
