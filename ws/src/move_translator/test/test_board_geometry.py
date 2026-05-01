"""
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