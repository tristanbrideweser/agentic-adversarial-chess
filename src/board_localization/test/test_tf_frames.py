"""
test_tf_frames.py
=================
Tests for TF frame naming conventions, full frame coverage, and consistency
with the move_translator board_geometry constants.

No ROS required — tests the pure-Python geometry layer.

Run with:
    pytest board_localization/test/test_tf_frames.py -v
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from src.board_localization.board_localization.board_geometry import (
    BOARD_FRAME,
    BOARD_SURFACE_Z,
    SQUARE_SIZE,
    WORLD_FRAME,
    A1_X, A1_Y,
    all_graveyard_poses,
    all_square_poses,
    all_squares,
    graveyard_tf_frame,
    square_to_board_pose,
    square_to_tf_frame,
    tf_frame_to_square,
)


# ---------------------------------------------------------------------------
# Frame name conventions
# ---------------------------------------------------------------------------

class TestFrameNamingConventions:
    """All TF frame names must match the architecture spec exactly."""

    def test_world_frame_name(self):
        assert WORLD_FRAME == "world"

    def test_board_frame_name(self):
        assert BOARD_FRAME == "chess_board"

    def test_square_frame_prefix(self):
        for sq in all_squares():
            frame = square_to_tf_frame(sq)
            assert frame.startswith("square_"), (
                f"Expected 'square_' prefix, got '{frame}'"
            )

    def test_square_frame_suffix_is_algebraic(self):
        for sq in all_squares():
            frame = square_to_tf_frame(sq)
            suffix = frame[len("square_"):]
            assert len(suffix) == 2
            assert suffix[0] in "abcdefgh"
            assert suffix[1] in "12345678"

    def test_graveyard_frame_format_white(self):
        for i in range(16):
            frame = graveyard_tf_frame("white", i)
            assert frame == f"graveyard_white_{i}"

    def test_graveyard_frame_format_black(self):
        for i in range(16):
            frame = graveyard_tf_frame("black", i)
            assert frame == f"graveyard_black_{i}"

    def test_specific_frames_from_architecture(self):
        """Spot-check the exact frame names listed in the architecture doc §2.4.2."""
        assert square_to_tf_frame("a1") == "square_a1"
        assert square_to_tf_frame("h8") == "square_h8"
        assert graveyard_tf_frame("white", 0)  == "graveyard_white_0"
        assert graveyard_tf_frame("black", 15) == "graveyard_black_15"


# ---------------------------------------------------------------------------
# Full frame coverage: 64 + 32 = 96 child frames under chess_board
# (plus the world→chess_board root = 97 total transforms)
# ---------------------------------------------------------------------------

class TestFullFrameCoverage:
    def test_64_square_frames(self):
        poses = all_square_poses()
        assert len(poses) == 64

    def test_32_graveyard_frames(self):
        poses = all_graveyard_poses()
        assert len(poses) == 32

    def test_total_child_frames(self):
        # 64 squares + 32 graveyard = 96 child frames of chess_board
        total = len(all_square_poses()) + len(all_graveyard_poses())
        assert total == 96

    def test_no_frame_name_collisions(self):
        sq_frames = {square_to_tf_frame(sq) for sq in all_squares()}
        gy_frames = set(all_graveyard_poses().keys())
        overlap = sq_frames & gy_frames
        assert overlap == set(), f"Frame name collisions: {overlap}"

    def test_all_frame_names_are_strings(self):
        for frame in all_square_poses():
            assert isinstance(frame, str)
        for frame in all_graveyard_poses():
            assert isinstance(frame, str)


# ---------------------------------------------------------------------------
# Coordinate consistency with architecture spec
# ---------------------------------------------------------------------------

class TestCoordinateSpec:
    """
    Verify specific coordinates from architecture §6.3 (world-frame table).
    """

    @pytest.mark.parametrize("square,expected_x,expected_y", [
        ("a1", -0.175, -0.175),
        ("e1", +0.025, -0.175),
        ("h1", +0.175, -0.175),
        ("d4", -0.025, -0.025),
        ("e4", +0.025, -0.025),
        ("a8", -0.175, +0.175),
        ("e8", +0.025, +0.175),
        ("h8", +0.175, +0.175),
    ])
    def test_architecture_coordinate_table(self, square, expected_x, expected_y):
        p = square_to_board_pose(square)
        assert p.x == pytest.approx(expected_x, abs=1e-4), (
            f"{square}: expected x={expected_x}, got {p.x}"
        )
        assert p.y == pytest.approx(expected_y, abs=1e-4), (
            f"{square}: expected y={expected_y}, got {p.y}"
        )

    def test_all_squares_z_is_board_surface(self):
        for sq, pose in all_square_poses().items():
            assert pose.z == pytest.approx(BOARD_SURFACE_Z, abs=1e-9), sq


# ---------------------------------------------------------------------------
# Cross-package consistency
# ---------------------------------------------------------------------------

class TestCrossPackageConsistency:
    """
    The move_translator.board_geometry and grasp_planner.lookup_grasps
    modules carry their own copies of the coordinate constants.  This test
    class verifies they agree with the canonical values here.

    If move_translator or grasp_planner are not installed these tests are
    skipped rather than failing (import guards).
    """

    @pytest.fixture(autouse=True)
    def _skip_if_unavailable(self):
        pass   # individual tests handle their own skipping

    def test_move_translator_a1_matches(self):
        try:
            from move_translator.board_geometry import A1_X as MT_A1_X, A1_Y as MT_A1_Y
        except ImportError:
            pytest.skip("move_translator not installed")
        assert MT_A1_X == pytest.approx(A1_X, abs=1e-9)
        assert MT_A1_Y == pytest.approx(A1_Y, abs=1e-9)

    def test_move_translator_board_z_matches(self):
        try:
            from move_translator.board_geometry import BOARD_SURFACE_Z as MT_Z
        except ImportError:
            pytest.skip("move_translator not installed")
        assert MT_Z == pytest.approx(BOARD_SURFACE_Z, abs=1e-9)

    def test_move_translator_square_size_matches(self):
        try:
            from move_translator.board_geometry import SQUARE_SIZE as MT_SQ
        except ImportError:
            pytest.skip("move_translator not installed")
        assert MT_SQ == pytest.approx(SQUARE_SIZE, abs=1e-9)

    def test_move_translator_square_xy_matches(self):
        try:
            from move_translator.board_geometry import square_to_world
        except ImportError:
            pytest.skip("move_translator not installed")
        for sq in ["a1", "e4", "h8", "d5"]:
            mt = square_to_world(sq)
            bl = square_to_board_pose(sq)
            assert mt.x == pytest.approx(bl.x, abs=1e-6), sq
            assert mt.y == pytest.approx(bl.y, abs=1e-6), sq
            assert mt.z == pytest.approx(bl.z, abs=1e-6), sq

    def test_grasp_planner_square_xy_matches(self):
        try:
            from grasp_planner.lookup_grasps import _square_xy
        except ImportError:
            pytest.skip("grasp_planner not installed")
        for sq in ["a1", "e4", "h8"]:
            gx, gy = _square_xy(sq)
            bl = square_to_board_pose(sq)
            assert gx == pytest.approx(bl.x, abs=1e-6), sq
            assert gy == pytest.approx(bl.y, abs=1e-6), sq