"""
test_board_geometry.py
======================
Unit tests for board_geometry.py.  No ROS required.

Run with:
    pytest board_localization/test/test_board_geometry.py -v
"""

import math
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from board_localization.board_geometry import (
    A1_X, A1_Y, BOARD_SURFACE_Z, PANDA_MAX_REACH, SQUARE_SIZE,
    WHITE_ARM_BASE, BLACK_ARM_BASE,
    BoardPose,
    all_graveyard_poses,
    all_square_poses,
    all_squares,
    apply_board_transform,
    arm_reach_to_square,
    board_pose_to_square,
    estimate_board_origin,
    graveyard_slot_pose,
    graveyard_tf_frame,
    square_to_board_pose,
    square_to_tf_frame,
    tf_frame_to_square,
    validate_arm_reach,
    worst_case_reach,
)


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

class TestConstants:
    def test_board_surface_z(self):
        assert BOARD_SURFACE_Z == pytest.approx(0.762, abs=1e-9)

    def test_square_size(self):
        assert SQUARE_SIZE == pytest.approx(0.05, abs=1e-9)

    def test_a1_x(self):
        assert A1_X == pytest.approx(-0.175, abs=1e-9)

    def test_a1_y(self):
        assert A1_Y == pytest.approx(-0.175, abs=1e-9)

    def test_board_spans_correct_range(self):
        # a1 to h8 span = 7 * 0.05 = 0.35 m each axis
        assert (A1_X + 7 * SQUARE_SIZE) == pytest.approx(+0.175, abs=1e-6)
        assert (A1_Y + 7 * SQUARE_SIZE) == pytest.approx(+0.175, abs=1e-6)


# ---------------------------------------------------------------------------
# square_to_board_pose
# ---------------------------------------------------------------------------

class TestSquareToBoardPose:
    def test_a1(self):
        p = square_to_board_pose("a1")
        assert p.x == pytest.approx(-0.175, abs=1e-6)
        assert p.y == pytest.approx(-0.175, abs=1e-6)
        assert p.z == pytest.approx(BOARD_SURFACE_Z, abs=1e-6)

    def test_h8(self):
        p = square_to_board_pose("h8")
        assert p.x == pytest.approx(+0.175, abs=1e-6)
        assert p.y == pytest.approx(+0.175, abs=1e-6)

    def test_e1(self):
        p = square_to_board_pose("e1")
        assert p.x == pytest.approx(+0.025, abs=1e-6)
        assert p.y == pytest.approx(-0.175, abs=1e-6)

    def test_e4(self):
        p = square_to_board_pose("e4")
        assert p.x == pytest.approx(+0.025, abs=1e-6)
        assert p.y == pytest.approx(-0.025, abs=1e-6)

    def test_d4(self):
        p = square_to_board_pose("d4")
        assert p.x == pytest.approx(-0.025, abs=1e-6)
        assert p.y == pytest.approx(-0.025, abs=1e-6)

    def test_h1(self):
        p = square_to_board_pose("h1")
        assert p.x == pytest.approx(+0.175, abs=1e-6)
        assert p.y == pytest.approx(-0.175, abs=1e-6)

    def test_a8(self):
        p = square_to_board_pose("a8")
        assert p.x == pytest.approx(-0.175, abs=1e-6)
        assert p.y == pytest.approx(+0.175, abs=1e-6)

    def test_e8(self):
        p = square_to_board_pose("e8")
        assert p.x == pytest.approx(+0.025, abs=1e-6)
        assert p.y == pytest.approx(+0.175, abs=1e-6)

    def test_default_orientation_is_identity(self):
        p = square_to_board_pose("d4")
        assert p.qw == pytest.approx(1.0, abs=1e-9)
        assert p.qx == pytest.approx(0.0, abs=1e-9)
        assert p.qy == pytest.approx(0.0, abs=1e-9)
        assert p.qz == pytest.approx(0.0, abs=1e-9)

    def test_uppercase_normalised(self):
        p_lower = square_to_board_pose("e4")
        p_upper = square_to_board_pose("E4")
        assert p_lower == p_upper

    def test_invalid_file(self):
        with pytest.raises(ValueError, match="Invalid file"):
            square_to_board_pose("z4")

    def test_invalid_rank(self):
        with pytest.raises(ValueError, match="Invalid rank"):
            square_to_board_pose("a9")

    def test_wrong_length(self):
        with pytest.raises(ValueError):
            square_to_board_pose("e44")

    def test_all_64_squares_unique_xy(self):
        seen = {}
        for sq in all_squares():
            p = square_to_board_pose(sq)
            key = (round(p.x, 6), round(p.y, 6))
            assert key not in seen, f"Duplicate XY for {sq}: {key}"
            seen[key] = sq

    def test_adjacent_squares_5cm_apart(self):
        p_e4 = square_to_board_pose("e4")
        p_f4 = square_to_board_pose("f4")
        p_e5 = square_to_board_pose("e5")
        assert abs(p_f4.x - p_e4.x) == pytest.approx(SQUARE_SIZE, abs=1e-6)
        assert abs(p_e5.y - p_e4.y) == pytest.approx(SQUARE_SIZE, abs=1e-6)

    def test_all_squares_at_board_surface_z(self):
        for sq in all_squares():
            assert square_to_board_pose(sq).z == pytest.approx(BOARD_SURFACE_Z, abs=1e-9)

    def test_to_dict_structure(self):
        d = square_to_board_pose("e4").to_dict()
        assert "position" in d
        assert "orientation" in d
        assert "x" in d["position"]
        assert "w" in d["orientation"]


# ---------------------------------------------------------------------------
# board_pose_to_square (reverse mapping)
# ---------------------------------------------------------------------------

class TestBoardPoseToSquare:
    def test_exact_centres_round_trip(self):
        for sq in all_squares():
            p = square_to_board_pose(sq)
            result = board_pose_to_square(p.x, p.y)
            assert result == sq, f"Round-trip failed for {sq}: got {result}"

    def test_within_tolerance(self):
        p = square_to_board_pose("e4")
        # 1 cm offset — within the default 2 cm tolerance
        result = board_pose_to_square(p.x + 0.009, p.y - 0.009)
        assert result == "e4"

    def test_outside_board_raises(self):
        with pytest.raises(ValueError, match="outside the board"):
            board_pose_to_square(1.0, 1.0)

    def test_outside_tolerance_raises(self):
        # Place the point exactly on the boundary between e4 and f4 (2.5 cm from each)
        # With tolerance_m=0.02 (2 cm) this should raise because 2.5 cm > 2 cm
        p_e4 = square_to_board_pose("e4")
        p_f4 = square_to_board_pose("f4")
        midpoint_x = (p_e4.x + p_f4.x) / 2.0  # exactly 2.5 cm from each centre
        with pytest.raises(ValueError, match="deviates"):
            board_pose_to_square(midpoint_x, p_e4.y, tolerance_m=0.02)


# ---------------------------------------------------------------------------
# TF frame name conversions
# ---------------------------------------------------------------------------

class TestTFFrameNames:
    def test_square_to_tf_frame(self):
        assert square_to_tf_frame("e4") == "square_e4"
        assert square_to_tf_frame("A1") == "square_a1"
        assert square_to_tf_frame("H8") == "square_h8"

    def test_tf_frame_to_square(self):
        assert tf_frame_to_square("square_e4") == "e4"
        assert tf_frame_to_square("square_a1") == "a1"

    def test_tf_frame_round_trip(self):
        for sq in all_squares():
            frame = square_to_tf_frame(sq)
            recovered = tf_frame_to_square(frame)
            assert recovered == sq

    def test_non_square_frame_raises(self):
        with pytest.raises(ValueError, match="not a square frame"):
            tf_frame_to_square("panda_link0")

    def test_all_64_tf_frames_unique(self):
        frames = [square_to_tf_frame(sq) for sq in all_squares()]
        assert len(set(frames)) == 64


# ---------------------------------------------------------------------------
# all_squares / all_square_poses
# ---------------------------------------------------------------------------

class TestAllSquares:
    def test_exactly_64_squares(self):
        squares = list(all_squares())
        assert len(squares) == 64

    def test_all_valid_notation(self):
        for sq in all_squares():
            assert len(sq) == 2
            assert sq[0] in "abcdefgh"
            assert sq[1] in "12345678"

    def test_all_square_poses_has_64_entries(self):
        poses = all_square_poses()
        assert len(poses) == 64

    def test_all_square_poses_keys_match_squares(self):
        poses = all_square_poses()
        for sq in all_squares():
            assert sq in poses


# ---------------------------------------------------------------------------
# Graveyard
# ---------------------------------------------------------------------------

class TestGraveyard:
    def test_white_slot_0_is_left_of_board(self):
        p = graveyard_slot_pose("white", 0)
        assert p.x < A1_X, "White graveyard should be left of the a-file"

    def test_black_slot_0_is_right_of_board(self):
        p = graveyard_slot_pose("black", 0)
        assert p.x > -A1_X, "Black graveyard should be right of the h-file"

    def test_slots_advance_in_y(self):
        poses = [graveyard_slot_pose("white", i) for i in range(4)]
        for i in range(1, 4):
            assert poses[i].y > poses[i-1].y

    def test_slot_spacing_is_square_size(self):
        p0 = graveyard_slot_pose("black", 0)
        p1 = graveyard_slot_pose("black", 1)
        assert abs(p1.y - p0.y) == pytest.approx(SQUARE_SIZE, abs=1e-6)

    def test_16_slots_per_side(self):
        for color in ("white", "black"):
            for i in range(16):
                graveyard_slot_pose(color, i)  # should not raise

    def test_invalid_color_raises(self):
        with pytest.raises(ValueError, match="color"):
            graveyard_slot_pose("green", 0)

    def test_invalid_slot_raises(self):
        with pytest.raises(ValueError, match="slot_index"):
            graveyard_slot_pose("white", 16)

    def test_tf_frame_name(self):
        assert graveyard_tf_frame("white", 0)  == "graveyard_white_0"
        assert graveyard_tf_frame("black", 15) == "graveyard_black_15"

    def test_all_graveyard_poses_32_entries(self):
        poses = all_graveyard_poses()
        assert len(poses) == 32

    def test_all_graveyard_poses_at_board_z(self):
        for frame, pose in all_graveyard_poses().items():
            assert pose.z == pytest.approx(BOARD_SURFACE_Z, abs=1e-6), frame


# ---------------------------------------------------------------------------
# Reach analysis
# ---------------------------------------------------------------------------

class TestReachAnalysis:
    def test_white_arm_reaches_all_squares(self):
        for sq in all_squares():
            d = arm_reach_to_square(WHITE_ARM_BASE, sq)
            assert d < PANDA_MAX_REACH, (
                f"White arm cannot reach {sq}: {d:.4f}m > {PANDA_MAX_REACH}m"
            )

    def test_black_arm_reaches_all_squares(self):
        for sq in all_squares():
            d = arm_reach_to_square(BLACK_ARM_BASE, sq)
            assert d < PANDA_MAX_REACH, (
                f"Black arm cannot reach {sq}: {d:.4f}m > {PANDA_MAX_REACH}m"
            )

    def test_worst_case_within_reach(self):
        for base in (WHITE_ARM_BASE, BLACK_ARM_BASE):
            sq, dist = worst_case_reach(base)
            assert dist < PANDA_MAX_REACH

    def test_validate_arm_reach_no_warnings(self):
        warnings = validate_arm_reach()
        assert warnings == [], f"Unexpected reach warnings: {warnings}"

    def test_closest_square_for_white(self):
        # White arm is at y=-0.45; closest rank is rank 1 (y=-0.175)
        # a1 or h1 should be among the nearest
        h1_d = arm_reach_to_square(WHITE_ARM_BASE, "h1")
        a8_d = arm_reach_to_square(WHITE_ARM_BASE, "a8")
        assert h1_d < a8_d, "Rank 1 squares should be closer than rank 8 for white"


# ---------------------------------------------------------------------------
# apply_board_transform (calibration path)
# ---------------------------------------------------------------------------

class TestApplyBoardTransform:
    def test_zero_yaw_matches_analytic(self):
        """With nominal origin and zero yaw, transform matches analytic."""
        origin = (0.0, 0.0, BOARD_SURFACE_Z)
        for sq in all_squares():
            transformed = apply_board_transform(sq, origin, 0.0)
            analytic    = square_to_board_pose(sq)
            assert transformed.x == pytest.approx(analytic.x, abs=1e-5), sq
            assert transformed.y == pytest.approx(analytic.y, abs=1e-5), sq

    def test_translation_offset(self):
        """A pure translation shifts all squares equally."""
        shift_x, shift_y = 0.1, 0.2
        origin = (shift_x, shift_y, BOARD_SURFACE_Z)
        for sq in ["a1", "h8", "e4"]:
            transformed = apply_board_transform(sq, origin, 0.0)
            analytic    = square_to_board_pose(sq)
            assert transformed.x == pytest.approx(analytic.x + shift_x, abs=1e-5)
            assert transformed.y == pytest.approx(analytic.y + shift_y, abs=1e-5)

    def test_90_degree_rotation(self):
        """90° yaw: a1 (bottom-left) maps to h1 (bottom-right) of rotated board."""
        origin = (0.0, 0.0, BOARD_SURFACE_Z)
        a1_rot = apply_board_transform("a1", origin, math.pi / 2)
        h8_rot = apply_board_transform("h8", origin, math.pi / 2)
        # After 90° CCW, a1(-3.5,-3.5)*sq → (+3.5, -3.5)*sq  ≈ (h, 1) position
        assert a1_rot.x == pytest.approx(+0.175, abs=1e-4)
        assert a1_rot.y == pytest.approx(-0.175, abs=1e-4)

    def test_transform_preserves_square_spacing(self):
        """Spacing between adjacent squares must be preserved under rotation."""
        origin = (0.0, 0.0, BOARD_SURFACE_Z)
        yaw = math.radians(15)
        p_e4 = apply_board_transform("e4", origin, yaw)
        p_f4 = apply_board_transform("f4", origin, yaw)
        d = math.hypot(p_f4.x - p_e4.x, p_f4.y - p_e4.y)
        assert d == pytest.approx(SQUARE_SIZE, abs=1e-5)


# ---------------------------------------------------------------------------
# estimate_board_origin
# ---------------------------------------------------------------------------

class TestEstimateBoardOrigin:
    def test_nominal_corners_give_zero_offset_zero_yaw(self):
        a1 = square_to_board_pose("a1")
        h8 = square_to_board_pose("h8")
        cx, cy, cz, yaw = estimate_board_origin(
            (a1.x, a1.y, a1.z),
            (h8.x, h8.y, h8.z),
        )
        assert cx == pytest.approx(0.0, abs=1e-5)
        assert cy == pytest.approx(0.0, abs=1e-5)
        assert yaw == pytest.approx(0.0, abs=1e-4)

    def test_offset_board_detected(self):
        shift = 0.05
        a1 = square_to_board_pose("a1")
        h8 = square_to_board_pose("h8")
        cx, cy, _, _ = estimate_board_origin(
            (a1.x + shift, a1.y + shift, a1.z),
            (h8.x + shift, h8.y + shift, h8.z),
        )
        assert cx == pytest.approx(shift, abs=1e-5)
        assert cy == pytest.approx(shift, abs=1e-5)

    def test_rotated_board_detected(self):
        yaw_in = math.radians(10)
        origin = (0.0, 0.0, BOARD_SURFACE_Z)
        a1_rot = apply_board_transform("a1", origin, yaw_in)
        h8_rot = apply_board_transform("h8", origin, yaw_in)
        _, _, _, yaw_out = estimate_board_origin(
            (a1_rot.x, a1_rot.y, a1_rot.z),
            (h8_rot.x, h8_rot.y, h8_rot.z),
        )
        assert yaw_out == pytest.approx(yaw_in, abs=1e-4)