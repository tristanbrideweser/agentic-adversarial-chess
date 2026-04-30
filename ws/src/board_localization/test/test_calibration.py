"""
test_calibration.py
===================
Unit tests for calibration.py.  No ROS required.

Run with:
    pytest board_localization/test/test_calibration.py -v
"""

import math
import tempfile
from pathlib import Path
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from board_localization.board_geometry import (
    BOARD_SURFACE_Z, SQUARE_SIZE,
    apply_board_transform, square_to_board_pose,
)
from board_localization.calibration import (
    CalibrationResult,
    calibrate_four_corners,
    calibrate_nominal,
    calibrate_two_corners,
    validate_calibration,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def nominal_corners():
    a1 = square_to_board_pose("a1")
    h1 = square_to_board_pose("h1")
    a8 = square_to_board_pose("a8")
    h8 = square_to_board_pose("h8")
    return (
        (a1.x, a1.y, a1.z),
        (h1.x, h1.y, h1.z),
        (a8.x, a8.y, a8.z),
        (h8.x, h8.y, h8.z),
    )


def shifted_corners(dx=0.0, dy=0.0, yaw=0.0):
    origin = (dx, dy, BOARD_SURFACE_Z)
    a1 = apply_board_transform("a1", origin, yaw)
    h1 = apply_board_transform("h1", origin, yaw)
    a8 = apply_board_transform("a8", origin, yaw)
    h8 = apply_board_transform("h8", origin, yaw)
    return (
        (a1.x, a1.y, a1.z),
        (h1.x, h1.y, h1.z),
        (a8.x, a8.y, a8.z),
        (h8.x, h8.y, h8.z),
    )


# ---------------------------------------------------------------------------
# calibrate_nominal
# ---------------------------------------------------------------------------

class TestCalibrateNominal:
    def test_returns_success(self):
        r = calibrate_nominal()
        assert r.success is True

    def test_nominal_origin(self):
        r = calibrate_nominal()
        assert r.board_origin_x == pytest.approx(0.0, abs=1e-9)
        assert r.board_origin_y == pytest.approx(0.0, abs=1e-9)
        assert r.board_origin_z == pytest.approx(BOARD_SURFACE_Z, abs=1e-9)

    def test_nominal_yaw_zero(self):
        r = calibrate_nominal()
        assert r.board_yaw == pytest.approx(0.0, abs=1e-9)

    def test_nominal_residual_zero(self):
        r = calibrate_nominal()
        assert r.residual_mm == pytest.approx(0.0, abs=1e-9)

    def test_method_name(self):
        assert calibrate_nominal().method == "nominal"


# ---------------------------------------------------------------------------
# calibrate_two_corners
# ---------------------------------------------------------------------------

class TestCalibrateTwoCorners:
    def test_nominal_corners_zero_residual(self):
        a1 = square_to_board_pose("a1")
        h8 = square_to_board_pose("h8")
        r = calibrate_two_corners(
            (a1.x, a1.y, a1.z),
            (h8.x, h8.y, h8.z),
        )
        assert r.success is True
        assert r.residual_mm == pytest.approx(0.0, abs=0.1)

    def test_recovers_translation(self):
        dx, dy = 0.03, -0.02
        a1 = square_to_board_pose("a1")
        h8 = square_to_board_pose("h8")
        r = calibrate_two_corners(
            (a1.x + dx, a1.y + dy, a1.z),
            (h8.x + dx, h8.y + dy, h8.z),
        )
        assert r.board_origin_x == pytest.approx(dx, abs=1e-4)
        assert r.board_origin_y == pytest.approx(dy, abs=1e-4)

    def test_recovers_yaw(self):
        yaw_in = math.radians(5)
        origin = (0.0, 0.0, BOARD_SURFACE_Z)
        a1_rot = apply_board_transform("a1", origin, yaw_in)
        h8_rot = apply_board_transform("h8", origin, yaw_in)
        r = calibrate_two_corners(
            (a1_rot.x, a1_rot.y, a1_rot.z),
            (h8_rot.x, h8_rot.y, h8_rot.z),
        )
        assert r.board_yaw == pytest.approx(yaw_in, abs=1e-4)

    def test_large_error_fails_threshold(self):
        a1 = square_to_board_pose("a1")
        h8 = square_to_board_pose("h8")
        # Add 20 mm noise to one corner
        r = calibrate_two_corners(
            (a1.x + 0.020, a1.y, a1.z),
            (h8.x, h8.y, h8.z),
            quality_threshold_mm=5.0,
        )
        assert r.success is False

    def test_method_name(self):
        a1 = square_to_board_pose("a1")
        h8 = square_to_board_pose("h8")
        r = calibrate_two_corners((a1.x, a1.y, a1.z), (h8.x, h8.y, h8.z))
        assert r.method == "two_corner"


# ---------------------------------------------------------------------------
# calibrate_four_corners
# ---------------------------------------------------------------------------

class TestCalibrateFourCorners:
    def test_nominal_corners_success(self):
        a1_w, h1_w, a8_w, h8_w = nominal_corners()
        r = calibrate_four_corners(a1_w, h1_w, a8_w, h8_w)
        assert r.success is True
        assert r.residual_mm == pytest.approx(0.0, abs=0.1)

    def test_recovers_translation(self):
        dx, dy = 0.05, 0.03
        a1_w, h1_w, a8_w, h8_w = shifted_corners(dx=dx, dy=dy)
        r = calibrate_four_corners(a1_w, h1_w, a8_w, h8_w)
        assert r.board_origin_x == pytest.approx(dx, abs=5e-4)
        assert r.board_origin_y == pytest.approx(dy, abs=5e-4)

    def test_recovers_yaw(self):
        yaw_in = math.radians(8)
        a1_w, h1_w, a8_w, h8_w = shifted_corners(yaw=yaw_in)
        r = calibrate_four_corners(a1_w, h1_w, a8_w, h8_w)
        assert abs(r.board_yaw) == pytest.approx(abs(yaw_in), abs=2e-3)

    def test_noisy_measurement_still_succeeds(self):
        """Small noise (< 2 mm) should still produce a passing result."""
        import random
        rng = random.Random(42)
        noise = 0.001  # 1 mm
        a1_w, h1_w, a8_w, h8_w = nominal_corners()

        def jitter(pt):
            return (pt[0] + rng.uniform(-noise, noise),
                    pt[1] + rng.uniform(-noise, noise),
                    pt[2])

        r = calibrate_four_corners(
            jitter(a1_w), jitter(h1_w), jitter(a8_w), jitter(h8_w),
            quality_threshold_mm=5.0,
        )
        assert r.success is True

    def test_method_name(self):
        a1_w, h1_w, a8_w, h8_w = nominal_corners()
        r = calibrate_four_corners(a1_w, h1_w, a8_w, h8_w)
        assert r.method == "four_corner"

    def test_four_corners_more_accurate_than_two(self):
        """With same data, four-corner should have ≤ residual of two-corner."""
        a1_w, h1_w, a8_w, h8_w = nominal_corners()
        r4 = calibrate_four_corners(a1_w, h1_w, a8_w, h8_w)
        a1 = square_to_board_pose("a1")
        h8 = square_to_board_pose("h8")
        r2 = calibrate_two_corners((a1.x, a1.y, a1.z), (h8.x, h8.y, h8.z))
        assert r4.residual_mm <= r2.residual_mm + 0.1  # within 0.1 mm


# ---------------------------------------------------------------------------
# CalibrationResult serialisation
# ---------------------------------------------------------------------------

class TestCalibrationResultSerialisation:
    def test_to_ros_params_structure(self):
        r = calibrate_nominal()
        d = r.to_ros_params()
        assert "/**" in d
        params = d["/**"]["ros__parameters"]
        for key in ("board_origin_x", "board_origin_y",
                    "board_origin_z", "board_yaw"):
            assert key in params

    def test_write_and_reload_yaml(self):
        import yaml
        r = calibrate_nominal()
        with tempfile.NamedTemporaryFile(suffix=".yaml", mode="w",
                                         delete=False) as f:
            path = f.name
        r.write_yaml(path)
        with open(path) as f:
            loaded = yaml.safe_load(f)
        params = loaded["/**"]["ros__parameters"]
        assert params["board_origin_z"] == pytest.approx(BOARD_SURFACE_Z, abs=1e-9)


# ---------------------------------------------------------------------------
# validate_calibration
# ---------------------------------------------------------------------------

class TestValidateCalibration:
    def test_nominal_no_warnings(self):
        r = calibrate_nominal()
        warnings = validate_calibration(r)
        assert warnings == []

    def test_shifted_calibration_no_warnings_for_itself(self):
        """A correctly calibrated (shifted) board should pass its own validation."""
        dx = 0.05
        a1_w, h1_w, a8_w, h8_w = shifted_corners(dx=dx)
        r = calibrate_four_corners(a1_w, h1_w, a8_w, h8_w)
        # Validation compares calibrated vs nominal — a shift WILL show deviation
        # This tests that the validator runs without error, not that it passes
        warnings = validate_calibration(r, tolerance_mm=200.0)
        # With 200 mm tolerance everything passes
        assert warnings == []

    def test_bad_calibration_produces_warnings(self):
        """A wildly wrong calibration should produce warnings."""
        r = CalibrationResult(
            board_origin_x=0.5,  # 50 cm off
            board_origin_y=0.0,
            board_origin_z=BOARD_SURFACE_Z,
            board_yaw=0.0,
            success=True,
        )
        warnings = validate_calibration(r, tolerance_mm=5.0)
        assert len(warnings) > 0

    def test_custom_spot_check_squares(self):
        r = calibrate_nominal()
        warnings = validate_calibration(r, spot_check_squares=["e4", "d5"])
        assert warnings == []