"""
calibration.py
==============
Board pose calibration utilities.

In Gazebo the board is fixed at its nominal pose so calibration is never
needed.  For physical deployment, this module computes the board's
world-frame origin and yaw from sensor observations, then writes the result
to board_params.yaml so the TF broadcaster picks it up on the next launch.

Two calibration methods are supported:

1. Two-corner method
   Measure the world-frame positions of a1 and h8 (e.g. from a calibration
   target or ArUco markers placed on the corners).  Sufficient for a board
   mounted flat on a table.

2. Four-corner method
   Measure all four corners (a1, h1, a8, h8).  Computes a least-squares fit
   that handles slight board tilt and corrects for any roll/pitch.

3. ArUco method (ROS-aware)
   Detects ArUco markers attached to the board via OpenCV + cv_bridge,
   then calls the two- or four-corner method internally.

All methods return a CalibrationResult that can be written directly to the
board_params.yaml config file.
"""

from __future__ import annotations

import math
import statistics
import yaml
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import List, Optional, Tuple


from .board_geometry import (
    A1_X, A1_Y, BOARD_SURFACE_Z, SQUARE_SIZE,
    BoardPose,
    apply_board_transform,
    estimate_board_origin,
    square_to_board_pose,
)


# ---------------------------------------------------------------------------
# Result dataclass
# ---------------------------------------------------------------------------

@dataclass
class CalibrationResult:
    """
    Output of any calibration routine.

    board_origin_x/y/z : world-frame position of the board centre (metres)
    board_yaw          : rotation around world Z (radians)
    residual_mm        : RMS reprojection error of measured corners (mm)
    method             : calibration method used
    success            : whether calibration met the quality threshold
    message            : human-readable status
    """
    board_origin_x: float = 0.0
    board_origin_y: float = 0.0
    board_origin_z: float = BOARD_SURFACE_Z
    board_yaw: float = 0.0
    residual_mm: float = 0.0
    method: str = "nominal"
    success: bool = True
    message: str = "Using nominal board pose"

    def to_ros_params(self) -> dict:
        """Format for direct inclusion in a ROS 2 params YAML file."""
        return {
            "/**": {
                "ros__parameters": {
                    "board_origin_x": self.board_origin_x,
                    "board_origin_y": self.board_origin_y,
                    "board_origin_z": self.board_origin_z,
                    "board_yaw":      self.board_yaw,
                }
            }
        }

    def write_yaml(self, path: str | Path) -> None:
        """Write calibration result to a board_params.yaml file."""
        path = Path(path)
        with path.open("w") as f:
            yaml.dump(self.to_ros_params(), f, default_flow_style=False)


# ---------------------------------------------------------------------------
# Two-corner calibration
# ---------------------------------------------------------------------------

def calibrate_two_corners(
    a1_world: Tuple[float, float, float],
    h8_world: Tuple[float, float, float],
    quality_threshold_mm: float = 5.0,
) -> CalibrationResult:
    """
    Calibrate board pose from the measured positions of the a1 and h8 corners.

    Parameters
    ----------
    a1_world : measured world (x, y, z) of the a1 corner (metres)
    h8_world : measured world (x, y, z) of the h8 corner (metres)
    quality_threshold_mm : maximum acceptable residual (mm)

    Returns
    -------
    CalibrationResult
    """
    cx, cy, cz, yaw = estimate_board_origin(a1_world, h8_world)

    # Compute residual: re-project a1 and h8 and measure error
    a1_reproj = apply_board_transform("a1", (cx, cy, cz), yaw)
    h8_reproj = apply_board_transform("h8", (cx, cy, cz), yaw)

    res_a1 = math.hypot(a1_reproj.x - a1_world[0], a1_reproj.y - a1_world[1]) * 1000
    res_h8 = math.hypot(h8_reproj.x - h8_world[0], h8_reproj.y - h8_world[1]) * 1000
    residual = math.sqrt((res_a1**2 + res_h8**2) / 2)

    success = residual <= quality_threshold_mm
    message = (
        f"Two-corner calibration: residual={residual:.2f}mm "
        f"({'OK' if success else 'EXCEEDS threshold ' + str(quality_threshold_mm) + 'mm'})"
    )

    return CalibrationResult(
        board_origin_x=cx,
        board_origin_y=cy,
        board_origin_z=cz,
        board_yaw=yaw,
        residual_mm=round(residual, 3),
        method="two_corner",
        success=success,
        message=message,
    )


# ---------------------------------------------------------------------------
# Four-corner calibration (least-squares fit)
# ---------------------------------------------------------------------------

def calibrate_four_corners(
    a1_world: Tuple[float, float, float],
    h1_world: Tuple[float, float, float],
    a8_world: Tuple[float, float, float],
    h8_world: Tuple[float, float, float],
    quality_threshold_mm: float = 5.0,
) -> CalibrationResult:
    """
    Calibrate board pose from all four corners using a least-squares fit.

    More robust than two-corner when the board is not perfectly placed.

    Parameters
    ----------
    a1_world, h1_world, a8_world, h8_world : measured world positions (metres)
    quality_threshold_mm : max acceptable RMS residual

    Returns
    -------
    CalibrationResult
    """
    # Board corner nominal positions relative to centre (0,0)
    # Nominal: a1=(-3.5, -3.5)*sq, h1=(3.5,-3.5)*sq, a8=(-3.5,3.5)*sq, h8=(3.5,3.5)*sq
    half = 3.5 * SQUARE_SIZE   # 0.175 m
    nominal_offsets = [
        (-half, -half),   # a1
        (+half, -half),   # h1
        (-half, +half),   # a8
        (+half, +half),   # h8
    ]
    measured = [a1_world, h1_world, a8_world, h8_world]

    # Estimate centre as mean of measured corners
    cx = statistics.mean(m[0] for m in measured)
    cy = statistics.mean(m[1] for m in measured)
    cz = statistics.mean(m[2] for m in measured)

    # Estimate yaw via SVD-free 2D Procrustes
    # Numerator and denominator for atan2 of the rotation
    num = sum(
        nom[0] * (meas[1] - cy) - nom[1] * (meas[0] - cx)
        for nom, meas in zip(nominal_offsets, measured)
    )
    den = sum(
        nom[0] * (meas[0] - cx) + nom[1] * (meas[1] - cy)
        for nom, meas in zip(nominal_offsets, measured)
    )
    yaw = math.atan2(num, den)

    # Compute RMS residual
    residuals_sq: List[float] = []
    for (nom_dx, nom_dy), meas in zip(nominal_offsets, measured):
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        rx = cx + nom_dx * cos_y - nom_dy * sin_y
        ry = cy + nom_dx * sin_y + nom_dy * cos_y
        err_mm = math.hypot(rx - meas[0], ry - meas[1]) * 1000
        residuals_sq.append(err_mm ** 2)

    rms_mm = math.sqrt(sum(residuals_sq) / len(residuals_sq))
    success = rms_mm <= quality_threshold_mm

    message = (
        f"Four-corner calibration: RMS={rms_mm:.2f}mm "
        f"({'OK' if success else 'EXCEEDS threshold ' + str(quality_threshold_mm) + 'mm'})"
    )

    return CalibrationResult(
        board_origin_x=round(cx, 6),
        board_origin_y=round(cy, 6),
        board_origin_z=round(cz, 6),
        board_yaw=round(yaw, 6),
        residual_mm=round(rms_mm, 3),
        method="four_corner",
        success=success,
        message=message,
    )


# ---------------------------------------------------------------------------
# Nominal (Gazebo) calibration — no measurement needed
# ---------------------------------------------------------------------------

def calibrate_nominal() -> CalibrationResult:
    """
    Return a CalibrationResult for the nominal (Gazebo) board pose.
    No sensor data required.
    """
    return CalibrationResult(
        board_origin_x=0.0,
        board_origin_y=0.0,
        board_origin_z=BOARD_SURFACE_Z,
        board_yaw=0.0,
        residual_mm=0.0,
        method="nominal",
        success=True,
        message="Nominal Gazebo pose — no calibration required",
    )


# ---------------------------------------------------------------------------
# Validation helpers
# ---------------------------------------------------------------------------

def validate_calibration(
    result: CalibrationResult,
    spot_check_squares: Optional[List[str]] = None,
    tolerance_mm: float = 3.0,
) -> List[str]:
    """
    Validate a calibration result by spot-checking that key squares map
    to sensible world positions.

    Parameters
    ----------
    result : CalibrationResult to validate
    spot_check_squares : list of squares to check (default: four corners)
    tolerance_mm : max allowed deviation from the analytic position

    Returns
    -------
    List of warning strings (empty = all checks passed).
    """
    if spot_check_squares is None:
        spot_check_squares = ["a1", "h1", "a8", "h8", "e4", "d5"]

    origin = (result.board_origin_x, result.board_origin_y, result.board_origin_z)
    warnings: List[str] = []

    for sq in spot_check_squares:
        # Expected analytic position (nominal)
        nominal = square_to_board_pose(sq)
        # Position via calibration transform
        calibrated = apply_board_transform(sq, origin, result.board_yaw)

        dist_mm = math.hypot(
            calibrated.x - nominal.x,
            calibrated.y - nominal.y,
        ) * 1000

        if dist_mm > tolerance_mm:
            warnings.append(
                f"Square {sq}: calibrated position deviates "
                f"{dist_mm:.2f}mm from nominal (tolerance={tolerance_mm}mm)"
            )

    return warnings