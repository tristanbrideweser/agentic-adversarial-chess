"""
test_waypoint_planner.py
========================
Unit tests for waypoint_planner.py.  No ROS required.

Run with:
    pytest arm_controller/test/test_waypoint_planner.py -v
"""

import math
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from arm_controller.waypoint_planner import (
    GripperState,
    MotionWaypoint,
    WaypointConfig,
    WaypointPlanner,
)

BOARD_Z = 0.762
PAWN_GRASP_Z = 0.789


def make_planner(**kwargs) -> WaypointPlanner:
    cfg = WaypointConfig(**kwargs)
    return WaypointPlanner(cfg)


def standard_plan(planner=None):
    """Standard e2→e4 pawn move."""
    p = planner or WaypointPlanner()
    pick  = (0.025, -0.125, PAWN_GRASP_Z)   # e2
    place = (0.025, -0.025, PAWN_GRASP_Z)   # e4
    return p.plan_pick_place(pick, place, finger_separation=0.028)


# ---------------------------------------------------------------------------
# Waypoint sequence structure
# ---------------------------------------------------------------------------

class TestWaypointSequence:
    def test_returns_8_waypoints(self):
        wps = standard_plan()
        assert len(wps) == 8

    def test_labels_in_order(self):
        wps = standard_plan()
        expected = ["approach", "descend", "grasp", "lift",
                    "transit", "lower", "release", "retract"]
        assert [wp.label for wp in wps] == expected

    def test_all_waypoints_are_motion_waypoints(self):
        for wp in standard_plan():
            assert isinstance(wp, MotionWaypoint)

    def test_to_dict_has_required_keys(self):
        wp = standard_plan()[0]
        d = wp.to_dict()
        for k in ("label", "position", "orientation", "gripper",
                  "velocity_scaling", "acceleration_scaling"):
            assert k in d


# ---------------------------------------------------------------------------
# Gripper state sequence
# ---------------------------------------------------------------------------

class TestGripperSequence:
    def test_approach_opens_gripper(self):
        wps = standard_plan()
        assert wps[0].gripper == GripperState.OPEN

    def test_grasp_closes_gripper(self):
        wps = standard_plan()
        grasp_wp = next(w for w in wps if w.label == "grasp")
        assert grasp_wp.gripper == GripperState.CLOSED

    def test_lift_through_transit_stays_closed(self):
        wps = standard_plan()
        for label in ("lift", "transit", "lower"):
            wp = next(w for w in wps if w.label == label)
            assert wp.gripper == GripperState.CLOSED, (
                f"Expected CLOSED at {label}, got {wp.gripper}"
            )

    def test_release_opens_gripper(self):
        wps = standard_plan()
        rel = next(w for w in wps if w.label == "release")
        assert rel.gripper == GripperState.OPEN

    def test_retract_gripper_open(self):
        wps = standard_plan()
        ret = next(w for w in wps if w.label == "retract")
        assert ret.gripper == GripperState.OPEN


# ---------------------------------------------------------------------------
# Z-height logic
# ---------------------------------------------------------------------------

class TestZHeights:
    def test_approach_above_pick(self):
        p = WaypointPlanner(WaypointConfig(approach_z_offset=0.10))
        wps = p.plan_pick_place((0.0, 0.0, PAWN_GRASP_Z), (0.1, 0.0, PAWN_GRASP_Z))
        approach = next(w for w in wps if w.label == "approach")
        descend  = next(w for w in wps if w.label == "descend")
        assert approach.position[2] == pytest.approx(
            descend.position[2] + 0.10, abs=1e-6
        )

    def test_descend_at_grasp_z(self):
        gz = PAWN_GRASP_Z
        p = WaypointPlanner()
        wps = p.plan_pick_place((0.0, 0.0, gz), (0.1, 0.0, gz))
        descend = next(w for w in wps if w.label == "descend")
        assert descend.position[2] == pytest.approx(gz, abs=1e-6)

    def test_lift_above_pick(self):
        p = WaypointPlanner(WaypointConfig(lift_z_offset=0.10))
        gz = PAWN_GRASP_Z
        wps = p.plan_pick_place((0.0, 0.0, gz), (0.1, 0.0, gz))
        lift = next(w for w in wps if w.label == "lift")
        assert lift.position[2] == pytest.approx(gz + 0.10, abs=1e-6)

    def test_transit_above_place(self):
        p = WaypointPlanner(WaypointConfig(transit_z_offset=0.10))
        gz = PAWN_GRASP_Z
        wps = p.plan_pick_place((0.0, 0.0, gz), (0.1, 0.0, gz))
        transit = next(w for w in wps if w.label == "transit")
        assert transit.position[2] == pytest.approx(gz + 0.10, abs=1e-6)

    def test_lower_at_place_z(self):
        gz = 0.810   # queen
        p = WaypointPlanner()
        wps = p.plan_pick_place((0.0, 0.0, 0.789), (0.1, 0.0, gz))
        lower = next(w for w in wps if w.label == "lower")
        assert lower.position[2] == pytest.approx(gz, abs=1e-6)

    def test_release_same_z_as_lower(self):
        wps = standard_plan()
        lower   = next(w for w in wps if w.label == "lower")
        release = next(w for w in wps if w.label == "release")
        assert release.position[2] == pytest.approx(lower.position[2], abs=1e-6)

    def test_retract_above_place(self):
        p = WaypointPlanner(WaypointConfig(retract_z_offset=0.10))
        gz = PAWN_GRASP_Z
        wps = p.plan_pick_place((0.0, 0.0, gz), (0.1, 0.0, gz))
        lower   = next(w for w in wps if w.label == "lower")
        retract = next(w for w in wps if w.label == "retract")
        assert retract.position[2] == pytest.approx(
            lower.position[2] + 0.10, abs=1e-6
        )


# ---------------------------------------------------------------------------
# XY positions
# ---------------------------------------------------------------------------

class TestXYPositions:
    def test_approach_xy_matches_pick(self):
        px, py = 0.025, -0.125
        wps = WaypointPlanner().plan_pick_place(
            (px, py, PAWN_GRASP_Z), (0.0, 0.0, PAWN_GRASP_Z)
        )
        approach = next(w for w in wps if w.label == "approach")
        assert approach.position[0] == pytest.approx(px, abs=1e-6)
        assert approach.position[1] == pytest.approx(py, abs=1e-6)

    def test_descend_xy_matches_pick(self):
        px, py = -0.075, 0.025
        wps = WaypointPlanner().plan_pick_place(
            (px, py, PAWN_GRASP_Z), (0.0, 0.0, PAWN_GRASP_Z)
        )
        descend = next(w for w in wps if w.label == "descend")
        assert descend.position[0] == pytest.approx(px, abs=1e-6)
        assert descend.position[1] == pytest.approx(py, abs=1e-6)

    def test_transit_xy_matches_place(self):
        lx, ly = 0.175, 0.125
        wps = WaypointPlanner().plan_pick_place(
            (0.0, 0.0, PAWN_GRASP_Z), (lx, ly, PAWN_GRASP_Z)
        )
        transit = next(w for w in wps if w.label == "transit")
        assert transit.position[0] == pytest.approx(lx, abs=1e-6)
        assert transit.position[1] == pytest.approx(ly, abs=1e-6)

    def test_lower_xy_matches_place(self):
        lx, ly = -0.025, 0.175
        wps = WaypointPlanner().plan_pick_place(
            (0.0, 0.0, PAWN_GRASP_Z), (lx, ly, PAWN_GRASP_Z)
        )
        lower = next(w for w in wps if w.label == "lower")
        assert lower.position[0] == pytest.approx(lx, abs=1e-6)
        assert lower.position[1] == pytest.approx(ly, abs=1e-6)


# ---------------------------------------------------------------------------
# Velocity scaling
# ---------------------------------------------------------------------------

class TestVelocityScaling:
    def test_transit_is_fastest(self):
        wps = standard_plan()
        transit = next(w for w in wps if w.label == "transit")
        descend = next(w for w in wps if w.label == "descend")
        assert transit.velocity_scaling > descend.velocity_scaling

    def test_near_piece_steps_are_slow(self):
        wps = standard_plan()
        for label in ("descend", "lower"):
            wp = next(w for w in wps if w.label == label)
            assert wp.velocity_scaling <= 0.4, (
                f"Expected slow velocity at '{label}', got {wp.velocity_scaling}"
            )

    def test_all_scalings_in_range(self):
        for wp in standard_plan():
            assert 0.0 < wp.velocity_scaling <= 1.0
            assert 0.0 < wp.acceleration_scaling <= 1.0


# ---------------------------------------------------------------------------
# Arm color / home pose
# ---------------------------------------------------------------------------

class TestArmColor:
    def test_white_arm_uses_white_home(self):
        cfg = WaypointConfig(
            home_position_white=(0.0, -0.45, 1.00),
            home_position_black=(0.0, +0.45, 1.00),
        )
        p = WaypointPlanner(cfg)
        home = p.plan_home((0.0, 0.0, 1.0), arm_color="white")
        assert home[0].position[1] == pytest.approx(-0.45, abs=1e-6)

    def test_black_arm_uses_black_home(self):
        cfg = WaypointConfig(
            home_position_white=(0.0, -0.45, 1.00),
            home_position_black=(0.0, +0.45, 1.00),
        )
        p = WaypointPlanner(cfg)
        home = p.plan_home((0.0, 0.0, 1.0), arm_color="black")
        assert home[0].position[1] == pytest.approx(+0.45, abs=1e-6)

    def test_home_gripper_is_open(self):
        p = WaypointPlanner()
        home = p.plan_home((0.0, 0.0, 1.0))
        assert home[0].gripper == GripperState.OPEN


# ---------------------------------------------------------------------------
# estimate_duration
# ---------------------------------------------------------------------------

class TestEstimateDuration:
    def test_returns_positive_float(self):
        p = WaypointPlanner()
        wps = standard_plan(p)
        d = p.estimate_duration(wps)
        assert d > 0.0

    def test_longer_move_takes_more_time(self):
        p = WaypointPlanner()
        short = p.plan_pick_place((0.0, 0.0, PAWN_GRASP_Z), (0.05, 0.0, PAWN_GRASP_Z))
        long_ = p.plan_pick_place((0.0, 0.0, PAWN_GRASP_Z), (0.35, 0.0, PAWN_GRASP_Z))
        assert p.estimate_duration(long_) > p.estimate_duration(short)

    def test_empty_sequence_zero(self):
        p = WaypointPlanner()
        assert p.estimate_duration([]) == 0.0


# ---------------------------------------------------------------------------
# validate_waypoints
# ---------------------------------------------------------------------------

class TestValidateWaypoints:
    def test_valid_sequence_no_warnings(self):
        p = WaypointPlanner()
        wps = standard_plan(p)
        warns = p.validate_waypoints(wps)
        assert warns == [], f"Unexpected warnings: {warns}"

    def test_below_floor_triggers_warning(self):
        p = WaypointPlanner()
        bad_wp = MotionWaypoint(
            position=(0.0, 0.0, 0.70),  # 62 mm below board surface
            orientation=(1.0, 0.0, 0.0, 0.0),
            label="bad",
        )
        warns = p.validate_waypoints([bad_wp])
        assert any("below" in w.lower() for w in warns)

    def test_closed_first_waypoint_triggers_warning(self):
        p = WaypointPlanner()
        wps = standard_plan(p)
        wps[0] = MotionWaypoint(
            position=wps[0].position,
            orientation=wps[0].orientation,
            gripper=GripperState.CLOSED,
            label="approach",
        )
        warns = p.validate_waypoints(wps)
        assert any("open" in w.lower() for w in warns)