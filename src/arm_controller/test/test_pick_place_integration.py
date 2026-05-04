"""
test_pick_place_integration.py
===============================
Integration tests: PickPlaceTask (from move_translator) → waypoints.

Verifies that the waypoint planner produces geometrically correct sequences
for each task type that the move_translator can emit.  No ROS or MoveIt.

Run with:
    pytest arm_controller/test/test_pick_place_integration.py -v
"""

import math
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from src.arm_controller.arm_controller.waypoint_planner import (
    GripperState,
    WaypointConfig,
    WaypointPlanner,
)

# ---------------------------------------------------------------------------
# Board geometry constants (matches architecture spec)
# ---------------------------------------------------------------------------
BOARD_Z = 0.762
GRASP_Z = {
    "P": 0.789, "R": 0.795, "N": 0.798,
    "B": 0.801, "Q": 0.810, "K": 0.819,
}
FINGER_SEP = {
    "P": 0.028, "R": 0.032, "N": 0.034,
    "B": 0.030, "Q": 0.034, "K": 0.034,
}

def sq(square: str):
    """Return (x, y) world centre of a square."""
    fx = ord(square[0].lower()) - ord("a")
    ry = int(square[1]) - 1
    return (-0.175 + fx * 0.05, -0.175 + ry * 0.05)


def pick_pos(square: str, piece: str):
    x, y = sq(square)
    return (x, y, GRASP_Z[piece.upper()])


def place_pos(square: str, piece: str):
    x, y = sq(square)
    return (x, y, GRASP_Z[piece.upper()])


ORIENT_DOWN = (1.0, 0.0, 0.0, 0.0)

# ---------------------------------------------------------------------------
# Standard move: e2 pawn → e4
# ---------------------------------------------------------------------------

class TestStandardPawnMove:
    def setup_method(self):
        self.planner = WaypointPlanner()
        self.wps = self.planner.plan_pick_place(
            pick_position=pick_pos("e2", "P"),
            place_position=place_pos("e4", "P"),
            pick_orientation=ORIENT_DOWN,
            place_orientation=ORIENT_DOWN,
            finger_separation=FINGER_SEP["P"],
            arm_color="white",
        )

    def test_8_waypoints(self):
        assert len(self.wps) == 8

    def test_approach_over_e2(self):
        x, y = sq("e2")
        ap = next(w for w in self.wps if w.label == "approach")
        assert ap.position[0] == pytest.approx(x, abs=1e-5)
        assert ap.position[1] == pytest.approx(y, abs=1e-5)

    def test_descend_to_pawn_grasp_z(self):
        de = next(w for w in self.wps if w.label == "descend")
        assert de.position[2] == pytest.approx(GRASP_Z["P"], abs=1e-3)

    def test_transit_over_e4(self):
        x, y = sq("e4")
        tr = next(w for w in self.wps if w.label == "transit")
        assert tr.position[0] == pytest.approx(x, abs=1e-5)
        assert tr.position[1] == pytest.approx(y, abs=1e-5)

    def test_lower_to_e4_grasp_z(self):
        lo = next(w for w in self.wps if w.label == "lower")
        assert lo.position[2] == pytest.approx(GRASP_Z["P"], abs=1e-3)

    def test_no_waypoint_below_board(self):
        for wp in self.wps:
            assert wp.position[2] >= BOARD_Z - 0.005, (
                f"Waypoint '{wp.label}' at Z={wp.position[2]:.4f} below board"
            )

    def test_validates_cleanly(self):
        warns = self.planner.validate_waypoints(self.wps)
        assert warns == []


# ---------------------------------------------------------------------------
# Capture: queen d1 takes pawn on h5
# Two-task sequence: remove h5 pawn first, then move queen
# ---------------------------------------------------------------------------

class TestCaptureSequence:
    """
    Simulates the two-task sequence the coordinator sends for a capture:
      Task 1: remove h5 black pawn → graveyard
      Task 2: move d1 queen → h5
    """

    def setup_method(self):
        self.planner = WaypointPlanner()

    def _graveyard_pos(self, slot=0):
        return (0.275, -0.175 + slot * 0.05, GRASP_Z["P"])

    def test_removal_task_pick_from_h5(self):
        wps = self.planner.plan_pick_place(
            pick_position=pick_pos("h5", "p"),
            place_position=self._graveyard_pos(0),
            finger_separation=FINGER_SEP["P"],
            arm_color="black",
        )
        x, y = sq("h5")
        de = next(w for w in wps if w.label == "descend")
        assert de.position[0] == pytest.approx(x, abs=1e-5)
        assert de.position[1] == pytest.approx(y, abs=1e-5)

    def test_removal_task_place_in_graveyard(self):
        gy = self._graveyard_pos(0)
        wps = self.planner.plan_pick_place(
            pick_position=pick_pos("h5", "p"),
            place_position=gy,
            finger_separation=FINGER_SEP["P"],
            arm_color="black",
        )
        lo = next(w for w in wps if w.label == "lower")
        assert lo.position[0] == pytest.approx(gy[0], abs=1e-5)
        assert lo.position[1] == pytest.approx(gy[1], abs=1e-5)

    def test_move_task_pick_from_d1(self):
        wps = self.planner.plan_pick_place(
            pick_position=pick_pos("d1", "Q"),
            place_position=place_pos("h5", "Q"),
            finger_separation=FINGER_SEP["Q"],
            arm_color="white",
        )
        x, y = sq("d1")
        de = next(w for w in wps if w.label == "descend")
        assert de.position[0] == pytest.approx(x, abs=1e-5)
        assert de.position[1] == pytest.approx(y, abs=1e-5)


# ---------------------------------------------------------------------------
# Castling: two-task sequence, king then rook
# ---------------------------------------------------------------------------

class TestCastlingSequence:
    def setup_method(self):
        self.planner = WaypointPlanner()

    def test_king_task_e1_to_g1(self):
        wps = self.planner.plan_pick_place(
            pick_position=pick_pos("e1", "K"),
            place_position=place_pos("g1", "K"),
            finger_separation=FINGER_SEP["K"],
            arm_color="white",
        )
        assert len(wps) == 8
        # descend should be over e1
        x, y = sq("e1")
        de = next(w for w in wps if w.label == "descend")
        assert de.position[0] == pytest.approx(x, abs=1e-5)

    def test_rook_task_h1_to_f1(self):
        wps = self.planner.plan_pick_place(
            pick_position=pick_pos("h1", "R"),
            place_position=place_pos("f1", "R"),
            finger_separation=FINGER_SEP["R"],
            arm_color="white",
        )
        x, y = sq("f1")
        lo = next(w for w in wps if w.label == "lower")
        assert lo.position[0] == pytest.approx(x, abs=1e-5)
        assert lo.position[1] == pytest.approx(y, abs=1e-5)


# ---------------------------------------------------------------------------
# All 6 piece types: verify correct grasp Z for each
# ---------------------------------------------------------------------------

class TestAllPieceTypes:
    @pytest.mark.parametrize("piece,grasp_z,sep", [
        ("P", 0.789, 0.028),
        ("R", 0.795, 0.032),
        ("N", 0.798, 0.034),
        ("B", 0.801, 0.030),
        ("Q", 0.810, 0.034),
        ("K", 0.819, 0.034),
    ])
    def test_descend_at_correct_grasp_z(self, piece, grasp_z, sep):
        p = WaypointPlanner()
        wps = p.plan_pick_place(
            pick_position=(0.0, 0.0, grasp_z),
            place_position=(0.05, 0.0, grasp_z),
            finger_separation=sep,
        )
        de = next(w for w in wps if w.label == "descend")
        assert de.position[2] == pytest.approx(grasp_z, abs=1e-3), (
            f"Piece {piece}: expected grasp Z {grasp_z}, got {de.position[2]}"
        )

    @pytest.mark.parametrize("piece,grasp_z,sep", [
        ("P", 0.789, 0.028),
        ("Q", 0.810, 0.034),
        ("K", 0.819, 0.034),
    ])
    def test_no_collision_with_board(self, piece, grasp_z, sep):
        p = WaypointPlanner()
        wps = p.plan_pick_place(
            pick_position=(0.025, -0.025, grasp_z),
            place_position=(0.025, +0.025, grasp_z),
            finger_separation=sep,
        )
        warns = p.validate_waypoints(wps)
        assert warns == [], f"Piece {piece} produced warnings: {warns}"


# ---------------------------------------------------------------------------
# Config parameter effects
# ---------------------------------------------------------------------------

class TestConfigEffects:
    def test_larger_approach_offset_raises_waypoint(self):
        cfg_small = WaypointConfig(approach_z_offset=0.05)
        cfg_large = WaypointConfig(approach_z_offset=0.15)
        args = dict(
            pick_position=(0.0, 0.0, 0.789),
            place_position=(0.05, 0.0, 0.789),
        )
        ap_small = next(w for w in WaypointPlanner(cfg_small).plan_pick_place(**args)
                        if w.label == "approach")
        ap_large = next(w for w in WaypointPlanner(cfg_large).plan_pick_place(**args)
                        if w.label == "approach")
        assert ap_large.position[2] > ap_small.position[2]

    def test_duration_scales_with_distance(self):
        p = WaypointPlanner()
        short = p.plan_pick_place((0.0, 0.0, 0.789), (0.05, 0.0, 0.789))
        long_ = p.plan_pick_place((-.175, -.175, 0.789), (.175, .175, 0.789))
        assert p.estimate_duration(long_) > p.estimate_duration(short)