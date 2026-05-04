"""
test_gripper_interface.py
=========================
Unit tests for gripper_interface.py.  No ROS required.

Run with:
    pytest arm_controller/test/test_gripper_interface.py -v
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from src.arm_controller.arm_controller.gripper_interface import (
    GripperCommand,
    GripperConfig,
    GripperInterface,
    GripperStateEnum,
)


# ---------------------------------------------------------------------------
# GripperConfig
# ---------------------------------------------------------------------------

class TestGripperConfig:
    def test_defaults_are_sane(self):
        cfg = GripperConfig()
        assert 0 < cfg.open_width_m <= cfg.max_width_m
        assert cfg.safe_min_width_m > 0
        assert cfg.grasp_force_n > 0
        assert cfg.grasp_speed_mps > 0

    def test_open_width_within_limits(self):
        cfg = GripperConfig()
        assert cfg.min_width_m <= cfg.open_width_m <= cfg.max_width_m


# ---------------------------------------------------------------------------
# GripperInterface.open_command
# ---------------------------------------------------------------------------

class TestOpenCommand:
    def test_returns_move_action(self):
        gi = GripperInterface()
        cmd = gi.open_command()
        assert cmd.action == "move"

    def test_width_is_open_width(self):
        cfg = GripperConfig(open_width_m=0.065)
        gi = GripperInterface(cfg)
        cmd = gi.open_command()
        assert cmd.width_m == pytest.approx(0.065, abs=1e-6)

    def test_to_dict_keys(self):
        gi = GripperInterface()
        d = gi.open_command().to_dict()
        for k in ("action", "width_m", "speed", "force"):
            assert k in d


# ---------------------------------------------------------------------------
# GripperInterface.close_command
# ---------------------------------------------------------------------------

class TestCloseCommand:
    def test_returns_grasp_action(self):
        gi = GripperInterface()
        cmd = gi.close_command(0.030)
        assert cmd.action == "grasp"

    def test_width_matches_finger_separation(self):
        gi = GripperInterface()
        cmd = gi.close_command(0.028)
        assert cmd.width_m == pytest.approx(0.028, abs=1e-6)

    def test_clamps_to_safe_min(self):
        gi = GripperInterface(GripperConfig(safe_min_width_m=0.010))
        cmd = gi.close_command(0.005)   # too narrow
        assert cmd.width_m >= 0.010

    def test_clamps_to_max(self):
        gi = GripperInterface(GripperConfig(max_width_m=0.08))
        cmd = gi.close_command(0.10)    # too wide
        assert cmd.width_m <= 0.08

    def test_force_is_positive(self):
        gi = GripperInterface()
        cmd = gi.close_command(0.030)
        assert cmd.force > 0

    def test_epsilon_present(self):
        gi = GripperInterface()
        cmd = gi.close_command(0.030)
        assert cmd.epsilon_inner >= 0
        assert cmd.epsilon_outer >= 0

    @pytest.mark.parametrize("sep", [0.028, 0.030, 0.032, 0.034])
    def test_all_piece_separations_valid(self, sep):
        gi = GripperInterface()
        cmd = gi.close_command(sep)
        err = gi.validate_command(cmd)
        assert err is None, f"sep={sep}: {err}"


# ---------------------------------------------------------------------------
# GripperInterface.release_command
# ---------------------------------------------------------------------------

class TestReleaseCommand:
    def test_same_as_open(self):
        gi = GripperInterface()
        assert gi.release_command().action == gi.open_command().action
        assert gi.release_command().width_m == gi.open_command().width_m


# ---------------------------------------------------------------------------
# GripperInterface.validate_command
# ---------------------------------------------------------------------------

class TestValidateCommand:
    def test_valid_open_command(self):
        gi = GripperInterface()
        err = gi.validate_command(gi.open_command())
        assert err is None

    def test_valid_close_command(self):
        gi = GripperInterface()
        err = gi.validate_command(gi.close_command(0.030))
        assert err is None

    def test_rejects_zero_speed(self):
        gi = GripperInterface()
        bad = GripperCommand(action="move", width_m=0.05, speed=0.0)
        err = gi.validate_command(bad)
        assert err is not None
        assert "speed" in err

    def test_rejects_unknown_action(self):
        gi = GripperInterface()
        bad = GripperCommand(action="spin", width_m=0.05, speed=0.1)
        err = gi.validate_command(bad)
        assert err is not None

    def test_rejects_negative_width(self):
        gi = GripperInterface()
        bad = GripperCommand(action="move", width_m=-0.01, speed=0.1)
        err = gi.validate_command(bad)
        assert err is not None

    def test_rejects_over_max_width(self):
        gi = GripperInterface(GripperConfig(max_width_m=0.08))
        bad = GripperCommand(action="move", width_m=0.09, speed=0.1)
        err = gi.validate_command(bad)
        assert err is not None


# ---------------------------------------------------------------------------
# GripperInterface state tracking
# ---------------------------------------------------------------------------

class TestGripperState:
    def test_initial_state_unknown(self):
        gi = GripperInterface()
        assert gi.state == GripperStateEnum.UNKNOWN

    def test_update_state_open(self):
        gi = GripperInterface()
        gi.update_state(GripperStateEnum.OPEN, 0.07)
        assert gi.state == GripperStateEnum.OPEN
        assert gi.current_width_m == pytest.approx(0.07)

    def test_is_holding_when_closed_with_width(self):
        gi = GripperInterface(GripperConfig(safe_min_width_m=0.010))
        gi.update_state(GripperStateEnum.CLOSED, 0.030)
        assert gi.is_holding is True

    def test_not_holding_when_open(self):
        gi = GripperInterface()
        gi.update_state(GripperStateEnum.OPEN, 0.07)
        assert gi.is_holding is False

    def test_not_holding_when_fully_closed(self):
        gi = GripperInterface(GripperConfig(safe_min_width_m=0.010))
        gi.update_state(GripperStateEnum.CLOSED, 0.005)  # below safe min
        assert gi.is_holding is False