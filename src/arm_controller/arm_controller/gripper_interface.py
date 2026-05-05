"""
gripper_interface.py
====================
Abstraction layer for Franka Panda gripper control.

Provides a clean Python interface over the two ROS 2 action servers that
drive the gripper:
  - /franka_gripper/grasp   (franka_msgs/action/Grasp)
  - /franka_gripper/move    (franka_msgs/action/Move)

The GripperInterface class is ROS-aware but all configuration, state
tracking, and parameter validation live in pure-Python dataclasses so
they can be unit-tested without a running robot.

Franka gripper conventions
--------------------------
  - width = 0.0 → fully closed
  - width = 0.08 → fully open (80 mm max)
  - grasp() uses force + epsilon (inner/outer tolerance) for compliant grasping
  - move() uses speed for open/reposition commands

The interface maps our piece-type finger_separation values to Franka widths
and automatically selects grasp vs. move depending on context.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional


# ---------------------------------------------------------------------------
# Gripper state tracking
# ---------------------------------------------------------------------------

class GripperStateEnum(Enum):
    UNKNOWN  = auto()
    OPEN     = auto()
    CLOSED   = auto()
    MOVING   = auto()
    FAULT    = auto()


@dataclass
class GripperConfig:
    """
    Physical and control parameters for the Franka gripper.
    Values match the Franka Hand datasheet and typical chess-piece dimensions.
    """
    max_width_m: float        = 0.08    # fully open (m)
    open_width_m: float       = 0.07    # "open" position for approach
    min_width_m: float        = 0.0     # fully closed

    # Grasp action parameters
    grasp_speed_mps: float    = 0.05    # jaw closing speed (m/s)
    grasp_force_n: float      = 20.0    # grasping force (N)
    grasp_epsilon_inner: float = 0.005  # inner width tolerance (m)
    grasp_epsilon_outer: float = 0.005  # outer width tolerance (m)

    # Move action parameters
    move_speed_mps: float     = 0.1     # jaw movement speed (m/s)

    # Safety: minimum width we'll ever command (avoids crushing pieces)
    safe_min_width_m: float   = 0.010   # 10 mm


@dataclass
class GripperCommand:
    """
    A structured gripper command ready to be sent to the action server.

    action   : "grasp" or "move"
    width_m  : target width in metres
    speed    : jaw speed (m/s)
    force    : grasping force N (grasp only)
    epsilon_inner / epsilon_outer : grasp tolerance (m)
    """
    action: str                    # "grasp" | "move"
    width_m: float
    speed: float
    force: float = 0.0
    epsilon_inner: float = 0.005
    epsilon_outer: float = 0.005
    description: str = ""

    def to_dict(self) -> dict:
        return {
            "action": self.action,
            "width_m": self.width_m,
            "speed": self.speed,
            "force": self.force,
            "epsilon_inner": self.epsilon_inner,
            "epsilon_outer": self.epsilon_outer,
        }


# ---------------------------------------------------------------------------
# Gripper interface (pure Python layer — ROS calls are in the node)
# ---------------------------------------------------------------------------

class GripperInterface:
    """
    Builds gripper commands for open, close, and grasp operations.

    This class is ROS-free — it only computes GripperCommand objects.
    The actual action calls are made by PickPlaceServer.
    """

    def __init__(self, config: Optional[GripperConfig] = None) -> None:
        self.config = config or GripperConfig()
        self._state = GripperStateEnum.UNKNOWN
        self._current_width: float = self.config.max_width_m

    # ------------------------------------------------------------------
    # Command builders
    # ------------------------------------------------------------------

    def open_command(self) -> GripperCommand:
        """Return a command to fully open the gripper."""
        cfg = self.config
        return GripperCommand(
            action="move",
            width_m=cfg.open_width_m,
            speed=cfg.move_speed_mps,
            description="open gripper",
        )

    def close_command(self, finger_separation_m: float) -> GripperCommand:
        """
        Return a grasp command targeting the given finger separation.

        The Franka grasp action closes until it detects the object
        (within epsilon tolerance) or reaches the target width.

        Parameters
        ----------
        finger_separation_m : recommended jaw gap at contact (from PieceProfile)
        """
        cfg = self.config
        # Clamp to safe range
        width = max(cfg.safe_min_width_m,
                    min(finger_separation_m, cfg.max_width_m))
        return GripperCommand(
            action="grasp",
            width_m=width,
            speed=cfg.grasp_speed_mps,
            force=cfg.grasp_force_n,
            epsilon_inner=cfg.grasp_epsilon_inner,
            epsilon_outer=cfg.grasp_epsilon_outer,
            description=f"grasp piece (target width={width*1000:.1f}mm)",
        )

    def release_command(self) -> GripperCommand:
        """Return a command to open the gripper (release piece)."""
        return self.open_command()

    # ------------------------------------------------------------------
    # State tracking (updated by the ROS node after each command)
    # ------------------------------------------------------------------

    def update_state(self, state: GripperStateEnum, width_m: float) -> None:
        self._state = state
        self._current_width = width_m

    @property
    def state(self) -> GripperStateEnum:
        return self._state

    @property
    def is_holding(self) -> bool:
        """Heuristic: gripper is closed and likely holding something."""
        return (self._state == GripperStateEnum.CLOSED and
                self._current_width > self.config.safe_min_width_m)

    @property
    def current_width_m(self) -> float:
        return self._current_width

    # ------------------------------------------------------------------
    # Validation
    # ------------------------------------------------------------------

    def validate_command(self, cmd: GripperCommand) -> Optional[str]:
        """
        Return an error string if the command is invalid, else None.
        """
        cfg = self.config
        if cmd.width_m < cfg.min_width_m:
            return f"width {cmd.width_m*1000:.1f}mm < min {cfg.min_width_m*1000:.1f}mm"
        if cmd.width_m > cfg.max_width_m:
            return f"width {cmd.width_m*1000:.1f}mm > max {cfg.max_width_m*1000:.1f}mm"
        if cmd.speed <= 0:
            return f"speed must be > 0, got {cmd.speed}"
        if cmd.action not in ("grasp", "move"):
            return f"unknown action '{cmd.action}'"
        return None


# ---------------------------------------------------------------------------
# ROS 2 gripper action client (imported lazily — node only)
# ---------------------------------------------------------------------------

class GripperActionClient:
    """
    ROS 2 action client wrapper for the Franka gripper.

    Call connect(node) before any gripper commands.
    All send_* methods are synchronous (spin until complete).
    """

    GRASP_ACTION = "/franka_gripper/grasp"
    MOVE_ACTION  = "/franka_gripper/move"

    def __init__(self, namespace: str = "") -> None:
        """
        Parameters
        ----------
        namespace : ROS 2 namespace prefix, e.g. "/white_panda" or "/black_panda"
        """
        self._ns = namespace.rstrip("/")
        self._grasp_client = None
        self._move_client  = None
        self._node         = None

    def connect(self, node) -> bool:
        """
        Connect to both gripper action servers.
        Returns True if both became available within 5 s.
        """
        try:
            from rclpy.action import ActionClient                    # noqa: PLC0415
            from franka_msgs.action import Grasp, Move               # type: ignore  # noqa: PLC0415
            self._node = node

            grasp_name = f"{self._ns}{self.GRASP_ACTION}"
            move_name  = f"{self._ns}{self.MOVE_ACTION}"

            self._grasp_client = ActionClient(node, Grasp, grasp_name)
            self._move_client  = ActionClient(node, Move,  move_name)

            g_ok = self._grasp_client.wait_for_server(timeout_sec=5.0)
            m_ok = self._move_client.wait_for_server(timeout_sec=5.0)

            if g_ok and m_ok:
                node.get_logger().info(
                    f"Gripper action clients connected ({self._ns})"
                )
            else:
                node.get_logger().warn(
                    f"Gripper action server(s) not available ({self._ns}): "
                    f"grasp={g_ok}, move={m_ok}"
                )
            return g_ok and m_ok

        except ImportError as e:
            if self._node:
                self._node.get_logger().warn(f"franka_msgs not available: {e}")
            return False

    def execute(self, cmd: GripperCommand, timeout_sec: float = 10.0) -> bool:
        """
        Execute a GripperCommand.  Returns True on success.
        """
        if cmd.action == "grasp":
            return self._send_grasp(cmd, timeout_sec)
        elif cmd.action == "move":
            return self._send_move(cmd, timeout_sec)
        return False

    def _send_grasp(self, cmd: GripperCommand, timeout: float) -> bool:
        try:
            import rclpy                                             # noqa: PLC0415
            from franka_msgs.action import Grasp                    # type: ignore  # noqa: PLC0415

            goal = Grasp.Goal()
            goal.width        = cmd.width_m
            goal.speed        = cmd.speed
            goal.force        = cmd.force
            goal.epsilon.inner = cmd.epsilon_inner
            goal.epsilon.outer = cmd.epsilon_outer

            future = self._grasp_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
            handle = future.result()
            if not handle or not handle.accepted:
                return False
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=timeout)
            return result_future.result() is not None

        except Exception as exc:   # noqa: BLE001
            if self._node:
                self._node.get_logger().error(f"Grasp action failed: {exc}")
            return False

    def _send_move(self, cmd: GripperCommand, timeout: float) -> bool:
        try:
            import rclpy                                             # noqa: PLC0415
            from franka_msgs.action import Move                     # type: ignore  # noqa: PLC0415

            goal = Move.Goal()
            goal.width = cmd.width_m
            goal.speed = cmd.speed

            future = self._move_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
            handle = future.result()
            if not handle or not handle.accepted:
                return False
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=timeout)
            return result_future.result() is not None

        except Exception as exc:   # noqa: BLE001
            if self._node:
                self._node.get_logger().error(f"Move action failed: {exc}")
            return False