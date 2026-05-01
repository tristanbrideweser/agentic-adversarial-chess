"""
pick_place_server.py
====================
ROS 2 action server that executes pick-and-place tasks for one Franka Panda arm.

One instance runs per arm, namespaced as:
  /white_panda/pick_place
  /black_panda/pick_place

Action interface  (chess_interfaces/action/PickPlace)
------------------------------------------------------
Goal:
    string   task_type        # "pick_and_place" | "remove_piece" | "place_reserve"
    string   piece_char       # FEN character (e.g. "P", "q")
    float64[3] pick_position   # world frame (m)
    float64[4] pick_orientation  # quaternion xyzw
    float64[3] place_position
    float64[4] place_orientation
    float64  finger_separation # metres
    float64  grasp_height     # absolute world Z for jaw centre
    bool     use_cartesian    # force Cartesian descent/lower phases

Result:
    bool    success
    string  failure_reason
    int32   waypoints_executed

Feedback:
    string  current_step      # e.g. "approach", "grasp", "transit"
    float32 progress          # 0.0–1.0

The server executes the 8-step sequence defined in architecture §2.6:
  approach → descend → grasp → lift → transit → lower → release → retract

Parameters (ROS 2 params, settable via arm_white.yaml / arm_black.yaml)
-----------------------------------------------------------------------
    approach_z_offset     : float  (default 0.10)
    lift_z_offset         : float  (default 0.10)
    transit_z_offset      : float  (default 0.10)
    retract_z_offset      : float  (default 0.10)
    max_retries           : int    (default 3)
    planning_time_sec     : float  (default 5.0)
    vel_transit           : float  (default 0.7)
    vel_near_piece        : float  (default 0.2)
    use_cartesian_descent : bool   (default true)
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from .gripper_interface import (
    GripperActionClient,
    GripperConfig,
    GripperInterface,
    GripperStateEnum,
)
from .moveit_client import MoveItClient, PlanningConfig
from .waypoint_planner import (
    GripperState,
    MotionWaypoint,
    WaypointConfig,
    WaypointPlanner,
)

try:
    from chess_interfaces.action import PickPlace           # type: ignore
    _INTERFACES_AVAILABLE = True
except ImportError:
    _INTERFACES_AVAILABLE = False

# ---------------------------------------------------------------------------
# Turn mutex — shared across both arm instances via a module-level lock
# Architecture §2.6: "only one arm moves at a time"
# ---------------------------------------------------------------------------
_ARM_MUTEX = threading.Lock()


class PickPlaceServer(Node):
    """
    ROS 2 action server for one Franka Panda arm's pick-and-place operations.

    Parameters
    ----------
    arm_color : "white" or "black"
    """

    def __init__(self, arm_color: str = "white") -> None:
        assert arm_color in ("white", "black"), \
            f"arm_color must be 'white' or 'black', got '{arm_color}'"

        self.arm_color = arm_color
        self.ns = f"{arm_color}_panda"

        super().__init__(f"{self.ns}_pick_place_server")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("approach_z_offset",     0.10)
        self.declare_parameter("lift_z_offset",         0.10)
        self.declare_parameter("transit_z_offset",      0.10)
        self.declare_parameter("retract_z_offset",      0.10)
        self.declare_parameter("max_retries",           3)
        self.declare_parameter("planning_time_sec",     5.0)
        self.declare_parameter("vel_transit",           0.7)
        self.declare_parameter("vel_near_piece",        0.2)
        self.declare_parameter("use_cartesian_descent", True)

        wp_cfg = WaypointConfig(
            approach_z_offset    = self.get_parameter("approach_z_offset").value,
            lift_z_offset        = self.get_parameter("lift_z_offset").value,
            transit_z_offset     = self.get_parameter("transit_z_offset").value,
            retract_z_offset     = self.get_parameter("retract_z_offset").value,
            vel_transit          = self.get_parameter("vel_transit").value,
            vel_near_piece       = self.get_parameter("vel_near_piece").value,
            home_position_white  = (0.0, -0.45, 1.00),
            home_position_black  = (0.0, +0.45, 1.00),
        )
        plan_cfg = PlanningConfig(
            planning_time_sec = self.get_parameter("planning_time_sec").value,
        )

        self._max_retries         = self.get_parameter("max_retries").value
        self._use_cartesian       = self.get_parameter("use_cartesian_descent").value

        # ------------------------------------------------------------------
        # Sub-components
        # ------------------------------------------------------------------
        self._waypoint_planner = WaypointPlanner(wp_cfg)
        self._gripper_iface    = GripperInterface(GripperConfig())
        self._gripper_client   = GripperActionClient(namespace=f"/{self.ns}")
        self._moveit           = MoveItClient(
            arm_name=self.ns,
            namespace=f"/{self.ns}",
            config=plan_cfg,
        )

        # Connect to hardware
        self._gripper_client.connect(self)
        self._moveit.connect(self)

        # ------------------------------------------------------------------
        # Action server
        # ------------------------------------------------------------------
        if not _INTERFACES_AVAILABLE:
            self.get_logger().warn(
                "chess_interfaces not found — action server not registered. "
                "Use execute_direct() for programmatic access."
            )
            return

        self._action_server = ActionServer(
            self,
            PickPlace,
            f"/{self.ns}/pick_place",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(
            f"PickPlaceServer ready on /{self.ns}/pick_place"
        )

    # ------------------------------------------------------------------
    # Action callbacks
    # ------------------------------------------------------------------

    def _goal_callback(self, goal_request):
        self.get_logger().info(
            f"{self.ns}: received {goal_request.task_type} goal "
            f"for piece '{goal_request.piece_char}'"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().warn(f"{self.ns}: goal cancelled — will stop after current waypoint")
        return CancelResponse.ACCEPT

    async def _execute(self, goal_handle) -> "PickPlace.Result":
        goal = goal_handle.request
        result = PickPlace.Result()

        exec_result = self.execute_direct(
            task_type=goal.task_type,
            piece_char=goal.piece_char,
            pick_position=tuple(goal.pick_position),
            pick_orientation=tuple(goal.pick_orientation),
            place_position=tuple(goal.place_position),
            place_orientation=tuple(goal.place_orientation),
            finger_separation=goal.finger_separation,
            feedback_cb=lambda step, prog: self._send_feedback(
                goal_handle, step, prog
            ),
        )

        result.success = exec_result["success"]
        result.failure_reason = exec_result.get("failure_reason", "")
        result.waypoints_executed = exec_result.get("waypoints_executed", 0)

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    def _send_feedback(self, goal_handle, step: str, progress: float) -> None:
        fb = PickPlace.Feedback()
        fb.current_step = step
        fb.progress = float(progress)
        goal_handle.publish_feedback(fb)

    # ------------------------------------------------------------------
    # Core execution logic (callable without action server overhead)
    # ------------------------------------------------------------------

    def execute_direct(
        self,
        task_type: str,
        piece_char: str,
        pick_position: tuple,
        pick_orientation: tuple,
        place_position: tuple,
        place_orientation: tuple,
        finger_separation: float = 0.030,
        feedback_cb=None,
    ) -> dict:
        """
        Execute a pick-and-place task directly (no action server needed).

        Acquires the global arm mutex before moving so only one arm is
        ever in motion at a time (architecture §2.6).

        Parameters
        ----------
        task_type   : "pick_and_place" | "remove_piece" | "place_reserve"
        piece_char  : FEN character (for logging)
        pick_position / pick_orientation : 3-tuple / 4-tuple world frame
        place_position / place_orientation : same
        finger_separation : metres
        feedback_cb : optional callable(step: str, progress: float)

        Returns
        -------
        dict with keys: success, failure_reason, waypoints_executed
        """
        def fb(step: str, idx: int, total: int):
            if feedback_cb:
                feedback_cb(step, idx / max(total, 1))
            self.get_logger().info(
                f"{self.ns} [{idx}/{total}] {step}"
            )

        # Generate waypoints
        waypoints = self._waypoint_planner.plan_pick_place(
            pick_position=pick_position,
            place_position=place_position,
            pick_orientation=pick_orientation,
            place_orientation=place_orientation,
            finger_separation=finger_separation,
            arm_color=self.arm_color,
        )

        # Sanity check
        warnings = self._waypoint_planner.validate_waypoints(waypoints)
        for w in warnings:
            self.get_logger().warn(f"{self.ns}: {w}")

        total = len(waypoints)
        executed = 0

        # Acquire mutex — blocks until other arm finishes its current move
        self.get_logger().debug(f"{self.ns}: waiting for arm mutex...")
        with _ARM_MUTEX:
            self.get_logger().debug(f"{self.ns}: arm mutex acquired")

            for i, wp in enumerate(waypoints):
                fb(wp.label, i + 1, total)

                # Handle gripper-only steps (same position as previous)
                if wp.gripper == GripperState.OPEN:
                    cmd = self._gripper_iface.open_command()
                    ok = self._gripper_client.execute(cmd)
                    if not ok:
                        self.get_logger().warn(
                            f"{self.ns}: gripper open failed at step '{wp.label}'"
                        )
                    self._gripper_iface.update_state(
                        GripperStateEnum.OPEN, self._gripper_iface.config.open_width_m
                    )

                elif wp.gripper == GripperState.CLOSED:
                    cmd = self._gripper_iface.close_command(finger_separation)
                    ok = self._gripper_client.execute(cmd)
                    if not ok:
                        # Grasp failure — retry up to max_retries
                        return self._handle_grasp_failure(
                            wp.label, pick_position, pick_orientation,
                            finger_separation, executed, feedback_cb
                        )
                    self._gripper_iface.update_state(
                        GripperStateEnum.CLOSED, finger_separation
                    )

                # Motion step
                if self._is_motion_step(wp, i, waypoints):
                    ok = self._execute_motion(wp, i, waypoints)
                    if not ok:
                        return {
                            "success": False,
                            "failure_reason": (
                                f"Motion planning/execution failed at step "
                                f"'{wp.label}'"
                            ),
                            "waypoints_executed": executed,
                        }

                executed += 1

        return {
            "success": True,
            "failure_reason": "",
            "waypoints_executed": executed,
        }

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _is_motion_step(
        self,
        wp: MotionWaypoint,
        idx: int,
        waypoints: list,
    ) -> bool:
        """Return True if this waypoint involves an arm motion (not just gripper)."""
        if idx == 0:
            return True
        prev = waypoints[idx - 1]
        dx = wp.position[0] - prev.position[0]
        dy = wp.position[1] - prev.position[1]
        dz = wp.position[2] - prev.position[2]
        dist = (dx*dx + dy*dy + dz*dz) ** 0.5
        return dist > 1e-4   # > 0.1 mm = actual motion

    def _execute_motion(
        self,
        wp: MotionWaypoint,
        idx: int,
        waypoints: list,
    ) -> bool:
        """Plan and execute motion to a single waypoint."""
        # Use Cartesian path for descent/lower phases if configured
        use_cartesian = (
            self._use_cartesian and
            wp.label in ("descend", "lower") and
            idx > 0
        )

        if use_cartesian:
            plan = self._moveit.plan_cartesian_path([waypoints[idx - 1], wp])
        else:
            plan = self._moveit.plan_to_pose(
                target_position=wp.position,
                target_orientation=wp.orientation,
                velocity_scaling=wp.velocity_scaling,
                acceleration_scaling=wp.acceleration_scaling,
            )

        if not plan.success:
            self.get_logger().error(
                f"{self.ns}: planning failed at '{wp.label}': {plan.failure_reason}"
            )
            return False

        result = self._moveit.execute(plan)
        if not result.success:
            self.get_logger().error(
                f"{self.ns}: execution failed at '{wp.label}': {result.failure_reason}"
            )
        return result.success

    def _handle_grasp_failure(
        self,
        label: str,
        pick_position: tuple,
        pick_orientation: tuple,
        finger_separation: float,
        waypoints_done: int,
        feedback_cb,
    ) -> dict:
        """
        Retry logic for a failed grasp.  Attempts up to max_retries times
        by re-approaching and re-closing.
        """
        self.get_logger().warn(
            f"{self.ns}: grasp failed at '{label}' — "
            f"retrying up to {self._max_retries} times"
        )

        for attempt in range(self._max_retries):
            self.get_logger().info(
                f"{self.ns}: grasp retry {attempt + 1}/{self._max_retries}"
            )
            # Re-open, re-approach, re-descend, re-grasp
            self._gripper_client.execute(self._gripper_iface.open_command())

            cfg = self._waypoint_planner.config
            approach_wp = MotionWaypoint(
                position=(
                    pick_position[0],
                    pick_position[1],
                    pick_position[2] + cfg.approach_z_offset,
                ),
                orientation=pick_orientation,
                label="retry_approach",
            )
            descend_wp = MotionWaypoint(
                position=pick_position,
                orientation=pick_orientation,
                label="retry_descend",
            )

            plan = self._moveit.plan_to_pose(approach_wp.position, approach_wp.orientation)
            if plan.success:
                self._moveit.execute(plan)
            plan2 = self._moveit.plan_cartesian_path([approach_wp, descend_wp])
            if plan2.success:
                self._moveit.execute(plan2)

            cmd = self._gripper_iface.close_command(finger_separation)
            if self._gripper_client.execute(cmd):
                self.get_logger().info(f"{self.ns}: grasp succeeded on retry {attempt + 1}")
                return None   # caller should continue execution

        return {
            "success": False,
            "failure_reason": (
                f"Grasp failed after {self._max_retries} attempts at '{label}'"
            ),
            "waypoints_executed": waypoints_done,
        }

    def go_home(self) -> bool:
        """Return the arm to its home pose. Used at game start/end/abort."""
        with _ARM_MUTEX:
            plan = self._moveit.plan_home()
            if not plan.success:
                self.get_logger().error(f"{self.ns}: home planning failed")
                return False
            result = self._moveit.execute(plan)
            return result.success


# ---------------------------------------------------------------------------
# Entry points (one per arm color)
# ---------------------------------------------------------------------------

def main_white(args=None) -> None:
    rclpy.init(args=args)
    node = PickPlaceServer(arm_color="white")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_black(args=None) -> None:
    rclpy.init(args=args)
    node = PickPlaceServer(arm_color="black")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()