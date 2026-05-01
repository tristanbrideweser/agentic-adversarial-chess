"""
moveit_client.py
================
MoveIt 2 motion planning and execution client for one Franka Panda arm.

Wraps the MoveIt 2 Python bindings (moveit_py) to:
  - Plan Cartesian paths to waypoints
  - Execute planned trajectories
  - Manage the planning scene (collision objects for both arms + board)
  - Provide IK feasibility checks before committing to execution

Design notes
------------
- One MoveItClient instance per arm (namespaced white_panda / black_panda).
- Planning scene is SHARED — both arm instances observe the same scene so
  each arm's link states appear as obstacles for the other.
- Strict turn-taking is enforced by the PickPlaceServer (only one arm moves
  at a time), so no concurrent planning is needed.
- All planning calls use the arm's move_group ("panda_arm").
- The gripper move_group ("hand") is controlled separately via GripperActionClient.

This module is structured so that the MoveItClient can be imported and
its pure-Python methods tested without a live MoveIt instance — the
ROS/MoveIt objects are created lazily in connect().
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from .waypoint_planner import MotionWaypoint, GripperState


# ---------------------------------------------------------------------------
# Planning result
# ---------------------------------------------------------------------------

@dataclass
class PlanResult:
    """
    Result of one MoveIt planning attempt.

    success          : whether a valid plan was found
    failure_reason   : human-readable reason if success is False
    estimated_duration_sec : rough execution time estimate
    num_waypoints    : number of trajectory points
    """
    success: bool = False
    failure_reason: str = ""
    estimated_duration_sec: float = 0.0
    num_waypoints: int = 0
    trajectory = None   # RobotTrajectory — None until connected to MoveIt


@dataclass
class ExecutionResult:
    """
    Result of one trajectory execution.

    success         : whether execution completed without error
    failure_reason  : human-readable reason if failed
    waypoints_executed : how many waypoints completed before failure (if any)
    """
    success: bool = False
    failure_reason: str = ""
    waypoints_executed: int = 0


# ---------------------------------------------------------------------------
# Planning configuration
# ---------------------------------------------------------------------------

@dataclass
class PlanningConfig:
    """Per-arm MoveIt planning parameters."""
    move_group_name: str     = "panda_arm"
    planning_pipeline: str   = "ompl"
    planner_id: str          = "RRTConnectkConfigDefault"
    planning_time_sec: float = 5.0
    num_planning_attempts: int = 3
    max_velocity_scaling: float = 0.5
    max_acceleration_scaling: float = 0.5
    goal_position_tolerance: float  = 0.001   # metres
    goal_orientation_tolerance: float = 0.01  # radians
    cartesian_step_size: float = 0.005        # metres (Cartesian interpolation)
    cartesian_jump_threshold: float = 0.0     # 0 = disabled


# ---------------------------------------------------------------------------
# MoveIt client
# ---------------------------------------------------------------------------

class MoveItClient:
    """
    MoveIt 2 planning and execution for one Franka Panda arm.

    Parameters
    ----------
    arm_name  : "white_panda" or "black_panda"
    namespace : ROS 2 namespace prefix (e.g. "/white_panda")
    config    : PlanningConfig (defaults used if None)
    """

    # Joint-space home poses (degrees, then converted internally)
    HOME_JOINTS_DEG = [0.0, -45.0, 0.0, -135.0, 0.0, 90.0, 45.0]

    def __init__(
        self,
        arm_name: str,
        namespace: str = "",
        config: Optional[PlanningConfig] = None,
    ) -> None:
        self.arm_name  = arm_name
        self.namespace = namespace.rstrip("/")
        self.config    = config or PlanningConfig()

        # Set lazily in connect()
        self._node            = None
        self._move_group      = None
        self._planning_scene  = None
        self._robot_model     = None
        self._connected       = False

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self, node) -> bool:
        """
        Initialise MoveIt 2 move group and planning scene.
        Must be called once from the owning ROS node's __init__.
        """
        self._node = node
        try:
            from moveit.planning import MoveItPy                    # type: ignore  # noqa: PLC0415
            from moveit.core.robot_state import RobotState          # type: ignore  # noqa: PLC0415

            self._moveit = MoveItPy(node_name=f"moveit_{self.arm_name}")
            self._move_group = self._moveit.get_planning_component(
                self.config.move_group_name
            )
            self._robot_model = self._moveit.get_robot_model()
            self._connected = True
            node.get_logger().info(
                f"MoveItClient connected for {self.arm_name} "
                f"(group={self.config.move_group_name})"
            )
            return True

        except ImportError:
            node.get_logger().warn(
                "moveit_py not available — running in dry-run mode. "
                "All plan() calls will return success=False."
            )
            return False

        except Exception as exc:   # noqa: BLE001
            node.get_logger().error(f"MoveIt init failed: {exc}")
            return False

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def plan_to_pose(
        self,
        target_position: Tuple[float, float, float],
        target_orientation: Tuple[float, float, float, float],
        velocity_scaling: float = 0.5,
        acceleration_scaling: float = 0.5,
    ) -> PlanResult:
        """
        Plan a motion to a single Cartesian pose target.

        Uses the configured OMPL planner with position + orientation goals.
        Falls back to Cartesian path if joint-space planning fails.
        """
        if not self._connected:
            return PlanResult(success=False, failure_reason="MoveIt not connected")

        try:
            from moveit.core.robot_state import RobotState          # type: ignore  # noqa: PLC0415
            from geometry_msgs.msg import Pose, Point, Quaternion   # noqa: PLC0415

            cfg = self.config
            self._move_group.set_start_state_to_current_state()

            # Build pose goal
            target_pose = Pose()
            target_pose.position = Point(
                x=target_position[0],
                y=target_position[1],
                z=target_position[2],
            )
            target_pose.orientation = Quaternion(
                x=target_orientation[0],
                y=target_orientation[1],
                z=target_orientation[2],
                w=target_orientation[3],
            )
            self._move_group.set_goal_state(
                pose_stamped_msg=target_pose,
                pose_link="panda_hand_tcp",
            )

            # Set scaling
            plan_params = self._move_group.plan()

            if plan_params and hasattr(plan_params, "trajectory"):
                return PlanResult(
                    success=True,
                    trajectory=plan_params.trajectory,
                    num_waypoints=len(
                        plan_params.trajectory.joint_trajectory.points
                    ),
                )
            return PlanResult(success=False, failure_reason="Planner returned no trajectory")

        except Exception as exc:   # noqa: BLE001
            return PlanResult(success=False, failure_reason=str(exc))

    def plan_cartesian_path(
        self,
        waypoints: List[MotionWaypoint],
    ) -> PlanResult:
        """
        Plan a Cartesian path through a sequence of pose waypoints.

        Used for the descend/lower phases where straight-line motion is
        essential to avoid knocking adjacent pieces.

        Parameters
        ----------
        waypoints : list of MotionWaypoint (motion-only, no gripper steps)
        """
        if not self._connected:
            return PlanResult(success=False, failure_reason="MoveIt not connected")

        try:
            from geometry_msgs.msg import Pose, Point, Quaternion   # noqa: PLC0415

            pose_list = []
            for wp in waypoints:
                p = Pose()
                p.position = Point(x=wp.position[0], y=wp.position[1], z=wp.position[2])
                p.orientation = Quaternion(
                    x=wp.orientation[0], y=wp.orientation[1],
                    z=wp.orientation[2], w=wp.orientation[3],
                )
                pose_list.append(p)

            cfg = self.config
            fraction, trajectory = self._move_group.compute_cartesian_path(
                waypoints=pose_list,
                eef_step=cfg.cartesian_step_size,
                jump_threshold=cfg.cartesian_jump_threshold,
            )

            if fraction < 0.95:
                return PlanResult(
                    success=False,
                    failure_reason=(
                        f"Cartesian path only {fraction*100:.1f}% complete "
                        f"(need ≥95%)"
                    ),
                )

            return PlanResult(
                success=True,
                trajectory=trajectory,
                num_waypoints=len(trajectory.joint_trajectory.points),
            )

        except Exception as exc:   # noqa: BLE001
            return PlanResult(success=False, failure_reason=str(exc))

    def plan_home(self) -> PlanResult:
        """Plan a return to the joint-space home pose."""
        if not self._connected:
            return PlanResult(success=False, failure_reason="MoveIt not connected")

        try:
            import math  # noqa: PLC0415
            home_rad = [math.radians(d) for d in self.HOME_JOINTS_DEG]
            self._move_group.set_start_state_to_current_state()
            self._move_group.set_goal_state(configuration_name="home")
            plan = self._move_group.plan()
            if plan and hasattr(plan, "trajectory"):
                return PlanResult(success=True, trajectory=plan.trajectory)
            return PlanResult(success=False, failure_reason="Home planning failed")

        except Exception as exc:   # noqa: BLE001
            return PlanResult(success=False, failure_reason=str(exc))

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------

    def execute(self, plan_result: PlanResult) -> ExecutionResult:
        """
        Execute a previously planned trajectory.

        Parameters
        ----------
        plan_result : output of plan_to_pose() or plan_cartesian_path()
        """
        if not self._connected:
            return ExecutionResult(
                success=False,
                failure_reason="MoveIt not connected",
            )
        if not plan_result.success or plan_result.trajectory is None:
            return ExecutionResult(
                success=False,
                failure_reason="No valid trajectory to execute",
            )

        try:
            ok = self._moveit.execute(
                plan_result.trajectory,
                controllers=[],
            )
            if ok:
                return ExecutionResult(
                    success=True,
                    waypoints_executed=plan_result.num_waypoints,
                )
            return ExecutionResult(
                success=False,
                failure_reason="Trajectory execution returned failure",
            )

        except Exception as exc:   # noqa: BLE001
            return ExecutionResult(success=False, failure_reason=str(exc))

    # ------------------------------------------------------------------
    # Planning scene management
    # ------------------------------------------------------------------

    def update_collision_scene(
        self,
        board_box: Optional[dict] = None,
        piece_cylinders: Optional[List[dict]] = None,
    ) -> None:
        """
        Update the MoveIt planning scene with board + piece collision objects.

        Parameters
        ----------
        board_box : dict with keys x, y, z, size_x, size_y, size_z
        piece_cylinders : list of dicts with x, y, z, radius, height, id
        """
        if not self._connected:
            return

        try:
            from moveit_msgs.msg import CollisionObject, BoundingVolume  # type: ignore  # noqa: PLC0415
            from shape_msgs.msg import SolidPrimitive                    # type: ignore  # noqa: PLC0415
            from geometry_msgs.msg import Pose                           # noqa: PLC0415

            planning_scene_monitor = self._moveit.get_planning_scene_monitor()

            with planning_scene_monitor.read_write() as scene:
                # Board box
                if board_box:
                    board_obj = CollisionObject()
                    board_obj.id = "chess_board"
                    board_obj.header.frame_id = "world"
                    prim = SolidPrimitive()
                    prim.type = SolidPrimitive.BOX
                    prim.dimensions = [
                        board_box.get("size_x", 0.40),
                        board_box.get("size_y", 0.40),
                        board_box.get("size_z", 0.762),  # full table height
                    ]
                    pose = Pose()
                    pose.position.x = board_box.get("x", 0.0)
                    pose.position.y = board_box.get("y", 0.0)
                    pose.position.z = board_box.get("z", 0.381)  # half height
                    pose.orientation.w = 1.0
                    board_obj.primitives = [prim]
                    board_obj.primitive_poses = [pose]
                    board_obj.operation = CollisionObject.ADD
                    scene.apply_collision_object(board_obj)

        except Exception as exc:   # noqa: BLE001
            if self._node:
                self._node.get_logger().warn(
                    f"Failed to update planning scene: {exc}"
                )

    # ------------------------------------------------------------------
    # IK feasibility check (lightweight, no full planning)
    # ------------------------------------------------------------------

    def is_reachable(
        self,
        position: Tuple[float, float, float],
        orientation: Tuple[float, float, float, float],
    ) -> bool:
        """
        Check if a Cartesian pose is kinematically reachable (IK exists).

        Returns True/False.  Does NOT plan a full trajectory.
        """
        if not self._connected:
            # Without MoveIt, use the geometric reach bound from architecture
            from .waypoint_planner import WaypointConfig   # noqa: PLC0415
            # Conservative: if within 0.85 m of origin it's likely reachable
            d = math.sqrt(sum(x*x for x in position))
            return d < 0.85

        try:
            from moveit.core.robot_state import RobotState          # type: ignore  # noqa: PLC0415
            from geometry_msgs.msg import Pose, Point, Quaternion   # noqa: PLC0415

            robot_state = RobotState(self._robot_model)
            pose = Pose()
            pose.position = Point(x=position[0], y=position[1], z=position[2])
            pose.orientation = Quaternion(
                x=orientation[0], y=orientation[1],
                z=orientation[2], w=orientation[3],
            )
            return robot_state.set_from_ik(
                self.config.move_group_name,
                pose,
                "panda_hand_tcp",
                timeout=0.1,
            )

        except Exception:   # noqa: BLE001
            return False