"""
moveit_client.py
================
MoveIt 2 motion planning and execution client for one Franka Panda arm.
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple
from .waypoint_planner import MotionWaypoint, GripperState


@dataclass
class PlanResult:
    success: bool = False
    failure_reason: str = ""
    estimated_duration_sec: float = 0.0
    num_waypoints: int = 0
    trajectory = None


@dataclass
class ExecutionResult:
    success: bool = False
    failure_reason: str = ""
    waypoints_executed: int = 0


@dataclass
class PlanningConfig:
    move_group_name: str = "white_arm"
    planning_pipeline: str = "ompl"
    planner_id: str = "RRTConnectkConfigDefault"
    planning_time_sec: float = 5.0
    num_planning_attempts: int = 3
    max_velocity_scaling: float = 0.5
    max_acceleration_scaling: float = 0.5
    goal_position_tolerance: float = 0.001
    goal_orientation_tolerance: float = 0.01
    cartesian_step_size: float = 0.005
    cartesian_jump_threshold: float = 0.0


class MoveItClient:
    HOME_JOINTS_DEG = [0.0, -45.0, 0.0, -135.0, 0.0, 90.0, 45.0]

    def __init__(self, arm_name: str, namespace: str = "",
                 config: Optional[PlanningConfig] = None) -> None:
        self.arm_name = arm_name
        self.namespace = namespace.rstrip("/")
        self.config = config or PlanningConfig()
        self._node = None
        self._moveit = None
        self._move_group = None
        self._planning_scene = None
        self._robot_model = None
        self._connected = False

    def connect(self, node) -> bool:
        self._node = node
        try:
            from moveit.planning import MoveItPy                   # type: ignore
            from moveit.core.robot_state import RobotState         # type: ignore
            import rclpy
            from rcl_interfaces.srv import GetParameters

            def get_param(service_name, param_name, timeout=5.0):
                client = node.create_client(GetParameters, service_name)
                if not client.wait_for_service(timeout_sec=timeout):
                    return ""
                req = GetParameters.Request()
                req.names = [param_name]
                future = client.call_async(req)
                rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
                if future.result() and future.result().values:
                    return future.result().values[0].string_value
                return ""

            robot_description = get_param(
                "/robot_state_publisher/get_parameters", "robot_description"
            )
            robot_description_semantic = get_param(
                "/move_group/get_parameters", "robot_description_semantic"
            )

            if not robot_description:
                node.get_logger().warn(
                    f"{self.arm_name}: no robot_description found — dry-run mode"
                )
                return False

            config_dict = {
                "robot_description": robot_description,
                "robot_description_semantic": robot_description_semantic,
                "robot_description_kinematics": {
                    self.config.move_group_name: {
                        "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                        "kinematics_solver_search_resolution": 0.005,
                        "kinematics_solver_timeout": 0.05,
                    }
                },
                "planning_pipelines": ["ompl"],
                "default_planning_pipeline": "ompl",
                "ompl": {
                    "planning_plugins": ["ompl_interface/OMPLPlanner"],
                    "request_adapters": [
                        "default_planning_request_adapters/ResolveConstraintFrames",
                        "default_planning_request_adapters/ValidateWorkspaceBounds",
                        "default_planning_request_adapters/CheckStartStateBounds",
                        "default_planning_request_adapters/CheckStartStateCollision",
                    ],
                    "response_adapters": [
                        "default_planning_response_adapters/AddTimeOptimalParameterization",
                        "default_planning_response_adapters/ValidateSolution",
                        "default_planning_response_adapters/DisplayMotionPath",
                    ],
                },
                "sensors": [""],
            }

            import yaml as _yaml
            # Write params file with node name matching what MoveItPy creates
            node_name = f"moveit_{self.arm_name}"
            params_file = f"/tmp/moveit_{self.arm_name}_params.yaml"
            moveit_node_params = {
                node_name: {
                    "ros__parameters": config_dict
                }
            }
            with open(params_file, "w") as _f:
                _yaml.dump(moveit_node_params, _f)

            self._moveit = MoveItPy(
                node_name=node_name,
                launch_params_filepaths=[params_file],
            )
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
        except ImportError as exc:
            node.get_logger().warn(
                f"moveit_py not available ({exc}) — dry-run mode"
            )
            return False
        except Exception as exc:                                   # noqa: BLE001
            node.get_logger().error(f"MoveIt init failed: {exc}")
            return False

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def plan_to_pose(
        self,
        target_position: Tuple[float, float, float],
        target_orientation: Tuple[float, float, float, float],
    ) -> PlanResult:
        if not self._connected:
            return PlanResult(success=False, failure_reason="MoveIt not connected")
        try:
            from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion  # noqa
            self._move_group.set_start_state_to_current_state()
            target_pose = PoseStamped()
            target_pose.header.frame_id = "world"
            target_pose.pose.position = Point(
                x=target_position[0],
                y=target_position[1],
                z=target_position[2],
            )
            target_pose.pose.orientation = Quaternion(
                x=target_orientation[0],
                y=target_orientation[1],
                z=target_orientation[2],
                w=target_orientation[3],
            )
            self._move_group.set_goal_state(
                pose_stamped_msg=target_pose,
                pose_link=f"{self.arm_name.replace('_panda', '_fr3')}_hand_tcp",
            )
            plan_result = self._move_group.plan()
            if plan_result and hasattr(plan_result, "trajectory"):
                return PlanResult(
                    success=True,
                    trajectory=plan_result.trajectory,
                    num_waypoints=len(
                        plan_result.trajectory.joint_trajectory.points
                    ),
                )
            return PlanResult(success=False, failure_reason="Planner returned no trajectory")
        except Exception as exc:                                   # noqa: BLE001
            return PlanResult(success=False, failure_reason=str(exc))

    def plan_cartesian_path(self, waypoints: List[MotionWaypoint]) -> PlanResult:
        if not self._connected:
            return PlanResult(success=False, failure_reason="MoveIt not connected")
        try:
            from geometry_msgs.msg import Pose, Point, Quaternion  # noqa
            pose_list = []
            for wp in waypoints:
                p = Pose()
                p.position = Point(
                    x=wp.position[0], y=wp.position[1], z=wp.position[2]
                )
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
                    failure_reason=f"Cartesian path only {fraction*100:.1f}% complete",
                )
            return PlanResult(
                success=True,
                trajectory=trajectory,
                num_waypoints=len(trajectory.joint_trajectory.points),
            )
        except Exception as exc:                                   # noqa: BLE001
            return PlanResult(success=False, failure_reason=str(exc))

    def plan_home(self) -> PlanResult:
        if not self._connected:
            return PlanResult(success=False, failure_reason="MoveIt not connected")
        try:
            self._move_group.set_start_state_to_current_state()
            self._move_group.set_goal_state(configuration_name="ready")
            plan = self._move_group.plan()
            if plan and hasattr(plan, "trajectory"):
                return PlanResult(success=True, trajectory=plan.trajectory)
            return PlanResult(success=False, failure_reason="Home planning failed")
        except Exception as exc:                                   # noqa: BLE001
            return PlanResult(success=False, failure_reason=str(exc))

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------

    def execute(self, plan_result: PlanResult) -> ExecutionResult:
        if not self._connected:
            return ExecutionResult(success=False, failure_reason="MoveIt not connected")
        if not plan_result.success or plan_result.trajectory is None:
            return ExecutionResult(success=False, failure_reason="No valid trajectory")
        try:
            ok = self._moveit.execute(plan_result.trajectory, controllers=[])
            if ok:
                return ExecutionResult(
                    success=True,
                    waypoints_executed=plan_result.num_waypoints,
                )
            return ExecutionResult(
                success=False,
                failure_reason="Trajectory execution returned failure",
            )
        except Exception as exc:                                   # noqa: BLE001
            return ExecutionResult(success=False, failure_reason=str(exc))

    # ------------------------------------------------------------------
    # Planning scene
    # ------------------------------------------------------------------

    def update_collision_scene(
        self,
        board_box: Optional[dict] = None,
        piece_cylinders: Optional[List[dict]] = None,
    ) -> None:
        if not self._connected:
            return
        try:
            from moveit_msgs.msg import CollisionObject              # type: ignore  # noqa
            from shape_msgs.msg import SolidPrimitive                # type: ignore  # noqa
            from geometry_msgs.msg import Pose                       # noqa
            planning_scene_monitor = self._moveit.get_planning_scene_monitor()
            with planning_scene_monitor.read_write() as scene:
                if board_box:
                    board_obj = CollisionObject()
                    board_obj.id = "chess_board"
                    board_obj.header.frame_id = "world"
                    prim = SolidPrimitive()
                    prim.type = SolidPrimitive.BOX
                    prim.dimensions = [
                        board_box.get("size_x", 0.45),
                        board_box.get("size_y", 0.45),
                        board_box.get("size_z", 0.762),
                    ]
                    pose = Pose()
                    pose.position.x = board_box.get("x", 0.0)
                    pose.position.y = board_box.get("y", 0.0)
                    pose.position.z = board_box.get("z", 0.381)
                    pose.orientation.w = 1.0
                    board_obj.primitives = [prim]
                    board_obj.primitive_poses = [pose]
                    board_obj.operation = CollisionObject.ADD
                    scene.apply_collision_object(board_obj)
        except Exception as exc:                                   # noqa: BLE001
            if self._node:
                self._node.get_logger().warn(f"Failed to update scene: {exc}")

    # ------------------------------------------------------------------
    # IK check
    # ------------------------------------------------------------------

    def is_reachable(
        self,
        position: Tuple[float, float, float],
        orientation: Tuple[float, float, float, float],
    ) -> bool:
        if not self._connected:
            d = math.sqrt(sum(x * x for x in position))
            return d < 0.85
        try:
            from moveit.core.robot_state import RobotState          # type: ignore  # noqa
            from geometry_msgs.msg import Pose, Point, Quaternion   # noqa
            robot_state = RobotState(self._robot_model)
            pose = Pose()
            pose.position = Point(x=position[0], y=position[1], z=position[2])
            pose.orientation = Quaternion(
                x=orientation[0], y=orientation[1],
                z=orientation[2], w=orientation[3],
            )
            return robot_state.set_from_ik(
                self.config.move_group_name, pose,
                f"{self.arm_name.replace('_panda', '_fr3')}_hand_tcp",
                timeout=0.1,
            )
        except Exception:                                          # noqa: BLE001
            return False
