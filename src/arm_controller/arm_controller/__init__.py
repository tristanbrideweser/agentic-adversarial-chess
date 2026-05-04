"""
arm_controller
==============
ROS 2 package: pick-and-place execution for both Franka Panda arms.

One PickPlaceServer instance runs per arm, namespaced as
/white_panda/pick_place and /black_panda/pick_place.

Architecture
------------
  WaypointPlanner   — pure-Python 8-step sequence generator (testable, no ROS)
  GripperInterface  — gripper command builder + state tracker (no ROS)
  GripperActionClient — Franka gripper action client (ROS-aware)
  MoveItClient      — MoveIt 2 planning + execution (ROS-aware)
  PickPlaceServer   — ROS 2 action server orchestrating all of the above

Public API (pure Python, no ROS required)
-----------------------------------------
  WaypointConfig, WaypointPlanner, MotionWaypoint, GripperState
  GripperConfig, GripperInterface, GripperCommand
  PlanningConfig, PlanResult, ExecutionResult
"""

from .waypoint_planner import (
    GripperState,
    MotionWaypoint,
    WaypointConfig,
    WaypointPlanner,
)

from .gripper_interface import (
    GripperActionClient,
    GripperCommand,
    GripperConfig,
    GripperInterface,
    GripperStateEnum,
)

from .moveit_client import (
    ExecutionResult,
    MoveItClient,
    PlanningConfig,
    PlanResult,
)

__all__ = [
    # waypoint_planner
    "GripperState",
    "MotionWaypoint",
    "WaypointConfig",
    "WaypointPlanner",
    # gripper_interface
    "GripperActionClient",
    "GripperCommand",
    "GripperConfig",
    "GripperInterface",
    "GripperStateEnum",
    # moveit_client
    "ExecutionResult",
    "MoveItClient",
    "PlanningConfig",
    "PlanResult",
]