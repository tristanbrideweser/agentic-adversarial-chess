"""
waypoint_planner.py
===================
Pure-Python waypoint generation for chess pick-and-place operations.

Computes the 8-step motion sequence defined in the architecture §2.6:

    1. APPROACH   — above pick pose at +approach_z_offset
    2. DESCEND    — lower to grasp height
    3. GRASP      — close gripper (gripper command, no motion)
    4. LIFT       — raise +lift_z_offset
    5. TRANSIT    — move above place pose at +transit_z_offset
    6. LOWER      — descend to place height
    7. RELEASE    — open gripper (gripper command, no motion)
    8. RETRACT    — raise +retract_z_offset, return to home

Each waypoint is a MotionWaypoint dataclass carrying:
  - pose (x, y, z, qx, qy, qz, qw) in world frame
  - gripper state: "open" | "closed" | "unchanged"
  - label (for logging / debugging)
  - velocity_scaling (0.0–1.0, slower near pieces)
  - acceleration_scaling

No ROS imports anywhere in this file — fully testable without a robot.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Tuple


# ---------------------------------------------------------------------------
# Gripper state
# ---------------------------------------------------------------------------

class GripperState(Enum):
    OPEN      = "open"
    CLOSED    = "closed"
    UNCHANGED = "unchanged"


# ---------------------------------------------------------------------------
# Waypoint
# ---------------------------------------------------------------------------

@dataclass
class MotionWaypoint:
    """
    A single waypoint in the pick-and-place motion sequence.

    position  : (x, y, z) world frame, metres
    orientation : (qx, qy, qz, qw) unit quaternion
    gripper   : gripper command at this waypoint
    label     : human-readable step name for logging
    velocity_scaling     : fraction of max joint velocity (0.0–1.0)
    acceleration_scaling : fraction of max joint acceleration (0.0–1.0)
    """
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]  # xyzw
    gripper: GripperState = GripperState.UNCHANGED
    label: str = ""
    velocity_scaling: float = 0.5
    acceleration_scaling: float = 0.5

    def to_dict(self) -> dict:
        return {
            "label": self.label,
            "position": list(self.position),
            "orientation": list(self.orientation),
            "gripper": self.gripper.value,
            "velocity_scaling": self.velocity_scaling,
            "acceleration_scaling": self.acceleration_scaling,
        }


# ---------------------------------------------------------------------------
# Waypoint planner configuration
# ---------------------------------------------------------------------------

@dataclass
class WaypointConfig:
    """
    Motion parameters for pick-and-place waypoint generation.
    All distances in metres, all scalings in [0.0, 1.0].
    """
    approach_z_offset:    float = 0.10   # height above pick for approach
    lift_z_offset:        float = 0.10   # height above pick after grasp
    transit_z_offset:     float = 0.10   # height above place for transit
    retract_z_offset:     float = 0.10   # height above place after release

    # Velocity/accel scaling per phase
    # Slower near pieces (approach/lower/lift), faster in transit
    vel_approach:   float = 0.3
    vel_transit:    float = 0.7
    vel_near_piece: float = 0.2   # descend / lower / retract phases
    vel_home:       float = 0.5

    accel_approach:   float = 0.3
    accel_transit:    float = 0.5
    accel_near_piece: float = 0.2
    accel_home:       float = 0.4

    # Home pose for each arm (joint values or Cartesian — stored as XYZ here
    # for simplicity; the executor uses joint-space home)
    home_position_white: Tuple[float, float, float] = (0.0, -0.45, 1.00)
    home_position_black: Tuple[float, float, float] = (0.0, +0.45, 1.00)

    # Top-down gripper orientation (180° about X → pointing down)
    # Pre-computed: top_down = Quaternion.top_down(0) = (1, 0, 0, 0) approx
    gripper_orientation_down: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# Waypoint planner
# ---------------------------------------------------------------------------

class WaypointPlanner:
    """
    Generates the 8-step pick-and-place waypoint sequence.

    The planner is stateless — call plan_pick_place() for every task.
    """

    def __init__(self, config: Optional[WaypointConfig] = None) -> None:
        self.config = config or WaypointConfig()

    def plan_pick_place(
        self,
        pick_position: Tuple[float, float, float],
        place_position: Tuple[float, float, float],
        pick_orientation: Tuple[float, float, float, float] = None,
        place_orientation: Tuple[float, float, float, float] = None,
        finger_separation: float = 0.030,
        arm_color: str = "white",
    ) -> List[MotionWaypoint]:
        """
        Generate the full 8-step waypoint sequence for one pick-and-place.

        Parameters
        ----------
        pick_position   : (x, y, z) world frame — gripper jaw centre at grasp
        place_position  : (x, y, z) world frame — gripper jaw centre at release
        pick_orientation  : quaternion (xyzw); defaults to top-down
        place_orientation : quaternion (xyzw); defaults to top-down
        finger_separation : jaw gap at contact (for logging/diagnostics)
        arm_color       : "white" or "black" (selects home pose)

        Returns
        -------
        List of MotionWaypoint in execution order.
        """
        cfg = self.config
        q_pick  = pick_orientation  or cfg.gripper_orientation_down
        q_place = place_orientation or cfg.gripper_orientation_down
        home_pos = (cfg.home_position_white if arm_color == "white"
                    else cfg.home_position_black)

        px, py, pz = pick_position
        lx, ly, lz = place_position

        waypoints: List[MotionWaypoint] = [

            # 1. APPROACH — move above the pick square
            MotionWaypoint(
                position=(px, py, pz + cfg.approach_z_offset),
                orientation=q_pick,
                gripper=GripperState.OPEN,
                label="approach",
                velocity_scaling=cfg.vel_approach,
                acceleration_scaling=cfg.accel_approach,
            ),

            # 2. DESCEND — lower to grasp height
            MotionWaypoint(
                position=(px, py, pz),
                orientation=q_pick,
                gripper=GripperState.UNCHANGED,
                label="descend",
                velocity_scaling=cfg.vel_near_piece,
                acceleration_scaling=cfg.accel_near_piece,
            ),

            # 3. GRASP — close gripper (position held)
            MotionWaypoint(
                position=(px, py, pz),
                orientation=q_pick,
                gripper=GripperState.CLOSED,
                label="grasp",
                velocity_scaling=cfg.vel_near_piece,
                acceleration_scaling=cfg.accel_near_piece,
            ),

            # 4. LIFT — raise above pick square
            MotionWaypoint(
                position=(px, py, pz + cfg.lift_z_offset),
                orientation=q_pick,
                gripper=GripperState.CLOSED,
                label="lift",
                velocity_scaling=cfg.vel_near_piece,
                acceleration_scaling=cfg.accel_near_piece,
            ),

            # 5. TRANSIT — move above place square
            MotionWaypoint(
                position=(lx, ly, lz + cfg.transit_z_offset),
                orientation=q_place,
                gripper=GripperState.CLOSED,
                label="transit",
                velocity_scaling=cfg.vel_transit,
                acceleration_scaling=cfg.accel_transit,
            ),

            # 6. LOWER — descend to place height
            MotionWaypoint(
                position=(lx, ly, lz),
                orientation=q_place,
                gripper=GripperState.CLOSED,
                label="lower",
                velocity_scaling=cfg.vel_near_piece,
                acceleration_scaling=cfg.accel_near_piece,
            ),

            # 7. RELEASE — open gripper (position held)
            MotionWaypoint(
                position=(lx, ly, lz),
                orientation=q_place,
                gripper=GripperState.OPEN,
                label="release",
                velocity_scaling=cfg.vel_near_piece,
                acceleration_scaling=cfg.accel_near_piece,
            ),

            # 8. RETRACT — raise and return to home
            MotionWaypoint(
                position=(lx, ly, lz + cfg.retract_z_offset),
                orientation=q_place,
                gripper=GripperState.OPEN,
                label="retract",
                velocity_scaling=cfg.vel_home,
                acceleration_scaling=cfg.accel_home,
            ),
        ]

        return waypoints

    def plan_home(
        self,
        current_position: Tuple[float, float, float],
        arm_color: str = "white",
    ) -> List[MotionWaypoint]:
        """
        Generate a single waypoint to return the arm to its home pose.
        Used after an abort or at game start/end.
        """
        cfg = self.config
        home_pos = (cfg.home_position_white if arm_color == "white"
                    else cfg.home_position_black)
        return [
            MotionWaypoint(
                position=home_pos,
                orientation=cfg.gripper_orientation_down,
                gripper=GripperState.OPEN,
                label="home",
                velocity_scaling=cfg.vel_home,
                acceleration_scaling=cfg.accel_home,
            )
        ]

    def estimate_duration(
        self,
        waypoints: List[MotionWaypoint],
        max_tcp_speed_mps: float = 0.5,
    ) -> float:
        """
        Rough estimate of total execution time in seconds.

        Computed as sum of (Euclidean distance / scaled_speed) between waypoints.
        Gripper-only steps (same position) get a fixed 0.5 s allowance.
        """
        total = 0.0
        for i in range(1, len(waypoints)):
            a = waypoints[i - 1].position
            b = waypoints[i].position
            dist = math.sqrt(sum((b[j] - a[j]) ** 2 for j in range(3)))
            if dist < 1e-4:
                total += 0.5   # gripper-only step
            else:
                speed = max_tcp_speed_mps * waypoints[i].velocity_scaling
                total += dist / max(speed, 1e-6)
        return round(total, 2)

    def validate_waypoints(
        self,
        waypoints: List[MotionWaypoint],
        board_surface_z: float = 0.762,
        min_clearance_z: float = 0.005,
    ) -> List[str]:
        """
        Basic sanity checks on a waypoint list.

        Returns a list of warning strings (empty = all OK).
        Checks:
          - No waypoint goes below board surface (minus small tolerance)
          - Approach and lift waypoints are above grasp Z
          - Sequence starts with gripper OPEN
        """
        warnings: List[str] = []
        floor = board_surface_z - min_clearance_z

        for i, wp in enumerate(waypoints):
            if wp.position[2] < floor:
                warnings.append(
                    f"Waypoint {i} ({wp.label}): Z={wp.position[2]:.4f}m is below "
                    f"board surface floor {floor:.4f}m"
                )

        if waypoints and waypoints[0].gripper != GripperState.OPEN:
            warnings.append(
                "First waypoint should open the gripper (approach phase)"
            )

        # Check approach is above descend
        labels = [wp.label for wp in waypoints]
        if "approach" in labels and "descend" in labels:
            idx_ap = labels.index("approach")
            idx_de = labels.index("descend")
            if (idx_de > idx_ap and
                    waypoints[idx_ap].position[2] <=
                    waypoints[idx_de].position[2]):
                warnings.append(
                    "Approach Z should be above descend Z"
                )

        return warnings