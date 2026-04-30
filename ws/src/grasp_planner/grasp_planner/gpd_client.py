"""
gpd_client.py
=============
GPD (Grasp Pose Detection) client for the chess robot grasp planner.

This module handles the full GPD pipeline:
  1. Crop the point cloud around the target square (±crop_box_m)
  2. Run GPD via its ROS action/service interface
  3. Parse the response into GraspCandidate objects
  4. Pass candidates to GraspFilter for ranking

Architecture note
-----------------
GPD is called as a ROS action (GraspPlanningAction).  If GPD is unavailable
(e.g. during unit tests or when running in lookup-only mode), the client
gracefully falls back to the LookupGraspPlanner.

The GPD action server is expected at /grasp_planner/gpd_action.
The point cloud topic is /overhead_camera/points (PointCloud2).

This module is ROS-aware but all pure-Python logic (cropping geometry,
candidate parsing, fallback) is testable without a ROS context.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

from .grasp_candidates import (
    GraspCandidate,
    GraspResult,
    PieceType,
    Quaternion,
    get_profile,
)
from .grasp_filter import GraspFilter
from .lookup_grasps import LookupGraspPlanner, _square_xy

# ---------------------------------------------------------------------------
# Board geometry constants
# ---------------------------------------------------------------------------

BOARD_SURFACE_Z = 0.762
CROP_BOX_HALF = 0.03        # ±3 cm around target square centre
CROP_Z_MIN = BOARD_SURFACE_Z - 0.005   # just below board surface
CROP_Z_MAX = BOARD_SURFACE_Z + 0.12    # 12 cm above surface (clears all pieces)


# ---------------------------------------------------------------------------
# Geometry helpers (no ROS dependency)
# ---------------------------------------------------------------------------

def compute_crop_box(square: str) -> dict:
    """
    Return axis-aligned bounding box parameters for cropping the point cloud
    around the given square.

    Returns a dict with keys: x_min, x_max, y_min, y_max, z_min, z_max
    """
    cx, cy = _square_xy(square)
    return {
        "x_min": cx - CROP_BOX_HALF,
        "x_max": cx + CROP_BOX_HALF,
        "y_min": cy - CROP_BOX_HALF,
        "y_max": cy + CROP_BOX_HALF,
        "z_min": CROP_Z_MIN,
        "z_max": CROP_Z_MAX,
    }


def approach_angle_from_quaternion(
    qx: float, qy: float, qz: float, qw: float
) -> float:
    """
    Compute the angle (degrees) between the gripper approach vector and
    the world -Z axis (straight down).

    The gripper approach direction in its own frame is +X.  We rotate
    (1, 0, 0) by the quaternion and measure its angle from (0, 0, -1).

    Returns degrees in [0, 180].
    """
    # Rotate unit X vector by quaternion
    # v' = q * (1,0,0) * q_conjugate
    # Optimised form for rotating (1,0,0):
    tx = 1.0 - 2.0 * (qy * qy + qz * qz)
    ty = 2.0 * (qx * qy + qz * qw)
    tz = 2.0 * (qx * qz - qy * qw)

    # Approach direction is the rotated X vector
    # Angle from world -Z axis = (0, 0, -1)
    dot = -tz   # dot product with (0, 0, -1) = -tz
    dot = max(-1.0, min(1.0, dot))
    angle_rad = math.acos(dot)
    return math.degrees(angle_rad)


def parse_gpd_grasp(
    position: Tuple[float, float, float],
    orientation_xyzw: Tuple[float, float, float, float],
    score: float,
    finger_separation: float = 0.030,
) -> GraspCandidate:
    """
    Parse one GPD output grasp into a GraspCandidate.

    Parameters
    ----------
    position : (x, y, z) in world frame (metres)
    orientation_xyzw : quaternion (x, y, z, w)
    score : GPD antipodal quality score
    finger_separation : jaw width at contact (metres)
    """
    qx, qy, qz, qw = orientation_xyzw
    angle_deg = approach_angle_from_quaternion(qx, qy, qz, qw)

    return GraspCandidate(
        position=position,
        orientation=Quaternion(qx, qy, qz, qw).normalised(),
        score=score,
        approach_angle_deg=angle_deg,
        finger_separation=finger_separation,
        source="gpd",
    )


# ---------------------------------------------------------------------------
# GPD client (ROS-aware, with graceful fallback)
# ---------------------------------------------------------------------------

class GPDClient:
    """
    Thin wrapper around the GPD ROS action server.

    In a live ROS environment, call ``connect()`` to establish the action
    client before calling ``plan()``.  When ``connect()`` has not been
    called (e.g. in unit tests), all calls fall back to the lookup table.

    Parameters
    ----------
    top_down_tolerance_deg : float
        Maximum deviation from vertical for a GPD grasp to be accepted.
    min_score : float
        Minimum GPD quality score to keep a candidate.
    max_candidates : int
        Maximum number of GPD candidates to request.
    fallback_on_failure : bool
        If True, fall back to lookup table when GPD returns no valid grasps.
    """

    GPD_ACTION_NAME = "/grasp_planner/gpd_action"
    POINTCLOUD_TOPIC = "/overhead_camera/points"

    def __init__(
        self,
        top_down_tolerance_deg: float = 15.0,
        min_score: float = 0.0,
        max_candidates: int = 20,
        fallback_on_failure: bool = True,
    ) -> None:
        self._tolerance = top_down_tolerance_deg
        self._min_score = min_score
        self._max_candidates = max_candidates
        self._fallback = fallback_on_failure
        self._filter = GraspFilter(top_down_tolerance_deg=top_down_tolerance_deg)
        self._lookup = LookupGraspPlanner()

        # ROS action client — populated by connect()
        self._action_client = None
        self._node = None

    # ------------------------------------------------------------------
    # ROS connection (call from a ROS node's __init__)
    # ------------------------------------------------------------------

    def connect(self, node) -> bool:
        """
        Connect to the GPD action server.

        Parameters
        ----------
        node : rclpy.node.Node
            The owning ROS node (used to create the action client).

        Returns
        -------
        bool : True if the server became available within the timeout.
        """
        try:
            from rclpy.action import ActionClient  # noqa: PLC0415
            # GPD uses gpd_ros GraspConfigList action — adapt to your GPD version.
            # Here we use a generic action type name; replace with the actual
            # action type from your GPD ROS package.
            from gpd_ros.action import GraspPlanning  # type: ignore  # noqa: PLC0415
            self._node = node
            self._action_client = ActionClient(
                node,
                GraspPlanning,
                self.GPD_ACTION_NAME,
            )
            available = self._action_client.wait_for_server(timeout_sec=5.0)
            if available:
                node.get_logger().info("GPD action server connected.")
            else:
                node.get_logger().warn(
                    "GPD action server not available — will use lookup table."
                )
            return available
        except ImportError:
            if self._node:
                self._node.get_logger().warn(
                    "gpd_ros not installed — running in lookup-only mode."
                )
            return False

    @property
    def is_connected(self) -> bool:
        return self._action_client is not None

    # ------------------------------------------------------------------
    # Main planning interface
    # ------------------------------------------------------------------

    def plan(
        self,
        square: str,
        fen_char: str,
        pointcloud=None,
        yaw_hint_rad: Optional[float] = None,
    ) -> GraspResult:
        """
        Plan a grasp for the piece on the given square.

        Tries GPD first (if connected and a point cloud is provided),
        then falls back to the lookup table.

        Parameters
        ----------
        square : algebraic notation, e.g. "e4"
        fen_char : FEN character of the piece
        pointcloud : sensor_msgs/PointCloud2 message, or None
        yaw_hint_rad : preferred yaw for the lookup fallback

        Returns
        -------
        GraspResult
        """
        piece_type = PieceType.from_fen_char(fen_char)

        # Try GPD if we have both a connection and a point cloud
        if self.is_connected and pointcloud is not None:
            gpd_result = self._plan_with_gpd(square, piece_type, pointcloud)
            if gpd_result.success:
                return gpd_result
            if not self._fallback:
                return gpd_result
            # Log and fall through to lookup
            if self._node:
                self._node.get_logger().warn(
                    f"GPD returned no valid grasps for {piece_type.name} on "
                    f"{square} — falling back to lookup table."
                )

        # Lookup table fallback
        return self._lookup.plan(square, fen_char, yaw_hint_rad)

    # ------------------------------------------------------------------
    # GPD pipeline (internal)
    # ------------------------------------------------------------------

    def _plan_with_gpd(
        self,
        square: str,
        piece_type: PieceType,
        pointcloud,
    ) -> GraspResult:
        """
        Run the GPD pipeline: crop → request → parse → filter.

        This is a synchronous wrapper around the async action call.
        In a real deployment, use the async send_goal / get_result pattern
        within a ROS executor.
        """
        crop_box = compute_crop_box(square)
        profile = get_profile(piece_type)

        try:
            raw_candidates = self._call_gpd_action(
                pointcloud, crop_box, self._max_candidates
            )
        except Exception as exc:  # noqa: BLE001
            return GraspResult(
                piece_type=piece_type,
                target_square=square,
                success=False,
                failure_reason=f"GPD action call failed: {exc}",
            )

        if not raw_candidates:
            return GraspResult(
                piece_type=piece_type,
                target_square=square,
                success=False,
                failure_reason="GPD returned zero candidates",
            )

        # Filter and rank
        filtered = self._filter.filter_and_rank(
            raw_candidates,
            piece_type=piece_type,
            target_z=profile.grasp_z,
        )

        if not filtered:
            return GraspResult(
                piece_type=piece_type,
                target_square=square,
                all_candidates=raw_candidates,
                success=False,
                failure_reason=(
                    f"All {len(raw_candidates)} GPD candidates rejected by filter "
                    f"(top-down tolerance={self._tolerance}°, "
                    f"min_score={self._min_score})"
                ),
            )

        return GraspResult(
            piece_type=piece_type,
            target_square=square,
            selected=filtered[0],
            all_candidates=raw_candidates,
            success=True,
        )

    def _call_gpd_action(
        self,
        pointcloud,
        crop_box: dict,
        max_candidates: int,
    ) -> List[GraspCandidate]:
        """
        Send a goal to the GPD action server and return parsed candidates.

        This method is intentionally thin — the actual action message type
        depends on your GPD ROS package version.  Fill in the goal fields
        to match your gpd_ros action definition.

        Returns a list of GraspCandidate objects (may be empty).
        """
        import rclpy  # noqa: PLC0415

        if self._action_client is None:
            raise RuntimeError("GPD action client not connected.")

        # Build goal — adapt to your gpd_ros action type
        from gpd_ros.action import GraspPlanning  # type: ignore  # noqa: PLC0415

        goal = GraspPlanning.Goal()
        goal.cloud_indexed.cloud = pointcloud
        # Crop box can be passed via a PassThrough filter node upstream,
        # or by filtering the cloud here before sending.
        # Many GPD setups receive the full cloud and use workspace params:
        goal.cloud_indexed.cloud = _crop_pointcloud(pointcloud, crop_box)

        send_goal_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_goal_future, timeout_sec=10.0)

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            raise RuntimeError("GPD goal rejected by action server.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=30.0)

        result = result_future.result()
        if result is None:
            raise RuntimeError("GPD action returned no result.")

        return _parse_gpd_result(result.result, self._min_score)


# ---------------------------------------------------------------------------
# GPD result parsing helpers
# ---------------------------------------------------------------------------

def _crop_pointcloud(cloud, crop_box: dict):
    """
    Crop a sensor_msgs/PointCloud2 to the axis-aligned box.

    In production, this is best done upstream with a PCL PassThrough filter
    node so that only the relevant region is ever sent to GPD.
    Here we do it in Python for completeness — replace with the PCL node
    approach in your deployment.

    Returns the original cloud (no-op stub) — implement with pcl_helper or
    open3d if you want Python-side cropping.
    """
    # TODO: implement with pcl_ros or open3d if needed
    # For now, rely on GPD's own workspace bounding box parameter.
    return cloud


def _parse_gpd_result(result, min_score: float) -> List[GraspCandidate]:
    """
    Parse a gpd_ros GraspPlanning result into GraspCandidate objects.

    Adapt the attribute names to match your specific gpd_ros message type.
    Common formats:
      result.grasps  — list of GraspConfig messages
      Each GraspConfig has: position, approach, binormal, score

    Returns a list of GraspCandidate objects with score >= min_score.
    """
    candidates = []

    for g in getattr(result, "grasps", []):
        try:
            # Position: geometry_msgs/Point
            pos = g.position
            position = (pos.x, pos.y, pos.z)

            # GPD outputs approach + binormal + axis vectors.
            # Convert the rotation matrix to a quaternion.
            # approach = gripper X axis, binormal = Y axis, axis = Z axis
            quat = _rotation_matrix_to_quaternion(
                g.approach, g.binormal, g.axis
            )

            score = float(getattr(g, "score", 1.0))
            if score < min_score:
                continue

            candidates.append(
                parse_gpd_grasp(
                    position=position,
                    orientation_xyzw=(quat.x, quat.y, quat.z, quat.w),
                    score=score,
                    finger_separation=getattr(g, "finger_width", 0.030),
                )
            )
        except (AttributeError, TypeError):
            continue

    return candidates


def _rotation_matrix_to_quaternion(approach, binormal, axis) -> Quaternion:
    """
    Convert GPD's approach/binormal/axis rotation matrix to a quaternion.

    approach  = gripper approach direction (unit vector, geometry_msgs/Vector3)
    binormal  = gripper binormal direction
    axis      = gripper axis direction (along finger width)

    The rotation matrix R = [approach | binormal | axis] (columns).
    """
    # Build 3×3 matrix
    r = [
        [approach.x, binormal.x, axis.x],
        [approach.y, binormal.y, axis.y],
        [approach.z, binormal.z, axis.z],
    ]

    # Shepperd's method
    trace = r[0][0] + r[1][1] + r[2][2]

    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (r[2][1] - r[1][2]) * s
        y = (r[0][2] - r[2][0]) * s
        z = (r[1][0] - r[0][1]) * s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = 2.0 * math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2])
        w = (r[2][1] - r[1][2]) / s
        x = 0.25 * s
        y = (r[0][1] + r[1][0]) / s
        z = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = 2.0 * math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2])
        w = (r[0][2] - r[2][0]) / s
        x = (r[0][1] + r[1][0]) / s
        y = 0.25 * s
        z = (r[1][2] + r[2][1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1])
        w = (r[1][0] - r[0][1]) / s
        x = (r[0][2] + r[2][0]) / s
        y = (r[1][2] + r[2][1]) / s
        z = 0.25 * s

    return Quaternion(x, y, z, w).normalised()