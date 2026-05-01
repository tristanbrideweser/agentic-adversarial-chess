"""
grasp_planner_node.py
=====================
ROS 2 node that exposes a GraspPlanning action server.

Interface
---------
Action server : /grasp_planner/plan  (chess_interfaces/action/GraspPlanning)

    Goal fields:
        string  square       # e.g. "e4"
        string  fen_char     # e.g. "P" (uppercase) or "q" (lowercase)
        bool    use_gpd      # true = GPD mode; false = lookup-only
        float32 yaw_hint     # optional preferred yaw (radians)

    Result fields:
        bool    success
        string  failure_reason
        float64[3]  position        # world frame (m)
        float64[4]  orientation     # quaternion (x, y, z, w)
        float64     finger_separation  # metres
        string  source           # "gpd" | "lookup"

    Feedback:
        string  status           # progress string

Subscriptions:
    /overhead_camera/points  (sensor_msgs/PointCloud2)
        Latest point cloud, cached and passed to GPD on demand.

Parameters (all settable via gpd_params.yaml):
    mode                  : string  "auto" | "gpd" | "lookup"  (default: "auto")
    top_down_tolerance_deg: float   (default: 15.0)
    min_gpd_score         : float   (default: 0.0)
    max_gpd_candidates    : int     (default: 20)
    fallback_to_lookup    : bool    (default: true)
    pointcloud_timeout_sec: float   (default: 2.0)

Mode semantics
--------------
  "lookup" : Always use the precomputed table — fast, deterministic.
  "gpd"    : Always use GPD — requires a point cloud; will fail if unavailable.
  "auto"   : Use GPD if connected + cloud available; else fall back to lookup.
"""

from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from .gpd_client import GPDClient
from .grasp_candidates import PieceType
from .lookup_grasps import LookupGraspPlanner

try:
    from sensor_msgs.msg import PointCloud2
except ImportError:
    PointCloud2 = None  # allow import outside ROS for unit testing

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=1,
)
_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10,
)

VALID_MODES = ("auto", "gpd", "lookup")


class GraspPlannerNode(Node):
    """
    ROS 2 action server for grasp planning.

    Internally owns both a LookupGraspPlanner (always available) and a
    GPDClient (connected lazily on first use).  Mode selection is handled
    per-request via the ``use_gpd`` goal field, subject to the node-level
    ``mode`` parameter.
    """

    def __init__(self) -> None:
        super().__init__("grasp_planner")

        # ---------------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------------
        self.declare_parameter("mode", "auto")
        self.declare_parameter("top_down_tolerance_deg", 15.0)
        self.declare_parameter("min_gpd_score", 0.0)
        self.declare_parameter("max_gpd_candidates", 20)
        self.declare_parameter("fallback_to_lookup", True)
        self.declare_parameter("pointcloud_timeout_sec", 2.0)

        mode = self.get_parameter("mode").value
        if mode not in VALID_MODES:
            self.get_logger().warn(
                f"Unknown mode '{mode}', defaulting to 'auto'."
            )
            mode = "auto"
        self._mode = mode

        tol = self.get_parameter("top_down_tolerance_deg").value
        min_score = self.get_parameter("min_gpd_score").value
        max_cand = self.get_parameter("max_gpd_candidates").value
        fallback = self.get_parameter("fallback_to_lookup").value

        # ---------------------------------------------------------------
        # Planners
        # ---------------------------------------------------------------
        self._lookup = LookupGraspPlanner(
            top_down_tolerance_deg=tol,
        )
        self._gpd = GPDClient(
            top_down_tolerance_deg=tol,
            min_score=min_score,
            max_candidates=max_cand,
            fallback_on_failure=fallback,
        )

        # ---------------------------------------------------------------
        # Point cloud cache
        # ---------------------------------------------------------------
        self._cloud_lock = threading.Lock()
        self._latest_cloud = None
        self._cloud_stamp = None

        if PointCloud2 is not None:
            self._cloud_sub = self.create_subscription(
                PointCloud2,
                "/overhead_camera/points",
                self._on_pointcloud,
                _SENSOR_QOS,
            )
        else:
            self.get_logger().warn(
                "sensor_msgs not available — running without point cloud subscription."
            )

        # ---------------------------------------------------------------
        # Action server (registered lazily to allow import without chess_interfaces)
        # ---------------------------------------------------------------
        self._action_server = None
        self._try_register_action_server()

        # ---------------------------------------------------------------
        # GPD connection (non-blocking)
        # ---------------------------------------------------------------
        if self._mode in ("gpd", "auto"):
            self._gpd.connect(self)

        self.get_logger().info(
            f"GraspPlannerNode started — mode={self._mode}, "
            f"top_down_tol={tol}°"
        )

    # ------------------------------------------------------------------
    # Point cloud subscription
    # ------------------------------------------------------------------

    def _on_pointcloud(self, msg) -> None:
        with self._cloud_lock:
            self._latest_cloud = msg
            self._cloud_stamp = self.get_clock().now()

    def _get_cloud(self) -> Optional[object]:
        """Return the latest point cloud if it is fresh enough."""
        timeout_sec = self.get_parameter("pointcloud_timeout_sec").value
        with self._cloud_lock:
            if self._latest_cloud is None:
                return None
            if self._cloud_stamp is not None:
                age = (self.get_clock().now() - self._cloud_stamp).nanoseconds * 1e-9
                if age > timeout_sec:
                    self.get_logger().warn(
                        f"Point cloud is {age:.1f}s old (timeout={timeout_sec}s)"
                    )
                    return None
            return self._latest_cloud

    # ------------------------------------------------------------------
    # Action server registration
    # ------------------------------------------------------------------

    def _try_register_action_server(self) -> None:
        try:
            from rclpy.action import ActionServer  # noqa: PLC0415
            from chess_interfaces.action import GraspPlanning  # type: ignore  # noqa: PLC0415
            self._action_server = ActionServer(
                self,
                GraspPlanning,
                "/grasp_planner/plan",
                self._handle_goal,
            )
            self.get_logger().info("GraspPlanning action server registered.")
        except ImportError:
            self.get_logger().warn(
                "chess_interfaces not found — action server not registered. "
                "Use plan_direct() for programmatic access."
            )

    async def _handle_goal(self, goal_handle) -> object:
        """
        Action server callback — called by the ROS executor for each goal.
        """
        from chess_interfaces.action import GraspPlanning  # type: ignore  # noqa: PLC0415

        goal = goal_handle.request
        square = goal.square.strip()
        fen_char = goal.fen_char.strip()
        use_gpd = bool(goal.use_gpd)
        yaw_hint = float(goal.yaw_hint) if hasattr(goal, "yaw_hint") else None

        # Send initial feedback
        fb = GraspPlanning.Feedback()
        fb.status = f"Planning grasp for {fen_char} on {square} ..."
        goal_handle.publish_feedback(fb)

        result = GraspPlanning.Result()
        grasp_result = self.plan_direct(square, fen_char, use_gpd, yaw_hint)

        result.success = grasp_result.success
        result.failure_reason = grasp_result.failure_reason

        if grasp_result.success and grasp_result.selected is not None:
            sel = grasp_result.selected
            result.position = list(sel.position)
            result.orientation = list(sel.orientation.to_tuple())
            result.finger_separation = sel.finger_separation
            result.source = sel.source
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    # ------------------------------------------------------------------
    # Public programmatic API (usable without a running action server)
    # ------------------------------------------------------------------

    def plan_direct(
        self,
        square: str,
        fen_char: str,
        use_gpd: bool = False,
        yaw_hint: Optional[float] = None,
    ):
        """
        Plan a grasp directly (bypassing the action server).

        Parameters
        ----------
        square : algebraic notation, e.g. "e4"
        fen_char : FEN character of the piece
        use_gpd : if True, attempt GPD (subject to mode override)
        yaw_hint : preferred gripper yaw in radians (for lookup fallback)

        Returns
        -------
        GraspResult
        """
        effective_use_gpd = self._resolve_mode(use_gpd)

        if effective_use_gpd:
            cloud = self._get_cloud()
            result = self._gpd.plan(square, fen_char, cloud, yaw_hint)
        else:
            result = self._lookup.plan(square, fen_char, yaw_hint)

        self.get_logger().info(
            f"Grasp plan [{result.source if result.selected else 'FAILED'}] "
            f"{fen_char}@{square} — "
            f"{'OK' if result.success else result.failure_reason}"
        )
        return result

    def _resolve_mode(self, goal_use_gpd: bool) -> bool:
        """Resolve whether to use GPD based on the node-level mode override."""
        if self._mode == "lookup":
            return False
        if self._mode == "gpd":
            return True
        # "auto" — honour the per-request flag
        return goal_use_gpd


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()