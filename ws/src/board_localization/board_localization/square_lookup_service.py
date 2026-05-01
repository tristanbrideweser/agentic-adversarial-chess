"""
square_lookup_service.py
========================
ROS 2 service node that resolves square and graveyard names to
geometry_msgs/PoseStamped in the world frame.

Services
--------
  /board_localization/get_square_pose
      Request : string square    (e.g. "e4")
      Response: PoseStamped pose (world frame)
               bool    success
               string  message

  /board_localization/get_graveyard_pose
      Request : string color       ("white" | "black")
                int32  slot_index  (0–15)
      Response: PoseStamped pose
               bool    success
               string  message

  /board_localization/world_to_square
      Request : float64 x, float64 y   (world frame, metres)
      Response: string  square          (e.g. "e4")
               bool     success
               string   message

Both analytic lookup (default) and TF-based lookup are supported.
Set ``use_tf_lookup: true`` in board_params.yaml to use TF (requires the
TF broadcaster to be running and the board to be localized).

Parameters
----------
  use_tf_lookup  : bool   (default false) — use TF buffer instead of math
  tf_timeout_sec : float  (default 0.5)  — TF lookup timeout
  world_frame    : string (default "world")
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Point, Quaternion

from .board_geometry import (
    WORLD_FRAME,
    BoardPose,
    board_pose_to_square,
    graveyard_slot_pose,
    graveyard_tf_frame,
    square_to_board_pose,
    square_to_tf_frame,
)

# Custom service interfaces — defined in chess_interfaces package.
# We import lazily so the module is importable in tests without ROS.
try:
    from chess_interfaces.srv import (   # type: ignore
        GetSquarePose,
        GetGraveyardPose,
        WorldToSquare,
    )
    _INTERFACES_AVAILABLE = True
except ImportError:
    _INTERFACES_AVAILABLE = False


def _board_pose_to_ros(pose: BoardPose, frame_id: str, stamp) -> PoseStamped:
    """Convert a BoardPose to a geometry_msgs/PoseStamped."""
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.header.stamp = stamp
    ps.pose.position = Point(x=pose.x, y=pose.y, z=pose.z)
    ps.pose.orientation = Quaternion(
        x=pose.qx, y=pose.qy, z=pose.qz, w=pose.qw
    )
    return ps


class SquareLookupService(Node):
    """
    Service node for resolving chess board positions to world-frame poses.
    """

    def __init__(self) -> None:
        super().__init__("square_lookup_service")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("use_tf_lookup",  False)
        self.declare_parameter("tf_timeout_sec", 0.5)
        self.declare_parameter("world_frame",    WORLD_FRAME)

        self._use_tf      = self.get_parameter("use_tf_lookup").value
        self._tf_timeout  = self.get_parameter("tf_timeout_sec").value
        self._world_frame = self.get_parameter("world_frame").value

        # ------------------------------------------------------------------
        # TF buffer (only used when use_tf_lookup is True)
        # ------------------------------------------------------------------
        self._tf_buffer = None
        if self._use_tf:
            from tf2_ros import Buffer, TransformListener  # noqa: PLC0415
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)

        # ------------------------------------------------------------------
        # Services
        # ------------------------------------------------------------------
        if not _INTERFACES_AVAILABLE:
            self.get_logger().warn(
                "chess_interfaces not found — services not registered. "
                "Build chess_interfaces first or use get_pose_direct() for "
                "programmatic access."
            )
            return

        self._sq_srv = self.create_service(
            GetSquarePose,
            "/board_localization/get_square_pose",
            self._handle_get_square_pose,
        )
        self._gy_srv = self.create_service(
            GetGraveyardPose,
            "/board_localization/get_graveyard_pose",
            self._handle_get_graveyard_pose,
        )
        self._rev_srv = self.create_service(
            WorldToSquare,
            "/board_localization/world_to_square",
            self._handle_world_to_square,
        )

        mode = "TF-based" if self._use_tf else "analytic"
        self.get_logger().info(
            f"SquareLookupService ready ({mode} mode). "
            f"Registered 3 services."
        )

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------

    def _handle_get_square_pose(self, request, response):
        """
        /board_localization/get_square_pose
        Request.square → Response.pose (PoseStamped, world frame)
        """
        square = request.square.strip().lower()
        try:
            pose = self.get_pose_direct(square)
            response.pose = _board_pose_to_ros(
                pose, self._world_frame, self.get_clock().now().to_msg()
            )
            response.success = True
            response.message = f"OK: {square} → ({pose.x:.4f}, {pose.y:.4f}, {pose.z:.4f})"
        except ValueError as e:
            response.success = False
            response.message = str(e)
        return response

    def _handle_get_graveyard_pose(self, request, response):
        """
        /board_localization/get_graveyard_pose
        Request.{color, slot_index} → Response.pose
        """
        color = request.color.strip().lower()
        slot  = int(request.slot_index)
        try:
            pose = self.get_graveyard_direct(color, slot)
            response.pose = _board_pose_to_ros(
                pose, self._world_frame, self.get_clock().now().to_msg()
            )
            response.success = True
            response.message = f"OK: graveyard_{color}_{slot}"
        except (ValueError, KeyError) as e:
            response.success = False
            response.message = str(e)
        return response

    def _handle_world_to_square(self, request, response):
        """
        /board_localization/world_to_square
        Request.{x, y} → Response.square
        """
        try:
            square = board_pose_to_square(float(request.x), float(request.y))
            response.square = square
            response.success = True
            response.message = f"OK: ({request.x:.4f}, {request.y:.4f}) → {square}"
        except ValueError as e:
            response.square = ""
            response.success = False
            response.message = str(e)
        return response

    # ------------------------------------------------------------------
    # Public programmatic API (no ROS service call overhead — use in tests
    # and in nodes that import board_localization directly)
    # ------------------------------------------------------------------

    def get_pose_direct(self, square: str) -> BoardPose:
        """
        Return the world-frame BoardPose for a square without a service call.

        Uses TF if ``use_tf_lookup=true``, else analytic math.
        """
        if self._use_tf and self._tf_buffer is not None:
            return self._tf_lookup_square(square)
        return square_to_board_pose(square)

    def get_graveyard_direct(self, color: str, slot_index: int) -> BoardPose:
        """Return the world-frame BoardPose for a graveyard slot."""
        if self._use_tf and self._tf_buffer is not None:
            return self._tf_lookup_frame(graveyard_tf_frame(color, slot_index))
        return graveyard_slot_pose(color, slot_index)

    # ------------------------------------------------------------------
    # TF lookups
    # ------------------------------------------------------------------

    def _tf_lookup_square(self, square: str) -> BoardPose:
        frame = square_to_tf_frame(square)
        return self._tf_lookup_frame(frame)

    def _tf_lookup_frame(self, child_frame: str) -> BoardPose:
        """Look up a frame in the TF tree and return a BoardPose."""
        try:
            timeout = Duration(seconds=self._tf_timeout)
            tf = self._tf_buffer.lookup_transform(
                self._world_frame,
                child_frame,
                rclpy.time.Time(),
                timeout,
            )
            t = tf.transform.translation
            r = tf.transform.rotation
            return BoardPose(
                x=t.x, y=t.y, z=t.z,
                qx=r.x, qy=r.y, qz=r.z, qw=r.w,
            )
        except Exception as exc:  # noqa: BLE001
            raise ValueError(
                f"TF lookup failed for frame '{child_frame}': {exc}"
            ) from exc


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SquareLookupService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()