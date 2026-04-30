"""
tf_broadcaster.py
=================
ROS 2 node that publishes the full board TF tree as static transforms.

Published frames
----------------
  world ──► chess_board                                (1 frame)
  chess_board ──► square_a1 … square_h8               (64 frames)
  chess_board ──► graveyard_white_0 … _15             (16 frames)
  chess_board ──► graveyard_black_0 … _15             (16 frames)
  ─────────────────────────────────────────────────
  Total: 97 static transforms

All poses are computed from the analytic geometry in board_geometry.py.
The ``board_origin`` and ``board_yaw`` parameters allow the board to be
placed at an arbitrary world position (for physical deployment or shifted
Gazebo worlds).  Defaults produce the nominal architecture pose.

Parameters (ROS 2 params, settable via board_params.yaml)
---------------------------------------------------------
  board_origin_x   : float  (default 0.0)
  board_origin_y   : float  (default 0.0)
  board_origin_z   : float  (default 0.762)
  board_yaw        : float  (default 0.0)  — radians, rotation around world Z
  publish_rate_hz  : float  (default 1.0)  — re-publish rate for robustness
                                             (static TFs are latched, but
                                             re-publishing helps late joiners)
"""

from __future__ import annotations

import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from .board_geometry import (
    BOARD_FRAME,
    BOARD_SURFACE_Z,
    SQUARE_SIZE,
    WORLD_FRAME,
    BoardPose,
    all_graveyard_poses,
    all_square_poses,
    apply_board_transform,
    graveyard_slot_pose,
    graveyard_tf_frame,
    square_to_tf_frame,
)


class BoardTFBroadcaster(Node):
    """
    Publishes static TF frames for all 64 squares and 32 graveyard slots.

    This node should be launched once at startup before any other node
    attempts a TF lookup on square frames.
    """

    def __init__(self) -> None:
        super().__init__("board_tf_broadcaster")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("board_origin_x",  0.0)
        self.declare_parameter("board_origin_y",  0.0)
        self.declare_parameter("board_origin_z",  BOARD_SURFACE_Z)
        self.declare_parameter("board_yaw",        0.0)
        self.declare_parameter("publish_rate_hz",  1.0)

        ox  = self.get_parameter("board_origin_x").value
        oy  = self.get_parameter("board_origin_y").value
        oz  = self.get_parameter("board_origin_z").value
        yaw = self.get_parameter("board_yaw").value
        hz  = self.get_parameter("publish_rate_hz").value

        self._origin = (ox, oy, oz)
        self._yaw    = yaw

        # ------------------------------------------------------------------
        # Static broadcaster
        # ------------------------------------------------------------------
        self._broadcaster = StaticTransformBroadcaster(self)

        # Build and publish all transforms once at startup
        transforms = self._build_all_transforms()
        self._broadcaster.sendTransform(transforms)
        self.get_logger().info(
            f"Published {len(transforms)} static TF frames "
            f"(origin={ox:.3f},{oy:.3f},{oz:.3f}, yaw={math.degrees(yaw):.1f}°)"
        )

        # Re-publish periodically so late-joining nodes receive the frames
        # (StaticTransformBroadcaster is latched but a timer is belt-and-suspenders)
        if hz > 0.0:
            self.create_timer(1.0 / hz, self._republish)

    # ------------------------------------------------------------------
    # Transform builders
    # ------------------------------------------------------------------

    def _build_all_transforms(self) -> List[TransformStamped]:
        """Build TransformStamped messages for every frame in the TF tree."""
        transforms: List[TransformStamped] = []
        now = self.get_clock().now().to_msg()

        # 1. world → chess_board
        transforms.append(
            self._make_transform(
                now,
                parent=WORLD_FRAME,
                child=BOARD_FRAME,
                x=self._origin[0],
                y=self._origin[1],
                z=self._origin[2],
                qx=0.0, qy=0.0,
                qz=math.sin(self._yaw / 2),
                qw=math.cos(self._yaw / 2),
            )
        )

        # 2. chess_board → square_XX  (64 frames)
        for square, world_pose in all_square_poses().items():
            # Re-express relative to chess_board origin (subtract board origin,
            # un-rotate by board_yaw)
            board_rel = self._world_to_board_relative(world_pose)
            transforms.append(
                self._make_transform(
                    now,
                    parent=BOARD_FRAME,
                    child=square_to_tf_frame(square),
                    **board_rel,
                )
            )

        # 3. chess_board → graveyard_white_N / graveyard_black_N  (32 frames)
        for frame, world_pose in all_graveyard_poses().items():
            board_rel = self._world_to_board_relative(world_pose)
            transforms.append(
                self._make_transform(
                    now,
                    parent=BOARD_FRAME,
                    child=frame,
                    **board_rel,
                )
            )

        return transforms

    def _world_to_board_relative(self, world_pose: BoardPose) -> dict:
        """
        Convert a world-frame BoardPose to a position relative to chess_board.

        chess_board sits at (origin_x, origin_y, origin_z) rotated by yaw.
        We need the inverse: translate then un-rotate.
        """
        dx = world_pose.x - self._origin[0]
        dy = world_pose.y - self._origin[1]
        dz = world_pose.z - self._origin[2]

        cos_y = math.cos(-self._yaw)
        sin_y = math.sin(-self._yaw)

        rx = dx * cos_y - dy * sin_y
        ry = dx * sin_y + dy * cos_y

        return {"x": round(rx, 6), "y": round(ry, 6), "z": round(dz, 6),
                "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}

    @staticmethod
    def _make_transform(
        stamp,
        parent: str,
        child: str,
        x: float, y: float, z: float,
        qx: float, qy: float, qz: float, qw: float,
    ) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        return t

    def _republish(self) -> None:
        """Re-send the static transforms (ensures late-joining nodes receive them)."""
        transforms = self._build_all_transforms()
        self._broadcaster.sendTransform(transforms)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoardTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()