import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import MultiArrayDimension, UInt8MultiArray


def load_yaml(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def square_to_xy(rank_idx: int, file_idx: int, center_xy, square_size: float):
    # Mirrors chess_robot_description/scripts/spawn_pieces.py:
    # rank 1->8 along +X, file a->h along +Y, a1 at most-negative (x, y).
    dx = (rank_idx - 3.5) * square_size
    dy = (file_idx - 3.5) * square_size
    return (center_xy[0] + dx, center_xy[1] + dy)


class BoardVerifier(Node):
    def __init__(self):
        super().__init__('board_verifier')

        board_cfg = load_yaml(Path(get_package_share_directory('chess_robot_description'))
                              / 'config' / 'board_params.yaml')['board']
        cam_cfg = load_yaml(Path(get_package_share_directory('perception'))
                            / 'config' / 'camera_params.yaml')

        self.center_xy = tuple(board_cfg['center_xy'])
        self.square_size = float(board_cfg['square_size'])
        self.board_top_z = float(board_cfg['top_z'])

        cam = cam_cfg['camera']
        det = cam_cfg['detection']
        self.cam_xyz = tuple(cam['pose_xyz'])
        hfov = float(cam['hfov'])
        self.image_w = int(cam['image_w'])
        self.image_h = int(cam['image_h'])

        self.empty_threshold = float(det['empty_depth_threshold'])
        self.roi_shrink = float(det['roi_shrink'])
        self.min_occupied_fraction = float(det.get('min_occupied_fraction', 0.1))
        publish_rate = float(det['publish_rate_hz'])
        depth_topic = str(det['depth_topic'])
        occupancy_topic = str(det['occupancy_topic'])

        # Pinhole intrinsics (square pixels, same hfov on both axes).
        self.fx = (self.image_w / 2.0) / math.tan(hfov / 2.0)
        self.fy = self.fx
        self.cx = self.image_w / 2.0
        self.cy = self.image_h / 2.0

        # Distance from camera to board top (meters). Camera looks straight down.
        self.reference_depth = self.cam_xyz[2] - self.board_top_z

        self.rois = self._precompute_rois()
        self.get_logger().info(
            f'64 ROIs precomputed. fx={self.fx:.1f}, '
            f'reference_depth={self.reference_depth:.3f} m. '
            f'a1 ROI={self.rois[0]}, h8 ROI={self.rois[63]}'
        )

        self.bridge = CvBridge()
        self._latest_depth = None

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, depth_topic, self._on_depth, qos)
        self.pub = self.create_publisher(UInt8MultiArray, occupancy_topic, 10)
        self.create_timer(1.0 / publish_rate, self._on_tick)

    def _precompute_rois(self):
        rois = []
        half = 0.5 * self.square_size * self.roi_shrink
        dz = self.reference_depth
        for rank_idx in range(8):
            for file_idx in range(8):
                xw, yw = square_to_xy(rank_idx, file_idx,
                                      self.center_xy, self.square_size)
                # Straight-down pinhole. Gazebo camera with pitch=π/2 around
                # Y: optical +X points along world -Y, optical +Y along world -X.
                # Verified empirically by spawning a single piece at e5 and
                # checking it lands in the right (rank, file) index.
                u_center = self.cx - self.fy * (yw - self.cam_xyz[1]) / dz
                v_center = self.cy - self.fx * (xw - self.cam_xyz[0]) / dz
                du = self.fy * half / dz
                dv = self.fx * half / dz
                u0 = max(0, int(round(u_center - du)))
                u1 = min(self.image_w, int(round(u_center + du)))
                v0 = max(0, int(round(v_center - dv)))
                v1 = min(self.image_h, int(round(v_center + dv)))
                rois.append((u0, v0, u1, v1))
        return rois

    def _on_depth(self, msg: Image):
        self._latest_depth = msg

    def _on_tick(self):
        msg = self._latest_depth
        if msg is None:
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge conversion failed: {e}')
            return
        depth = np.asarray(depth, dtype=np.float32)
        bad = ~np.isfinite(depth)
        if bad.any():
            depth = np.where(bad, self.reference_depth, depth)

        # A square is "occupied" if a meaningful fraction of its ROI pixels
        # read closer than (board_top - threshold). Robust to small pieces
        # (pawns only cover ~40% of each square from above).
        closer_than = self.reference_depth - self.empty_threshold
        data = bytearray(64)
        for idx, (u0, v0, u1, v1) in enumerate(self.rois):
            if u1 <= u0 or v1 <= v0:
                continue
            patch = depth[v0:v1, u0:u1]
            if patch.size == 0:
                continue
            frac = float(np.mean(patch < closer_than))
            if frac > self.min_occupied_fraction:
                data[idx] = 1

        out = UInt8MultiArray()
        out.layout.dim = [
            MultiArrayDimension(label='rank', size=8, stride=64),
            MultiArrayDimension(label='file', size=8, stride=8),
        ]
        out.layout.data_offset = 0
        out.data = list(data)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = BoardVerifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
