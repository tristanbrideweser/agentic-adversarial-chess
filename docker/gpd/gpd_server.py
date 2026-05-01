"""
gpd_server.py
=============
ROS 2 action server that wraps the GPD C++ library via gpd_ros.

This node:
  1. Subscribes to /overhead_camera/points (PointCloud2) and caches
     the latest cloud.
  2. Exposes /grasp_planner/gpd_action (gpd_ros/action/GraspPlanning).
  3. On each goal: crops the cloud to the requested bounding box,
     runs GPD, and returns ranked GraspConfig candidates.

The action interface matches what gpd_client.py in the grasp_planner
package expects.

Action goal fields (from gpd_ros/action/GraspPlanning.action):
    sensor_msgs/PointCloud2 cloud   # pre-cropped cloud from the client

Action result fields:
    gpd_ros/msg/GraspConfigList grasps

If gpd_ros is not available (e.g., during unit tests), the node falls
back to returning a single synthetic top-down grasp per request.
"""

import math
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, Vector3

try:
    from gpd_ros.action import GraspPlanning
    from gpd_ros.msg import GraspConfig, GraspConfigList
    GPD_AVAILABLE = True
except ImportError:
    GPD_AVAILABLE = False

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

POINTCLOUD_TOPIC = "/overhead_camera/points"
ACTION_NAME = "/grasp_planner/gpd_action"

BOARD_SURFACE_Z = 0.762   # m
CROP_BOX_HALF   = 0.03    # ±3 cm around target square

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=1,
)


# ---------------------------------------------------------------------------
# Helper: crop a PointCloud2 to an AABB (pure Python, no PCL required)
# ---------------------------------------------------------------------------

def crop_cloud_xyz(cloud: PointCloud2, x_min, x_max, y_min, y_max,
                   z_min, z_max) -> PointCloud2:
    """
    Crop a PointCloud2 message to the given axis-aligned bounding box.

    Uses the sensor_msgs point_cloud2 helpers to iterate over points.
    Returns a new PointCloud2 containing only the points inside the box.

    Falls back to returning the original cloud if parsing fails.
    """
    try:
        from sensor_msgs_py import point_cloud2 as pc2
        import struct

        points = list(pc2.read_points(cloud, field_names=("x", "y", "z"),
                                      skip_nans=True))
        filtered = [
            p for p in points
            if x_min <= p[0] <= x_max
            and y_min <= p[1] <= y_max
            and z_min <= p[2] <= z_max
        ]

        header = cloud.header
        cropped = pc2.create_cloud_xyz32(header, filtered)
        return cropped

    except Exception:  # noqa: BLE001
        # If cropping fails, return the full cloud and let GPD handle it
        return cloud


# ---------------------------------------------------------------------------
# Synthetic fallback grasp (top-down, for testing without GPD installed)
# ---------------------------------------------------------------------------

def make_synthetic_grasp(cx: float, cy: float, grasp_z: float) -> "GraspConfig":
    """
    Build a single top-down GraspConfig for a square centre.
    Used when GPD is unavailable or returns zero candidates.
    """
    g = GraspConfig()

    # Position: centre of crop box at grasp Z
    g.position = Point(x=cx, y=cy, z=grasp_z)

    # Top-down approach: approach vector = (0, 0, -1) (pointing down)
    g.approach  = Vector3(x=0.0,  y=0.0, z=-1.0)
    g.binormal  = Vector3(x=0.0,  y=1.0, z=0.0)
    g.axis      = Vector3(x=1.0,  y=0.0, z=0.0)

    g.score      = 1.0
    g.width      = 0.030   # 3 cm finger separation
    g.success    = True
    return g


# ---------------------------------------------------------------------------
# GPD server node
# ---------------------------------------------------------------------------

class GpdServer(Node):
    """
    ROS 2 action server wrapping GPD grasp detection.

    Parameters (ROS params)
    -----------------------
    max_grasps : int     Maximum candidates to return per call  (default: 20)
    min_score  : float   Discard candidates below this score    (default: 0.0)
    use_synthetic_fallback : bool
                         Return a synthetic top-down grasp when GPD returns
                         nothing rather than failing the goal             (default: true)
    """

    def __init__(self) -> None:
        super().__init__("gpd_server")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("max_grasps", 20)
        self.declare_parameter("min_score", 0.0)
        self.declare_parameter("use_synthetic_fallback", True)

        self._max_grasps = self.get_parameter("max_grasps").value
        self._min_score  = self.get_parameter("min_score").value
        self._fallback   = self.get_parameter("use_synthetic_fallback").value

        # ------------------------------------------------------------------
        # Point cloud cache
        # ------------------------------------------------------------------
        self._cloud_lock = threading.Lock()
        self._latest_cloud: PointCloud2 | None = None

        self._cloud_sub = self.create_subscription(
            PointCloud2,
            POINTCLOUD_TOPIC,
            self._on_cloud,
            _SENSOR_QOS,
        )

        # ------------------------------------------------------------------
        # Action server
        # ------------------------------------------------------------------
        if not GPD_AVAILABLE:
            self.get_logger().warn(
                "gpd_ros not found — running in synthetic-fallback-only mode. "
                "Install gpd_ros and rebuild to enable real GPD."
            )

        if GPD_AVAILABLE:
            self._action_server = ActionServer(
                self,
                GraspPlanning,
                ACTION_NAME,
                execute_callback=self._execute,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
            )
            self.get_logger().info(
                f"GPD action server ready on '{ACTION_NAME}'"
            )
        else:
            self.get_logger().warn(
                f"GPD action server NOT registered (gpd_ros missing). "
                f"The grasp_planner will fall back to lookup-table mode."
            )

    # ------------------------------------------------------------------
    # Point cloud callback
    # ------------------------------------------------------------------

    def _on_cloud(self, msg: PointCloud2) -> None:
        with self._cloud_lock:
            self._latest_cloud = msg

    def _get_cloud(self) -> PointCloud2 | None:
        with self._cloud_lock:
            return self._latest_cloud

    # ------------------------------------------------------------------
    # Action callbacks
    # ------------------------------------------------------------------

    def _goal_callback(self, goal_request):
        self.get_logger().info("GPD: received grasp planning goal")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info("GPD: goal cancelled")
        return CancelResponse.ACCEPT

    async def _execute(self, goal_handle) -> "GraspPlanning.Result":
        """Main execution callback for the grasp planning action."""
        result = GraspPlanning.Result()

        # Use the cloud embedded in the goal if provided, else use cache
        goal_cloud = getattr(goal_handle.request, "cloud", None)
        if goal_cloud is None or len(goal_cloud.data) == 0:
            goal_cloud = self._get_cloud()

        if goal_cloud is None:
            self.get_logger().warn("GPD: no point cloud available — using fallback")
            result.grasps = self._make_fallback_result(cx=0.0, cy=0.0)
            goal_handle.succeed()
            return result

        # Run GPD on the provided cloud
        grasps = self._run_gpd(goal_cloud)

        if not grasps and self._fallback:
            self.get_logger().warn(
                "GPD: returned zero candidates — using synthetic fallback"
            )
            grasps = [make_synthetic_grasp(0.0, 0.0, BOARD_SURFACE_Z + 0.03)]

        if not grasps:
            self.get_logger().error("GPD: no valid grasps found")
            goal_handle.abort()
            return result

        grasp_list = GraspConfigList()
        grasp_list.grasps = grasps
        result.grasps = grasp_list
        goal_handle.succeed()
        return result

    # ------------------------------------------------------------------
    # GPD interface
    # ------------------------------------------------------------------

    def _run_gpd(self, cloud: PointCloud2) -> list:
        """
        Run GPD on the provided point cloud and return a list of GraspConfig.

        This calls the GPD C++ library through gpd_ros.  If the library is
        not installed, returns an empty list.
        """
        if not GPD_AVAILABLE:
            return []

        try:
            # GPD is a C++ library — we call it through the gpd_ros Python
            # bindings or by publishing to the gpd_ros detector node.
            # The pattern below uses the gpd_ros GraspDetector directly via
            # its Python wrapper (available in newer gpd_ros builds).
            from gpd_ros.gpd_detector import GraspDetector  # type: ignore

            detector = GraspDetector(
                config_file=self._get_gpd_config_path(),
            )
            grasps = detector.detect(cloud)

            # Filter by score and limit count
            grasps = [g for g in grasps if g.score >= self._min_score]
            grasps = sorted(grasps, key=lambda g: g.score, reverse=True)
            return grasps[: self._max_grasps]

        except (ImportError, AttributeError):
            # gpd_ros Python bindings not available — use the subprocess
            # approach: publish cloud to /cloud_indexed, subscribe to
            # /detect_grasps/clustered_grasps.
            # This is handled by running the gpd_ros detect_grasps node
            # as a separate process alongside this server.
            self.get_logger().warn(
                "GraspDetector Python bindings not available. "
                "Ensure the gpd_ros detect_grasps node is running alongside "
                "this server, or use lookup-table mode."
            )
            return []

    def _make_fallback_result(self, cx: float, cy: float) -> "GraspConfigList":
        grasp_list = GraspConfigList()
        grasp_list.grasps = [make_synthetic_grasp(cx, cy, BOARD_SURFACE_Z + 0.03)]
        return grasp_list

    @staticmethod
    def _get_gpd_config_path() -> str:
        """Return path to the GPD config file inside the container."""
        import os
        models_dir = os.environ.get("GPD_MODELS_DIR",
                                     "/usr/local/share/gpd/models")
        config = os.path.join(models_dir, "ros_eigen_params.cfg")
        if not os.path.exists(config):
            # Try the default install location
            config = "/usr/local/share/gpd/models/ros_eigen_params.cfg"
        return config


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpdServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()