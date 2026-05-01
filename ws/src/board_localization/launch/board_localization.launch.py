"""
board_localization.launch.py
============================
Launches the two board_localization nodes together:
  1. board_tf_broadcaster   — publishes 97 static TF frames
  2. square_lookup_service  — serves /board_localization/get_square_pose etc.

Usage
-----
  # Nominal Gazebo pose (default):
  ros2 launch board_localization board_localization.launch.py

  # Physical deployment with a shifted board:
  ros2 launch board_localization board_localization.launch.py \\
      board_origin_x:=0.02 board_origin_y:=-0.01 board_yaw:=0.05

  # Load calibration output from a yaml file:
  ros2 launch board_localization board_localization.launch.py \\
      params_file:=/path/to/calibrated_board_params.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    pkg = FindPackageShare("board_localization")
    default_params = PathJoinSubstitution([pkg, "config", "board_params.yaml"])

    # ------------------------------------------------------------------
    # Declare overrideable arguments
    # ------------------------------------------------------------------
    args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to board_params.yaml (use calibration output here)",
        ),
        DeclareLaunchArgument(
            "board_origin_x", default_value="0.0",
            description="World X of board centre (m)",
        ),
        DeclareLaunchArgument(
            "board_origin_y", default_value="0.0",
            description="World Y of board centre (m)",
        ),
        DeclareLaunchArgument(
            "board_origin_z", default_value="0.762",
            description="World Z of board surface (m)",
        ),
        DeclareLaunchArgument(
            "board_yaw", default_value="0.0",
            description="Board rotation around world Z (radians)",
        ),
        DeclareLaunchArgument(
            "use_tf_lookup", default_value="false",
            description="Use TF buffer for pose lookups (true) or analytic math (false)",
        ),
        DeclareLaunchArgument(
            "publish_rate_hz", default_value="1.0",
            description="Rate at which static TF frames are re-broadcast (Hz)",
        ),
    ]

    # ------------------------------------------------------------------
    # board_tf_broadcaster
    # ------------------------------------------------------------------
    tf_broadcaster = Node(
        package="board_localization",
        executable="board_tf_broadcaster",
        name="board_tf_broadcaster",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "board_origin_x": LaunchConfiguration("board_origin_x"),
                "board_origin_y": LaunchConfiguration("board_origin_y"),
                "board_origin_z": LaunchConfiguration("board_origin_z"),
                "board_yaw":      LaunchConfiguration("board_yaw"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
            },
        ],
    )

    # ------------------------------------------------------------------
    # square_lookup_service
    # ------------------------------------------------------------------
    lookup_service = Node(
        package="board_localization",
        executable="square_lookup_service",
        name="square_lookup_service",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "use_tf_lookup": LaunchConfiguration("use_tf_lookup"),
            },
        ],
    )

    return LaunchDescription(args + [tf_broadcaster, lookup_service])