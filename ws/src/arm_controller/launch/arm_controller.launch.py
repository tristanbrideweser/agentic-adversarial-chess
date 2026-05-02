"""
arm_controller.launch.py
========================
Launches both arm controller nodes (white and black) simultaneously.

Usage
-----
  # Both arms, defaults:
  ros2 launch arm_controller arm_controller.launch.py

  # Override velocity scaling at launch:
  ros2 launch arm_controller arm_controller.launch.py vel_transit:=0.5

  # Single arm only (useful during development):
  ros2 launch arm_controller arm_controller.launch.py arm:=white
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("arm_controller")

    args = [
        DeclareLaunchArgument(
            "arm",
            default_value="both",
            description="Which arm to launch: 'white', 'black', or 'both'",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulated time from Gazebo clock",
        ),
    ]

    launch_white = PythonExpression(
        ["'", LaunchConfiguration("arm"), "' in ('white', 'both')"]
    )
    launch_black = PythonExpression(
        ["'", LaunchConfiguration("arm"), "' in ('black', 'both')"]
    )

    white_node = Node(
        package="arm_controller",
        executable="pick_place_server_white",
        additional_env={"PYTHONPATH": "/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages:/root/chess_ws/install/perception/lib/python3.12/site-packages:/root/chess_ws/install/move_translator/lib/python3.12/site-packages:/root/chess_ws/install/grasp_planner/lib/python3.12/site-packages:/root/chess_ws/install/game_coordinator/lib/python3.12/site-packages:/root/chess_ws/install/chess_engine/lib/python3.12/site-packages:/root/chess_ws/install/board_localization/lib/python3.12/site-packages:/root/chess_ws/install/arm_controller/lib/python3.12/site-packages:/root/chess_ws/install/chess_interfaces/lib/python3.12/site-packages:/opt/franka_ws/install/franka_msgs/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages"},
        name="white_panda_pick_place_server",
        namespace="white_panda",
        output="screen",
        parameters=[
            PathJoinSubstitution([pkg, "config", "arm_white.yaml"]),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        condition=IfCondition(launch_white),
    )

    black_node = Node(
        package="arm_controller",
        executable="pick_place_server_black",
        additional_env={"PYTHONPATH": "/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages:/root/chess_ws/install/perception/lib/python3.12/site-packages:/root/chess_ws/install/move_translator/lib/python3.12/site-packages:/root/chess_ws/install/grasp_planner/lib/python3.12/site-packages:/root/chess_ws/install/game_coordinator/lib/python3.12/site-packages:/root/chess_ws/install/chess_engine/lib/python3.12/site-packages:/root/chess_ws/install/board_localization/lib/python3.12/site-packages:/root/chess_ws/install/arm_controller/lib/python3.12/site-packages:/root/chess_ws/install/chess_interfaces/lib/python3.12/site-packages:/opt/franka_ws/install/franka_msgs/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages"},
        name="black_panda_pick_place_server",
        namespace="black_panda",
        output="screen",
        parameters=[
            PathJoinSubstitution([pkg, "config", "arm_black.yaml"]),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        condition=IfCondition(launch_black),
    )

    return LaunchDescription(args + [white_node, black_node])