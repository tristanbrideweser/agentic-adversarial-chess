"""
nodes_only.launch.py
====================
Launches the full ROS node stack WITHOUT starting Gazebo.

Use this when:
  - Gazebo is already running from a previous launch
  - You want to restart just the software stack after a crash
  - You're testing individual subsystems

Usage
-----
  ros2 launch chess_robot_bringup nodes_only.launch.py
  ros2 launch chess_robot_bringup nodes_only.launch.py launch_coordinator:=false
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    bringup_pkg  = FindPackageShare("chess_robot_bringup")
    board_loc_pkg = FindPackageShare("board_localization")
    arm_ctrl_pkg  = FindPackageShare("arm_controller")
    grasp_pkg     = FindPackageShare("grasp_planner")

    args = [
        DeclareLaunchArgument("use_sim_time",        default_value="true"),
        DeclareLaunchArgument("launch_coordinator",  default_value="true"),
        DeclareLaunchArgument("white_depth",         default_value="15"),
        DeclareLaunchArgument("black_depth",         default_value="15"),
        DeclareLaunchArgument("black_elo",           default_value=""),
        DeclareLaunchArgument("grasp_mode",          default_value="lookup"),
        DeclareLaunchArgument("auto_start",          default_value="true"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")

    board_loc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([board_loc_pkg, "launch", "board_localization.launch.py"])
        ]),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    board_state = Node(
        package="chess_engine", executable="board_state_node",
        name="board_state_node", output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    stockfish_white = Node(
        package="chess_engine", executable="stockfish_node",
        name="stockfish_white", output="screen",
        parameters=[
            PathJoinSubstitution([bringup_pkg, "config", "stockfish_white.yaml"]),
            {"use_sim_time": use_sim_time, "color": "white",
             "depth": LaunchConfiguration("white_depth")},
        ],
        remappings=[("~/get_move", "/stockfish/white/get_move")],
    )

    stockfish_black = Node(
        package="chess_engine", executable="stockfish_node",
        name="stockfish_black", output="screen",
        parameters=[
            PathJoinSubstitution([bringup_pkg, "config", "stockfish_black.yaml"]),
            {"use_sim_time": use_sim_time, "color": "black",
             "depth": LaunchConfiguration("black_depth"),
             "elo":   LaunchConfiguration("black_elo")},
        ],
        remappings=[("~/get_move", "/stockfish/black/get_move")],
    )

    move_translator = Node(
        package="move_translator", executable="move_translator_node",
        name="move_translator", output="screen",
        parameters=[
            PathJoinSubstitution([bringup_pkg, "config", "move_translator.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
    )

    grasp_planner = Node(
        package="grasp_planner", executable="grasp_planner_node",
        name="grasp_planner", output="screen",
        parameters=[
            PathJoinSubstitution([grasp_pkg, "config", "gpd_params.yaml"]),
            {"use_sim_time": use_sim_time,
             "mode": LaunchConfiguration("grasp_mode")},
        ],
    )

    arm_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([arm_ctrl_pkg, "launch", "arm_controller.launch.py"])
        ]),
        launch_arguments={"use_sim_time": use_sim_time, "arm": "both"}.items(),
    )

    coordinator = Node(
        package="game_coordinator", executable="coordinator_node",
        name="game_coordinator", output="screen",
        condition=IfCondition(LaunchConfiguration("launch_coordinator")),
        parameters=[
            PathJoinSubstitution([bringup_pkg, "config", "coordinator.yaml"]),
            {"use_sim_time": use_sim_time,
             "auto_start": LaunchConfiguration("auto_start")},
        ],
    )

    return LaunchDescription(args + [
        board_loc,
        board_state,
        stockfish_white,
        stockfish_black,
        move_translator,
        grasp_planner,
        arm_controllers,
        TimerAction(period=2.0, actions=[coordinator]),
    ])