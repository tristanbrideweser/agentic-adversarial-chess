"""
sim.launch.py
=============
Top-level launch file for the chess robot system in Gazebo simulation.

Starts everything in the correct order:
  1. Gazebo Harmonic (chess world + both arms + pieces)
  2. robot_state_publisher
  3. board_localization  (TF broadcaster + square lookup service)
  4. chess_engine        (board_state_node + stockfish white + stockfish black)
  5. move_translator     (FEN + UCI → pick/place task queue)
  6. grasp_planner       (top-down lookup mode by default)
  7. arm_controller      (white and black pick-place action servers)
  8. game_coordinator    (top-level FSM — starts paused, waits for /start_game)

Usage
-----
  # Full simulation, default settings:
  ros2 launch chess_robot_bringup sim.launch.py

  # Start paused (manual step-through):
  ros2 launch chess_robot_bringup sim.launch.py auto_start:=false

  # Asymmetric play (white full strength, black ELO-limited):
  ros2 launch chess_robot_bringup sim.launch.py black_elo:=1500

  # Skip the game coordinator (just bring up the robot stack):
  ros2 launch chess_robot_bringup sim.launch.py launch_coordinator:=false
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    # -----------------------------------------------------------------------
    # Package share paths
    # -----------------------------------------------------------------------
    bringup_pkg    = FindPackageShare("chess_robot_bringup")
    description_pkg = FindPackageShare("chess_robot_description")
    board_loc_pkg  = FindPackageShare("board_localization")
    arm_ctrl_pkg   = FindPackageShare("arm_controller")
    grasp_pkg      = FindPackageShare("grasp_planner")

    # -----------------------------------------------------------------------
    # Launch arguments
    # -----------------------------------------------------------------------
    args = [
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Use Gazebo simulated clock",
        ),
        DeclareLaunchArgument(
            "world_file", default_value="chess_table.sdf",
            description="Gazebo world SDF filename (relative to chess_robot_description/worlds/)",
        ),
        DeclareLaunchArgument(
            "auto_start", default_value="true",
            description="Automatically start the game after launch (false = wait for /start_game)",
        ),
        DeclareLaunchArgument(
            "launch_coordinator", default_value="true",
            description="Launch the game coordinator FSM",
        ),
        DeclareLaunchArgument(
            "white_depth", default_value="15",
            description="Stockfish search depth for White",
        ),
        DeclareLaunchArgument(
            "black_depth", default_value="15",
            description="Stockfish search depth for Black",
        ),
        DeclareLaunchArgument(
            "black_elo", default_value="",
            description="ELO limit for Black (empty = no limit)",
        ),
        DeclareLaunchArgument(
            "grasp_mode", default_value="lookup",
            description="Grasp planner mode: 'lookup' | 'gpd' | 'auto'",
        ),
        DeclareLaunchArgument(
            "log_level", default_value="info",
            description="ROS logging level: debug | info | warn | error",
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")

    # -----------------------------------------------------------------------
    # 1. Gazebo world + robot description
    # -----------------------------------------------------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([description_pkg, "launch", "spawn_robot.launch.py"])
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world_file":   LaunchConfiguration("world_file"),
        }.items(),
    )

    # -----------------------------------------------------------------------
    # 2. board_localization — TF broadcaster + square lookup service
    #    Delay slightly to let Gazebo and robot_state_publisher settle
    # -----------------------------------------------------------------------
    board_loc_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([board_loc_pkg, "launch", "board_localization.launch.py"])
                ]),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                }.items(),
            )
        ],
    )

    # -----------------------------------------------------------------------
    # 3. chess_engine — board_state_node + stockfish nodes
    # -----------------------------------------------------------------------
    board_state_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="chess_engine",
                executable="board_state_node",
                name="board_state_node",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                }],
            ),
        ],
    )

    stockfish_white = TimerAction(
        period=4.5,
        actions=[
            Node(
                package="chess_engine",
                executable="stockfish_node",
                name="stockfish_white",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_pkg, "config", "stockfish_white.yaml"]),
                    {
                        "use_sim_time": use_sim_time,
                        "color": "white",
                        "depth": LaunchConfiguration("white_depth"),
                    },
                ],
                remappings=[
                    ("~/get_move", "/stockfish/white/get_move"),
                ],
            ),
        ],
    )

    stockfish_black = TimerAction(
        period=4.5,
        actions=[
            Node(
                package="chess_engine",
                executable="stockfish_node",
                name="stockfish_black",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_pkg, "config", "stockfish_black.yaml"]),
                    {
                        "use_sim_time": use_sim_time,
                        "color": "black",
                        "depth": LaunchConfiguration("black_depth"),
                        "elo":   LaunchConfiguration("black_elo"),
                    },
                ],
                remappings=[
                    ("~/get_move", "/stockfish/black/get_move"),
                ],
            ),
        ],
    )

    # -----------------------------------------------------------------------
    # 4. move_translator
    # -----------------------------------------------------------------------
    move_translator = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="move_translator",
                executable="move_translator_node",
                name="move_translator",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_pkg, "config", "move_translator.yaml"]),
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ],
    )

    # -----------------------------------------------------------------------
    # 5. grasp_planner
    # -----------------------------------------------------------------------
    grasp_planner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="grasp_planner",
                executable="grasp_planner_node",
                name="grasp_planner",
                output="screen",
                parameters=[
                    PathJoinSubstitution([grasp_pkg, "config", "gpd_params.yaml"]),
                    {
                        "use_sim_time": use_sim_time,
                        "mode": LaunchConfiguration("grasp_mode"),
                    },
                ],
            ),
        ],
    )

    # -----------------------------------------------------------------------
    # 6. arm_controller — white and black
    # -----------------------------------------------------------------------
    arm_controller_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([arm_ctrl_pkg, "launch", "arm_controller.launch.py"])
                ]),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "arm": "both",
                }.items(),
            ),
        ],
    )

    # -----------------------------------------------------------------------
    # 7. game_coordinator
    # -----------------------------------------------------------------------
    game_coordinator = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="game_coordinator",
                executable="coordinator_node",
                name="game_coordinator",
                output="screen",
                condition=IfCondition(LaunchConfiguration("launch_coordinator")),
                parameters=[
                    PathJoinSubstitution([bringup_pkg, "config", "coordinator.yaml"]),
                    {
                        "use_sim_time": use_sim_time,
                        "auto_start": LaunchConfiguration("auto_start"),
                    },
                ],
            ),
        ],
    )

    # -----------------------------------------------------------------------
    # Startup log
    # -----------------------------------------------------------------------
    startup_log = LogInfo(msg="[chess_robot_bringup] Launching full chess robot simulation...")

    return LaunchDescription(
        args + [
            startup_log,
            gazebo_launch,
            board_loc_launch,
            board_state_node,
            stockfish_white,
            stockfish_black,
            move_translator,
            grasp_planner,
            arm_controller_launch,
            game_coordinator,
        ]
    )