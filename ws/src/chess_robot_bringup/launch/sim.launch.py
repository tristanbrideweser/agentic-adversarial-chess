"""
sim.launch.py
=============
Top-level launch file for the chess robot system in Gazebo simulation.

Usage
-----
  ros2 launch chess_robot_bringup sim.launch.py
  ros2 launch chess_robot_bringup sim.launch.py launch_coordinator:=false
  ros2 launch chess_robot_bringup sim.launch.py gz_headless:=true
  ros2 launch chess_robot_bringup sim.launch.py black_elo:=1500
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
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

    bringup_pkg     = FindPackageShare("chess_robot_bringup")
    description_pkg = FindPackageShare("chess_robot_description")
    board_loc_pkg   = FindPackageShare("board_localization")
    arm_ctrl_pkg    = FindPackageShare("arm_controller")
    grasp_pkg       = FindPackageShare("grasp_planner")

    args = [
        DeclareLaunchArgument("use_sim_time",        default_value="true"),
        DeclareLaunchArgument("world_file",          default_value="chess_table.sdf"),
        DeclareLaunchArgument("auto_start",          default_value="true"),
        DeclareLaunchArgument("launch_coordinator",  default_value="true"),
        DeclareLaunchArgument("white_depth",         default_value="15"),
        DeclareLaunchArgument("black_depth",         default_value="15"),
        DeclareLaunchArgument("black_elo",           default_value=""),
        DeclareLaunchArgument("grasp_mode",          default_value="lookup"),
        DeclareLaunchArgument("gz_headless",         default_value="false",
            description="Run Gazebo without GUI"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")

    # 1. Gazebo Harmonic
    gazebo = ExecuteProcess(
        cmd=[
            "bash", "-c",
            [
                'if [ "', LaunchConfiguration("gz_headless"), '" = "true" ]; then '
                'gz sim -r -s ',
                PathJoinSubstitution([description_pkg, "worlds", LaunchConfiguration("world_file")]),
                '; else gz sim -r ',
                PathJoinSubstitution([description_pkg, "worlds", LaunchConfiguration("world_file")]),
                '; fi',
            ],
        ],
        output="screen",
    )

    # 2. Robot description + spawn (3 s after Gazebo)
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([description_pkg, "launch", "spawn_robot.launch.py"])
                ]),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            )
        ],
    )

    # 2b. Spawn chess pieces (5 s — after robot is in Gazebo)
    spawn_pieces = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "python3",
                    PathJoinSubstitution([description_pkg, "scripts", "spawn_pieces.py"]),
                    "--config",
                    PathJoinSubstitution([description_pkg, "config", "board_params.yaml"]),
                ],
                output="screen",
            )
        ],
    )

    # 3. board_localization (6 s)
    board_loc = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([board_loc_pkg, "launch", "board_localization.launch.py"])
                ]),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            )
        ],
    )

    # 4. chess_engine (7 s)
    board_state = TimerAction(period=7.0, actions=[
        Node(package="chess_engine", executable="board_state_node",
             name="board_state_node", output="screen",
             parameters=[{"use_sim_time": use_sim_time}]),
    ])

    stockfish_white = TimerAction(period=7.5, actions=[
        Node(package="chess_engine", executable="stockfish_node",
             name="stockfish_white", output="screen",
             parameters=[
                 PathJoinSubstitution([bringup_pkg, "config", "stockfish_white.yaml"]),
                 {"use_sim_time": use_sim_time, "color": "white",
                  "depth": LaunchConfiguration("white_depth")},
             ],
             remappings=[("~/get_move", "/stockfish/white/get_move")]),
    ])

    stockfish_black = TimerAction(period=7.5, actions=[
        Node(package="chess_engine", executable="stockfish_node",
             name="stockfish_black", output="screen",
             parameters=[
                 PathJoinSubstitution([bringup_pkg, "config", "stockfish_black.yaml"]),
                 {"use_sim_time": use_sim_time, "color": "black",
                  "depth": LaunchConfiguration("black_depth"),
                  "elo": LaunchConfiguration("black_elo")},
             ],
             remappings=[("~/get_move", "/stockfish/black/get_move")]),
    ])

    # 5. move_translator (8 s)
    move_translator = TimerAction(period=8.0, actions=[
        Node(package="move_translator", executable="move_translator_node",
             name="move_translator", output="screen",
             parameters=[
                 PathJoinSubstitution([bringup_pkg, "config", "move_translator.yaml"]),
                 {"use_sim_time": use_sim_time},
             ]),
    ])

    # 6. grasp_planner (8 s)
    grasp_planner = TimerAction(period=8.0, actions=[
        Node(package="grasp_planner", executable="grasp_planner_node",
             name="grasp_planner", output="screen",
             parameters=[
                 PathJoinSubstitution([grasp_pkg, "config", "gpd_params.yaml"]),
                 {"use_sim_time": use_sim_time,
                  "mode": LaunchConfiguration("grasp_mode")},
             ]),
    ])

    # 7. arm_controller (9 s)
    arm_controllers = TimerAction(period=9.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([arm_ctrl_pkg, "launch", "arm_controller.launch.py"])
            ]),
            launch_arguments={"use_sim_time": use_sim_time, "arm": "both"}.items(),
        ),
    ])

    # 7b. MoveIt move_group (10 s)
    move_group_launch = TimerAction(period=10.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([description_pkg, "launch", "move_group.launch.py"])
            ]),
        ),
    ])

    # 7c. Set initial arm pose (12 s — after controllers active)
    set_pose = TimerAction(period=12.0, actions=[
        ExecuteProcess(
            cmd=["python3",
                 PathJoinSubstitution([bringup_pkg, "scripts", "set_initial_pose.py"])],
            output="screen",
        )
    ])

    # 8. game_coordinator (11 s, optional)
    coordinator = TimerAction(period=11.0, actions=[
        Node(package="game_coordinator", executable="coordinator_node",
             name="game_coordinator", output="screen",
             condition=IfCondition(LaunchConfiguration("launch_coordinator")),
             parameters=[
                 PathJoinSubstitution([bringup_pkg, "config", "coordinator.yaml"]),
                 {"use_sim_time": use_sim_time,
                  "auto_start": LaunchConfiguration("auto_start")},
             ]),
    ])

    return LaunchDescription(args + [
        LogInfo(msg="[chess_robot_bringup] Launching full chess robot simulation..."),
        gazebo,
        spawn_robot,
        spawn_pieces,
        board_loc,
        board_state,
        stockfish_white,
        stockfish_black,
        move_translator,
        grasp_planner,
        arm_controllers,
        move_group_launch,
        set_pose,
        coordinator,
    ])