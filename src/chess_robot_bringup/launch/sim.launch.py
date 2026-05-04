from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_pkg     = FindPackageShare("chess_robot_bringup")
    description_pkg = FindPackageShare("chess_robot_description")
    board_loc_pkg   = FindPackageShare("board_localization")
    arm_ctrl_pkg    = FindPackageShare("arm_controller")
    grasp_pkg       = FindPackageShare("grasp_planner")

    args = [
        DeclareLaunchArgument("use_sim_time",       default_value="true"),
        DeclareLaunchArgument("world_file",         default_value="chess_table.sdf"),
        DeclareLaunchArgument("auto_start",         default_value="true"),
        DeclareLaunchArgument("launch_coordinator", default_value="true"),
        DeclareLaunchArgument("white_depth",        default_value="15"),
        DeclareLaunchArgument("black_depth",        default_value="15"),
        DeclareLaunchArgument("grasp_mode",         default_value="lookup"),
        DeclareLaunchArgument("gz_headless",        default_value="false",
                              description="Run Gazebo without GUI"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")

    # -------------------------------------------------------------------------
    # 0. Environment
    # -------------------------------------------------------------------------
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=PathJoinSubstitution([description_pkg, "models"]),
    )
    set_franka_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value="/opt/franka_ws/install/franka_description/share:"
            + "/root/chess_ws/install/chess_robot_description/share/chess_robot_description/models",
    )

    # -------------------------------------------------------------------------
    # 1. Gazebo Harmonic
    # -------------------------------------------------------------------------
    gazebo_gui = ExecuteProcess(
        cmd=[
            "gz", "sim", "-r",
            PathJoinSubstitution([
                description_pkg, "worlds", LaunchConfiguration("world_file")
            ]),
        ],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("gz_headless")),
    )

    gazebo_headless = ExecuteProcess(
        cmd=[
            "gz", "sim", "-r", "-s",
            PathJoinSubstitution([
                description_pkg, "worlds", LaunchConfiguration("world_file")
            ]),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("gz_headless")),
    )

    # -------------------------------------------------------------------------
    # 2. ROS-Gazebo bridge (2 s)
    # -------------------------------------------------------------------------
    gz_bridge = TimerAction(period=2.0, actions=[
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/overhead_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/overhead_camera/depth@sensor_msgs/msg/Image[gz.msgs.Image",
            ],
            parameters=[{"use_sim_time": True}],
            output="screen",
        )
    ])

    # -------------------------------------------------------------------------
    # 3. Robot description + spawn + ros2_control (6 s)
    # -------------------------------------------------------------------------
    spawn_robot = TimerAction(period=6.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    description_pkg, "launch", "spawn_robot.launch.py"
                ])
            ]),
            launch_arguments=[("use_sim_time", use_sim_time)],
        )
    ])

    # -------------------------------------------------------------------------
    # 4. Chess pieces (10 s)
    # -------------------------------------------------------------------------
    spawn_pieces = TimerAction(period=10.0, actions=[
        ExecuteProcess(
            cmd=[
                "python3",
                PathJoinSubstitution([
                    description_pkg, "scripts", "spawn_pieces.py"
                ]),
                "--config",
                PathJoinSubstitution([
                    description_pkg, "config", "board_params.yaml"
                ]),
            ],
            output="screen",
        )
    ])

    # -------------------------------------------------------------------------
    # 5. Board localization (9 s)
    # -------------------------------------------------------------------------
    board_loc = TimerAction(period=9.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    board_loc_pkg, "launch", "board_localization.launch.py"
                ])
            ]),
            launch_arguments=[("use_sim_time", use_sim_time)],
        )
    ])

    # -------------------------------------------------------------------------
    # 6. Chess engine (10 s / 10.5 s)
    # -------------------------------------------------------------------------
    board_state = TimerAction(period=10.0, actions=[
        Node(
            package="chess_engine",
            executable="board_state_node",
            name="board_state_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])

    stockfish_white = TimerAction(period=10.5, actions=[
        Node(
            package="chess_engine",
            executable="stockfish_node",
            name="stockfish_white",
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    bringup_pkg, "config", "stockfish_white.yaml"
                ]),
                {
                    "use_sim_time": use_sim_time,
                    "color": "white",
                    "depth": LaunchConfiguration("white_depth"),
                },
            ],
            remappings=[("~/get_move", "/stockfish/white/get_move")],
        ),
    ])

    stockfish_black = TimerAction(period=10.5, actions=[
        Node(
            package="chess_engine",
            executable="stockfish_node",
            name="stockfish_black",
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    bringup_pkg, "config", "stockfish_black.yaml"
                ]),
                {
                    "use_sim_time": use_sim_time,
                    "color": "black",
                    "depth": LaunchConfiguration("black_depth"),
                },
                # elo is set in stockfish_black.yaml
            ],
            remappings=[("~/get_move", "/stockfish/black/get_move")],
        ),
    ])

    # -------------------------------------------------------------------------
    # 7. Move translator + grasp planner (11 s)
    # -------------------------------------------------------------------------
    move_translator = TimerAction(period=11.0, actions=[
        Node(
            package="move_translator",
            executable="move_translator_node",
            name="move_translator",
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    bringup_pkg, "config", "move_translator.yaml"
                ]),
                {"use_sim_time": use_sim_time},
            ],
        ),
    ])

    grasp_planner = TimerAction(period=11.0, actions=[
        Node(
            package="grasp_planner",
            executable="grasp_planner_node",
            name="grasp_planner",
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    grasp_pkg, "config", "gpd_params.yaml"
                ]),
                {
                    "use_sim_time": use_sim_time,
                    "mode": LaunchConfiguration("grasp_mode"),
                },
            ],
        ),
    ])

    # -------------------------------------------------------------------------
    # 8. MoveIt move_group (12 s)
    # -------------------------------------------------------------------------
    move_group = TimerAction(period=12.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    description_pkg, "launch", "move_group.launch.py"
                ])
            ]),
        )
    ])

    # -------------------------------------------------------------------------
    # 9. Arm controllers (14 s)
    # -------------------------------------------------------------------------
    arm_controllers = TimerAction(period=14.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    arm_ctrl_pkg, "launch", "arm_controller.launch.py"
                ])
            ]),
            launch_arguments=[
                ("use_sim_time", use_sim_time),
                ("arm", "both"),
            ],
        ),
    ])

    # -------------------------------------------------------------------------
    # 10. Set initial arm pose (16 s)
    # -------------------------------------------------------------------------
    set_pose = TimerAction(period=16.0, actions=[
        ExecuteProcess(
            cmd=[
                "python3",
                PathJoinSubstitution([
                    bringup_pkg, "scripts", "set_initial_pose.py"
                ]),
            ],
            output="screen",
        )
    ])

    # -------------------------------------------------------------------------
    # 11. Game coordinator (18 s)
    # -------------------------------------------------------------------------
    coordinator = TimerAction(period=18.0, actions=[
        Node(
            package="game_coordinator",
            executable="coordinator_node",
            name="game_coordinator",
            output="screen",
            condition=IfCondition(LaunchConfiguration("launch_coordinator")),
            parameters=[
                PathJoinSubstitution([
                    bringup_pkg, "config", "coordinator.yaml"
                ]),
                {
                    "use_sim_time": use_sim_time,
                    "auto_start": LaunchConfiguration("auto_start"),
                },
            ],
        ),
    ])

    # -------------------------------------------------------------------------
    # Assembly
    # -------------------------------------------------------------------------
    return LaunchDescription(args + [
        LogInfo(msg="[chess_robot_bringup] Launching chess robot simulation..."),
        set_gz_resource_path,
        gazebo_gui,
        gazebo_headless,
        gz_bridge,
        spawn_robot,
        spawn_pieces,
        board_loc,
        board_state,
        stockfish_white,
        stockfish_black,
        move_translator,
        grasp_planner,
        move_group,
        arm_controllers,
        set_pose,
        coordinator,
    ])