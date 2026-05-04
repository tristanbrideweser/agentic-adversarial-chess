from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("board_localization")
    default_params = PathJoinSubstitution([pkg, "config", "board_params.yaml"])

    args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
        ),
        DeclareLaunchArgument("board_origin_x",   default_value="0.0"),
        DeclareLaunchArgument("board_origin_y",   default_value="0.0"),
        DeclareLaunchArgument("board_origin_z",   default_value="0.762"),
        DeclareLaunchArgument("board_yaw",        default_value="0.0"),
        DeclareLaunchArgument("use_tf_lookup",    default_value="false"),
        DeclareLaunchArgument("publish_rate_hz",  default_value="1.0"),
        DeclareLaunchArgument("use_sim_time",     default_value="true"),
    ]

    tf_broadcaster = Node(
        package="board_localization",
        executable="board_tf_broadcaster",
        name="board_tf_broadcaster",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "board_origin_x":  LaunchConfiguration("board_origin_x"),
                "board_origin_y":  LaunchConfiguration("board_origin_y"),
                "board_origin_z":  LaunchConfiguration("board_origin_z"),
                "board_yaw":       LaunchConfiguration("board_yaw"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                "use_sim_time":    LaunchConfiguration("use_sim_time"),
            },
        ],
    )

    lookup_service = Node(
        package="board_localization",
        executable="square_lookup_service",
        name="square_lookup_service",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "use_tf_lookup": LaunchConfiguration("use_tf_lookup"),
                "use_sim_time":  LaunchConfiguration("use_sim_time"),
            },
        ],
    )

    return LaunchDescription(args + [tf_broadcaster, lookup_service])
