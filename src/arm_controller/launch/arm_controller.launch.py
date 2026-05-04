from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
