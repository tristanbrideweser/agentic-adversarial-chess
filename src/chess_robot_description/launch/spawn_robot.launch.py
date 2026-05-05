import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction


def _resolve_controllers_yaml(src_path: str, out_path: str) -> str:
    with open(src_path) as f:
        data = yaml.safe_load(f)
    cm_params = data["controller_manager"]["ros__parameters"]
    for name, spec in cm_params.items():
        if isinstance(spec, dict) and "type" in spec:
            spec["params_file"] = src_path
    with open(out_path, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
    return out_path


def generate_launch_description():
    description_pkg_share = get_package_share_directory("chess_robot_description")
    xacro_file = os.path.join(description_pkg_share, "urdf", "dual_fr3.urdf.xacro")
    src_yaml   = os.path.join(description_pkg_share, "config", "fr3_controllers.yaml")

    resolved_yaml = _resolve_controllers_yaml(src_yaml, "/tmp/fr3_controllers_resolved.yaml")

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            xacro_file, " ",
            f"controllers_yaml:={resolved_yaml}",
        ]),
        value_type=str,
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "dual_fr3", "-topic", "robot_description"],
        output="screen",
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--param-file", resolved_yaml],
        )

    jsb       = spawner("joint_state_broadcaster")
    white_arm = spawner("white_arm_controller")
    black_arm = spawner("black_arm_controller")

    spawn_with_delay    = TimerAction(period=3.0,  actions=[spawn])
    controller_spawners = TimerAction(period=10.0, actions=[jsb, white_arm, black_arm])

    return LaunchDescription([rsp, spawn_with_delay, controller_spawners])