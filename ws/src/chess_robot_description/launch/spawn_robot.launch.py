import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_controllers_yaml(src_path: str, out_path: str) -> str:
    """
    Workaround for a Jazzy controller_manager quirk: when loading each
    controller it builds the controller node's args as
        --ros-args --params-file <controller>.params_file -p use_sim_time:=true
    If `<controller>.params_file` is unset (the common case for inline-spec
    controllers) the empty value collapses to `--params-file -p`, which then
    fails arg parsing and every spawner dies. Setting params_file per
    controller — even pointing back at this same YAML — gives controller_manager
    a non-empty path so the construction is well-formed.
    """
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
    src_yaml = os.path.join(description_pkg_share, "config", "fr3_controllers.yaml")

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
        parameters=[{"robot_description": robot_description}],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "dual_fr3",
            "-topic", "robot_description",
        ],
        output="screen",
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--param-file", resolved_yaml],
        )

    jsb = spawner("joint_state_broadcaster")
    white_arm = spawner("white_arm_controller")
    black_arm = spawner("black_arm_controller")
    # white_hand_controller and black_hand_controller are declared in
    # fr3_controllers.yaml as position_controllers/GripperActionController,
    # which moved to the gripper_controllers package in Jazzy and isn't
    # installed in this workspace. Not needed for perception/world bringup.
    # Add them back once gripper_controllers is installed and the YAML type
    # is updated.

    return LaunchDescription([rsp, spawn, jsb, white_arm, black_arm])
