import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue

MOVEIT_PARAMS_FILE = "/tmp/chess_moveit_params.yaml"

def generate_launch_description():
    pkg = get_package_share_directory("chess_robot_description")
    xacro_file      = os.path.join(pkg, "urdf", "dual_fr3.urdf.xacro")
    srdf_file       = os.path.join(pkg, "moveit_config", "config", "dual_fr3.srdf")
    kinematics_file = os.path.join(pkg, "moveit_config", "kinematics.yaml")
    moveit_controllers_file = os.path.join(pkg, "moveit_config", "moveit_controllers.yaml")
    ompl_yaml       = "/opt/ros/jazzy/share/moveit_configs_utils/default_configs/ompl_planning.yaml"
    fr3_controllers_yaml = "/tmp/fr3_controllers_resolved.yaml"  # written by spawn_robot

    with open(srdf_file) as f:
        robot_description_semantic = f.read()

    with open(kinematics_file) as f:
        kinematics_raw = yaml.safe_load(f)

    kinematics_yaml = kinematics_raw.get("/**", kinematics_raw).get("ros__parameters", kinematics_raw)


    with open(moveit_controllers_file) as f:
        moveit_controllers = yaml.safe_load(f)

    with open(ompl_yaml) as f:
        ompl_config = yaml.safe_load(f)

    robot_description = {"robot_description": ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ", xacro_file,
            " controllers_yaml:=", fr3_controllers_yaml,
        ]),
        value_type=str,
    )}

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            {"robot_description_semantic": robot_description_semantic},
            kinematics_yaml,
            moveit_controllers,
            {
                "planning_pipelines": ["ompl"],
                "default_planning_pipeline": "ompl",
                "ompl": ompl_config,
                "use_sim_time": True,
                "sensors": [""],
                "publish_robot_description_semantic": True,
            },
        ],
    )

    return LaunchDescription([move_group])