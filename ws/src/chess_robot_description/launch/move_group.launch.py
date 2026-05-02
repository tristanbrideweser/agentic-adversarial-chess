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
    xacro_file   = os.path.join(pkg, "urdf", "dual_fr3.urdf.xacro")
    srdf_file    = os.path.join(pkg, "moveit_config", "config", "dual_fr3.srdf")
    kinematics   = os.path.join(pkg, "moveit_config", "kinematics.yaml")
    controllers  = os.path.join(pkg, "moveit_config", "moveit_controllers.yaml")
    ompl_yaml    = "/opt/ros/jazzy/share/moveit_configs_utils/default_configs/ompl_planning.yaml"
    controllers_yaml = "/tmp/fr3_controllers_resolved.yaml"

    with open(srdf_file) as f:
        robot_description_semantic = f.read()

    with open(kinematics) as f:
        kinematics_yaml = yaml.safe_load(f)

    with open(controllers) as f:
        controllers_yaml = yaml.safe_load(f)

    with open(ompl_yaml) as f:
        ompl_config = yaml.safe_load(f)

    # Write persistent params file for MoveItPy to read
    moveit_params = {
        "move_group": {
            "ros__parameters": {
                "robot_description_semantic": robot_description_semantic,
                "robot_description_kinematics": kinematics_yaml,
                "planning_pipelines": ["ompl"],
                "default_planning_pipeline": "ompl",
                "ompl": ompl_config,
                "sensors": [""],
                "use_sim_time": True,
            }
        }
    }
    with open(MOVEIT_PARAMS_FILE, "w") as f:
        yaml.dump(moveit_params, f)

    robot_description = {"robot_description": ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file,
                 " controllers_yaml:=", controllers_yaml]),
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
