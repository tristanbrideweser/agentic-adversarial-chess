from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    world = PathJoinSubstitution([
        FindPackageShare("fr3_chess_sim"),
        "worlds",
        "chess_world.sdf"
    ])

    # Workaround for WSLg Ignition/Fortress OGRE crash:
    force_sw = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1")

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": ["-r -v 4 ", world]
        }.items(),
    )

    return LaunchDescription([force_sw, gz])