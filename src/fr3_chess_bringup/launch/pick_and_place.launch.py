import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Package directories
    description_pkg_share = FindPackageShare('fr3_chess_description')
    sim_pkg_share = FindPackageShare('fr3_chess_sim')
    bringup_pkg_share = FindPackageShare('fr3_chess_bringup')
    ros_gz_sim_pkg_share = FindPackageShare('ros_gz_sim')

    # Path to the xacro file
    xacro_file = PathJoinSubstitution([description_pkg_share, 'urdf', 'fr3_chess.urdf.xacro'])

    # Robot description
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Gazebo World
    world_file = PathJoinSubstitution([sim_pkg_share, 'worlds', 'chess_world.sdf'])

    # Environment variable for Gazebo OGRE crash on some systems
    force_sw = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1")

    # Launch Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_pkg_share, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r -v 4 ', world_file]}.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'fr3_chess',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.78'
        ],
        output='screen'
    )

    # Bridge for clock
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Spawners for controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fr3_arm_controller'],
    )

    hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fr3_hand_controller'],
    )

    return LaunchDescription([
        force_sw,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        hand_controller_spawner
    ])
