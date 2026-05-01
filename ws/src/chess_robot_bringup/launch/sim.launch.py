from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution([
        FindPackageShare('chess_robot_description'),
        'worlds',
        'chess_table.sdf',
    ])

    # Workaround for WSLg Ignition/Fortress OGRE crash.
    force_sw = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')

    # Let Gazebo find chess_robot_description models (e.g. for `ros_gz_sim create`).
    models_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        PathJoinSubstitution([FindPackageShare('chess_robot_description'), 'models']),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r -v 4 ', world]}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/overhead_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/overhead_camera/depth@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen',
    )

    board_verifier = Node(
        package='perception',
        executable='board_verifier',
        name='board_verifier',
        output='screen',
    )

    return LaunchDescription([force_sw, models_path, gz_sim, bridge, board_verifier])
