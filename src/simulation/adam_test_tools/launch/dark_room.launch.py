"""Launch dark room test environment with test tools."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf_path = get_package_share_directory('robot_description')
    tools_path = get_package_share_directory('adam_test_tools')

    return LaunchDescription([
        # Dark room world in Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('gazebo_ros'),
                '/launch/gazebo.launch.py'
            ]),
            launch_arguments={
                'world': f'{tools_path}/worlds/dark_room_test.world',
                'verbose': 'true',
            }.items()
        ),
        # Spawn robot (laser_2wd)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', '/robot_description',
                '-entity', 'laser_2wd',
                '-x', '0.0', '-y', '0.0', '-z', '0.1',
            ],
        ),
        # Test tools
        Node(
            package='adam_test_tools',
            executable='control_light.py',
            name='test_control_light',
            output='screen',
        ),
        Node(
            package='adam_test_tools',
            executable='teleport_robot.py',
            name='test_teleport_robot',
            parameters=[{'robot_name': 'laser_2wd'}],
            output='screen',
        ),
        Node(
            package='adam_test_tools',
            executable='pose_jump_detector.py',
            name='test_pose_jump_detector',
            output='screen',
        ),
    ])
