"""Launch all adam_test_tools nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Scene injection tools
        Node(
            package='adam_test_tools',
            executable='spawn_object.py',
            name='test_spawn_object',
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
            executable='control_light.py',
            name='test_control_light',
            output='screen',
        ),
        Node(
            package='adam_test_tools',
            executable='spawn_actor.py',
            name='test_spawn_actor',
            output='screen',
        ),
        Node(
            package='adam_test_tools',
            executable='apply_force.py',
            name='test_apply_force',
            parameters=[{'robot_name': 'laser_2wd'}],
            output='screen',
        ),
        Node(
            package='adam_test_tools',
            executable='set_friction.py',
            name='test_set_friction',
            output='screen',
        ),
        # Verification nodes
        Node(
            package='adam_test_tools',
            executable='nav2_lifecycle_check.py',
            name='test_nav2_lifecycle_check',
            output='screen',
        ),
        Node(
            package='adam_test_tools',
            executable='tf_monitor_node.py',
            name='test_tf_monitor',
            output='screen',
        ),
        Node(
            package='adam_test_tools',
            executable='odom_health_node.py',
            name='test_odom_health',
            output='screen',
        ),
        Node(
            package='adam_test_tools',
            executable='voxel_decay_meter.py',
            name='test_voxel_decay_meter',
            output='screen',
        ),
        Node(
            package='adam_test_tools',
            executable='pose_jump_detector.py',
            name='test_pose_jump_detector',
            output='screen',
        ),
    ])
