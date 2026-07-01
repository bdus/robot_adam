import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("adam_localization")

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local_node',
            parameters=[{'use_sim_time': True},
                        os.path.join(pkg_dir, 'config', 'ekf_local.yaml')],
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global_node',
            parameters=[{'use_sim_time': True},
                        os.path.join(pkg_dir, 'config', 'ekf_global.yaml')],
            output='screen',
        ),
    ])
