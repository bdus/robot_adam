import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = get_package_share_directory('adam_slam')

    return LaunchDescription([
        DeclareLaunchArgument('load_state_filename', default_value=''),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '-configuration_directory', config_dir + '/config',
                '-configuration_basename', 'cartographer_localization.lua',
                '-load_state_filename', LaunchConfiguration('load_state_filename'),
            ],
            output='screen',
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
        ),
    ])