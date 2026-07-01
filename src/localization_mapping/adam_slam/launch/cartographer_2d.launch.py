import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = get_package_share_directory('adam_slam')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', config_dir + '/config',
            '-configuration_basename', 'cartographer_2d.lua',
        ],
        output='screen',
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
    )

    explore_lite = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('explore_lite'),
            '/launch', '/explore.launch.py'
        ]),
        launch_arguments={
            'params_file': config_dir + '/config/explore_lite.yaml',
        }.items()
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node,
        explore_lite,
    ])