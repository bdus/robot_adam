from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = get_package_share_directory('adam_navigation')

    return LaunchDescription([
        # Nav2 标准 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('nav2_bringup'),
                '/launch', '/navigation_launch.py'
            ]),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': config_dir + '/config/nav2_2d_config.yaml',
            }.items()
        ),
    ])
