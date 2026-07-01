"""一键启动 SPEC 02 全栈：Gazebo + EKF + Cartographer + Nav2 + test_tools。

启动模式:
  mode=mapping    — 建图模式 (默认): Cartographer 建图 + explore_lite
  mode=localize   — 纯定位模式: Cartographer 加载 pbstream 定位 + Nav2
"""
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_desc_dir = get_package_share_directory('robot_description')
    loc_dir = get_package_share_directory('adam_localization')
    slam_dir = get_package_share_directory('adam_slam')
    nav_dir = get_package_share_directory('adam_navigation')
    tools_dir = get_package_share_directory('adam_test_tools')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping',
                              description='mapping | localize'),
        DeclareLaunchArgument('load_state_filename', default_value='',
                              description='pbstream path (localize mode only)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # L1: Gazebo + robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                robot_desc_dir + '/launch/laser_2wd.launch.py'
            ),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        # L2: EKF 里程计
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                loc_dir + '/launch/localization.launch.py'
            ),
        ),

        # L3: Cartographer (建图模式)
        GroupAction(
            condition=IfCondition(LaunchConfiguration('mode') == 'mapping'),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        slam_dir + '/launch/cartographer_2d.launch.py'
                    ),
                ),
            ],
        ),

        # L3: Cartographer (纯定位模式)
        GroupAction(
            condition=IfCondition(LaunchConfiguration('mode') == 'localize'),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        slam_dir + '/launch/cartographer_localization.launch.py'
                    ),
                    launch_arguments={
                        'load_state_filename': LaunchConfiguration('load_state_filename'),
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        nav_dir + '/launch/nav2_bringup.launch.py'
                    ),
                ),
            ],
        ),

        # Test tools
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                tools_dir + '/launch/test_tools.launch.py'
            ),
        ),
    ])
