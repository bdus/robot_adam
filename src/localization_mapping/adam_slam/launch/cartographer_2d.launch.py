from launch import LaunchDescription
from launch_ros.actions import Node
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
        remappings=[
            ('odom', '/odometry/filtered'),
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

    explorer_node = Node(
        package='custom_explorer',
        executable='explorer',
        name='explorer_node',
        parameters=[config_dir + '/config/custom_explorer.yaml'],
        output='screen',
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node,
        explorer_node,
    ])