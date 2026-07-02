import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    use_rviz = launch.substitutions.LaunchConfiguration('rviz', default='false')

    robot_name = "laser_2wd"
    urdf_path = get_package_share_directory('robot_description')
    default_model_path = urdf_path + '/urdf/robots/laser_2wd.urdf.xacro'
    default_world_path = urdf_path + '/worlds/bigH.world'
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')

    action_declare_arg_model = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='URDF 的绝对路径')

    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}, {'use_sim_time': use_sim_time}]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py'
        ]),
        launch_arguments=[
            ('world', default_world_path),
            ('verbose', 'true'),
            ('use_sim_time', use_sim_time)
        ]
    )

    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name, ])

    delayed_spawn = launch.actions.TimerAction(
        period=2.0,
        actions=[spawn_entity_node]
    )

    rviz_path = os.path.join(urdf_path, 'config', 'rviz', 'laser_2wd.rviz')
    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_path],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('rviz', default_value='false',
                                             description='Open RViz'),
        action_declare_arg_model,
        robot_state_publisher_node,
        joint_state_publisher_node,
        launch_gazebo,
        delayed_spawn,
        rviz
    ])
