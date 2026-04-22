import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 配置参数
    use_rviz = LaunchConfiguration('rviz', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_model = LaunchConfiguration('model', default='ackermann_mid360')

    # 获取包路径
    urdf_path = get_package_share_directory('adam_description')

    # 根据选择的模型设置 URDF 路径
    model_paths = {
        'ackermann_mid360': urdf_path + '/urdf/ackermann_mid360.urdf.xacro',
        'ackermann_laser': urdf_path + '/urdf/ackermann_laser.urdf.xacro'
    }

    # 声明参数
    declare_model_cmd = launch.actions.DeclareLaunchArgument(
        'model',
        default_value='ackermann_mid360',
        description='选择机器人模型：ackermann_mid360 或 ackermann_laser')

    declare_rviz_cmd = launch.actions.DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='是否启动 RViz')

    # 生成机器人描述
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str)

    # 机器人状态发布器
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 启动 Gazebo
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py'
        ]),
        launch_arguments=[('use_sim_time', use_sim_time)]
    )

    # 生成机器人实体
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_model],
    )

    delayed_spawn = launch.actions.TimerAction(
        period=2.0,
        actions=[spawn_entity_node]
    )

    # 加载控制器
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_ackermann_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'ackermann_steering_controller'],
        output='screen'
    )

    load_controller_event = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[load_joint_state_controller]
        )
    )

    load_controller_event2 = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_ackermann_controller]
        )
    )

    # RViz 配置
    rviz_config = launch.conditions.IfCondition(use_rviz)
    rviz_mid360_path = os.path.join(urdf_path, 'config', 'rviz', 'ackermann_mid360.rviz')
    rviz_laser_path = os.path.join(urdf_path, 'config', 'rviz', 'ackermann_laser.rviz')

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_mid360_path],
        condition=rviz_config
    )

    return launch.LaunchDescription([
        declare_model_cmd,
        declare_rviz_cmd,
        robot_state_publisher_node,
        joint_state_publisher_node,
        launch_gazebo,
        delayed_spawn,
        load_controller_event,
        load_controller_event2,
        rviz_node
    ])
