"""Bring up the physical or mock STS camera head with MoveIt and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def load_yaml(path):
    with open(path, encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def launch_setup(context):
    def argument(name):
        return LaunchConfiguration(name).perform(context)

    use_mock = argument('use_mock').lower()
    if use_mock not in ('true', 'false'):
        raise ValueError('use_mock must be true or false')

    def bounded_integer(name, minimum, maximum):
        value = int(argument(name))
        if not minimum <= value <= maximum:
            raise ValueError(f'{name} must be in [{minimum}, {maximum}]')
        return str(value)

    yaw_id = bounded_integer('yaw_motor_id', 1, 253)
    pitch_id = bounded_integer('pitch_motor_id', 1, 253)
    if yaw_id == pitch_id:
        raise ValueError('yaw_motor_id and pitch_motor_id must be different')
    yaw_center = bounded_integer('yaw_center_steps', 0, 4095)
    pitch_center = bounded_integer('pitch_center_steps', 0, 4095)
    yaw_direction = argument('yaw_direction')
    pitch_direction = argument('pitch_direction')
    if yaw_direction not in ('1', '-1') or pitch_direction not in ('1', '-1'):
        raise ValueError('yaw_direction and pitch_direction must be 1 or -1')

    joint_state_sources = [
        source.strip()
        for source in argument('joint_state_sources').split(',')
        if source.strip()
    ]
    if not joint_state_sources:
        raise ValueError('joint_state_sources must contain at least one topic')
    if '/joint_states' in joint_state_sources or 'joint_states' in joint_state_sources:
        raise ValueError(
            'joint_state_sources cannot contain the aggregate /joint_states topic')

    config_share = get_package_share_directory('head_moveit_config')
    description_share = get_package_share_directory('robot_description')
    model = os.path.join(
        description_share, 'urdf', 'sim', 'my_car_rx200.urdf.xacro')
    robot_description = xacro.process_file(model, mappings={
        'arm_model': 'rx200',
        'enable_aluminium_frame': 'true',
        'enable_head_gimbal': 'true',
        'head_control_backend': 'sts',
        'head_serial_port': argument('serial_port'),
        'head_use_mock': use_mock,
        'head_yaw_motor_id': yaw_id,
        'head_pitch_motor_id': pitch_id,
        'head_yaw_center_steps': yaw_center,
        'head_pitch_center_steps': pitch_center,
        'head_yaw_direction': yaw_direction,
        'head_pitch_direction': pitch_direction,
    }).toxml()
    robot_description_semantic = xacro.process_file(
        os.path.join(config_share, 'config', 'head.srdf.xacro')).toxml()

    ompl = load_yaml(os.path.join(config_share, 'config', 'ompl_planning.yaml'))
    joint_limits = load_yaml(os.path.join(
        config_share, 'config', 'joint_limits.yaml'))
    moveit_controllers = load_yaml(os.path.join(
        config_share, 'config', 'moveit_controllers.yaml'))

    robot_parameters = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'robot_description_kinematics': {},
        'robot_description_planning': joint_limits,
    }
    planning_pipeline = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints'
            ),
            'start_state_max_bounds_error': 0.05,
            **ompl,
        },
    }
    controller_parameters = {
        'moveit_controller_manager': (
            'moveit_simple_controller_manager/MoveItSimpleControllerManager'),
        'moveit_simple_controller_manager': moveit_controllers,
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.5,
        'trajectory_execution.allowed_goal_duration_margin': 1.0,
        'trajectory_execution.allowed_start_tolerance': 0.04,
    }
    monitor_parameters = {
        'planning_scene_monitor_options': {
            'robot_description': 'robot_description',
            'joint_state_topic': '/joint_states',
        },
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
        'use_sim_time': False,
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0,
            'use_sim_time': False,
        }],
    )
    joint_state_aggregator = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='head_joint_state_aggregator',
        output='screen',
        parameters=[{
            'rate': 50,
            'source_list': joint_state_sources,
            'publish_default_positions': True,
            'publish_default_velocities': True,
            'publish_default_efforts': True,
            'use_sim_time': False,
        }],
    )
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='head',
        output='screen',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': False},
            os.path.join(config_share, 'config', 'ros2_controllers.yaml'),
        ],
    )
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/head/controller_manager',
        ],
    )
    motor_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'motor_controller',
            '--controller-manager', '/head/controller_manager',
        ],
    )
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            robot_parameters,
            planning_pipeline,
            controller_parameters,
            monitor_parameters,
        ],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='head_moveit_rviz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=[
            '-d', os.path.join(config_share, 'rviz', 'head_moveit.rviz'),
            '-f', 'base_footprint',
        ],
        parameters=[
            robot_parameters,
            planning_pipeline,
            {'use_sim_time': False},
        ],
    )

    return [
        robot_state_publisher,
        joint_state_aggregator,
        controller_manager,
        TimerAction(period=1.0, actions=[joint_state_spawner]),
        TimerAction(period=1.5, actions=[motor_controller_spawner]),
        TimerAction(period=2.0, actions=[move_group]),
        TimerAction(period=3.0, actions=[rviz]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mock', default_value='true',
            description='Use STSHardwareInterface mock mode; set false for motors.'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('yaw_motor_id', default_value='1'),
        DeclareLaunchArgument('pitch_motor_id', default_value='2'),
        DeclareLaunchArgument('yaw_center_steps', default_value='2048'),
        DeclareLaunchArgument('pitch_center_steps', default_value='2048'),
        DeclareLaunchArgument(
            'yaw_direction', default_value='1',
            description='Physical-to-URDF yaw sign: 1 or -1.'),
        DeclareLaunchArgument(
            'pitch_direction', default_value='-1',
            description='Physical-to-URDF pitch sign: 1 or -1.'),
        DeclareLaunchArgument(
            'joint_state_sources', default_value='/head/joint_states',
            description=(
                'Comma-separated raw joint-state topics to merge into '
                '/joint_states. Add arm/base controller topics when present.')),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
