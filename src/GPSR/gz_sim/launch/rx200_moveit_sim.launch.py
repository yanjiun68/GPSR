"""Attach RX200 MoveIt and trajectory relays to an already-running Gazebo simulation."""

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
    sim_share = get_package_share_directory('gz_sim')
    description_share = get_package_share_directory('robot_description')
    moveit_share = get_package_share_directory('interbotix_xsarm_moveit')

    robot_description = xacro.process_file(
        os.path.join(description_share, 'urdf', 'sim', 'my_car_rx200.urdf.xacro'),
        mappings={
            'arm_model': 'rx200',
            'enable_aluminium_frame': 'true',
            'enable_head_gimbal': 'true',
            'head_control_backend': 'gazebo',
        },
    ).toxml()
    robot_description_semantic = xacro.process_file(
        os.path.join(sim_share, 'config', 'rx200_sim.srdf.xacro'),
    ).toxml()

    kinematics = load_yaml(os.path.join(
        sim_share, 'config', 'rx200_sim_kinematics.yaml'))
    controllers = load_yaml(os.path.join(
        sim_share, 'config', 'rx200_sim_moveit_controllers.yaml'))
    ompl = load_yaml(os.path.join(
        moveit_share, 'config', 'ompl_planning.yaml'))
    joint_limits = load_yaml(os.path.join(
        moveit_share, 'config', 'joint_limits', 'rx200_joint_limits.yaml'))
    # The upstream file also contains a lift joint not present in this model.
    joint_limits.get('joint_limits', {}).pop('lift_joint', None)
    joint_limits.setdefault('joint_limits', {}).update({
        'head_yaw_joint': {
            'has_velocity_limits': True,
            'max_velocity': 0.5,
            'has_acceleration_limits': True,
            'max_acceleration': 0.8,
        },
        'head_pitch_joint': {
            'has_velocity_limits': True,
            'max_velocity': 0.5,
            'has_acceleration_limits': True,
            'max_acceleration': 0.8,
        },
    })
    ompl.setdefault('planner_configs', {}).setdefault(
        'RRTConnect', {'type': 'geometric::RRTConnect', 'range': 0.1})
    ompl['camera_head'] = {
        'default_planner_config': 'RRTConnect',
        'longest_valid_segment_fraction': 0.01,
        'planner_configs': ['RRTConnect'],
    }

    robot_parameters = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'robot_description_kinematics': kinematics,
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
            'start_state_max_bounds_error': 0.1,
            **ompl,
        },
    }
    controller_parameters = {
        'moveit_controller_manager': (
            'moveit_simple_controller_manager/MoveItSimpleControllerManager'),
        'moveit_simple_controller_manager': controllers,
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.5,
        'trajectory_execution.allowed_goal_duration_margin': 1.0,
        'trajectory_execution.allowed_start_tolerance': 0.02,
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
        'use_sim_time': True,
    }

    relay = Node(
        package='gz_sim',
        executable='rx200_trajectory_action_relay.py',
        name='rx200_trajectory_action_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    head_relay = Node(
        package='gz_sim',
        executable='head_trajectory_action_relay.py',
        name='head_trajectory_action_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
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
        name='rx200_moveit_rviz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=[
            '-d', LaunchConfiguration('rviz_config'),
            '-f', 'base_footprint',
        ],
        parameters=[
            robot_parameters,
            planning_pipeline,
            {'use_sim_time': True},
        ],
    )
    manipulation = Node(
        package='gz_sim',
        executable='rx200_pick_place.py',
        name='rx200_pick_place',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_manipulation')),
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('~/grasp_pose', '/rx200_manipulation/grasp_pose'),
            ('~/place_pose', '/rx200_manipulation/place_pose'),
            ('~/status', '/rx200_manipulation/status'),
        ],
    )

    return [
        relay,
        head_relay,
        TimerAction(period=2.0, actions=[move_group]),
        TimerAction(period=3.0, actions=[manipulation]),
        TimerAction(period=4.0, actions=[rviz]),
    ]


def generate_launch_description():
    sim_share = get_package_share_directory('gz_sim')
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                sim_share, 'config', 'rx200_moveit.rviz'),
            description=(
                'RViz configuration containing the integrated RX200 and '
                'camera-head MotionPlanning panel.')),
        DeclareLaunchArgument(
            'use_manipulation', default_value='true',
            description='Start the RX200 grasp/place pose executor.'),
        OpaqueFunction(function=launch_setup),
    ])
