#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    grasp_ready_monitor = Node(
        package='carry_my_luggage_follow_demo',
        executable='grasp_ready_monitor.py',
        name='grasp_ready_monitor',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    waypoint_store = Node(
        package='carry_my_luggage_follow_demo',
        executable='waypoint_store_node',
        name='waypoint_store',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )


    state_machine = Node(
        package='carry_my_luggage_follow_demo',
        executable='carry_my_luggage_follow_demo',
        name='carry_my_luggage_follow_demo',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        grasp_ready_monitor,
        waypoint_store,
        state_machine,
    ])
