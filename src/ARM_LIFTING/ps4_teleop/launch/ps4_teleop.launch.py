#!/usr/bin/env python3
"""
PS4 Teleop Launch File for RX200 + Lift

This launch file combines:
1. Official Interbotix xsarm_joy for arm control
2. Custom lift controller for lift axis

Usage:
    ros2 launch ps4_teleop ps4_teleop.launch.py
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, 
    TimerAction, LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    robot_model_arg = DeclareLaunchArgument(
        'robot_model', default_value='rx200',
        description='Robot model'
    )
    controller_arg = DeclareLaunchArgument(
        'controller', default_value='ps4',
        description='Controller type: ps3, ps4, xbox360'
    )
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev', default_value='/dev/input/js0',
        description='Joystick device path'
    )

    # ========================================
    # Option 1: Use official Interbotix xsarm_joy
    # ========================================
    # Uncomment below to use official package (if installed):
    #
    # xsarm_joy_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('interbotix_xsarm_joy'),
    #             'launch', 'xsarm_joy.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'robot_model': LaunchConfiguration('robot_model'),
    #         'controller': LaunchConfiguration('controller'),
    #     }.items()
    # )

    # ========================================
    # Option 2: Standalone joy + custom controller
    # ========================================
    # Joy node (gamepad driver)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )

    # Custom teleop node (arm + lift)
    teleop_node = Node(
        package='ps4_teleop',
        executable='ps4_controller.py',
        name='ps4_teleop',
        parameters=[{
            'linear_speed': 0.05,
            'angular_speed': 0.3,
            'lift_speed': 0.02,
            'deadzone': 0.15,
            'control_rate': 20.0,
        }],
        output='screen'
    )

    return LaunchDescription([
        robot_model_arg,
        controller_arg,
        joy_dev_arg,

        LogInfo(msg=''),
        LogInfo(msg='=' * 50),
        LogInfo(msg='PS4 Teleop for RX200 + Lift'),
        LogInfo(msg='=' * 50),
        LogInfo(msg=''),

        joy_node,
        TimerAction(period=1.0, actions=[teleop_node]),
    ])
