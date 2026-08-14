#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_lift = LaunchConfiguration('use_lift')
    launch_rviz = LaunchConfiguration('launch_rviz')

    rx200_startup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rc_rx200'),
                'upstream',
                'rx200_handbag_grasp',
                'launch',
                'step1_startup.launch.py',
            ])
        ),
        launch_arguments={
            'use_lift': use_lift,
            'launch_rviz': launch_rviz,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_lift',
            default_value='false',
            description='Start the real lift serial driver with the RX200 stack.',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Open the RX200 MoveIt RViz view.',
        ),
        LogInfo(msg='[rc_rx200] Starting RX200 controller stack for receptionist seat pointing.'),
        rx200_startup,
    ])
