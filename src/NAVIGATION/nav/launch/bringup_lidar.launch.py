#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # -------------------------------------------------
    # LIDAR PORT ARGUMENT
    # -------------------------------------------------
    lidar_port = LaunchConfiguration('lidar_port')

    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
        description='Serial port used by RPLIDAR'
    )

    # -------------------------------------------------
    # BASE BRINGUP (cmd_vel, odom, serial, URDF)
    # -------------------------------------------------
    putrabot_base_share = get_package_share_directory('putrabot_base')
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(putrabot_base_share, 'launch', 'start.launch.py')
        )
    )

    # -------------------------------------------------
    # RPLIDAR
    # -------------------------------------------------
    rplidar_share = get_package_share_directory('rplidar_ros')
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_share, 'launch', 'rplidar_a2m8_launch.py')
        ),
        launch_arguments={
            'serial_port': lidar_port,
        }.items()
    )

    # -------------------------------------------------
    # EKF (robot_localization)
    # -------------------------------------------------
    ekf_params = os.path.join(
        get_package_share_directory('nav'),
        'config',
        'ekf.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params]
    )

    # -------------------------------------------------
    # LAUNCH
    # -------------------------------------------------
    return LaunchDescription([
        declare_lidar_port,
        base_launch,
        rplidar_launch,
        ekf_node,
    ])
