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
    # ---------------- LIDAR PORT ARG ----------------
    lidar_port = LaunchConfiguration('lidar_port')

    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
        description='Serial port used by RPLIDAR'
    )

    # ---------------- BASE BRINGUP ----------------
    putrabot_base_share = get_package_share_directory('putrabot_base')
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(putrabot_base_share, 'launch', 'start.launch.py')
        )
    )

    # ---------------- RPLIDAR ----------------
    rplidar_share = get_package_share_directory('rplidar_ros')
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_share, 'launch', 'rplidar_a2m8_launch.py')
        ),
        launch_arguments={
            'serial_port': lidar_port,
            'frame_id': 'lidar_link',
            'channel_type': 'serial',
        }.items()
    )

    # ---------------- REALSENSE (NAVIGATION PROFILE) ----------------
    realsense_share = get_package_share_directory('realsense2_camera')
    realsense_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'camera_namespace': 'camera_nav',
            'camera_name': 'camera',
            'enable_color': 'true',
            'enable_depth': 'true',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',

            # --- Navigation filters ---
            'decimation_filter.enable': 'true',
            'spatial_filter.enable': 'true',
            'temporal_filter.enable': 'true',
            'hole_filling_filter.enable': 'true',

            # Optional but safe
            'publish_tf': 'true',
        }.items()
    )

    # ---------------- EKF ----------------
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
        parameters=[ekf_params],

    )

    # ---------------- SLAM TOOLBOX ----------------
    slam_params = os.path.join(
        get_package_share_directory('nav'),
        'config',
        'slam.yaml'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odometry/filtered'),
        ]
    )

    return LaunchDescription([
        declare_lidar_port,
        base_launch,
        rplidar_launch,
        realsense_nav_launch,
        ekf_node,
        slam_node,
    ])
