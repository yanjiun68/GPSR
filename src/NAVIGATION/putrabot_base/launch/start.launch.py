#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # ============================================================
    # 1) Include robot_description -> base.launch.py
    #    (URDF + robot_state_publisher + joint_state_publisher)
    # ============================================================
    robot_description_pkg = get_package_share_directory('robot_description')
    base_launch_path = os.path.join(robot_description_pkg, 'launch', 'base.launch.py')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_path),
        # If you want to override args from base.launch.py, uncomment:
        # launch_arguments={
        #     'use_sim_time': 'false',
        #     'model': os.path.join(robot_description_pkg, 'urdf', 'main', 'base.urdf.xacro'),
        # }.items(),
    )

    # ============================================================
    # 2) Node: /cmd_vel  --->  /wheel_targets (ticks/sec)
    # ============================================================
    cmdvel_node = Node(
        package='putrabot_base',
        executable='cmd_vel.py',          # installed by CMake from scripts/
        name='cmdvel_to_targets',
        output='screen',
        parameters=[{
            # --- geometry (must match your real robot) ---
            'wheel_radius': 0.05,
            'lx': 0.2525,
            'ly': 0.2475,

            # --- encoder ticks per rev ---
            'tpr_fl': 3332.0,     # FL
            'tpr_fr': 3332.0,     # FR
            'tpr_rr': 3332.0,     # RR
            'tpr_rl': 3332.0,     # RL

            # --- optional motor inversion ---
            'invert_fl': False,
            'invert_fr': False,
            'invert_rr': False,
            'invert_rl': False,

            # --- motion limits ---
            'vx_max': 0.8,
            'vy_max': 0.8,
            'wz_max': 2.5,
            'deadband': 0.0,

            # --- max ticks/sec (huge = no limit) ---
            'max_tps_m1': 1.0e9,
            'max_tps_m2': 1.0e9,
            'max_tps_m3': 1.0e9,
            'max_tps_m4': 1.0e9,

            # acceleration limits
            'ax_max': 1.5,
            'ay_max': 1.5,
            'aw_max': 4.0,

            # --- watchdog + publish rate ---
            'timeout': 0.2,
            'publish_rate': 20.0,     # Hz
        }],
        # remappings=[]  # keep default: /cmd_vel and /wheel_targets
    )

    # ============================================================
    # 3) Node: /wheel_ticks  --->  /odom + TF  (omni odometry)
    # ============================================================
    odom_node = Node(
        package='putrabot_base',
        executable='wheel_odometry.py',   # installed by CMake from scripts/
        name='wheel_odometry',
        output='screen',
        parameters=[{
            # --- TF frames ---
            'odom_frame': 'odom',
            'base_footprint_frame': 'base_footprint',

            # TF switch (IMPORTANT)
            'publish_tf': True,

            # --- geometry (must match robot) ---
            'wheel_radius': 0.05,
            'lx': 0.2051,
            'ly': 0.2051,

            # --- encoder ticks per rev ---
            'tpr_m1': 3332.0,     # FL
            'tpr_m2': 3332.0,     # FR
            'tpr_m3': 3332.0,     # RR
            'tpr_m4': 3332.0,     # RL

            # --- encoder polarity (sign) ---
            'enc_sign_m1':  1,
            'enc_sign_m2':  1,
            'enc_sign_m3':  1,
            'enc_sign_m4':  1,

            # --- covariance (for EKF later) ---
            'pose_cov_xy': 1e-3,
            'pose_cov_th': 1e-2,
            'twist_cov':   1e-2,


        }],
        # remappings=[]  # default: wheel_ticks, odom
    )

    # ============================================================
    # 4) Node: serial bridge (ROS2 <-> Arduino)
    # ============================================================
    serial_bridge_node = Node(
        package='putrabot_base',
        executable='serial_bridge.py',    # installed by CMake from scripts/
        name='base_serial_bridge',
        output='screen',
        parameters=[{
            'port': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',   # change if your Arduino on other port
            'baud': 115200,           # must match DBG_BAUD in Arduino
            ##'poll_rate': 100.0,        # Hz, matches your node default
        }],
    )

    # ============================================================
    # 5) Return full launch description
    # ============================================================
    return LaunchDescription([
        base_launch,         # URDF + robot_state_publisher + joint_state_publisher
        cmdvel_node,         # /cmd_vel  ->  /wheel_targets
        odom_node,           # /wheel_ticks  ->  /odom + TF
        serial_bridge_node,  # Serial <-> Arduino
    ])
