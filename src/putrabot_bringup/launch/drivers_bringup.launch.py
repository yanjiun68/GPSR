#!/usr/bin/env python3

import json
import os
import subprocess

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def _resolve_serial_roles():
    default_base = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
    default_lift = "auto"
    default_nano_d2 = "/dev/ttyUSB0"
    resolver = os.path.join(
        get_package_prefix("putrabot_bringup"),
        "lib",
        "putrabot_bringup",
        "resolve_serial_roles.py",
    )

    try:
        result = subprocess.run(
            [resolver, "--json"],
            check=True,
            capture_output=True,
            text=True,
        )
        payload = json.loads(result.stdout)
        base_port = payload.get("base_port") or default_base
        lift_port = payload.get("lift_port") or default_lift
        nano_d2_port = payload.get("nano_d2_port") or default_nano_d2
        return {
            "base_port": base_port,
            "lift_port": lift_port,
            "nano_d2_port": nano_d2_port,
            "ports": payload.get("ports") or [],
            "debug": payload.get("debug") or [],
        }
    except Exception:
        return {
            "base_port": default_base,
            "lift_port": default_lift,
            "nano_d2_port": default_nano_d2,
            "ports": [],
            "debug": ["resolver failed, using defaults"],
        }


def generate_launch_description():
    resolved_ports = _resolve_serial_roles()
    base_port = resolved_ports["base_port"]
    lift_port = resolved_ports["lift_port"]
    nano_d2_port = resolved_ports["nano_d2_port"]
    lidar_serial_port = LaunchConfiguration('serial_port')
    enable_arm_lift_drivers = LaunchConfiguration('enable_arm_lift_drivers')
    button_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("dio13_monitor_example"),
                "launch",
                "dio13_monitor.launch.py",
            )
        ),
        launch_arguments={
            "port": nano_d2_port,
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('enable_person_following'))
    )

    person_follow_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        launch_arguments={
            "camera_namespace": "camera",
            "camera_name": "camera",
            "serial_no": "_022422071806",
            "enable_color": "true",
            "enable_depth": "true",
            "enable_gyro": "true",
            "enable_accel": "true",
            "align_depth.enable": "true",
            "enable_sync": "true",
            "rgb_camera.color_profile": "640x480x30",
            "depth_module.depth_profile": "640x480x30",
            "unite_imu_method": "1",
            "publish_tf": "false",
            "pointcloud.enable": "true",
            "pointcloud.stream_filter": "2",
            "pointcloud.stream_index_filter": "0",
            "filters": "spatial,decimation",
            "decimation_filter.filter_magnitude": "2",
            "gyro_fps": "200",
            "accel_fps": "63",
            "wait_for_device_timeout": "10.0",
            "reconnect_timeout": "3.0",
            "use_sim_time": "false",
            "output": "screen",
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'",
                LaunchConfiguration('enable_robot_vision'),
                "' == 'true' or '",
                LaunchConfiguration('enable_person_following'),
                "'",
                " == 'true'"
            ])
        )
    )

    arm_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        launch_arguments={
            "camera_namespace": "camera",
            "camera_name": "rx200_camera",
            "serial_no": "_947122071187",
            "enable_color": "true",
            "enable_depth": "true",
            "enable_infra": "false",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "enable_gyro": "false",
            "enable_accel": "false",
            "enable_motion": "false",
            "unite_imu_method": "0",
            "align_depth.enable": "true",
            "pointcloud.enable": "false",
            "wait_for_device_timeout": "10.0",
            "reconnect_timeout": "3.0",
            "use_sim_time": "false",
            "output": "screen",
            "log_level": "warn",
            'enable_sync': 'true',
            'enable_rgbd': 'true',
            'rgb_camera.color_profile': '640x480x30',
            'rgb_camera.color_format': 'RGB8',
            'depth_module.depth_profile': '640x480x30',
            'depth_module.depth_format': 'Z16',
            'publish_tf': 'true',

        }.items(),
        condition=UnlessCondition(LaunchConfiguration('enable_person_following'))
    )

    arm_lift_drivers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rx200_handbag_grasp"),
                "launch",
                "step1_startup.launch.py",
            )
        ),
        launch_arguments={
            # "use_rviz": "false",
            # "use_external_drivers": "false",
            "launch_moveit": "false",
            # "xs_port": "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT2XP6Z8-if00-port0",
            "lift_port": lift_port,
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'",
                enable_arm_lift_drivers,
                "' == 'true' and '",
                LaunchConfiguration('enable_person_following'),
                "' != 'true'",
            ])
        )
    )

    # twist_mux_node = Node(
    #     package='twist_mux',
    #     executable='twist_mux',
    #     name='twist_mux',
    #     output='screen',
    #     remappings=[('/cmd_vel_out', '/cmd_vel')],
    #     parameters=[twist_mux_config],
    #     condition=IfCondition(LaunchConfiguration('enable_person_following'))
    # )

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("my_robot_nav"),
                "launch",
                "drivers_only.launch.py",
            )
        ),
        launch_arguments={
            # "use_rviz": "false",
            # "use_external_drivers": "false",
            # "xs_port": "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT2XP6Z8-if00-port0",
            "serial_port": lidar_serial_port,
            "arduino_port": base_port,
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('enable_person_following'))
    )

    resolver_log = LogInfo(
        msg=(
            "[drivers_bringup] serial ports found="
            f"{resolved_ports['ports']} base_port={base_port} lift_port={lift_port} "
            f"nano_d2_port={nano_d2_port} "
            f"debug={resolved_ports['debug']}"
        )
    )

    # kinematics_node = Node(
    #     package='my_robot_nav',
    #     executable='cmd_vel_to_targets.py',
    #     name='cmdvel_to_targets',
    #     output='screen',
    #     condition=IfCondition(LaunchConfiguration('enable_person_following'))
    # )

    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[ekf_config_path],
    #     remappings=[('odometry/filtered', '/odom_filtered')],
    #     condition=IfCondition(LaunchConfiguration('enable_person_following'))
    # )
    

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'serial_port',
                default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                description='Serial port for the lidar passed through to drivers_only.launch.py',
            ),
            DeclareLaunchArgument(
                'enable_robot_vision',
                default_value='false',
                description='Serial port for the lidar passed through to drivers_only.launch.py',
            ),
            DeclareLaunchArgument(
                'enable_person_following',
                default_value='false',
                description='Serial port for the lidar passed through to drivers_only.launch.py',
            ),
            DeclareLaunchArgument(
                'enable_arm_lift_drivers',
                default_value='true',
                description='Start RX200 lift startup drivers.',
            ),
            button_launch,
            resolver_log,
            person_follow_camera_launch,
            arm_camera_launch,
            arm_lift_drivers_launch,
            base_launch,
            # twist_mux_node,
            # kinematics_node,
            # ekf_node,
        ]
    )
