#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # -------- Launch 参数 --------
    use_hardware_driver = LaunchConfiguration("use_hardware_driver")
    use_static_tf = LaunchConfiguration("use_static_tf")
    auto_home = LaunchConfiguration("auto_home")

    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    min_m = LaunchConfiguration("min_m")
    max_m = LaunchConfiguration("max_m")

    # -------- 包路径 --------
    pkg_share = get_package_share_directory("lift_moveit_config")

    # -------- MoveIt config（robot_description/srdf/kinematics/joint_limits 等）--------
    moveit_config = (
        MoveItConfigsBuilder("lift", package_name="lift_moveit_config")
        .to_moveit_configs()
    )

    # ✅ controllers 参数文件（必须是 ROS2 参数文件格式）
    controllers_yaml = os.path.join(pkg_share, "config", "moveit_controllers.yaml")

    # -------- robot_state_publisher --------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # -------- 可选 static TF（仅当 URDF 没有 world 固定 joint 才开）--------
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_world_to_base",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "lift_base_link"],
        condition=IfCondition(use_static_tf),
    )

    # -------- 可选硬件驱动（你的包名/可执行名：lift_bridge / lift_serial_driver）--------
    # ⚠️ 这里只传你 driver 确定 declare 过的参数，避免“未声明参数直接启动失败”
    lift_driver = Node(
        package="lift_bridge",
        executable="lift_serial_driver",
        name="lift_serial_driver",
        output="screen",
        parameters=[
            {
                "port": port,
                "baud": baud,
                "joint_name": "lift_joint",
                "min_m": min_m,
                "max_m": max_m,
                "pos_publish_hz": 30.0,
                "tolerance_m": 0.002,  # 2mm 容差（与 Arduino DEAD_BAND=0.2mm 匹配）
                "trajectory_mode": "final",
                "cmd_rate_hz": 3.0,
                "step_size_m": 0.02,  # W/S 步进 20mm
                "auto_home_on_start": auto_home,  # 启动时自动归零
            }
        ],
        condition=IfCondition(use_hardware_driver),
    )

    # -------- move_group（显式命名为 move_group，和 controllers_yaml 里的 node scope 对齐）--------
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",  # ✅ 关键：让 moveit_controllers.yaml 的 move_group: 生效
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            controllers_yaml,  # ✅ 用 params-file 方式加载
        ],
    )

    # -------- RViz（显式命名为 rviz2，和 controllers_yaml 里的 node scope 对齐）--------
    rviz_config = os.path.join(pkg_share, "config", "moveit.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",  # ✅ 关键：让 moveit_controllers.yaml 的 rviz2: 生效
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.to_dict(),
            controllers_yaml,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_hardware_driver",
                default_value="true",
                description="Start lift_serial_driver if true",
            ),
            DeclareLaunchArgument(
                "use_static_tf",
                default_value="false",
                description="Publish static TF world->lift_base_link (ONLY if URDF has no world fixed joint)",
            ),
            DeclareLaunchArgument(
                "auto_home",
                default_value="true",
                description="Auto-home on startup (recommended)",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="/dev/ttyUSB0",
                description="Serial port (recommend /dev/serial/by-id/...)",
            ),
            DeclareLaunchArgument(
                "baud",
                default_value="115200",
                description="Serial baudrate",
            ),
            DeclareLaunchArgument(
                "min_m",
                default_value="0.0",
                description="Soft lower limit (m)",
            ),
            DeclareLaunchArgument(
                "max_m",
                default_value="0.275",
                description="Soft upper limit (m) - 275mm",
            ),
            rsp,
            static_tf,
            lift_driver,
            move_group,
            rviz,
        ]
    )
