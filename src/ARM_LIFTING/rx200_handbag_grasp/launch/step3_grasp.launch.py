#!/usr/bin/env python3
"""
Step 3: Grasp Execution Launch
启动抓取执行节点 - Eye-in-Hand 模式，无需 AprilTag 标定

Usage:
    ros2 launch rx200_handbag_grasp step3_grasp.launch.py

Prerequisites:
    - Step 1 (MoveIt + Lift) must be running
    - Step 2 (Camera + Detection) must be running
    - Lift must be homed (should happen automatically in Step 1)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ========================================
    # Paths & Constants
    # ========================================
    grasp_pkg_share = FindPackageShare('rx200_handbag_grasp')
    grasp_params = PathJoinSubstitution([grasp_pkg_share, 'config', 'grasp_params.yaml'])
    guard_pkg_share = FindPackageShare('rx200_gripper_guard')
    guard_params = PathJoinSubstitution([guard_pkg_share, 'config', 'gripper_guard.yaml'])

    # ========================================
    # Launch Arguments
    # ========================================
    viz_only_arg = DeclareLaunchArgument(
        'visualization_only', default_value='false',
        description='If true, run diagnostics and visualization but do not move arm'
    )
    use_saved_tf_arg = DeclareLaunchArgument(
        'use_saved_tf', default_value='false',
        description='Compatibility argument for all-in-one launch; Step3 uses the active camera TF publisher.'
    )

    # ========================================
    # Gripper guard starts first and remains the sole PWM owner during guarded close.
    # ========================================
    gripper_guard_node = Node(
        package='rx200_gripper_guard',
        executable='gripper_guard_node',
        name='rx200_gripper_guard',
        parameters=[guard_params],
        output='screen',
        emulate_tty=False,
    )

    # Main Grasp Node
    grasp_node = Node(
        package='rx200_handbag_grasp',
        executable='grasp_node.py',
        name='grasp_node',
        parameters=[
            grasp_params,
            {
                'debug.visualization_only': LaunchConfiguration('visualization_only'),
                'yolo_detections_topic': '/handbag/selected_detections',
            },
        ],
        output='screen',
        emulate_tty=False,
        additional_env={
            'PYTHONNOUSERSITE': '1',
        },
    )

    # ========================================
    # Launch Description
    # ========================================
    return LaunchDescription([
        viz_only_arg,
        use_saved_tf_arg,

        # Companion guard tolerates startup delay while xs_sdk feedback becomes available.
        gripper_guard_node,

        # Grasp node (slightly delayed to ensure other systems are ready)
        TimerAction(
            period=3.0,
            actions=[grasp_node],
        ),
    ])
