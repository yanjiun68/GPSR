#!/usr/bin/env python3
"""
All-in-One launch for the RX200 chair pointing flow.

Sequence:
1. Step1 startup: robot controllers + MoveIt
2. Step2 perception: camera + YOLO chair detector
3. Step3 pointing: ready -> wait for chair 2s -> point -> sleep
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_grasp = FindPackageShare('rx200_handbag_grasp')

    # ========================================
    # Launch Arguments
    # ========================================
    use_saved_tf_arg = DeclareLaunchArgument(
        'use_saved_tf',
        default_value='false',
        description='Use saved camera TF instead of runtime calibration'
    )
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Launch RealSense camera (set to false to bypass)'
    )
    use_lift_arg = DeclareLaunchArgument(
        'use_lift',
        default_value='false',
        description='Launch real lift hardware. Keep false to use fake lift controller.'
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz from Step 1.'
    )
    show_camera_arg = DeclareLaunchArgument(
        'show_camera',
        default_value='false',
        description='Open the Step 2 camera viewer.'
    )
    show_yolo_arg = DeclareLaunchArgument(
        'show_yolo',
        default_value='true',
        description='Open the Step 2 YOLO viewer.'
    )
    step2_delay_arg = DeclareLaunchArgument(
        'step2_delay_sec',
        default_value='8.0',
        description='Delay before launching Step 2 perception'
    )
    step3_delay_arg = DeclareLaunchArgument(
        'step3_delay_sec',
        default_value='15.0',
        description='Delay before launching Step 3 chair pointing'
    )
    ready_msg_delay_arg = DeclareLaunchArgument(
        'ready_msg_delay_sec',
        default_value='20.0',
        description='Delay before printing the all-in-one ready message'
    )
    # ========================================
    # Step 1: Robot + MoveIt (immediate)
    # ========================================
    step1_startup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_grasp, 'launch', 'step1_startup.launch.py'])
        ]),
        launch_arguments={
            'use_lift': LaunchConfiguration('use_lift'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
        }.items()
    )

    # ========================================
    # Step 2: Perception
    # ========================================
    step2_perception = TimerAction(
        period=LaunchConfiguration('step2_delay_sec'),
        actions=[
            LogInfo(msg='[ALL-IN-ONE] Starting Step 2 perception: camera + chair detector...'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([pkg_grasp, 'launch', 'step2_perception.launch.py'])
                    ]),
                    launch_arguments={
                        'use_camera': LaunchConfiguration('use_camera'),
                        'show_camera': LaunchConfiguration('show_camera'),
                        'show_yolo': LaunchConfiguration('show_yolo'),
                        'target_class': 'chair',
                    }.items()
                )
        ]
    )

    # ========================================
    # Step 3: Pointing stage
    # ========================================
    step3_point = TimerAction(
        period=LaunchConfiguration('step3_delay_sec'),
        actions=[
            LogInfo(msg='[ALL-IN-ONE] Starting Step 3 pointing: ready -> detect chair 2s -> point -> sleep...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([pkg_grasp, 'launch', 'step3_point.launch.py'])
                ])
            )
        ]
    )

    # ========================================
    # Ready Message
    # ========================================
    ready_msg = TimerAction(
        period=LaunchConfiguration('ready_msg_delay_sec'),
        actions=[
            LogInfo(msg=''),
            LogInfo(msg='=' * 60),
            LogInfo(msg='[ALL-IN-ONE] SYSTEM RUNNING'),
            LogInfo(msg='=' * 60),
            LogInfo(msg=''),
            LogInfo(msg='Step1: controllers + MoveIt launched.'),
            LogInfo(msg='Step2: camera + YOLO chair detection launched.'),
            LogInfo(msg='Step3: robot should go ready, wait for chair 2s, then point for 10s and return to sleep.'),
            LogInfo(msg=''),
        ]
    )

    return LaunchDescription([
        use_saved_tf_arg,
        use_camera_arg,
        use_lift_arg,
        launch_rviz_arg,
        show_camera_arg,
        show_yolo_arg,
        step2_delay_arg,
        step3_delay_arg,
        ready_msg_delay_arg,
        
        LogInfo(msg=''),
        LogInfo(msg='=' * 60),
        LogInfo(msg='[ALL-IN-ONE] RX200 Chair Pointing System'),
        LogInfo(msg='=' * 60),
        LogInfo(msg=''),
        
        step1_startup,
        step2_perception,
        step3_point,
        ready_msg,
    ])
