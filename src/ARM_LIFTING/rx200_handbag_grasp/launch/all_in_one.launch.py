#!/usr/bin/env python3
"""
All-in-one orchestrated workflow for the RX200 handbag grasping system.

Sequence:
1. Start Step1: robot + lift + MoveIt.
2. Start Step2 YOLO11n + MediaPipe.
3. Start Step3, then call /grasp/execute_ready.
4. Stop Step2 YOLO11n, then start Step2 perception.
5. Call /grasp/execute, /grasp/execute_ready, and /grasp/rest.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_external_drivers_arg = DeclareLaunchArgument(
        'use_external_drivers',
        default_value='false',
        description='Assume shared RX200/camera drivers are already launched elsewhere.'
    )
    use_saved_tf_arg = DeclareLaunchArgument(
        'use_saved_tf',
        default_value='false',
        description='Reserved for Step3 compatibility; camera TF is loaded by the perception nodes.'
    )
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Launch RealSense camera in the active Step2 pipeline.'
    )
    show_yolo_window_arg = DeclareLaunchArgument(
        'show_yolo_window',
        default_value='false',
        description='Show YOLO11n detector window during the first Step2 phase.'
    )
    show_mediapipe_window_arg = DeclareLaunchArgument(
        'show_mediapipe_window',
        default_value='true',
        description='Show MediaPipe + bag overlay window during the first Step2 phase.'
    )
    show_selector_arg = DeclareLaunchArgument(
        'show_selector',
        default_value='true',
        description='Show selector visualization window in Step2 perception.'
    )
    step2_yolo_delay_arg = DeclareLaunchArgument(
        'step2_yolo_delay_s',
        default_value='8.0',
        description='Delay after Step1 launch before starting Step2 YOLO11n.'
    )
    step3_delay_arg = DeclareLaunchArgument(
        'step3_delay_s',
        default_value='15.0',
        description='Delay after Step2 YOLO11n launch before starting Step3.'
    )
    ready_after_step3_delay_arg = DeclareLaunchArgument(
        'ready_after_step3_delay_s',
        default_value='8.0',
        description='Delay after Step3 launch before calling /grasp/execute_ready.'
    )
    step2_perception_settle_arg = DeclareLaunchArgument(
        'step2_perception_settle_s',
        default_value='12.0',
        description='Delay after Step2 perception launch before calling /grasp/execute.'
    )

    # Integration switches used when this launch is driven as the
    # carry_my_luggage_phase1 phase 2 bundle.  In that mode step1/step3
    # are already resident (top-level launch owns them) and /grasp/rest
    # must NOT be called at the end (phase 3 handover opens the gripper).
    # Defaults preserve the standalone demo behaviour.
    skip_step1_arg = DeclareLaunchArgument(
        'skip_step1', default_value='false',
        description='Assume step1 (arm + MoveIt + lift) is already running externally.'
    )
    skip_step3_arg = DeclareLaunchArgument(
        'skip_step3', default_value='false',
        description='Assume step3 (grasp_node) is already running externally.'
    )
    skip_step2_preview_arg = DeclareLaunchArgument(
        'skip_step2_preview', default_value='false',
        description=(
            'Skip the step2_yolo11n_mediapipe preview stage (MediaPipe '
            'hand-pointing debug view).  Used in integrated pipelines '
            'where bag selection already happened upstream.'
        )
    )
    skip_rest_at_end_arg = DeclareLaunchArgument(
        'skip_rest_at_end', default_value='false',
        description='Do NOT call /grasp/rest after /grasp/execute succeeds.'
    )

    orchestrator = Node(
        package='rx200_handbag_grasp',
        executable='all_in_one_orchestrator.py',
        name='all_in_one_orchestrator',
        output='screen',
        parameters=[{
            'use_external_drivers': LaunchConfiguration('use_external_drivers'),
            'use_saved_tf': LaunchConfiguration('use_saved_tf'),
            'use_camera': LaunchConfiguration('use_camera'),
            'show_yolo_window': LaunchConfiguration('show_yolo_window'),
            'show_mediapipe_window': LaunchConfiguration('show_mediapipe_window'),
            'show_selector': LaunchConfiguration('show_selector'),
            'step2_yolo_delay_s': LaunchConfiguration('step2_yolo_delay_s'),
            'step3_delay_s': LaunchConfiguration('step3_delay_s'),
            'ready_after_step3_delay_s': LaunchConfiguration('ready_after_step3_delay_s'),
            'step2_perception_settle_s': LaunchConfiguration('step2_perception_settle_s'),
            'skip_step1': LaunchConfiguration('skip_step1'),
            'skip_step3': LaunchConfiguration('skip_step3'),
            'skip_step2_preview': LaunchConfiguration('skip_step2_preview'),
            'skip_rest_at_end': LaunchConfiguration('skip_rest_at_end'),
        }],
        emulate_tty=True,
    )

    return LaunchDescription([
        use_external_drivers_arg,
        use_saved_tf_arg,
        use_camera_arg,
        show_yolo_window_arg,
        show_mediapipe_window_arg,
        show_selector_arg,
        step2_yolo_delay_arg,
        step3_delay_arg,
        ready_after_step3_delay_arg,
        step2_perception_settle_arg,
        skip_step1_arg,
        skip_step3_arg,
        skip_step2_preview_arg,
        skip_rest_at_end_arg,
        LogInfo(msg='[ALL-IN-ONE] Starting orchestrated RX200 handbag grasp workflow'),
        orchestrator,
    ])
