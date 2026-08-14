#!/usr/bin/env python3
"""
Step 2: Perception System Launch
Start RealSense Camera

Usage:
    ros2 launch rx200_handbag_grasp step2_perception.launch.py
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ========================================
    # Paths & Constants
    # ========================================
    grasp_pkg_share = FindPackageShare('rx200_handbag_grasp')

    # ========================================
    # Launch Arguments
    # ========================================
    use_camera_arg = DeclareLaunchArgument(
        'use_camera', default_value='true',
        description='Launch RealSense camera'
    )

    enable_depth_arg = DeclareLaunchArgument(
        'enable_depth', default_value='true',
        description='Enable RealSense depth stream'
    )

    align_depth_arg = DeclareLaunchArgument(
        'align_depth', default_value='true',
        description='Align depth to color frame'
    )

    rgb_profile_arg = DeclareLaunchArgument(
        'rgb_profile', default_value='1280x720x30',
        description='RealSense RGB stream profile: WIDTHxHEIGHTxFPS'
    )

    depth_profile_arg = DeclareLaunchArgument(
        'depth_profile', default_value='640x480x30',
        description='RealSense depth stream profile: WIDTHxHEIGHTxFPS'
    )

    open_realsense_viewer_arg = DeclareLaunchArgument(
        'open_realsense_viewer', default_value='false',
        description='Auto-open realsense-viewer GUI for tuning exposure/depth'
    )

    show_camera_arg = DeclareLaunchArgument(
        'show_camera', default_value='false',
        description='Open a simple camera window using OpenCV'
    )

    show_yolo_arg = DeclareLaunchArgument(
        'show_yolo', default_value='true',
        description='Open YOLO annotated detection window'
    )

    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value=PathJoinSubstitution([grasp_pkg_share, 'models', 'chair.pt']),
        description='Absolute path to the YOLO model used for Step 2 detection'
    )

    target_class_arg = DeclareLaunchArgument(
        'target_class', default_value='chair',
        description='Class name to keep from YOLO detections'
    )

    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.30',
        description='YOLO confidence threshold'
    )

    # ========================================
    # 1. Camera (RealSense) - Reverted to simple config matching calibrate_official
    # ========================================
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': 'camera',
            'enable_color': 'true',
            'enable_depth': LaunchConfiguration('enable_depth'),
            'align_depth.enable': LaunchConfiguration('align_depth'),
            'rgb_camera.color_profile': LaunchConfiguration('rgb_profile'),
            'depth_module.depth_profile': LaunchConfiguration('depth_profile'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_camera'))
    )

    # ========================================
    # 1.1 RealSense Viewer (optional GUI)
    # ========================================
    realsense_viewer = ExecuteProcess(
        cmd=['realsense-viewer'],
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'",
                LaunchConfiguration('use_camera'),
                "' == 'true' and '",
                LaunchConfiguration('open_realsense_viewer'),
                "'",
                " == 'true'"
            ])
        )
    )

    # ========================================
    # 1.2 Simple Camera Viewer (optional GUI)
    # ========================================
    camera_viewer_node = Node(
        package='rx200_handbag_grasp',
        executable='camera_viewer.py',
        name='camera_viewer',
        parameters=[{
            'image_topic': '/camera/camera/color/image_raw/compressed',
            'window_name': 'Camera View',
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('show_camera'))
    )

    yolo_detector_node = Node(
        package='rx200_handbag_grasp',
        executable='yolo_handle_detector.py',
        name='yolo_handle_detector',
        output='screen',
        parameters=[{
            'image_topic': '/camera/camera/color/image_raw/compressed',
            'detections_topic': '/yolo/detections',
            'annotated_image_topic': '/yolo/image_annotated/compressed',
            'model_path': LaunchConfiguration('yolo_model'),
            'target_class': LaunchConfiguration('target_class'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
        }],
    )

    yolo_viewer_node = Node(
        package='rx200_handbag_grasp',
        executable='yolo_viewer.py',
        name='yolo_viewer',
        output='screen',
        parameters=[{
            'image_topic': '/yolo/image_annotated/compressed',
            'window_name': 'YOLO Chair Detection',
        }],
        condition=IfCondition(LaunchConfiguration('show_yolo'))
    )

    # ========================================
    # 3. Camera TF Publisher (Loads calibration)
    # ========================================
    camera_tf_node = Node(
        package='rx200_handbag_grasp',
        executable='camera_tf_publisher.py',
        name='camera_tf_publisher',
        output='screen'
    )

    # ========================================
    # Startup Sequence
    # ========================================
    return LaunchDescription([
        use_camera_arg,
        enable_depth_arg,
        align_depth_arg,
        rgb_profile_arg,
        depth_profile_arg,
        open_realsense_viewer_arg,
        show_camera_arg,
        show_yolo_arg,
        yolo_model_arg,
        target_class_arg,
        confidence_threshold_arg,

        # Camera first
        camera_launch,
        realsense_viewer,
        camera_viewer_node,
        TimerAction(period=2.0, actions=[yolo_detector_node]),
        TimerAction(period=3.0, actions=[yolo_viewer_node]),

        # TF Publisher
        camera_tf_node,
    ])
