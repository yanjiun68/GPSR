#!/usr/bin/env python3
"""
Step 2 variant: RealSense + Step2 bag-handle detector + MediaPipe display.

This launch intentionally uses unique node names, topics, and window names so it
can be tested without colliding with the normal Step2 perception pipeline.

Usage:
    ros2 launch rx200_handbag_grasp step2_yolo11n_mediapipe.launch.py
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
    grasp_pkg_share = FindPackageShare('rx200_handbag_grasp')
    tags_yaml = PathJoinSubstitution([grasp_pkg_share, 'config', 'tags.yaml'])
    default_model = PathJoinSubstitution([grasp_pkg_share, 'models', 'best_bag_handle.pt'])

    use_camera_arg = DeclareLaunchArgument(
        'use_camera', default_value='true',
        description='Launch RealSense camera'
    )
    use_yolo_arg = DeclareLaunchArgument(
        'use_yolo', default_value='true',
        description='Launch Step2 bag-handle detection'
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
        description='Auto-open realsense-viewer GUI'
    )
    use_apriltag_arg = DeclareLaunchArgument(
        'use_apriltag', default_value='false',
        description='Launch AprilTag detection'
    )
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value=default_model,
        description='Step2 bag-handle model path'
    )
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.25',
        description='YOLO confidence threshold'
    )
    max_detections_arg = DeclareLaunchArgument(
        'max_detections', default_value='2',
        description='Maximum number of bag detections to show'
    )
    show_yolo_window_arg = DeclareLaunchArgument(
        'show_yolo_window', default_value='false',
        description='Show extra detector-only window. The MediaPipe window already overlays detections.'
    )
    show_mediapipe_window_arg = DeclareLaunchArgument(
        'show_mediapipe_window', default_value='true',
        description='Show MediaPipe + bag overlay window'
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
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
        condition=IfCondition(LaunchConfiguration('use_camera')),
    )

    realsense_viewer = ExecuteProcess(
        cmd=['realsense-viewer'],
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'",
                LaunchConfiguration('use_camera'),
                "' == 'true' and '",
                LaunchConfiguration('open_realsense_viewer'),
                "' == 'true'",
            ])
        ),
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_ros_continuous_detector_node',
        name='bag_detector_apriltag_node',
        remappings=[
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info'),
            ('detections', '/bag_detector/apriltag/detections'),
        ],
        parameters=[tags_yaml],
        condition=IfCondition(LaunchConfiguration('use_apriltag')),
    )

    yolo_node = Node(
        package='rx200_handbag_grasp',
        executable='yolo_handle_detector.py',
        name='bag_detector_two_bag_detector',
        parameters=[{
            'model_path': LaunchConfiguration('yolo_model'),
            'image_topic': '/camera/camera/color/image_raw/compressed',
            'detections_topic': '/bag_detector/detections',
            'annotated_image_topic': '/bag_detector/image_annotated/compressed',
            'mask_topic': '/bag_detector/mask',
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'device': 'cuda',
            'model_type': 'yolo',
            'model_size': 'l',
            'max_detections': LaunchConfiguration('max_detections'),
            'one_per_class': False,
            'use_sam_segmentation': False,
            'target_classes': [
                'bag handle',
                'handbag handle',
                'strap',
            ],
            'filter_target_classes': False,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_yolo')),
    )

    yolo_two_bag_viewer = Node(
        package='rx200_handbag_grasp',
        executable='yolo_viewer.py',
        name='bag_detector_two_bag_viewer',
        output='screen',
        parameters=[{
            'image_topic': '/bag_detector/image_annotated/compressed',
            'window_name': 'Bag Detector: Two Bags',
        }],
        condition=IfCondition(LaunchConfiguration('show_yolo_window')),
    )

    mediapipe_bag_viewer = Node(
        package='rx200_handbag_grasp',
        executable='mediapipe_bag_viewer.py',
        name='bag_detector_mediapipe_bag_viewer',
        output='screen',
        parameters=[{
            'image_topic': '/camera/camera/color/image_raw/compressed',
            'detections_topic': '/bag_detector/detections',
            'output_topic': '/bag_detector/mediapipe_annotated/compressed',
            'window_name': 'MediaPipe + Two Bag Detections',
            'show_window': LaunchConfiguration('show_mediapipe_window'),
            'max_draw_detections': LaunchConfiguration('max_detections'),
        }],
    )

    camera_tf_node = Node(
        package='rx200_handbag_grasp',
        executable='camera_tf_publisher.py',
        name='bag_detector_camera_tf_publisher',
        output='screen',
    )

    return LaunchDescription([
        use_camera_arg,
        use_yolo_arg,
        enable_depth_arg,
        align_depth_arg,
        rgb_profile_arg,
        depth_profile_arg,
        open_realsense_viewer_arg,
        use_apriltag_arg,
        yolo_model_arg,
        confidence_threshold_arg,
        max_detections_arg,
        show_yolo_window_arg,
        show_mediapipe_window_arg,
        camera_launch,
        realsense_viewer,
        TimerAction(period=5.0, actions=[apriltag_node]),
        TimerAction(period=8.0, actions=[yolo_node]),
        TimerAction(period=9.0, actions=[yolo_two_bag_viewer, mediapipe_bag_viewer]),
        camera_tf_node,
    ])
