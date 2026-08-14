#!/usr/bin/env python3
"""
Step 2: Perception System Launch
Start RealSense Camera + YOLO Detection + AprilTag Calibration

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
    grasp_params = PathJoinSubstitution([grasp_pkg_share, 'config', 'grasp_params.yaml'])
    tags_yaml = PathJoinSubstitution([grasp_pkg_share, 'config', 'tags.yaml'])

    # ========================================
    # Launch Arguments
    # ========================================
    use_external_drivers_arg = LaunchConfiguration('use_external_drivers')

    use_camera_arg = DeclareLaunchArgument(
        'use_camera', default_value='true',
        description='Launch RealSense camera'
    )
    use_external_drivers_decl = DeclareLaunchArgument(
        'use_external_drivers',
        default_value='false',
        description='Assume camera drivers are already launched elsewhere'
    )

    use_yolo_arg = DeclareLaunchArgument(
        'use_yolo', default_value='true',
        description='Launch YOLO object detection'
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

    show_vision_arg = DeclareLaunchArgument(
        'show_vision', default_value='false',
        description='Show YOLO detection visualization window'
    )

    model_type_arg = DeclareLaunchArgument(
        'model_type', default_value='yolo',
        description='Model type: [yolo, yoloworld, sam, nas]'
    )

    model_size_arg = DeclareLaunchArgument(
        'model_size', default_value='l',
        description='Model size: [n, s, m, l, x] - l recommended for better accuracy'
    )

    max_detections_arg = DeclareLaunchArgument(
        'max_detections', default_value='2',
        description='Maximum number of YOLO detections per frame'
    )

    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.25',
        description='YOLO confidence threshold (lower catches farther/smaller objects but adds false positives)'
    )

    use_selector_arg = DeclareLaunchArgument(
        'use_selector', default_value='true',
        description='Launch hand/finger selector (Step4) in this Step2 pipeline'
    )

    show_selector_arg = DeclareLaunchArgument(
        'show_selector', default_value='true',
        description='Show selector visualization window'
    )
    # ========================================
    use_apriltag_arg = DeclareLaunchArgument(
        'use_apriltag', default_value='false',
        description='Launch AprilTag detection (disabled by default for Eye-in-Hand mode)'
    )

    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value=PathJoinSubstitution([grasp_pkg_share, 'models', 'best_bag_handle.pt']),
        description='YOLO model file path (filename in models/ directory or absolute path)'
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
        condition=IfCondition(
            PythonExpression([
                "'",
                LaunchConfiguration('use_camera'),
                "' == 'true' and '",
                use_external_drivers_arg,
                "' != 'true'"
            ])
        )
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
                "' == 'true' and '",
                use_external_drivers_arg,
                "' != 'true'"
            ])
        )
    )

    # ========================================
    # 2. AprilTag Detection
    # ========================================
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_ros_continuous_detector_node',
        name='apriltag_node',
        remappings=[
            ('image_rect', '/camera/rx200_camera/color/image_raw'),
            ('camera_info', '/camera/rx200_camera/color/camera_info'),
            ('detections', '/apriltag/detections'),
        ],
        parameters=[tags_yaml],
        condition=IfCondition(LaunchConfiguration('use_apriltag'))
    )

    # ========================================
    # 3. YOLO Object Detection
    # ========================================
    yolo_node = Node(
        package='rx200_handbag_grasp',
        executable='yolo_handle_detector.py',
        name='yolo_detector',
        parameters=[{
            'model_path': LaunchConfiguration('yolo_model'),
            'image_topic': '/camera/rx200_camera/color/image_raw/compressed',
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'device': 'cuda',
            'model_type': LaunchConfiguration('model_type'),
            'model_size': LaunchConfiguration('model_size'),
            'max_detections': LaunchConfiguration('max_detections'),
            'one_per_class': False,
            'use_sam_segmentation': False,
            # Target classes for bag/handle detection
            'target_classes': [
                'bag handle',
                'handbag handle',
                'strap',
            ],
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_yolo'))
    )

    # ========================================
    # 3.1 YOLO Viewer (Visualization)
    # ========================================
    yolo_viewer_node = Node(
        package='rx200_handbag_grasp',
        executable='yolo_viewer.py',
        name='yolo_viewer',
        output='screen',
        condition=IfCondition(
            # Show if use_yolo AND show_vision are both true
            # (LaunchConfiguration in Python math is tricky, simpler to just check show_vision usually
            # but nicely we can just chain conditions or put it here)
            LaunchConfiguration('show_vision')
        )
    )

    # ========================================
    # 3.2 Hand-Bag Selector (Step4 integrated)
    # ========================================
    hand_bag_selector = Node(
        package='rx200_handbag_grasp',
        executable='hand_bag_selector.py',
        name='hand_bag_selector',
        output='screen',
        parameters=[{
            'image_topic': '/camera/rx200_camera/color/image_raw/compressed',
            'detections_topic': '/yolo/detections',
            'max_detection_age_sec': 1.2,
            'stability.center_tolerance_px': 80.0,
            'stability.size_ratio_tolerance': 0.35,
            'auto_lock.stable_time_sec': 2.0,
            'lock.keep_frames_on_jitter': 20,
            'selection_hold_sec': 3.0,
            'reuse_last_selected_when_missing': True,
            'show_window': False,
        }],
        condition=IfCondition(LaunchConfiguration('use_selector'))
    )

    selector_viewer_node = Node(
        package='rx200_handbag_grasp',
        executable='yolo_viewer.py',
        name='handbag_pointing_viewer',
        output='screen',
        parameters=[{
            'image_topic': '/handbag/pointing_annotated/compressed',
            'window_name': 'Step4: Auto Lock Bag Handle',
        }],
        condition=IfCondition(
            PythonExpression([
                "'",
                LaunchConfiguration('use_selector'),
                "' == 'true' and '",
                LaunchConfiguration('show_selector'),
                "' == 'true'"
            ])
        )
    )

    # ========================================
    # 4. Camera TF Publisher (Loads calibration)
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
        use_external_drivers_decl,
        use_yolo_arg,
        enable_depth_arg,
        align_depth_arg,
        rgb_profile_arg,
        depth_profile_arg,
        open_realsense_viewer_arg,
        show_vision_arg,
        model_type_arg,
        model_size_arg,
        max_detections_arg,
        confidence_threshold_arg,
        use_selector_arg,
        show_selector_arg,
        use_apriltag_arg,
        yolo_model_arg,

        # Camera first
        camera_launch,
        realsense_viewer,

        # Detection nodes (delayed for camera to initialize)
        TimerAction(
            period=5.0,
            actions=[
                apriltag_node,
            ]
        ),

        # YOLO (delayed more for model loading)
        TimerAction(
            period=8.0,
            actions=[
                yolo_node,
                hand_bag_selector,
            ]
        ),

        # Viewer (delayed slightly to ensure publisher is ready)
        TimerAction(
            period=9.0,
            actions=[
                yolo_viewer_node,
                selector_viewer_node,
            ]
        ),

        # TF Publisher
        camera_tf_node,
    ])
