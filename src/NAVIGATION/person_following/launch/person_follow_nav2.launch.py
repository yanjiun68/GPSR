import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    person_following_share = get_package_share_directory('person_following')
    default_retinaface_onnx = os.path.join(person_following_share, 'models', 'RetinaFace_int.onnx')
    default_osnet_onnx = os.path.join(person_following_share, 'models', 'osnet_x0_5.onnx')
    default_yolo_pt = os.path.join(person_following_share, 'models', 'yolo11n.pt')
    default_boundary_config = os.path.join(person_following_share, 'config', 'boundary_zone_checker.yaml')

    follow_profile = LaunchConfiguration('follow_profile')
    track_frame = LaunchConfiguration('track_frame')
    global_frame = LaunchConfiguration('global_frame')
    base_frame = LaunchConfiguration('base_frame')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    camera_frame = LaunchConfiguration('camera_frame')
    scan_topic = LaunchConfiguration('scan_topic')
    yolo_model_path = LaunchConfiguration('yolo_model_path')
    yolo_device = LaunchConfiguration('yolo_device')
    enable_gui = LaunchConfiguration('enable_gui')
    publish_cam_markers = LaunchConfiguration('publish_cam_markers')
    publish_lidar_markers = LaunchConfiguration('publish_lidar_markers')
    retinaface_onnx_path = LaunchConfiguration('retinaface_onnx_path')
    retinaface_providers = LaunchConfiguration('retinaface_providers')
    enable_tracker = LaunchConfiguration('enable_tracker')
    reid_model_path = LaunchConfiguration('reid_model_path')
    reid_providers = LaunchConfiguration('reid_providers')
    target_facing_topic = LaunchConfiguration('target_facing_topic')
    nav2_action_name = LaunchConfiguration('nav2_action_name')
    cmd_vel_raw_topic = LaunchConfiguration('cmd_vel_raw_topic')
    default_follow_mode = LaunchConfiguration('default_follow_mode')
    follow_mode_topic = LaunchConfiguration('follow_mode_topic')
    follow_enable_topic = LaunchConfiguration('follow_enable_topic')
    stop_reason_topic = LaunchConfiguration('stop_reason_topic')
    session_active_topic = LaunchConfiguration('session_active_topic')
    boundary_config_file = LaunchConfiguration('boundary_config_file')
    boundary_clicked_point_topic = LaunchConfiguration('boundary_clicked_point_topic')
    boundary_enable_rviz_override = LaunchConfiguration('boundary_enable_rviz_override')
    boundary_rviz_target_zone = LaunchConfiguration('boundary_rviz_target_zone')
    runtime_timing_debug = LaunchConfiguration('runtime_timing_debug')
    enable_follow_ready_voice_server = LaunchConfiguration('enable_follow_ready_voice_server')
    follow_ready_piper_model_path = LaunchConfiguration('follow_ready_piper_model_path')
    follow_ready_piper_bin = LaunchConfiguration('follow_ready_piper_bin')
    follow_ready_aplay_bin = LaunchConfiguration('follow_ready_aplay_bin')

    vision_params = PathJoinSubstitution([person_following_share, 'config', 'profiles', follow_profile, 'vision.yaml'])
    lidar_params = PathJoinSubstitution([person_following_share, 'config', 'profiles', follow_profile, 'lidar_candidates.yaml'])
    tracker_params = PathJoinSubstitution([person_following_share, 'config', 'profiles', follow_profile, 'tracker.yaml'])
    target_lock_params = PathJoinSubstitution([person_following_share, 'config', 'profiles', follow_profile, 'target_lock.yaml'])
    behavior_params = PathJoinSubstitution([person_following_share, 'config', 'profiles', follow_profile, 'behavior.yaml'])
    mapped_follow_params = PathJoinSubstitution([person_following_share, 'config', 'profiles', follow_profile, 'mapped_follow.yaml'])
    unmapped_follow_params = PathJoinSubstitution([person_following_share, 'config', 'profiles', follow_profile, 'unmapped_follow.yaml'])
    safety_params = PathJoinSubstitution([person_following_share, 'config', 'profiles', follow_profile, 'safety.yaml'])
    server_params = PathJoinSubstitution([person_following_share, 'config', 'profiles', follow_profile, 'server.yaml'])
    follow_ready_voice_params = PathJoinSubstitution([person_following_share, 'config', 'profiles', follow_profile, 'follow_ready_voice.yaml'])
    boundary_checker_params = PathJoinSubstitution([person_following_share, 'config', 'profiles', follow_profile, 'boundary_zone_checker.yaml'])

    nav_backend_status_topic = '/person_follow/nav_backend_status'
    local_backend_status_topic = '/person_follow/local_backend_status'

    vision_node = Node(
        package='person_following',
        executable='vision_node.py',
        name='vision_node',
        output='screen',
        additional_env={
            'DISPLAY': os.environ.get('DISPLAY', ':0'),
            'QT_QPA_FONTDIR': '/usr/share/fonts/truetype/dejavu',
            'QT_X11_NO_MITSHM': '1',
        },
        parameters=[
            vision_params,
            {
                'rgb_topic': rgb_topic,
                'depth_topic': depth_topic,
                'camera_info_topic': camera_info_topic,
                'camera_frame': camera_frame,
                'global_frame': track_frame,
                'yolo_model_path': yolo_model_path,
                'yolo_device': yolo_device,
                'enable_gui': ParameterValue(enable_gui, value_type=bool),
                'publish_markers': ParameterValue(publish_cam_markers, value_type=bool),
                'retinaface_onnx_path': retinaface_onnx_path,
                'retinaface_providers': retinaface_providers,
                'runtime_timing_debug': ParameterValue(runtime_timing_debug, value_type=bool),
            },
        ],
    )

    lidar_people_candidates = Node(
        package='person_following',
        executable='follow_person.py',
        name='lidar_people_candidates',
        output='screen',
        parameters=[
            lidar_params,
            {
                'scan_topic': scan_topic,
                'base_frame': track_frame,
                'publish_markers': ParameterValue(publish_lidar_markers, value_type=bool),
            },
        ],
    )

    track_fusion_node = Node(
        package='person_following',
        executable='person_track_manager.py',
        name='person_track_manager',
        output='screen',
        condition=IfCondition(enable_tracker),
        parameters=[
            tracker_params,
            {
                'cam_detections_topic': '/cam_detections',
                'people_candidates_topic': '/people_candidates',
                'rgb_topic': rgb_topic,
                'track_frame': track_frame,
                'global_frame': global_frame,
                'base_frame': base_frame,
                'target_facing_topic': target_facing_topic,
                'reid_model_path': reid_model_path,
                'reid_providers': reid_providers,
                'runtime_timing_debug': ParameterValue(runtime_timing_debug, value_type=bool),
            },
        ],
    )

    target_lock_manager = Node(
        package='person_following',
        executable='target_lock_manager.py',
        name='target_lock_manager',
        output='screen',
        parameters=[
            target_lock_params,
            {
                'target_topic': '/person_target_locked',
                'target_facing_topic': target_facing_topic,
                'track_frame': track_frame,
                'global_frame': global_frame,
                'base_frame': base_frame,
            },
        ],
    )

    behavior_manager = Node(
        package='person_following',
        executable='follow_behavior_manager.py',
        name='follow_behavior_manager',
        output='screen',
        parameters=[
            behavior_params,
            {
                'target_topic': '/person_target_locked',
                'facing_topic': target_facing_topic,
                'follow_enable_topic': follow_enable_topic,
                'nav_backend_status_topic': nav_backend_status_topic,
                'local_backend_status_topic': local_backend_status_topic,
                'track_frame': track_frame,
                'global_frame': global_frame,
                'base_frame': base_frame,
            },
        ],
    )

    nav_follow_interface = Node(
        package='person_following',
        executable='nav_follow_interface.py',
        name='nav_follow_interface',
        output='screen',
        parameters=[
            mapped_follow_params,
            {
                'default_follow_mode': default_follow_mode,
                'track_frame': track_frame,
                'global_frame': global_frame,
                'base_frame': base_frame,
                'nav2_action_name': nav2_action_name,
                'cmd_vel_raw_topic': cmd_vel_raw_topic,
                'follow_enable_topic': follow_enable_topic,
                'follow_mode_topic': follow_mode_topic,
                'nav_backend_status_topic': nav_backend_status_topic,
                'local_backend_status_topic': local_backend_status_topic,
            },
        ],
    )

    local_follow_interface = Node(
        package='person_following',
        executable='local_follow_interface.py',
        name='local_follow_interface',
        output='screen',
        parameters=[
            unmapped_follow_params,
            {
                'default_follow_mode': default_follow_mode,
                'target_topic': '/person_target_locked',
                'motion_intent_topic': '/person_follow/motion_intent',
                'safety_status_topic': '/person_follow/safety_status',
                'follow_enable_topic': follow_enable_topic,
                'follow_mode_topic': follow_mode_topic,
                'cmd_vel_raw_topic': cmd_vel_raw_topic,
                'scan_topic': scan_topic,
                'depth_topic': depth_topic,
                'base_frame': base_frame,
                'track_frame': track_frame,
                'nav2_action_name': nav2_action_name,
                'local_backend_status_topic': local_backend_status_topic,
                'nav_backend_status_topic': nav_backend_status_topic,
            },
        ],
    )

    safety_supervisor = Node(
        package='person_following',
        executable='safety_supervisor.py',
        name='safety_supervisor',
        output='screen',
        parameters=[
            safety_params,
            {
                'default_follow_mode': default_follow_mode,
                'follow_enable_topic': follow_enable_topic,
                'follow_mode_topic': follow_mode_topic,
                'tracks_state_topic': '/person_tracks_state',
                'target_topic': '/person_target_locked',
                'stop_reason_topic': stop_reason_topic,
                'track_frame': track_frame,
                'global_frame': global_frame,
                'base_frame': base_frame,
                'nav_backend_status_topic': nav_backend_status_topic,
                'local_backend_status_topic': local_backend_status_topic,
            },
        ],
    )

    boundary_zone_checker = Node(
        package='person_following',
        executable='boundary_zone_checker.py',
        name='boundary_zone_checker',
        output='screen',
        parameters=[
            boundary_checker_params,
            {
                'boundary_config_file': boundary_config_file,
                'boundary_enable_rviz_override': ParameterValue(boundary_enable_rviz_override, value_type=bool),
                'boundary_clicked_point_topic': boundary_clicked_point_topic,
                'boundary_rviz_target_zone': boundary_rviz_target_zone,
            },
        ],
    )

    control_server = Node(
        package='person_following',
        executable='person_follow_server.py',
        name='person_follow_server',
        output='screen',
        parameters=[
            server_params,
            {
                'default_follow_mode': default_follow_mode,
                'follow_enable_topic': follow_enable_topic,
                'follow_mode_topic': follow_mode_topic,
                'session_active_topic': session_active_topic,
                'behavior_node_name': '/follow_behavior_manager',
            },
        ],
    )

    follow_ready_voice_server = Node(
        package='person_following',
        executable='follow_ready_voice_server.py',
        name='follow_ready_voice_server',
        output='screen',
        condition=IfCondition(enable_follow_ready_voice_server),
        parameters=[
            follow_ready_voice_params,
            {
                'session_active_topic': session_active_topic,
                'safety_status_topic': '/person_follow/safety_status',
                'target_topic': '/person_target_locked',
                'piper_model_path': follow_ready_piper_model_path,
                'piper_bin': follow_ready_piper_bin,
                'aplay_bin': follow_ready_aplay_bin,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('follow_profile', default_value='competition_default'),
        DeclareLaunchArgument('track_frame', default_value='base_footprint'),
        DeclareLaunchArgument('global_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera/color/camera_info'),
        DeclareLaunchArgument('camera_frame', default_value='camera_color_optical_frame'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('yolo_model_path', default_value=default_yolo_pt),
        DeclareLaunchArgument('yolo_device', default_value='auto'),
        DeclareLaunchArgument('enable_gui', default_value='true'),
        DeclareLaunchArgument('publish_cam_markers', default_value='true'),
        DeclareLaunchArgument('publish_lidar_markers', default_value='true'),
        DeclareLaunchArgument('retinaface_onnx_path', default_value=default_retinaface_onnx),
        DeclareLaunchArgument('retinaface_providers', default_value='CUDAExecutionProvider,CPUExecutionProvider'),
        DeclareLaunchArgument('enable_tracker', default_value='true'),
        DeclareLaunchArgument('reid_model_path', default_value=default_osnet_onnx),
        DeclareLaunchArgument('reid_providers', default_value='CUDAExecutionProvider,CPUExecutionProvider'),
        DeclareLaunchArgument('target_facing_topic', default_value='/person_target_facing_robot'),
        DeclareLaunchArgument('nav2_action_name', default_value='/navigate_to_pose'),
        DeclareLaunchArgument('cmd_vel_raw_topic', default_value='/cmd_vel_follow'),
        DeclareLaunchArgument('default_follow_mode', default_value='unmapped'),
        DeclareLaunchArgument('follow_mode_topic', default_value='/person_follow/mode'),
        DeclareLaunchArgument('follow_enable_topic', default_value='/person_follow/enable'),
        DeclareLaunchArgument('stop_reason_topic', default_value='/person_follow/stop_reason'),
        DeclareLaunchArgument('session_active_topic', default_value='/person_follow/session_active'),
        DeclareLaunchArgument('boundary_config_file', default_value=default_boundary_config),
        DeclareLaunchArgument('boundary_enable_rviz_override', default_value='false'),
        DeclareLaunchArgument('boundary_clicked_point_topic', default_value='/clicked_point'),
        DeclareLaunchArgument('boundary_rviz_target_zone', default_value=''),
        DeclareLaunchArgument('runtime_timing_debug', default_value='false'),
        DeclareLaunchArgument('enable_follow_ready_voice_server', default_value='true'),
        DeclareLaunchArgument('follow_ready_piper_model_path', default_value=''),
        DeclareLaunchArgument('follow_ready_piper_bin', default_value='piper'),
        DeclareLaunchArgument('follow_ready_aplay_bin', default_value='aplay'),
        vision_node,
        lidar_people_candidates,
        track_fusion_node,
        target_lock_manager,
        behavior_manager,
        nav_follow_interface,
        local_follow_interface,
        safety_supervisor,
        boundary_zone_checker,
        control_server,
        follow_ready_voice_server,
    ])
