from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('person_following')

    follow_profile = LaunchConfiguration('follow_profile')
    track_frame = LaunchConfiguration('track_frame')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    camera_frame = LaunchConfiguration('camera_frame')
    yolo_model_path = LaunchConfiguration('yolo_model_path')
    yolo_device = LaunchConfiguration('yolo_device')
    enable_gui = LaunchConfiguration('enable_gui')
    publish_cam_markers = LaunchConfiguration('publish_cam_markers')
    scan_topic = LaunchConfiguration('scan_topic')
    publish_lidar_markers = LaunchConfiguration('publish_lidar_markers')
    enable_tracker = LaunchConfiguration('enable_tracker')
    reid_model_path = LaunchConfiguration('reid_model_path')
    reid_providers = LaunchConfiguration('reid_providers')
    retinaface_onnx_path = LaunchConfiguration('retinaface_onnx_path')
    retinaface_providers = LaunchConfiguration('retinaface_providers')
    runtime_timing_debug = LaunchConfiguration('runtime_timing_debug')

    vision_params = PathJoinSubstitution([pkg_share, 'config', 'profiles', follow_profile, 'vision.yaml'])
    lidar_params = PathJoinSubstitution([pkg_share, 'config', 'profiles', follow_profile, 'lidar_candidates.yaml'])
    tracker_params = PathJoinSubstitution([pkg_share, 'config', 'profiles', follow_profile, 'tracker.yaml'])

    vision_node = Node(
        package='person_following',
        executable='vision_node.py',
        name='vision_node',
        output='screen',
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

    person_track_manager = Node(
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
                'reid_model_path': reid_model_path,
                'reid_providers': reid_providers,
                'runtime_timing_debug': ParameterValue(runtime_timing_debug, value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('follow_profile', default_value='competition_default'),
        DeclareLaunchArgument('track_frame', default_value='base_footprint', description='Common robot frame for perception outputs'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera/color/camera_info'),
        DeclareLaunchArgument('camera_frame', default_value='camera_color_optical_frame'),
        DeclareLaunchArgument('yolo_model_path', default_value=PathJoinSubstitution([pkg_share, 'models', 'yolo11n.pt'])),
        DeclareLaunchArgument('yolo_device', default_value='auto'),
        DeclareLaunchArgument('enable_gui', default_value='true'),
        DeclareLaunchArgument('publish_cam_markers', default_value='true'),
        DeclareLaunchArgument('retinaface_onnx_path', default_value=PathJoinSubstitution([pkg_share, 'models', 'RetinaFace_int.onnx'])),
        DeclareLaunchArgument('retinaface_providers', default_value='CUDAExecutionProvider,CPUExecutionProvider'),
        DeclareLaunchArgument('runtime_timing_debug', default_value='false'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('publish_lidar_markers', default_value='true'),
        DeclareLaunchArgument('enable_tracker', default_value='true'),
        DeclareLaunchArgument('reid_model_path', default_value=PathJoinSubstitution([pkg_share, 'models', 'osnet_x0_5.onnx'])),
        DeclareLaunchArgument('reid_providers', default_value='CUDAExecutionProvider,CPUExecutionProvider'),
        vision_node,
        lidar_people_candidates,
        person_track_manager,
    ])
