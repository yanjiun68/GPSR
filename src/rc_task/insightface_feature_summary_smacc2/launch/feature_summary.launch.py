from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Section: launch arguments passed into both the lifecycle and child SMACC2 node.
    personal_test_mode = LaunchConfiguration('personal_test_mode')
    camera_index = LaunchConfiguration('camera_index')
    camera_backend = LaunchConfiguration('camera_backend')
    image_topic = LaunchConfiguration('image_topic')
    enable_depth = LaunchConfiguration('enable_depth')
    face_topic = LaunchConfiguration('face_topic')
    appearance_topic = LaunchConfiguration('appearance_topic')
    tts_service_name = LaunchConfiguration('tts_service_name')
    start_tts = LaunchConfiguration('start_tts')
    piper_model_path = LaunchConfiguration('piper_model_path')
    face_wait_timeout_sec = LaunchConfiguration('face_wait_timeout_sec')
    appearance_wait_timeout_sec = LaunchConfiguration('appearance_wait_timeout_sec')
    collection_duration_sec = LaunchConfiguration('collection_duration_sec')
    min_stable_samples = LaunchConfiguration('min_stable_samples')
    show_appearance_gui = LaunchConfiguration('show_appearance_gui')
    stream_info_period_sec = LaunchConfiguration('stream_info_period_sec')

    # Section: optional local TTS service node started before the state machine.
    tts_node = Node(
        package='voice',
        executable='tts_node.py',
        name='voice_tts',
        namespace='',
        output='screen',
        condition=IfCondition(start_tts),
        parameters=[
            {
                'service_name': ParameterValue(tts_service_name, value_type=str),
                'piper_model_path': ParameterValue(piper_model_path, value_type=str),
            }
        ],
    )

    # Section: lifecycle wrapper that supervises the detached SMACC2 child process.
    lifecycle_node = LifecycleNode(
        package='insightface_feature_summary_smacc2',
        executable='feature_summary_lifecycle_node',
        name='insightface_feature_summary_lifecycle',
        namespace='',
        output='screen',
        parameters=[
            {
                'personal_test_mode': ParameterValue(personal_test_mode, value_type=bool),
                'camera_index': ParameterValue(camera_index, value_type=str),
                'camera_backend': ParameterValue(camera_backend, value_type=str),
                'image_topic': ParameterValue(image_topic, value_type=str),
                'enable_depth': ParameterValue(enable_depth, value_type=bool),
                'face_topic': ParameterValue(face_topic, value_type=str),
                'appearance_topic': ParameterValue(appearance_topic, value_type=str),
                'tts_service_name': ParameterValue(tts_service_name, value_type=str),
                'face_wait_timeout_sec': ParameterValue(face_wait_timeout_sec, value_type=float),
                'appearance_wait_timeout_sec': ParameterValue(
                    appearance_wait_timeout_sec,
                    value_type=float,
                ),
                'collection_duration_sec': ParameterValue(
                    collection_duration_sec,
                    value_type=float,
                ),
                'min_stable_samples': ParameterValue(min_stable_samples, value_type=int),
                'show_appearance_gui': ParameterValue(show_appearance_gui, value_type=bool),
                'stream_info_period_sec': ParameterValue(
                    stream_info_period_sec,
                    value_type=float,
                ),
            }
        ],
    )

    # Section: Nav2 lifecycle manager autostarts the lifecycle wrapper.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_insightface_feature_summary',
        namespace='',
        output='screen',
        parameters=[
            {
                'autostart': True,
                'bond_timeout': 0.0,
                'node_names': ['insightface_feature_summary_lifecycle'],
            }
        ],
    )

    return LaunchDescription(
        [
            # Section: camera and InsightFace defaults; personal_test_mode is enabled by default.
            DeclareLaunchArgument('personal_test_mode', default_value='true'),
            DeclareLaunchArgument('camera_index', default_value='0'),
            DeclareLaunchArgument('camera_backend', default_value='realsense'),
            DeclareLaunchArgument('image_topic', default_value='/camera/camera/color/image_raw'),
            DeclareLaunchArgument('enable_depth', default_value='true'),
            DeclareLaunchArgument('face_topic', default_value='/perception/largest_face'),
            DeclareLaunchArgument('appearance_topic', default_value='/appearance/current_estimate'),
            # Section: speech defaults; start_tts controls whether this launch starts the TTS node.
            DeclareLaunchArgument('tts_service_name', default_value='/voice/tts/say'),
            DeclareLaunchArgument('start_tts', default_value='true'),
            DeclareLaunchArgument('piper_model_path', default_value=''),
            # Section: timing and stability controls for face and appearance collection.
            DeclareLaunchArgument('face_wait_timeout_sec', default_value='10.0'),
            DeclareLaunchArgument('appearance_wait_timeout_sec', default_value='10.0'),
            DeclareLaunchArgument('collection_duration_sec', default_value='2.0'),
            DeclareLaunchArgument('min_stable_samples', default_value='3'),
            DeclareLaunchArgument('show_appearance_gui', default_value='true'),
            DeclareLaunchArgument('stream_info_period_sec', default_value='1.0'),
            tts_node,
            lifecycle_node,
            lifecycle_manager,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=lifecycle_node,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason='InsightFace feature summary lifecycle node exited.'
                            )
                        )
                    ],
                )
            ),
        ]
    )
