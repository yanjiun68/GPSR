"""Launch static-map Nav2, its diagnostic proxy, and optional diagnostic speech."""

# Section: Standard-library path construction import.
import os

# Section: ROS launch imports for composable includes and conditional nodes.
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Section: GPSR static-map integrated launch-description factory.
def generate_launch_description() -> LaunchDescription:
    """Launch existing static-map Nav2 plus the independent diagnostic and speech components."""

    navigation_share = get_package_share_directory('my_robot_nav')
    reporter_share = get_package_share_directory('nav2_diagnostic_reporter')
    default_map = os.path.join(navigation_share, 'maps', 'new_my_map.yaml')
    default_reporter_params = os.path.join(reporter_share, 'config', 'diagnostic_reporter.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    use_realsense_imu = LaunchConfiguration('use_realsense_imu')
    use_rviz = LaunchConfiguration('use_rviz')
    nav2_delay = LaunchConfiguration('nav2_delay')
    nav2_autostart = LaunchConfiguration('nav2_autostart')
    rviz_delay = LaunchConfiguration('rviz_delay')
    use_semantic_nav = LaunchConfiguration('use_semantic_nav')
    reporter_params_file = LaunchConfiguration('diagnostic_reporter_params_file')
    enable_diagnostic_speech = LaunchConfiguration('enable_diagnostic_speech')
    start_voice_tts = LaunchConfiguration('start_voice_tts')
    user_message_topic = LaunchConfiguration('diagnostic_user_message_topic')
    tts_service_name = LaunchConfiguration('tts_service_name')

    # Section: Reuse the hardware, localization, map-server, and Nav2 setup unchanged.
    static_map_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, 'launch', 'static_map_nav.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'use_realsense_imu': use_realsense_imu,
            'use_rviz': use_rviz,
            'nav2_delay': nav2_delay,
            'nav2_autostart': nav2_autostart,
            'rviz_delay': rviz_delay,
            'use_semantic_nav': use_semantic_nav,
        }.items(),
    )

    # Section: Include the standalone proxy launch instead of duplicating its node definition.
    diagnostic_reporter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(reporter_share, 'launch', 'diagnostic_reporter.launch.py')),
        launch_arguments={'params_file': reporter_params_file}.items(),
    )

    # Section: Optional bridge sends only the reporter's safe user text to voice TTS.
    diagnostic_speech_bridge = Node(
        package='nav2_diagnostic_reporter',
        executable='diagnostic_user_message_tts_bridge',
        name='diagnostic_user_message_tts_bridge',
        output='screen',
        condition=IfCondition(enable_diagnostic_speech),
        parameters=[{
            'user_message_topic': user_message_topic,
            'tts_service_name': tts_service_name,
        }],
    )

    # Section: Start a local TTS server only when one is not already supplied elsewhere.
    voice_tts_node = Node(
        package='voice',
        executable='tts_node.py',
        name='voice_tts',
        output='screen',
        condition=IfCondition(start_voice_tts),
    )

    return LaunchDescription([
        # Section: Common static-map navigation arguments exposed by this GPSR wrapper.
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('use_realsense_imu', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('nav2_delay', default_value='10.0'),
        DeclareLaunchArgument('nav2_autostart', default_value='false'),
        DeclareLaunchArgument('rviz_delay', default_value='12.0'),
        DeclareLaunchArgument(
            'use_semantic_nav', default_value='false',
            description='Disable direct semantic navigation by default so GPSR uses the diagnostic proxy.'),
        # Section: Independent reporter and speech-routing controls.
        DeclareLaunchArgument(
            'diagnostic_reporter_params_file', default_value=default_reporter_params,
            description='Diagnostic reporter parameter YAML file.'),
        DeclareLaunchArgument(
            'enable_diagnostic_speech', default_value='true',
            description='Speak /nav2_diagnostics/user_message through the configured TTS service.'),
        DeclareLaunchArgument(
            'start_voice_tts', default_value='false',
            description='Start voice tts_node.py only if no existing /voice/tts/say server is running.'),
        DeclareLaunchArgument(
            'diagnostic_user_message_topic', default_value='/nav2_diagnostics/user_message',
            description='Reporter user-message topic consumed by the diagnostic speech bridge.'),
        DeclareLaunchArgument(
            'tts_service_name', default_value='/voice/tts/say',
            description='SayText service used to speak diagnostic user messages.'),
        static_map_navigation,
        diagnostic_reporter,
        diagnostic_speech_bridge,
        voice_tts_node,
    ])
