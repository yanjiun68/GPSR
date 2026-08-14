from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_test_profile_publisher = LaunchConfiguration('use_test_profile_publisher')
    use_json_renderer = LaunchConfiguration('use_json_renderer')
    config_file = os.path.join(
        get_package_share_directory('my_tts_system'),
        'config',
        'tts_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_test_profile_publisher', default_value='false'),
        DeclareLaunchArgument('use_json_renderer', default_value='false'),
        Node(
            package='my_tts_system',
            executable='guest_text_renderer',
            name='guest_text_renderer',
            output='screen',
            parameters=[config_file],
            condition=UnlessCondition(use_json_renderer),
        ),
        Node(
            package='my_tts_system',
            executable='json_guest_text_renderer',
            name='json_guest_text_renderer',
            output='screen',
            parameters=[config_file],
            condition=IfCondition(use_json_renderer),
        ),
        Node(
            package='my_tts_system',
            executable='piper_tts_node',
            name='piper_tts_node',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='my_tts_system',
            executable='test_guest_profile_publisher',
            name='test_guest_profile_publisher',
            output='screen',
            parameters=[config_file],
            condition=IfCondition(use_test_profile_publisher),
        )
    ])
