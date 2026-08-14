from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('test_tools'),
                'config',
                'test_tools.yaml',
            ]),
            description='Optional YAML parameter file for the test publisher and subscriber.',
        ),
        Node(
            package='test_tools',
            executable='test_publisher',
            name='test_publisher',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='test_tools',
            executable='test_subscriber',
            name='test_subscriber',
            output='screen',
            parameters=[params_file],
        ),
    ])
