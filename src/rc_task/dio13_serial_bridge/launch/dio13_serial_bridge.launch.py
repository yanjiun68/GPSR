from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the Arduino Nano D2 serial bridge with configurable settings."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Linux serial device connected to the Arduino.',
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Serial baud rate used by the Arduino sketch.',
        ),
        DeclareLaunchArgument(
            'publish_only_on_change',
            default_value='false',
            description='Publish only when the Nano D2 state changes.',
        ),
        Node(
            package='dio13_serial_bridge',
            executable='nano_d2_serial_publisher',
            name='nano_d2_serial_publisher',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'publish_only_on_change': LaunchConfiguration('publish_only_on_change'),
            }],
        ),
    ])
