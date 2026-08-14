from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the DIO13 serial bridge with configurable serial settings."""
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
            description='Publish only when the DIO13 value changes.',
        ),
        Node(
            package='dio13_serial_bridge',
            executable='dio13_serial_publisher',
            name='dio13_serial_publisher',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'publish_only_on_change': LaunchConfiguration('publish_only_on_change'),
            }],
        ),
    ])
