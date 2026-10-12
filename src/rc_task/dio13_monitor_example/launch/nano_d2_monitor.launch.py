from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    """Compatibility-friendly named launch entry point for the Nano D2 stack."""
    argument_names_and_defaults = [
        ('port', '/dev/ttyUSB0'),
        ('baud_rate', '115200'),
        ('publish_only_on_change', 'false'),
        ('read_period_sec', '0.02'),
        ('reconnect_period_sec', '2.0'),
        ('input_topic', '/nano_d2_grounded'),
        ('text_topic', '/nano_d2_state'),
        ('event_topic', '/nano_d2_event'),
        ('latch_grounded_event', 'true'),
        ('periodic_status_sec', '2.0'),
    ]
    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=default)
            for name, default in argument_names_and_defaults
        ],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('dio13_monitor_example'),
                    'launch',
                    'dio13_monitor.launch.py',
                )
            ),
            launch_arguments={
                name: LaunchConfiguration(name)
                for name, _ in argument_names_and_defaults
            }.items(),
        )
    ])
