from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('gpsr_sys_ui')
    config = os.path.join(share, 'config', 'monitor.yaml')
    return LaunchDescription([
        Node(
            package='gpsr_sys_ui',
            executable='monitor_core',
            name='gpsr_sys_ui_core',
            output='screen',
            parameters=[config],
        ),
    ])
