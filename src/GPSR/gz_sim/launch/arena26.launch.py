"""Short name for the RoboCup 2026 arena Gazebo launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory('gz_sim')
    target = os.path.join(share, 'launch', 'robocup_arena.launch.py')
    return LaunchDescription([
        LogInfo(msg='Starting gz_sim/arena26.launch.py'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(target)),
    ])
