"""Short entry point for the RoboCup@Home 2025 simulation world."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory('robocup_home_2025_sim')
    target = os.path.join(share, 'launch', 'robocup_home_2025.launch.py')
    return LaunchDescription([
        LogInfo(msg='Starting robocup_home_2025_sim/home25.launch.py'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(target)),
    ])
