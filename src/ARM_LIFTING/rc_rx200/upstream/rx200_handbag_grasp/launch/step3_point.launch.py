#!/usr/bin/env python3
"""
Step 3 launch entry for the chair pointing sequence.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_grasp = FindPackageShare('rx200_handbag_grasp')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_grasp, 'launch', 'point.launch.py'])
            ])
        )
    ])
