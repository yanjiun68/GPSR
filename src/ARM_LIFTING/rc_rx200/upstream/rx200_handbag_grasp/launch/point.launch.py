#!/usr/bin/env python3
"""
Pointing launch entry.

This launch runs the Step 3 chair pointing sequence.
"""

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='[POINT] Step 3 selected: ready -> wait for chair 2s -> point -> sleep'),
        LogInfo(msg='[POINT] Make sure Step 1 and Step 2 are already running.'),
        Node(
            package='rx200_handbag_grasp',
            executable='step3_chair_pointing.py',
            name='step3_chair_pointing',
            output='screen',
        ),
    ])
