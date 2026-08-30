#!/usr/bin/env python3

"""Section: Start the standalone speech-to-text and text-to-speech nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build the configurable launch description for both voice nodes."""
    # Section: Declare independent switches so either audio direction can be disabled.
    start_stt = LaunchConfiguration("start_stt")
    start_tts = LaunchConfiguration("start_tts")

    # Section: Start speech recognition with its stable graph name and screen logging.
    stt_node = Node(
        package="voice",
        executable="stt_node.py",
        name="voice_stt",
        output="screen",
        condition=IfCondition(start_stt),
    )

    # Section: Start speech playback with its stable graph name and screen logging.
    tts_node = Node(
        package="voice",
        executable="tts_node.py",
        name="voice_tts",
        output="screen",
        condition=IfCondition(start_tts),
    )

    # Section: Return launch controls before the processes that consume them.
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_stt",
                default_value="true",
                description="Start the voice STT node.",
            ),
            DeclareLaunchArgument(
                "start_tts",
                default_value="true",
                description="Start the voice TTS node.",
            ),
            stt_node,
            tts_node,
        ]
    )
