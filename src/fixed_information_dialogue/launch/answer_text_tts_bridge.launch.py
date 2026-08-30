"""Section: Launch the fixed-information answer-text to TTS service bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Special function: Expose bridge interface names and start its ROS node."""
    # Section: Declare the provider topic containing only successful answer text.
    answer_text_topic_argument = DeclareLaunchArgument(
        "answer_text_topic",
        default_value="/fixed_information/answer_text",
        description="Plain fixed-information answer text input topic.",
    )

    # Section: Declare the existing tts_node.py service used for speech output.
    tts_service_name_argument = DeclareLaunchArgument(
        "tts_service_name",
        default_value="/voice/tts/say",
        description="SayText service exposed by the voice TTS node.",
    )

    # Section: Start only the one-way answer-text bridge node.
    bridge_node = Node(
        package="fixed_information_dialogue",
        executable="answer_text_tts_bridge",
        name="fixed_information_answer_text_tts_bridge",
        output="screen",
        parameters=[{
            "answer_text_topic": LaunchConfiguration("answer_text_topic"),
            "tts_service_name": LaunchConfiguration("tts_service_name"),
        }],
    )

    # Section: Return arguments before the node that consumes them.
    return LaunchDescription([
        answer_text_topic_argument,
        tts_service_name_argument,
        bridge_node,
    ])
