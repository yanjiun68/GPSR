"""Launch the persistent Qwen text planner with optional STT and debug inputs."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Special function: Expose safe input-mode overrides and start the Qwen model node."""
    # Section: Use the installed YAML as the common default configuration.
    default_config = PathJoinSubstitution(
        [FindPackageShare("qwen_vlm_bridge"), "config", "qwen_text_planner.yaml"]
    )

    # Section: Keep speech and debug sources independently selectable from the launch command.
    arguments = [
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("enable_stt_input", default_value="true"),
        DeclareLaunchArgument("enable_vad_input", default_value="false"),
        DeclareLaunchArgument("enable_debug_input", default_value="true"),
        DeclareLaunchArgument(
            "debug_input_topic", default_value="/ai/model/qwen_vlm/debug/input"
        ),
    ]

    # Section: The executable loads Qwen once and publishes plans without submitting robot commands.
    planner = Node(
        package="qwen_vlm_bridge",
        executable="qwen_text_planner.py",
        name="qwen_text_planner",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "enable_stt_input": LaunchConfiguration("enable_stt_input"),
                "enable_vad_input": LaunchConfiguration("enable_vad_input"),
                "enable_debug_input": LaunchConfiguration("enable_debug_input"),
                "debug_input_topic": LaunchConfiguration("debug_input_topic"),
            },
        ],
    )
    return LaunchDescription([*arguments, planner])
