"""Section: Integrated launch for GPSR voice command and executor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Special function: Launch TTS, STT, executor, and voice command intake."""
    use_local_voice = LaunchConfiguration("use_local_voice")
    use_voice_command = LaunchConfiguration("use_voice_command")
    start_lifecycle_manager = LaunchConfiguration("start_lifecycle_manager")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_local_voice", default_value="true", description="Start local voice TTS/STT nodes."),
            DeclareLaunchArgument("use_voice_command", default_value="true", description="Start spoken GPSR command intake."),
            DeclareLaunchArgument("use_llm_parser", default_value="false", description="Use optional /llm parser service if available."),
            DeclareLaunchArgument(
                "start_lifecycle_manager",
                default_value="true",
                description="Start the central manager required by every GPSR ActionStep.",
            ),
            Node(package="voice", executable="tts_node.py", name="voice_tts", output="screen", condition=IfCondition(use_local_voice)),
            Node(package="voice", executable="stt_node.py", name="voice_stt", output="screen", condition=IfCondition(use_local_voice)),
            # Section: Every state-machine dispatch uses this manager, including persistent-resource skills.
            Node(
                package="robot_yasmin_tasks",
                executable="gpsr_lifecycle_manager",
                name="gpsr_lifecycle_manager",
                output="screen",
                condition=IfCondition(start_lifecycle_manager),
            ),
            # Section: Transient skill nodes are instantiated UNCONFIGURED by gpsr_lifecycle_skills.launch.py.
            Node(package="robot_yasmin_tasks", executable="gpsr_executor", name="gpsr_executor", output="screen"),
            Node(
                package="robot_yasmin_tasks",
                executable="gpsr_voice_command",
                name="gpsr_voice_command",
                output="screen",
                condition=IfCondition(use_voice_command),
            ),
        ]
    )
