"""Section: Integrated launch for GPSR voice command and executor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Special function: Launch TTS, STT, executor, and voice command intake."""
    use_local_voice = LaunchConfiguration("use_local_voice")
    use_voice_command = LaunchConfiguration("use_voice_command")
    command_input_timeout_sec = LaunchConfiguration("command_input_timeout_sec")
    # Section: Match the scheduler launch's explicit TTS-to-STT echo handoff.
    post_tts_guard_ms = LaunchConfiguration("post_tts_guard_ms")
    # Section: Choose the Piper ONNX voice used by standalone executor prompts.
    piper_model_path = LaunchConfiguration("piper_model_path")
    start_lifecycle_manager = LaunchConfiguration("start_lifecycle_manager")
    # Section: Standalone execution uses the same installed waypoint policies as the scheduler.
    find_person_search_config_file = PathJoinSubstitution(
        [FindPackageShare("robot_yasmin_tasks"), "config", "find_person_search.yaml"]
    )
    find_object_search_config_file = PathJoinSubstitution(
        [FindPackageShare("robot_yasmin_tasks"), "config", "find_object_search.yaml"]
    )
    return LaunchDescription(
        [
            # Section: The integrated GPSR scheduler owns voice by default; standalone use must opt in.
            DeclareLaunchArgument("use_local_voice", default_value="false", description="Start local voice TTS/STT nodes only for standalone executor use."),
            DeclareLaunchArgument("use_voice_command", default_value="true", description="Start spoken GPSR command intake."),
            DeclareLaunchArgument(
                "command_input_timeout_sec",
                default_value="12.0",
                description="Seconds to wait for the initial spoken GPSR command transcript.",
            ),
            DeclareLaunchArgument(
                "post_tts_guard_ms",
                default_value="400",
                description="Milliseconds to wait after TTS playback before GPSR starts STT.",
            ),
            # Section: Default standalone executor prompts to the installed Piper medium voice.
            DeclareLaunchArgument(
                "piper_model_path",
                default_value="~/piper_models/en_US-lessac-medium.onnx",
                description="Path to the Piper ONNX model used by executor TTS.",
            ),
            DeclareLaunchArgument("use_llm_parser", default_value="false", description="Use optional /llm parser service if available."),
            DeclareLaunchArgument(
                "start_lifecycle_manager",
                default_value="true",
                description="Start the central manager required by every GPSR ActionStep.",
            ),
            # Section: Forward the selected Piper medium model to local standalone TTS.
            Node(
                package="voice",
                executable="tts_node.py",
                name="voice_tts",
                output="screen",
                condition=IfCondition(use_local_voice),
                parameters=[{"piper_model_path": piper_model_path}],
            ),
            # Section: Apply the shared echo guard even for direct local STT topic requests.
            Node(
                package="voice",
                executable="stt_node.py",
                name="voice_stt",
                output="screen",
                condition=IfCondition(use_local_voice),
                parameters=[{
                    "post_tts_guard_ms": ParameterValue(post_tts_guard_ms, value_type=int),
                }],
            ),
            # Section: Provide deterministic runtime and configured fact answers to the dedicated skill state.
            Node(
                package="fixed_information_dialogue",
                executable="fixed_information_node",
                name="fixed_information_dialogue",
                output="screen",
            ),
            # Section: Every state-machine dispatch uses this manager, including persistent-resource skills.
            Node(
                package="robot_yasmin_tasks",
                executable="gpsr_lifecycle_manager",
                name="gpsr_lifecycle_manager",
                output="screen",
                condition=IfCondition(start_lifecycle_manager),
            ),
            # Section: Transient skill nodes are instantiated UNCONFIGURED by gpsr_lifecycle_skills.launch.py.
            Node(
                package="robot_yasmin_tasks",
                executable="gpsr_executor",
                name="gpsr_executor",
                output="screen",
                parameters=[{
                    "find_person_search_config_file": find_person_search_config_file,
                    "find_object_search_config_file": find_object_search_config_file,
                    # Section: Protect every executor dialogue exchange from final TTS audio.
                    "post_tts_guard_ms": ParameterValue(post_tts_guard_ms, value_type=int),
                }],
            ),
            Node(
                package="robot_yasmin_tasks",
                executable="gpsr_voice_command",
                name="gpsr_voice_command",
                output="screen",
                condition=IfCondition(use_voice_command),
                # Section: Forward the launch timeout as a strongly typed ROS parameter.
                parameters=[{
                    "listen_timeout_sec": ParameterValue(
                        command_input_timeout_sec,
                        value_type=float,
                    ),
                    # Section: Keep initial command STT aligned with the executor dialogue guard.
                    "post_tts_guard_ms": ParameterValue(post_tts_guard_ms, value_type=int),
                }],
            ),
        ]
    )
