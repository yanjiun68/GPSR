#!/usr/bin/env python3

"""Section: Start the standalone speech-to-text and text-to-speech nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Build the configurable launch description for both voice nodes."""
    # Section: Declare independent switches so either audio direction can be disabled.
    start_stt = LaunchConfiguration("start_stt")
    start_tts = LaunchConfiguration("start_tts")
    enable_vad_utterance = LaunchConfiguration("enable_vad_utterance")
    whisper_model = LaunchConfiguration("whisper_model")
    whisper_language = LaunchConfiguration("whisper_language")
    whisper_device = LaunchConfiguration("whisper_device")
    piper_model_path = LaunchConfiguration("piper_model_path")
    post_tts_guard_ms = LaunchConfiguration("post_tts_guard_ms")
    vad_start_ratio = LaunchConfiguration("vad_start_ratio")
    vad_continue_ratio = LaunchConfiguration("vad_continue_ratio")
    vad_start_consecutive_frames = LaunchConfiguration("vad_start_consecutive_frames")
    vad_partial_update_ms = LaunchConfiguration("vad_partial_update_ms")
    vad_rolling_context_sec = LaunchConfiguration("vad_rolling_context_sec")
    vad_end_silence_ms = LaunchConfiguration("vad_end_silence_ms")

    # Section: Start speech recognition with its stable graph name and screen logging.
    stt_node = Node(
        package="voice",
        executable="stt_node.py",
        name="voice_stt",
        output="screen",
        condition=IfCondition(start_stt),
        # Section: Pass opt-in VAD utterance settings without changing existing default STT behavior.
        parameters=[
            {
                "enable_vad_utterance": ParameterValue(
                    enable_vad_utterance,
                    value_type=bool,
                ),
                "whisper_model": whisper_model,
                "whisper_language": whisper_language,
                "whisper_device": whisper_device,
                "post_tts_guard_ms": ParameterValue(post_tts_guard_ms, value_type=int),
                "vad_start_ratio": ParameterValue(vad_start_ratio, value_type=float),
                "vad_continue_ratio": ParameterValue(vad_continue_ratio, value_type=float),
                "vad_start_consecutive_frames": ParameterValue(vad_start_consecutive_frames, value_type=int),
                "vad_partial_update_ms": ParameterValue(
                    vad_partial_update_ms,
                    value_type=int,
                ),
                "vad_rolling_context_sec": ParameterValue(
                    vad_rolling_context_sec,
                    value_type=float,
                ),
                "vad_end_silence_ms": ParameterValue(
                    vad_end_silence_ms,
                    value_type=int,
                ),
            }
        ],
    )

    # Section: Start speech playback with its stable graph name and screen logging.
    tts_node = Node(
        package="voice",
        executable="tts_node.py",
        name="voice_tts",
        output="screen",
        condition=IfCondition(start_tts),
        parameters=[{"piper_model_path": piper_model_path}],
    )

    # Section: Return launch controls before the processes that consume them.
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_stt",
                default_value="true",
                description="Start the voice STT node.",
            ),
            DeclareLaunchArgument("whisper_model", default_value="turbo", description="OpenAI Whisper model name."),
            DeclareLaunchArgument("whisper_language", default_value="en", description="Whisper language hint."),
            DeclareLaunchArgument("whisper_device", default_value="cuda", description="Whisper device: cuda, cpu, or auto."),
            DeclareLaunchArgument("piper_model_path", default_value="~/piper_models/en_US-lessac-high.onnx", description="Piper high-quality voice model path."),
            DeclareLaunchArgument("post_tts_guard_ms", default_value="400", description="STT echo-suppression time after TTS playback."),
            DeclareLaunchArgument("vad_start_ratio", default_value="0.60", description="Minimum voiced 10 ms ratio to begin VAD."),
            DeclareLaunchArgument("vad_continue_ratio", default_value="0.30", description="Minimum voiced 10 ms ratio to continue VAD."),
            DeclareLaunchArgument("vad_start_consecutive_frames", default_value="2", description="Consecutive VAD blocks required to start an utterance."),
            DeclareLaunchArgument(
                "start_tts",
                default_value="true",
                description="Start the voice TTS node.",
            ),
            # Section: Leave continuous microphone ownership off unless the operator explicitly enables it.
            DeclareLaunchArgument(
                "enable_vad_utterance",
                default_value="false",
                description="Enable VAD-based complete-utterance transcription.",
            ),
            DeclareLaunchArgument(
                "vad_partial_update_ms",
                default_value="900",
                description="VAD transcript preview interval in milliseconds.",
            ),
            DeclareLaunchArgument(
                "vad_rolling_context_sec",
                default_value="5.0",
                description="Rolling audio context used for VAD transcript previews.",
            ),
            DeclareLaunchArgument(
                "vad_end_silence_ms",
                default_value="600",
                description="Silence required before publishing a final VAD utterance.",
            ),
            stt_node,
            tts_node,
        ]
    )
