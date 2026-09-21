"""Section: Persistent GPSR runtime with centrally managed lifecycle skills."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start persistent core and UNCONFIGURED transient skill processes."""
    # Section: Persistent runtime options forwarded into existing launches.
    start_camera = LaunchConfiguration("start_camera")
    camera_device = LaunchConfiguration("camera_device")
    camera_index = LaunchConfiguration("camera_index")
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    skill_show_gui = LaunchConfiguration("skill_show_gui")
    use_local_voice = LaunchConfiguration("use_local_voice")
    use_voice_command = LaunchConfiguration("use_voice_command")
    instruction_point = LaunchConfiguration("instruction_point")
    max_commands = LaunchConfiguration("max_commands")
    enable_error_recovery = LaunchConfiguration("enable_error_recovery")
    error_recovery_max_attempts = LaunchConfiguration("error_recovery_max_attempts")
    error_recovery_retry_delay_sec = LaunchConfiguration("error_recovery_retry_delay_sec")
    executor_goal_response_timeout_sec = LaunchConfiguration("executor_goal_response_timeout_sec")
    reset_state_service = LaunchConfiguration("reset_state_service")

    # Section: Camera hardware remains persistent and outside lifecycle skill ownership.
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("vision"), "launch", "external_camera.launch.py"]
            )
        ),
        condition=IfCondition(start_camera),
        launch_arguments={
            "camera_device": camera_device,
            "camera_index": camera_index,
            "image_topic": rgb_topic,
        }.items(),
    )

    # Section: Reuse the complete scheduler stack so registered person skills are present.
    scheduler = IncludeLaunchDescription(  # Keep one owner for scheduler, executor, and skill processes.
        PythonLaunchDescriptionSource(  # Resolve the authoritative integrated launch.
            PathJoinSubstitution([  # Use installed package paths on the robot.
                FindPackageShare("gpsr_task_scheduler"),  # Locate the scheduler package.
                "launch",  # Select its launch directory.
                "gpsr_scheduler.launch.py",  # Instantiate all default perception skills.
            ])  # Finish the integrated launch path.
        ),  # Finish the launch source.
        launch_arguments={  # Preserve the managed runtime's public controls.
            "dry_run": "false",  # Execute accepted commands on the robot.
            "start_executor": "true",  # Start exactly one queue executor and lifecycle manager.
            "command_input_mode": "topic",  # Receive commands through the existing topic.
            "command_topic": "/gpsr/command",  # Preserve the established command endpoint.
            "instruction_point": instruction_point,  # Retain explicit later-return configuration.
            "max_commands": max_commands,  # Preserve the mission command limit.
            "navigate_to_instruction_point_on_start": "false",  # Temporarily start at the current location.
            "rgb_topic": rgb_topic,  # Consume the persistent external camera.
            "depth_topic": depth_topic,  # Share aligned depth with perception skills.
            "camera_info_topic": camera_info_topic,  # Share camera calibration with perception skills.
            "show_gui": skill_show_gui,  # Preserve optional perception displays.
            "start_voice_tts": use_local_voice,  # Keep local speech output optional.
            "start_voice_stt": use_local_voice,  # Keep local speech recognition optional.
            "start_voice_command": use_voice_command,  # Preserve spoken command intake control.
            "enable_error_recovery": enable_error_recovery,  # Preserve post-command recovery policy.
            "error_recovery_max_attempts": error_recovery_max_attempts,  # Bound recovery attempts.
            "error_recovery_retry_delay_sec": error_recovery_retry_delay_sec,  # Preserve recovery pacing.
            "executor_goal_response_timeout_sec": executor_goal_response_timeout_sec,  # Bound goal admission.
            "reset_state_service": reset_state_service,  # Preserve the operator reset endpoint.
        }.items(),  # Forward launch substitutions without converting them to ROS parameters here.
    )  # Finish the single integrated GPSR stack.

    # Section: Launch order declares ownership but does not perform lifecycle transitions.
    return LaunchDescription([
        DeclareLaunchArgument(
            "start_camera",
            default_value="true",
            description="Start the persistent external camera adapter.",
        ),
        DeclareLaunchArgument(
            "camera_device",
            default_value="",
            description="External camera path; empty selects camera_index.",
        ),
        DeclareLaunchArgument(
            "camera_index",
            default_value="0",
            description="OpenCV index for the persistent camera.",
        ),
        DeclareLaunchArgument(
            "rgb_topic",
            default_value="/camera/image_raw",
            description="Persistent RGB topic shared by lifecycle skills.",
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/camera/aligned_depth_to_color/image_raw",
            description="Aligned depth topic used by the FIND_OBJECT lifecycle skill.",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/camera/color/camera_info",
            description="Color CameraInfo topic used by the FIND_OBJECT lifecycle skill.",
        ),
        DeclareLaunchArgument(
            "skill_show_gui",
            default_value="false",
            description="Enable ACTIVE-only perception debug windows.",
        ),
        DeclareLaunchArgument(
            "instruction_point",
            default_value="instruction_point",
            description="Named waypoint used between competition commands.",
        ),
        DeclareLaunchArgument(
            "max_commands",
            default_value="3",
            description="Maximum competition commands; zero allows unlimited commands.",
        ),
        DeclareLaunchArgument(
            "enable_error_recovery",
            default_value="true",
            description="Retry returning to the Instruction Point after scheduler ERROR.",
        ),
        DeclareLaunchArgument(
            "error_recovery_max_attempts",
            default_value="3",
            description="Maximum automatic recovery returns; zero retries indefinitely.",
        ),
        DeclareLaunchArgument(
            "error_recovery_retry_delay_sec",
            default_value="5.0",
            description="Delay between failed automatic recovery returns.",
        ),
        DeclareLaunchArgument(
            "executor_goal_response_timeout_sec",
            default_value="30.0",
            description="Seconds to wait for executor goal acceptance or rejection.",
        ),
        DeclareLaunchArgument(
            "reset_state_service",
            default_value="/gpsr/reset_state",
            description="Guarded service that re-arms recovery from scheduler ERROR.",
        ),
        DeclareLaunchArgument(
            "use_local_voice",
            default_value="true",
            description="Start persistent local TTS/STT nodes.",
        ),
        DeclareLaunchArgument(
            "use_voice_command",
            default_value="true",
            description="Start persistent spoken GPSR intake.",
        ),
        DeclareLaunchArgument(
            "use_llm_parser",
            default_value="false",
            description="Legacy compatibility argument; command parsing is deterministic.",
        ),
        camera,
        scheduler,
    ])
