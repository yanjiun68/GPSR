"""Section: Persistent GPSR runtime with centrally managed lifecycle skills."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    use_llm_parser = LaunchConfiguration("use_llm_parser")
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

    # Section: Instantiate all transient skill nodes in UNCONFIGURED state.
    lifecycle_skills = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("putrabot_bringup"),
                "launch",
                "gpsr_lifecycle_skills.launch.py",
            ])
        ),
        launch_arguments={
            "rgb_topic": rgb_topic,
            "depth_topic": depth_topic,
            "camera_info_topic": camera_info_topic,
            "show_gui": skill_show_gui,
        }.items(),
    )

    # Section: Scheduler remains the persistent ActionQueue producer.
    scheduler = Node(
        package="gpsr_task_scheduler",
        executable="gpsr_scheduler",
        name="gpsr_task_scheduler",
        output="screen",
        parameters=[{
            "dry_run": False,
            "command_input_mode": "topic",
            "command_topic": "/gpsr/command",
            "instruction_point": instruction_point,
            "max_commands": max_commands,
            "navigate_to_instruction_point_on_start": True,
            "enable_error_recovery": ParameterValue(enable_error_recovery, value_type=bool),
            "error_recovery_max_attempts": ParameterValue(error_recovery_max_attempts, value_type=int),
            "error_recovery_retry_delay_sec": ParameterValue(error_recovery_retry_delay_sec, value_type=float),
            "executor_goal_response_timeout_sec": ParameterValue(executor_goal_response_timeout_sec, value_type=float),
            "reset_state_service": reset_state_service,
        }],
    )

    # Section: Executor, command intake, TTS, and STT remain persistent core.
    executor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("robot_yasmin_tasks"),
                "launch",
                "gpsr_executor.launch.py",
            ])
        ),
        launch_arguments={
            "use_local_voice": use_local_voice,
            "use_voice_command": use_voice_command,
            "use_llm_parser": use_llm_parser,
        }.items(),
    )

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
            description="Use the optional LLM command parser.",
        ),
        camera,
        lifecycle_skills,
        scheduler,
        executor,
    ])
