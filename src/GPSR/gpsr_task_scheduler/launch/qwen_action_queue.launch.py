"""Section: Launch the isolated Qwen JSON-to-ActionQueue validation bridge."""

# Section: Launch primitives and ROS node construction.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Special function: Build a safe inspection-first Qwen queue launch."""
    # Section: Keep the Qwen input and typed queue topics separate from local GPSR command input.
    input_topic = LaunchConfiguration("input_topic")
    queue_topic = LaunchConfiguration("queue_topic")
    status_topic = LaunchConfiguration("status_topic")
    command_result_topic = LaunchConfiguration("command_result_topic")
    executor_action = LaunchConfiguration("executor_action")
    scheduler_state_topic = LaunchConfiguration("scheduler_state_topic")
    auto_execute = LaunchConfiguration("auto_execute")
    require_scheduler_idle = LaunchConfiguration("require_scheduler_idle")

    # Section: Declare the bridge's explicit ownership and safety controls.
    return LaunchDescription([
        DeclareLaunchArgument(
            "input_topic",
            default_value="/ai/qwen/action_queue_json",
            description="std_msgs/String topic containing Qwen ActionQueue JSON.",
        ),
        DeclareLaunchArgument(
            "queue_topic",
            default_value="/gpsr/qwen_action_queue",
            description="Typed robot_ai_interfaces/ActionQueue inspection topic.",
        ),
        DeclareLaunchArgument(
            "status_topic",
            default_value="/gpsr/qwen_action_queue_status",
            description="Bridge validation and execution status topic.",
        ),
        DeclareLaunchArgument(
            "command_result_topic",
            default_value="/gpsr/command_result",
            description="Command result topic used to release the local voice transaction.",
        ),
        DeclareLaunchArgument(
            "executor_action",
            default_value="/execute_action_queue",
            description="Typed ActionQueue executor action endpoint.",
        ),
        DeclareLaunchArgument(
            "scheduler_state_topic",
            default_value="/gpsr/state",
            description="Scheduler state topic used for execution ownership checks.",
        ),
        DeclareLaunchArgument(
            "auto_execute",
            default_value="false",
            description="Execute validated Qwen queues; false publishes for inspection only.",
        ),
        DeclareLaunchArgument(
            "require_scheduler_idle",
            default_value="true",
            description="Require WAITING_FOR_COMMAND before Qwen execution.",
        ),
        Node(
            package="gpsr_task_scheduler",
            executable="qwen_action_queue_bridge",
            name="qwen_action_queue_bridge",
            output="screen",
            parameters=[{
                "input_topic": input_topic,
                "queue_topic": queue_topic,
                "status_topic": status_topic,
                "command_result_topic": command_result_topic,
                "executor_action": executor_action,
                "scheduler_state_topic": scheduler_state_topic,
                "auto_execute": ParameterValue(auto_execute, value_type=bool),
                "require_scheduler_idle": ParameterValue(require_scheduler_idle, value_type=bool),
            }],
        ),
    ])
