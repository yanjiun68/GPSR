"""Section: Launch ROI depth resolution and the guarded GPSR ActionQueue bridge."""

# Section: ROS launch declarations and two-node composition.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Special function: Configure Qwen grounding intake, RGB-D geometry, and opt-in execution."""
    # Section: Resolve launch substitutions once for readable node parameter maps.
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    grounding_topic = LaunchConfiguration("grounding_topic")
    mask_topic = LaunchConfiguration("mask_topic")
    auto_execute = LaunchConfiguration("auto_execute")
    default_require_mask = LaunchConfiguration("default_require_mask")
    return LaunchDescription([
        # Section: Depth must already be registered into the RGB pixels shown to Qwen.
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/camera/aligned_depth_to_color/image_raw",
        ),
        # Section: Calibrated colour intrinsics are mandatory for metric projection.
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/camera/color/camera_info",
        ),
        # Section: The Qwen wrapper publishes a stamped JSON envelope after grounding inference.
        DeclareLaunchArgument("grounding_topic", default_value="/ai/qwen/grounding_json"),
        # Section: The segmentation process publishes the selected binary mask with the same RGB stamp.
        DeclareLaunchArgument("mask_topic", default_value="/ai/qwen/selected_mask"),
        # Section: Default false publishes the queue for inspection without moving the robot.
        DeclareLaunchArgument("auto_execute", default_value="false"),
        # Section: Disable only for bbox-only integration tests; production should require segmentation.
        DeclareLaunchArgument("default_require_mask", default_value="true"),
        # Section: Perception server owns depth quality, projection, TF, and stand-off generation.
        Node(
            package="vision",
            executable="qwen_roi_depth_resolver.py",
            name="qwen_roi_depth_resolver",
            output="screen",
            parameters=[{
                "depth_topic": depth_topic,
                "camera_info_topic": camera_info_topic,
            }],
        ),
        # Section: Scheduler bridge owns typed queue construction and optional executor submission.
        Node(
            package="gpsr_task_scheduler",
            executable="qwen_visual_navigation_bridge",
            name="qwen_visual_navigation_bridge",
            output="screen",
            parameters=[{
                "grounding_topic": grounding_topic,
                "mask_topic": mask_topic,
                "auto_execute": ParameterValue(auto_execute, value_type=bool),
                "default_require_mask": ParameterValue(default_require_mask, value_type=bool),
            }],
        ),
    ])
