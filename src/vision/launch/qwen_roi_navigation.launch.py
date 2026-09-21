"""Section: Launch Qwen ROI depth resolution against the aligned RGB-D camera."""

# Section: ROS launch declarations and node composition.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Special function: Expose sensor topics, frames, and safety thresholds as launch arguments."""
    # Section: Keep defaults aligned with the managed GPSR RealSense camera pipeline.
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    map_frame = LaunchConfiguration("map_frame")
    base_frame = LaunchConfiguration("base_frame")
    return LaunchDescription([
        # Section: The resolver needs registered depth in the same pixel coordinate system as Qwen RGB.
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/camera/aligned_depth_to_color/image_raw",
            description="Depth image registered to the RGB image supplied to Qwen.",
        ),
        # Section: Calibrated colour intrinsics project Qwen-selected pixels into metric camera coordinates.
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/camera/color/camera_info",
            description="Calibrated CameraInfo corresponding to the aligned RGB/depth pair.",
        ),
        # Section: Nav2 consumes only map-frame approach poses.
        DeclareLaunchArgument("map_frame", default_value="map"),
        # Section: The current robot base position determines the target-facing stand-off pose.
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        # Section: Start the bounded perception-only action server; it never sends motion commands itself.
        Node(
            package="vision",
            executable="qwen_roi_depth_resolver.py",
            name="qwen_roi_depth_resolver",
            output="screen",
            parameters=[{
                "depth_topic": depth_topic,
                "camera_info_topic": camera_info_topic,
                "map_frame": map_frame,
                "base_frame": base_frame,
            }],
        ),
    ])
