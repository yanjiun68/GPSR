"""Section: Launch Qwen ROI depth resolution against the aligned RGB-D camera."""

# Section: ROS launch declarations and node composition.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Special function: Expose sensor topics, frames, and safety thresholds as launch arguments."""
    # Section: Keep defaults aligned with the registered Astra colour-in-depth pipeline.
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    map_frame = LaunchConfiguration("map_frame")
    base_frame = LaunchConfiguration("base_frame")
    enable_tf = LaunchConfiguration("enable_tf")
    show_gui = LaunchConfiguration("show_gui")
    min_depth_m = LaunchConfiguration("min_depth_m")
    start_astra_registration = LaunchConfiguration("start_astra_registration")
    start_er_flow = LaunchConfiguration("start_er_flow")
    rgb_topic = LaunchConfiguration("rgb_topic")
    command_topic = LaunchConfiguration("command_topic")
    roi_topic = LaunchConfiguration("roi_topic")
    roi_depth_topic = LaunchConfiguration("roi_depth_topic")
    roi_point_topic = LaunchConfiguration("roi_point_topic")
    publish_selected_roi_depth = LaunchConfiguration("publish_selected_roi_depth")
    debug_image_topic = LaunchConfiguration("debug_image_topic")
    model_path = LaunchConfiguration("model_path")
    inference_rate_hz = LaunchConfiguration("inference_rate_hz")
    tracking_rate_hz = LaunchConfiguration("tracking_rate_hz")
    # Section: Astra registration remaps colour into the depth optical geometry used by ROI sampling.
    astra_registration_node = Node(
        package="astra_er_flow_perception",
        executable="astra_registration_node",
        name="astra_registration_node",
        output="screen",
        condition=IfCondition(start_astra_registration),
    )
    # Section: This is the sole model-owning node; the resolver remains GPU-free.
    er_flow_node = Node(
        package="vision",
        executable="astra_cam_qw.py",
        name="er_flow_roi_tracker",
        output="screen",
        condition=IfCondition(start_er_flow),
        parameters=[{
            "camera_topic": rgb_topic,
            "query_topic": command_topic,
            "typed_result_topic": roi_topic,
            "annotated_image_topic": debug_image_topic,
            "model_path": model_path,
            "inference_rate_hz": ParameterValue(inference_rate_hz, value_type=float),
            "tracking_rate_hz": ParameterValue(tracking_rate_hz, value_type=float),
            # Section: The same opt-in flag enables the OpenCV ROI window for local debugging.
            "show_gui": ParameterValue(show_gui, value_type=bool),
        }],
    )
    return LaunchDescription([
        # Section: Start registration by default so bbox and depth pixels always share Astra depth geometry.
        DeclareLaunchArgument("start_astra_registration", default_value="true"),
        # Section: Rate defaults separate expensive semantic acquisition from inexpensive tracking.
        DeclareLaunchArgument("start_er_flow", default_value="true"),
        DeclareLaunchArgument("rgb_topic", default_value="/camera/aligned_color_to_depth/image_raw"),
        DeclareLaunchArgument("command_topic", default_value="/vision/command"),
        DeclareLaunchArgument("roi_topic", default_value="/vision/roi/result"),
        # Section: Every valid selected bbox publishes its robust median metric depth here.
        DeclareLaunchArgument("roi_depth_topic", default_value="/vision/roi/depth"),
        # Section: The matching calibrated camera-frame point preserves source stamp and frame identity.
        DeclareLaunchArgument("roi_point_topic", default_value="/vision/roi/point"),
        DeclareLaunchArgument("publish_selected_roi_depth", default_value="true"),
        DeclareLaunchArgument("debug_image_topic", default_value="/vision/roi/debug_image"),
        DeclareLaunchArgument("model_path", default_value="/home/yanjiun68/ai/models/UnifoLM-ER-Flow"),
        DeclareLaunchArgument("inference_rate_hz", default_value="1.0"),
        DeclareLaunchArgument("tracking_rate_hz", default_value="20.0"),
        # Section: The resolver needs registered depth in the same pixel coordinate system as Qwen RGB.
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/depth/image_raw",
            description="Astra depth image whose pixel geometry owns the aligned Qwen RGB image.",
        ),
        # Section: Depth intrinsics project pixels because Astra colour is registered into the depth frame.
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/depth/camera_info",
            description="Calibrated depth CameraInfo corresponding to the aligned Astra pair.",
        ),
        # Section: Nav2 consumes only map-frame approach poses.
        DeclareLaunchArgument("map_frame", default_value="map"),
        # Section: The current robot base position determines the target-facing stand-off pose.
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        # Section: Operators can disable TF to return only camera-frame ROI depth during sensor validation.
        DeclareLaunchArgument("enable_tf", default_value="true"),
        # Section: A desktop-only debug window visualizes the scaled depth ROI selected by Qwen.
        DeclareLaunchArgument("show_gui", default_value="false"),
        # Section: Near-field experiments can lower the valid depth floor without changing the default navigation guard.
        DeclareLaunchArgument("min_depth_m", default_value="0.20"),
        astra_registration_node,
        er_flow_node,
        # Section: Start the bounded perception-only action server; it never sends motion commands itself.
        Node(
            package="vision",
            executable="qwen_roi_depth_resolver.py",
            name="qwen_roi_depth_resolver",
            output="screen",
            parameters=[{
                "depth_topic": depth_topic,
                "camera_info_topic": camera_info_topic,
                "roi_topic": roi_topic,
                "roi_depth_topic": roi_depth_topic,
                "roi_point_topic": roi_point_topic,
                "publish_selected_roi_depth": ParameterValue(publish_selected_roi_depth, value_type=bool),
                "map_frame": map_frame,
                "base_frame": base_frame,
                "enable_tf": ParameterValue(enable_tf, value_type=bool),
                "show_gui": ParameterValue(show_gui, value_type=bool),
                "min_depth_m": ParameterValue(min_depth_m, value_type=float),
            }],
        ),
    ])
