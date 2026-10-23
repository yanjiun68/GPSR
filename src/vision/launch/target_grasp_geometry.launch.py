"""Launch on-demand semantic grounding, target mesh reconstruction, and RViz visualization."""

# Section: Launch primitives expose the configuration file and optional operator visualization.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start the perception-only pipeline and optional dedicated RViz view."""
    # Section: Installed defaults keep sensor, model, and geometry thresholds together.
    default_config = PathJoinSubstitution(
        [FindPackageShare("vision"), "config", "target_grasp_geometry.yaml"]
    )
    default_rviz = PathJoinSubstitution(
        [FindPackageShare("vision"), "rviz", "target_grasp_geometry.rviz"]
    )
    config_file = LaunchConfiguration("config_file")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    start_er_flow = LaunchConfiguration("start_er_flow")
    rgb_topic = LaunchConfiguration("rgb_topic")
    target_topic = LaunchConfiguration("target_topic")
    roi_topic = LaunchConfiguration("roi_topic")
    model_path = LaunchConfiguration("model_path")
    inference_rate_hz = LaunchConfiguration("inference_rate_hz")
    tracking_rate_hz = LaunchConfiguration("tracking_rate_hz")
    show_gui = LaunchConfiguration("show_gui")

    # Section: A single shared model process publishes validated ROI updates for grasp geometry.
    er_flow_node = Node(
        package="vision",
        executable="astra_cam_qw.py",
        name="er_flow_roi_tracker",
        output="screen",
        condition=IfCondition(start_er_flow),
        parameters=[{
            "camera_topic": rgb_topic,
            "query_topic": target_topic,
            "typed_result_topic": roi_topic,
            "annotated_image_topic": "/vision/roi/debug_image",
            "model_path": model_path,
            "inference_rate_hz": ParameterValue(inference_rate_hz, value_type=float),
            "tracking_rate_hz": ParameterValue(tracking_rate_hz, value_type=float),
            # Section: HighGUI remains opt-in for desktop debugging and disabled on headless robots.
            "show_gui": ParameterValue(show_gui, value_type=bool),
        }],
    )

    # Section: The perception node publishes geometry only and owns no Nav2, MoveIt, or arm client.
    geometry_node = Node(
        package="vision",
        executable="target_grasp_geometry_node.py",
        name="target_grasp_geometry",
        output="screen",
        parameters=[config_file, {
            "target_topic": target_topic,
            "rgb_topic": rgb_topic,
            "roi_topic": roi_topic,
            # Section: The shared ER-Flow process owns Qwen, preventing a duplicate VRAM allocation here.
            "qwen_enabled": False,
        }],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="target_grasp_geometry_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(start_rviz),
    )
    return LaunchDescription([
        # Section: Set this false when another launch already owns the shared ER-Flow node.
        DeclareLaunchArgument("start_er_flow", default_value="true"),
        DeclareLaunchArgument("rgb_topic", default_value="/astra/color/image_raw"),
        DeclareLaunchArgument("target_topic", default_value="/vision/target_object"),
        DeclareLaunchArgument("roi_topic", default_value="/vision/roi/result"),
        DeclareLaunchArgument("model_path", default_value="/home/yanjiun68/ai/models/UnifoLM-ER-Flow"),
        DeclareLaunchArgument("inference_rate_hz", default_value="1.0"),
        # Section: Cap shared ROI tracker updates at the 15 Hz perception budget.
        DeclareLaunchArgument("tracking_rate_hz", default_value="15.0"),
        DeclareLaunchArgument("show_gui", default_value="false"),
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        er_flow_node,
        geometry_node,
        rviz_node,
    ])
