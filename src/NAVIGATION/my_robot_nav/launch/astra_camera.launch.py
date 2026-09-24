import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch the Astra driver and the RoboCup topic adapter."""
    astra_launch = os.path.join(
        get_package_share_directory("astra_camera"),
        "launch",
        "astra_mini.launch.py",
    )

    astra_namespace = LaunchConfiguration("astra_namespace")
    rgb_out = LaunchConfiguration("rgb_out")
    depth_out = LaunchConfiguration("depth_out")
    rgb_info_out = LaunchConfiguration("rgb_info_out")
    depth_info_out = LaunchConfiguration("depth_info_out")
    frame_timeout_sec = LaunchConfiguration("frame_timeout_sec")
    enable_qwen = LaunchConfiguration("enable_qwen")
    qwen_query_topic = LaunchConfiguration("qwen_query_topic")
    qwen_result_topic = LaunchConfiguration("qwen_result_topic")
    qwen_annotated_image_topic = LaunchConfiguration("qwen_annotated_image_topic")
    qwen_model_path = LaunchConfiguration("qwen_model_path")
    qwen_image_size = LaunchConfiguration("qwen_image_size")
    qwen_max_new_tokens = LaunchConfiguration("qwen_max_new_tokens")
    qwen_minimum_confidence = LaunchConfiguration("qwen_minimum_confidence")
    qwen_show_gui = LaunchConfiguration("qwen_show_gui")

    return LaunchDescription([
        # ---------- Launch arguments: driver namespace and normalized outputs ----------
        DeclareLaunchArgument("astra_namespace", default_value="astra"),
        DeclareLaunchArgument("rgb_out", default_value="/camera/image_raw"),
        DeclareLaunchArgument("depth_out", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("rgb_info_out", default_value="/camera/camera_info"),
        DeclareLaunchArgument("depth_info_out", default_value="/camera/depth/camera_info"),
        DeclareLaunchArgument("frame_timeout_sec", default_value="2.0"),
        # ---------- Launch arguments: optional Qwen ROI grounding ----------
        DeclareLaunchArgument("enable_qwen", default_value="false"),
        DeclareLaunchArgument("qwen_query_topic", default_value="/astra_cam_qw/query"),
        DeclareLaunchArgument("qwen_result_topic", default_value="/astra_cam_qw/result"),
        DeclareLaunchArgument("qwen_annotated_image_topic", default_value="/astra_cam_qw/annotated_image"),
        DeclareLaunchArgument(
            "qwen_model_path",
            default_value="/home/yanjiun68/ai/models/UnifoLM-ER-1",
        ),
        DeclareLaunchArgument("qwen_image_size", default_value="448"),
        DeclareLaunchArgument("qwen_max_new_tokens", default_value="192"),
        DeclareLaunchArgument("qwen_minimum_confidence", default_value="0.35"),
        DeclareLaunchArgument("qwen_show_gui", default_value="false"),

        # ---------- Astra driver: official package launch ----------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(astra_launch),
            launch_arguments={"namespace": astra_namespace}.items(),
        ),

        # ---------- RoboCup adapter: stable task-facing camera contract ----------
        Node(
            package="my_robot_nav",
            executable="robocup_astra_camera_node.py",
            name="robocup_astra_camera_node",
            output="screen",
            parameters=[{
                "rgb_in": ["/", astra_namespace, "/color/image_raw"],
                "depth_in": ["/", astra_namespace, "/depth/image_raw"],
                "rgb_info_in": ["/", astra_namespace, "/color/camera_info"],
                "depth_info_in": ["/", astra_namespace, "/depth/camera_info"],
                "rgb_out": rgb_out,
                "depth_out": depth_out,
                "rgb_info_out": rgb_info_out,
                "depth_info_out": depth_info_out,
                "frame_timeout_sec": frame_timeout_sec,
            }],
        ),
        # ---------- Optional Qwen ROI grounding: consumes the adapter's configurable RGB output ----------
        Node(
            package="my_robot_nav",
            executable="astra_cam_qw.py",
            name="astra_cam_qw",
            output="screen",
            condition=IfCondition(enable_qwen),
            parameters=[{
                "camera_topic": rgb_out,
                "query_topic": qwen_query_topic,
                "result_topic": qwen_result_topic,
                "annotated_image_topic": qwen_annotated_image_topic,
                "model_path": qwen_model_path,
                "image_size": ParameterValue(qwen_image_size, value_type=int),
                "max_new_tokens": ParameterValue(qwen_max_new_tokens, value_type=int),
                "minimum_confidence": ParameterValue(qwen_minimum_confidence, value_type=float),
                "show_gui": ParameterValue(qwen_show_gui, value_type=bool),
            }],
        ),
    ])
