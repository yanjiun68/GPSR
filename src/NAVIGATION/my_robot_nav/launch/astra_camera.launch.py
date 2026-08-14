import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    return LaunchDescription([
        # ---------- Launch arguments: driver namespace and normalized outputs ----------
        DeclareLaunchArgument("astra_namespace", default_value="astra"),
        DeclareLaunchArgument("rgb_out", default_value="/camera/image_raw"),
        DeclareLaunchArgument("depth_out", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("rgb_info_out", default_value="/camera/camera_info"),
        DeclareLaunchArgument("depth_info_out", default_value="/camera/depth/camera_info"),
        DeclareLaunchArgument("frame_timeout_sec", default_value="2.0"),

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
    ])
