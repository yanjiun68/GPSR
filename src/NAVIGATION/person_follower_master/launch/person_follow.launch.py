"""Launch the full single-master person-following stack.

Layers (composed with TimerAction to allow driver/camera/vision warm-up):
  t=0  drivers_only (lidar + base + odom + TF) + RealSense D435i
       (+ optional static map→odom TF)
  t=3  vision_node   (YOLO11n, CPU, bbox-enriched detections)
  t=5  reid_enricher_node
  t=6  master_tracker_node + follow_controller_node
  optional RViz with follow.rviz
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("person_follower_master")
    follow_params = os.path.join(pkg_share, "config", "follow_params.yaml")
    follow_rviz = os.path.join(pkg_share, "config", "follow.rviz")

    # --- Launch args ---
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    rviz = LaunchConfiguration("rviz", default="true")
    publish_map_tf = LaunchConfiguration("publish_map_tf", default="false")
    camera_enable = LaunchConfiguration("camera", default="true")
    yolo_device = LaunchConfiguration("yolo_device", default="cuda")
    yolo_stride = LaunchConfiguration("yolo_stride", default="1")
    use_external_drivers = LaunchConfiguration("use_external_drivers", default="false")
    auto_enroll = LaunchConfiguration("auto_enroll", default="true")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic", default="/cmd_vel")


    # --- Drivers (lidar + base + odom + TF) ---
    drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("my_robot_nav"),
                "launch",
                "drivers_only.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "enable_breadcrumb_teach_repeat": "false",
        }.items(),
        condition=UnlessCondition(use_external_drivers)
    )

    # --- RealSense camera (aligned depth, 640x480x30) ---
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch",
                "rs_launch.py",
            ])
        ]),
        launch_arguments={
            "align_depth.enable": "true",
            "rgb_camera.profile": "640x480x30",
            "depth_module.profile": "640x480x30",
            "pointcloud.enable": "false",
        }.items(),
        condition=UnlessCondition(use_external_drivers)

    )

    # --- Optional map→odom identity (so RViz's default map fixed frame works) ---
    static_map_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_static",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        condition=IfCondition(publish_map_tf),
    )

    # --- vision_node from the existing vision package ---
    vision_node = Node(
        package="vision",
        executable="vision_node.py",
        name="vision_node",
        output="screen",
        parameters=[{
            "rgb_topic": "/camera/camera/color/image_raw",
            "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "camera_frame": "camera_color_optical_frame",
            "global_frame": "base_footprint",
            "yolo_device": yolo_device,
            "yolo_stride": yolo_stride,
            "yolo_conf_thresh": 0.35,
            "debug_mode": False,
        }],
    )

    # --- Follow-stack nodes ---
    reid_enricher = Node(
        package="person_follower_master",
        executable="reid_enricher_node.py",
        name="reid_enricher_node",
        output="screen",
        parameters=[follow_params],
    )
    master_tracker = Node(
        package="person_follower_master",
        executable="master_tracker_node.py",
        name="master_tracker_node",
        output="screen",
        parameters=[follow_params, {"auto_enroll_enabled": auto_enroll}],
    )
    follow_controller = Node(
        package="person_follower_master",
        executable="follow_controller_node.py",
        name="follow_controller_node",
        output="screen",
        parameters=[follow_params, {"cmd_vel_topic": cmd_vel_topic}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", follow_rviz],
        condition=IfCondition(rviz),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("publish_map_tf", default_value="false"),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("yolo_device", default_value="cuda"),
        DeclareLaunchArgument("yolo_stride", default_value="1"),
        DeclareLaunchArgument("use_external_drivers", default_value="false"),
        DeclareLaunchArgument(
            "auto_enroll",
            default_value="true",
            description="Allow tracker enrollment without an explicit /enroll_master request.",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/cmd_vel",
            description="Velocity output used by the direct follow controller.",
        ),

        drivers,
        realsense,
        static_map_tf,

        TimerAction(period=3.0, actions=[vision_node]),
        TimerAction(period=5.0, actions=[reid_enricher]),
        TimerAction(period=6.0, actions=[master_tracker, follow_controller]),
        TimerAction(period=2.0, actions=[rviz_node]),
    ])
