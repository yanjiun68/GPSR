"""Composite launch: person-follow + breadcrumb teach + home-return.

This is the production entry point for the follow-then-return-home
behaviour. It layers on top of `my_robot_nav/drivers_only.launch.py`
(lidar + base + odom + TF) with the stock breadcrumb node disabled
so we can launch it ourselves with the session parameters applied.

Composition
-----------
  t=0  drivers_only (enable_breadcrumb_teach_repeat:=false)
       + RealSense D435i
       + scan_self_filter (/scan -> /scan_filtered)
       + static map -> odom identity (optional, off by default)
       + breadcrumb_teach_repeat (params from session_params.yaml)
       + cmd_vel_mux
       + return_safety_wrapper
       + session_manager
  t=3  vision_node (YOLO11)
  t=5  reid_enricher_node
  t=6  master_tracker_node + follow_controller_node
                                     (cmd_vel_topic overridden to
                                      /cmd_vel_follow via yaml)
  t=2  RViz (optional)

cmd_vel flow
------------
   follow_controller_node  -> /cmd_vel_follow
                                              ┐
                                              │
   breadcrumb_teach_repeat -> /cmd_vel_return_raw
                                │
                                ▼
                      return_safety_wrapper
                                │
                                ▼
                        /cmd_vel_return
                                │
                                │  (mux selects by /session_manager/mode)
                                ▼
                          cmd_vel_mux  -->  /cmd_vel
                                              │
                                              ▼
                                    cmd_vel_to_targets
                                    (in drivers_only)

Operator
--------
  ros2 service call /return_home/start    std_srvs/srv/Trigger  {}
  ros2 service call /return_home/go_home  std_srvs/srv/Trigger  {}
  ros2 service call /return_home/stop     std_srvs/srv/Trigger  {}
  ros2 service call /return_home/reset    std_srvs/srv/Trigger  {}
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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("person_follower_master")
    follow_params = os.path.join(pkg_share, "config", "follow_params.yaml")
    session_params = os.path.join(pkg_share, "config", "session_params.yaml")
    follow_rviz = os.path.join(pkg_share, "config", "follow.rviz")

    # --- Launch args ---
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    rviz = LaunchConfiguration("rviz", default="true")
    publish_map_tf = LaunchConfiguration("publish_map_tf", default="false")
    camera_enable = LaunchConfiguration("camera", default="true")
    yolo_device = LaunchConfiguration("yolo_device", default="cuda")
    yolo_stride = LaunchConfiguration("yolo_stride", default="1")

    # --- Drivers (lidar + base + odom + TF) with stock breadcrumb DISABLED. ---
    # We launch our own breadcrumb below so we can apply session_params.yaml
    # (route topic, scan topic, persistent route file, conservative speeds).
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
            # Disable motor-stiction deadband. breadcrumb + follow_controller
            # are closed-loop — the deadband amplifies small yaw corrections
            # above stiction and the PD loop then oscillates at ~15 Hz while
            # near the heading target (verified with /tmp/test_deadband_oscillation.py).
            "min_effective_wz": "0.0",
        }.items(),
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
        condition=IfCondition(camera_enable),
    )

    # --- Self-filtered scan for breadcrumb and safety wrapper ---
    # Produces /scan_filtered by removing footprint-interior returns from
    # /scan. Octagonal footprint matches the one in nav2_params.yaml so
    # obstacle judgements are conservative but not over-aggressive.
    scan_self_filter = Node(
        package="my_robot_nav",
        executable="scan_self_filter.py",
        name="scan_self_filter",
        output="screen",
        parameters=[{
            "input_scan_topic": "/scan",
            "output_scan_topic": "/scan_filtered",
            "base_frame": "base_footprint",
            "footprint_polygon": [
                0.2655, -0.11,
                0.2655,  0.11,
                0.212,   0.212,
                0.11,    0.2655,
                -0.11,   0.2655,
                -0.212,  0.212,
                -0.2655, 0.11,
                -0.2655, -0.11,
                -0.212,  -0.212,
                -0.11,   -0.2655,
                0.11,    -0.2655,
                0.212,   -0.212,
            ],
            "footprint_padding": 0.03,
            "timestamp_offset_sec": 0.08,
        }],
    )

    # --- Optional map -> odom identity so RViz defaults to a 'map' fixed frame. ---
    static_map_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_static",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        condition=IfCondition(publish_map_tf),
    )

    # --- Vision pipeline ---
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
        parameters=[follow_params],
    )
    # follow_controller_node's cmd_vel_topic is overridden to /cmd_vel_follow
    # via session_params.yaml so the mux can arbitrate between follow and
    # return outputs instead of both fighting for /cmd_vel.
    follow_controller = Node(
        package="person_follower_master",
        executable="follow_controller_node.py",
        name="follow_controller_node",
        output="screen",
        parameters=[follow_params, session_params],
    )

    # --- Return-home stack ---
    breadcrumb = Node(
        package="my_robot_nav",
        executable="breadcrumb_teach_repeat.py",
        name="breadcrumb_teach_repeat",
        output="screen",
        parameters=[session_params],
    )

    cmd_vel_mux = Node(
        package="person_follower_master",
        executable="cmd_vel_mux_node.py",
        name="cmd_vel_mux",
        output="screen",
        parameters=[session_params],
    )

    return_safety_wrapper = Node(
        package="person_follower_master",
        executable="return_safety_wrapper_node.py",
        name="return_safety_wrapper",
        output="screen",
        parameters=[session_params],
    )

    session_manager = Node(
        package="person_follower_master",
        executable="session_manager_node.py",
        name="session_manager",
        output="screen",
        parameters=[session_params],
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

        # t=0 — everything that doesn't depend on the camera stream.
        drivers,
        realsense,
        scan_self_filter,
        static_map_tf,
        breadcrumb,
        cmd_vel_mux,
        return_safety_wrapper,
        session_manager,

        # Staged to let upstream topics settle — mirrors person_follow.launch.py.
        TimerAction(period=2.0, actions=[rviz_node]),
        TimerAction(period=3.0, actions=[vision_node]),
        TimerAction(period=5.0, actions=[reid_enricher]),
        TimerAction(period=6.0, actions=[master_tracker, follow_controller]),
    ])

"""
ros2 launch person_follower_master person_follow_return.launch.py \
  launch_drivers:=false \
  launch_camera:=false

  This config is for integration with other nodes. 
  """