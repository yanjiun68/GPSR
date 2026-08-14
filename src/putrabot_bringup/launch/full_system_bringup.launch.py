#!/usr/bin/env python3

import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _installed_package_executable(package_name: str, executable: str) -> bool:
    try:
        prefix = get_package_prefix(package_name)
    except PackageNotFoundError:
        return False
    return os.path.exists(os.path.join(prefix, 'lib', package_name, executable))


def generate_launch_description():
    static_map_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('my_robot_nav'), 'launch', 'static_map_nav.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'set_initial_pose': 'true',
            'initial_pose_x': '0.0',
            'initial_pose_y': '0.0',
            'initial_pose_yaw': '0.0',
        }.items(),
    )

    person_follow_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('person_following'), 'launch', 'person_follow_nav2.launch.py'])
        ),
        launch_arguments={
            'include_real_robot': 'false',
            'use_sim_time': 'false',
            'rgb_topic': '/camera/rx200_camera/color/image_raw',
            'depth_topic': '/camera/rx200_camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/rx200_camera/color/camera_info',
            'camera_frame': 'camera_color_optical_frame',
        }.items(),
    )

    rx200_grasp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('rx200_handbag_grasp'), 'launch', 'all_in_one.launch.py'])
        ),
        launch_arguments={
            'use_camera': 'false',
            'use_saved_tf': 'false',
            'auto_grasp': 'false',
            'show_yolo_viewer': 'false',
            'use_candidate_notifier': 'false',
            'candidate_notifier_interval_ms': '200',
            'xs_port': '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT2XP6Z8-if00-port0',
            'lift_port': 'auto',
        }.items(),
    )

    phase_supervisor = Node(
        package='putrabot_bringup',
        executable='phase_supervisor.py',
        name='phase_supervisor',
        output='screen',
    )

    return LaunchDescription([
        static_map_nav_launch,
        person_follow_launch,
        rx200_grasp_launch,
        phase_supervisor,
    ])
