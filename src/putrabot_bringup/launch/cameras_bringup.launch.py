#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def determine_nodes_to_launch(context):
    config_file_path = LaunchConfiguration("config_file").perform(context)
    robot_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        launch_arguments={
            "camera_namespace": "robot_camera",
            "config_file": config_file_path,
        }.items(),
    )

    arm_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        launch_arguments={
            "camera_namespace": "arm_camera",
            "config_file": config_file_path,
        }.items(),
    )
    nodename_to_node = {
        '/robot_camera': robot_camera_launch,
        '/arm_camera': arm_camera_launch,
    }

    nodes = []
    with open(config_file_path, 'r') as f:
        content = f.read()
        for map in nodename_to_node:
            if map in content:
                nodes.append(nodename_to_node[map])
    return nodes

def generate_launch_description():
    return LaunchDescription(
        [
             DeclareLaunchArgument(
                "config_file",
                default_value=os.path.join(
                    get_package_share_directory("putrabot_bringup"),
                    "configs",
                    "cameras.yaml",
                ),
            ),
            OpaqueFunction(function=determine_nodes_to_launch),
        ]
    )
