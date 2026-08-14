#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Get package share dir
    pkg_robot_description = get_package_share_directory('robot_description')

    # 👉 Path to YOUR xacro file
    # If your file is in urdf/main/base.urdf.xacro, change to:
    # os.path.join(pkg_robot_description, 'urdf', 'main', 'base.urdf.xacro')
    default_model_path = os.path.join(
        pkg_robot_description,
        'urdf',
        'main',
        'base.urdf.xacro'
    )

    # Launch args
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')
    robot_state_publisher_name = LaunchConfiguration('robot_state_publisher_name')
    joint_state_publisher_name = LaunchConfiguration('joint_state_publisher_name')

    declare_model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot xacro file'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_joint_state_publisher_arg = DeclareLaunchArgument(
        name='use_joint_state_publisher',
        default_value='true',
        description='Start joint_state_publisher to publish default joint states'
    )

    declare_robot_state_publisher_name_arg = DeclareLaunchArgument(
        name='robot_state_publisher_name',
        default_value='robot_state_publisher',
        description='Node name for robot_state_publisher'
    )

    declare_joint_state_publisher_name_arg = DeclareLaunchArgument(
        name='joint_state_publisher_name',
        default_value='joint_state_publisher',
        description='Node name for joint_state_publisher'
    )

    # Run xacro → URDF
    robot_description = Command(['xacro ', model])

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=robot_state_publisher_name,
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
        output='screen'
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name=joint_state_publisher_name,
        condition=IfCondition(use_joint_state_publisher),
        output='screen'
    )

    return LaunchDescription([
        declare_model_arg,
        declare_use_sim_time_arg,
        declare_use_joint_state_publisher_arg,
        declare_robot_state_publisher_name_arg,
        declare_joint_state_publisher_name_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
    ])
