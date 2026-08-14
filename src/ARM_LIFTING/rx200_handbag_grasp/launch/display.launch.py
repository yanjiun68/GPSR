#!/usr/bin/env python3
"""
Simple display launch file for testing URDF visualization.
Only starts robot_state_publisher + joint_state_publisher + RViz.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Arguments
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='rx200')
    robot_name = LaunchConfiguration('robot_name')

    # Package paths
    pkg_share_grasp = FindPackageShare('rx200_handbag_grasp')

    # Generate URDF from xacro
    urdf_file = PathJoinSubstitution([pkg_share_grasp, 'urdf', 'combined.urdf.xacro'])
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_file,
        ' robot_name:=', robot_name,
        ' use_world_frame:=true',
        ' show_ar_tag:=true'
    ])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str)
        }]
    )

    # Joint State Publisher GUI (for manual joint control)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share_grasp, 'rviz', 'display.rviz'])],
    )

    return LaunchDescription([
        robot_name_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
