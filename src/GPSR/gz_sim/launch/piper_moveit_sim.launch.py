"""Start MoveIt against the Piper trajectory controllers inside Gazebo Sim."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    moveit_share = get_package_share_directory('agx_arm_moveit')
    common_arguments = {
        'arm_type': 'piper',
        'effector_type': 'agx_gripper',
        'revo2_type': 'left',
        'tcp_offset': LaunchConfiguration('tcp_offset'),
        'follow': 'true',
        'feedback_topic': '/joint_states',
        'control_topic': '/joint_states',
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, 'launch', 'move_group.launch.py')),
        launch_arguments=common_arguments.items(),
    )
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, 'launch', 'moveit_rviz.launch.py')),
        condition=IfCondition(LaunchConfiguration('moveit_rviz')),
        launch_arguments={
            **common_arguments,
            'rviz_config': os.path.join(moveit_share, 'config', 'moveit.rviz'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('moveit_rviz', default_value='false'),
        DeclareLaunchArgument(
            'tcp_offset', default_value='[0.0, 0.0, 0.085, 0.0, 0.0, 0.0]'),
        Node(
            package='gz_sim', executable='piper_trajectory_action_relay.py',
            name='piper_trajectory_action_relay', output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        move_group,
        moveit_rviz,
    ])
