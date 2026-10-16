"""Launch the RoboCup@Home 2025 training world with the project robot.

The shared gz_sim arena launcher owns Gazebo, robot spawning, state publishing,
ROS/Gazebo bridges, and RGB-D point-cloud correction. This wrapper supplies the
2025 world and its entrance-relative spawn defaults.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable


def generate_launch_description():
    arena_share = get_package_share_directory('robocup_home_2025_sim')
    gz_sim_share = get_package_share_directory('gz_sim')

    arena = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_share, 'launch', 'robocup_arena.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'door_x': '',
            'door_y': '',
            'door_yaw': '',
            'start_distance': LaunchConfiguration('start_distance'),
            'start_side': LaunchConfiguration('start_side'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            'spawn_z': LaunchConfiguration('spawn_z'),
            'headless': LaunchConfiguration('headless'),
            'enable_camera': LaunchConfiguration('enable_camera'),
            'washer_panel': 'false',
        }.items(),
    )

    # Make installed model://robocup25_* resources discoverable by both
    # Ignition Gazebo and Gazebo Sim when the world is launched directly.
    resource_dir = os.path.join(arena_share, 'worlds', 'models')

    return LaunchDescription([
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[resource_dir, ':', EnvironmentVariable(
                'IGN_GAZEBO_RESOURCE_PATH', default_value='')],
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[resource_dir, ':', EnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH', default_value='')],
        ),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                arena_share, 'worlds', 'robocup_home_2025_vision.sdf'
            ),
            description='Path to the RoboCup@Home 2025 SDF world',
        ),
        DeclareLaunchArgument(
            'start_distance',
            default_value='0.4',
            description='Distance from the entrance plane to base_footprint (m)',
        ),
        DeclareLaunchArgument(
            'start_side',
            default_value='outside',
            description='Spawn outside or inside the entrance; robot faces inward',
        ),
        DeclareLaunchArgument(
            'spawn_x',
            default_value='-2.06',
            description='Optional fixed world X; set with spawn_y and spawn_yaw',
        ),
        DeclareLaunchArgument(
            'spawn_y',
            default_value='-0.72',
            description='Optional fixed world Y; set with spawn_x and spawn_yaw',
        ),
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value='0.0',
            description='Optional fixed yaw; set with spawn_x and spawn_y',
        ),
        DeclareLaunchArgument('spawn_z', default_value='0.15'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument(
            'enable_camera',
            default_value='true',
            description='Enable RGB-D rendering and corrected ROS point cloud',
        ),
        arena,
    ])
