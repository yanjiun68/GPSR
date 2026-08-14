import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def resolve_default_semantic_locations_file(pkg_share, pkg_name):
    install_file = os.path.join(pkg_share, 'config', 'semantic_locations.yaml')
    workspace_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(pkg_share)))
    )
    source_file = os.path.join(
        workspace_root,
        'src',
        pkg_name,
        'config',
        'semantic_locations.yaml',
    )

    if os.path.isfile(source_file):
        return source_file

    return install_file


def generate_launch_description():
    pkg_name = 'my_robot_nav'
    pkg_share = get_package_share_directory(pkg_name)

    default_map = os.path.join(pkg_share, 'maps', 'new_my_map.yaml')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'my_nav_minimal.rviz')
    default_semantic_locations = resolve_default_semantic_locations_file(
        pkg_share,
        pkg_name,
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_file = LaunchConfiguration('map', default=default_map)
    rviz_config_file = LaunchConfiguration(
        'rviz_config',
        default=default_rviz_config,
    )
    semantic_locations_file = LaunchConfiguration(
        'semantic_locations_file',
        default=default_semantic_locations,
    )
    capture_name = LaunchConfiguration('capture_name', default='')
    capture_description = LaunchConfiguration('capture_description', default='')
    capture_input_mode = LaunchConfiguration('capture_input_mode', default='initial_pose')
    capture_timeout_sec = LaunchConfiguration('capture_timeout_sec', default='300.0')
    capture_overwrite = LaunchConfiguration('capture_overwrite', default='false')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file,
        }],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    capture_semantic_location = Node(
        package='my_robot_nav',
        executable='capture_rviz_semantic_location.py',
        name='capture_rviz_semantic_location',
        condition=IfCondition(
            PythonExpression(["'", capture_name, "' != ''"])
        ),
        parameters=[{
            'name': capture_name,
            'description': capture_description,
            'locations_file': semantic_locations_file,
            'input_mode': capture_input_mode,
            'timeout_sec': capture_timeout_sec,
            'overwrite': capture_overwrite,
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map yaml file to display in RViz',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='RViz config file to open',
        ),
        DeclareLaunchArgument(
            'semantic_locations_file',
            default_value=default_semantic_locations,
            description='Full path to semantic locations yaml file',
        ),
        DeclareLaunchArgument(
            'capture_name',
            default_value='',
            description='Semantic location name to save. Leave empty to just open RViz',
        ),
        DeclareLaunchArgument(
            'capture_description',
            default_value='',
            description='Optional description stored with the semantic location',
        ),
        DeclareLaunchArgument(
            'capture_input_mode',
            default_value='initial_pose',
            description='Use "initial_pose" for x/y/yaw or "clicked_point" for x/y only',
        ),
        DeclareLaunchArgument(
            'capture_timeout_sec',
            default_value='300.0',
            description='Seconds to wait for an RViz click before timing out',
        ),
        DeclareLaunchArgument(
            'capture_overwrite',
            default_value='false',
            description='Overwrite an existing semantic location if true',
        ),
        map_server,
        lifecycle_manager,
        rviz,
        capture_semantic_location,
    ])
