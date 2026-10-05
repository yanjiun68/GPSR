"""Section: Launch one configurable fixed-size String sample-and-hold cache."""

# Section: Package-share and launch APIs locate configuration without workspace-specific paths.
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description() -> LaunchDescription:
    """Special function: Start the reusable cache node with an overridable YAML configuration."""
    # Section: The installed example configuration owns safe capacity and freshness defaults.
    default_config = os.path.join(
        get_package_share_directory("robot_state_cache"),
        "config",
        "cache.yaml",
    )
    config_file = LaunchConfiguration("config_file")
    return LaunchDescription(
        [
            # Section: Callers can deploy independent instances by supplying another namespaced YAML file.
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="YAML file containing bounded cache topics, capacity, and timing policy.",
            ),
            Node(
                package="robot_state_cache",
                executable="robot_state_cache_node",
                name="robot_state_cache",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
