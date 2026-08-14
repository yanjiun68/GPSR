"""Section: Launch the standalone human identity node."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Special function: Launch identity processing with its package configuration."""
    share = Path(get_package_share_directory("vision"))
    show_gui = LaunchConfiguration("show_gui")
    return LaunchDescription(
        [
            # Section: Expose GUI visibility without requiring YAML edits.
            DeclareLaunchArgument(
                "show_gui",
                default_value="true",
                description="Show the annotated OpenCV human identity window.",
            ),
            # Section: Launch the identity node with the typed GUI override.
            Node(
                package="vision",
                executable="human_identity_node",
                name="human_identity_node",
                output="screen",
                parameters=[
                    str(share / "config" / "human_identity.yaml"),
                    {"show_gui": ParameterValue(show_gui, value_type=bool)},
                ],
            )
        ]
    )
