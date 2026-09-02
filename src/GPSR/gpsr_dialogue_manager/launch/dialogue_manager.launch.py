from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = str(
        Path(get_package_share_directory("gpsr_dialogue_manager"))
        / "config"
        / "dialogue.yaml"
    )

    return LaunchDescription([
        Node(
            package="gpsr_dialogue_manager",
            executable="dialogue_manager_node",
            name="gpsr_dialogue_manager",
            output="screen",
            parameters=[config],
        ),
    ])
