from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("fixed_information_dialogue"))
        / "config" / "fixed_information.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        Node(
            package="fixed_information_dialogue",
            executable="fixed_information_node",
            name="fixed_information_dialogue",
            output="screen",
            parameters=[{"config_file": LaunchConfiguration("config_file")}],
        ),
    ])
