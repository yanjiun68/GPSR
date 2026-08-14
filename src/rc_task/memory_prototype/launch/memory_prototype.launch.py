from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    json_path = LaunchConfiguration("json_path")
    appearance_topic = LaunchConfiguration("appearance_topic")
    enable_json_persistence = LaunchConfiguration("enable_json_persistence")

    return LaunchDescription(
        [
            DeclareLaunchArgument("json_path", default_value="~/.ros/rc_task/guest_memory.json"),
            DeclareLaunchArgument("appearance_topic", default_value="/appearance/current_estimate"),
            DeclareLaunchArgument("enable_json_persistence", default_value="true"),
            Node(
                package="memory_prototype",
                executable="guest_memory_node",
                name="guest_memory_node",
                output="screen",
                parameters=[
                    {
                        "json_path": json_path,
                        "appearance_topic": appearance_topic,
                        "enable_json_persistence": enable_json_persistence,
                    }
                ],
            ),
            Node(
                package="memory_prototype",
                executable="introduction_service_node",
                name="introduction_service_node",
                output="screen",
            ),
        ]
    )

