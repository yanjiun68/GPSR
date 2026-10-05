from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("astra_er_flow_perception")
    params = os.path.join(share, "config", "astra_registration.yaml")

    return LaunchDescription([
        Node(
            package="astra_er_flow_perception",
            executable="astra_registration_node",
            name="astra_registration_node",
            output="screen",
            parameters=[params],
        )
    ])
