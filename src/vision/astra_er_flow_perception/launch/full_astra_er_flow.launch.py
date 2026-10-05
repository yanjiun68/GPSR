from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("astra_er_flow_perception")
    registration_params = os.path.join(
        share, "config", "astra_registration.yaml"
    )
    tracking_params = os.path.join(
        share, "config", "er_flow_tracking.yaml"
    )

    return LaunchDescription([
        Node(
            package="astra_er_flow_perception",
            executable="astra_registration_node",
            name="astra_registration_node",
            output="screen",
            parameters=[registration_params],
        ),
        Node(
            package="astra_er_flow_perception",
            executable="er_flow_tracking_node",
            name="er_flow_tracking_node",
            output="screen",
            parameters=[tracking_params],
        ),
    ])
