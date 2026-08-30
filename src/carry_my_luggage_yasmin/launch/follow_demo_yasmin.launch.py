"""Launch the streamlined Carry My Luggage YASMIN follow demo."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start the follow-demo state-machine executable."""
    # Section: External robot bringup remains separately owned to prevent duplicate hardware nodes.
    state_machine = Node(
        package="carry_my_luggage_yasmin",
        executable="follow_demo_sm",
        name="carry_my_luggage_follow_demo_yasmin",
        output="screen",
    )
    return LaunchDescription([state_machine])
