"""Launch the full phase-one Carry My Luggage YASMIN state machine."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start the phase-one state-machine executable."""
    # Section: External robot bringup remains separately owned to prevent duplicate hardware nodes.
    state_machine = Node(
        package="carry_my_luggage_yasmin",
        executable="phase1_sm",
        name="carry_my_luggage_phase1_yasmin",
        output="screen",
    )
    return LaunchDescription([state_machine])
