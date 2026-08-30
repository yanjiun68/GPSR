"""Launch the RX200 gripper guard with its calibrated parameter file."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start one companion node without launching or opening the hardware driver."""
    # Section: Resolve installed configuration so source and install layouts behave identically.
    parameters = PathJoinSubstitution(
        [FindPackageShare("rx200_gripper_guard"), "config", "gripper_guard.yaml"]
    )
    guard = Node(
        package="rx200_gripper_guard",
        executable="gripper_guard_node",
        name="rx200_gripper_guard",
        output="screen",
        parameters=[parameters],
    )
    return LaunchDescription([guard])
