"""Launch on-demand semantic grounding, target mesh reconstruction, and RViz visualization."""

# Section: Launch primitives expose the configuration file and optional operator visualization.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start the perception-only pipeline and optional dedicated RViz view."""
    # Section: Installed defaults keep sensor, model, and geometry thresholds together.
    default_config = PathJoinSubstitution(
        [FindPackageShare("vision"), "config", "target_grasp_geometry.yaml"]
    )
    default_rviz = PathJoinSubstitution(
        [FindPackageShare("vision"), "rviz", "target_grasp_geometry.rviz"]
    )
    config_file = LaunchConfiguration("config_file")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    # Section: The perception node publishes geometry only and owns no Nav2, MoveIt, or arm client.
    geometry_node = Node(
        package="vision",
        executable="target_grasp_geometry_node.py",
        name="target_grasp_geometry",
        output="screen",
        parameters=[config_file],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="target_grasp_geometry_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(start_rviz),
    )
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        geometry_node,
        rviz_node,
    ])
