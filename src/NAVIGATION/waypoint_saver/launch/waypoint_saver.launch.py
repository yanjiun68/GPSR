"""Section: Standalone launch for the hardware-independent waypoint saver service node."""

# Section: Import launch arguments, substitutions, and ROS node actions.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Special function: Declare every saver setting and start only the waypoint saver node."""
    # Section: Start the service with configurable frames, lookup timeout, freshness, and clock.
    waypoint_saver_node = Node(
        package="waypoint_saver",
        executable="waypoint_saver_server.py",
        name="waypoint_saver_server",
        output="screen",
        parameters=[{
            "service_name": LaunchConfiguration("service_name"),
            "target_frame": LaunchConfiguration("target_frame"),
            "robot_frame": LaunchConfiguration("robot_frame"),
            "transform_timeout_sec": ParameterValue(
                LaunchConfiguration("transform_timeout_sec"),
                value_type=float,
            ),
            "max_transform_age_sec": ParameterValue(
                LaunchConfiguration("max_transform_age_sec"),
                value_type=float,
            ),
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"),
                value_type=bool,
            ),
        }],
    )

    # Section: Return the public launch contract before its single consumer node.
    return LaunchDescription([
        DeclareLaunchArgument(
            "service_name",
            default_value="/waypoint_saver/save_current",
            description="SaveWaypoint service name.",
        ),
        DeclareLaunchArgument(
            "target_frame",
            default_value="map",
            description="TF frame in which saved poses are expressed.",
        ),
        DeclareLaunchArgument(
            "robot_frame",
            default_value="base_footprint",
            description="Robot TF frame captured by the service.",
        ),
        DeclareLaunchArgument(
            "transform_timeout_sec",
            default_value="2.0",
            description="Maximum TF lookup wait in seconds.",
        ),
        DeclareLaunchArgument(
            "max_transform_age_sec",
            default_value="1.0",
            description="Maximum accepted TF age; the exact limit is accepted.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use ROS simulation time when true.",
        ),
        waypoint_saver_node,
    ])
