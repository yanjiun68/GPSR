"""Launch only the infrastructure owned by SemanticLocationServer."""

# Import section: declare overridable launch parameters without starting localization.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


# Launch entry point: expose all runtime controls documented by the standalone package.
def generate_launch_description() -> LaunchDescription:
    """Create the semantic server launch description without Nav2, AMCL, or robot drivers."""

    # Default section: resolve the installed illustrative semantic map.
    default_map = (
        get_package_share_directory("semantic_location_server")
        + "/config/semantic_locations.yaml"
    )

    # Argument section: keep frame, source, timing, uncertainty, and debug controls explicit.
    arguments = [
        DeclareLaunchArgument("semantic_map_file", default_value=default_map),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint"),
        DeclareLaunchArgument("pose_source", default_value="tf"),
        DeclareLaunchArgument("amcl_topic", default_value="/amcl_pose"),
        DeclareLaunchArgument("update_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("stable_duration_sec", default_value="1.0"),
        DeclareLaunchArgument("use_localization_covariance", default_value="true"),
        DeclareLaunchArgument("max_xy_variance", default_value="0.5"),
        DeclareLaunchArgument("pose_timeout_sec", default_value="1.0"),
        DeclareLaunchArgument("publish_debug_markers", default_value="false"),
    ]

    # Node section: launch only this package's semantic-location process.
    server = Node(
        package="semantic_location_server",
        executable="semantic_location_server",
        name="semantic_location_server",
        output="screen",
        parameters=[
            {
                "semantic_map_file": LaunchConfiguration("semantic_map_file"),
                "map_frame": LaunchConfiguration("map_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "pose_source": LaunchConfiguration("pose_source"),
                "amcl_topic": LaunchConfiguration("amcl_topic"),
                "update_rate_hz": ParameterValue(
                    LaunchConfiguration("update_rate_hz"),
                    value_type=float,
                ),
                "stable_duration_sec": ParameterValue(
                    LaunchConfiguration("stable_duration_sec"),
                    value_type=float,
                ),
                "use_localization_covariance": ParameterValue(
                    LaunchConfiguration("use_localization_covariance"),
                    value_type=bool,
                ),
                "max_xy_variance": ParameterValue(
                    LaunchConfiguration("max_xy_variance"),
                    value_type=float,
                ),
                "pose_timeout_sec": ParameterValue(
                    LaunchConfiguration("pose_timeout_sec"),
                    value_type=float,
                ),
                "publish_debug_markers": ParameterValue(
                    LaunchConfiguration("publish_debug_markers"),
                    value_type=bool,
                ),
            }
        ],
    )
    return LaunchDescription(arguments + [server])
