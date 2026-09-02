#!/usr/bin/env python3

"""Launch the GPSR robot base, static-map navigation, and depth camera."""

# Section: ROS 2 launch primitives used to compose the two existing launches.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


# Section: Absolute GPSR runtime files requested for this robot workspace.
GPSR_MAP_FILE = "/home/yanjiun68/workspace/WS1/PutrabotUPM-project/maps/gpsr_map.yaml"
GPSR_WAYPOINTS_FILE = "/home/yanjiun68/workspace/WS1/PutrabotUPM-project/waypoints/gpsr_waypoint.yaml"


# Special function: Build the complete hardware and navigation launch graph for GPSR.
def generate_launch_description() -> LaunchDescription:
    """Start RealSense depth and the GPSR static-map navigation stack."""
    # Section: Operator-facing settings shared with the included launches.
    map_file = LaunchConfiguration("map")
    waypoints_file = LaunchConfiguration("waypoints_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    nav2_autostart = LaunchConfiguration("nav2_autostart")
    start_realsense = LaunchConfiguration("start_realsense")
    enable_depth = LaunchConfiguration("enable_depth")

    # Section: Use deterministic absolute paths instead of the caller's working directory.
    default_map_file = GPSR_MAP_FILE
    default_waypoints_file = GPSR_WAYPOINTS_FILE

    # Section: Official RealSense launch with the depth stream explicitly enabled.
    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        condition=IfCondition(start_realsense),
        launch_arguments={
            "enable_depth": enable_depth,
        }.items(),
    )

    # Section: Existing base-driver, lidar, localization, Nav2, waypoint, and RViz stack.
    static_map_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("my_robot_nav"), "launch", "static_map_nav.launch.py"]
            )
        ),
        launch_arguments={
            "map": map_file,
            "waypoints_file": waypoints_file,
            "use_sim_time": use_sim_time,
            "use_rviz": use_rviz,
            "nav2_autostart": nav2_autostart,
            # Section: Prevent static_map_nav from starting a second RealSense instance.
            "use_realsense_imu": "false",
            "use_external_drivers": "false",
        }.items(),
    )

    # Section: Public GPSR base-driver interface and composed launch actions.
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=default_map_file,
                description="Full path to the GPSR map YAML file.",
            ),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=default_waypoints_file,
                description="Full path to the GPSR named-waypoint YAML file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the simulation clock instead of the hardware clock.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with the static-map navigation stack.",
            ),
            DeclareLaunchArgument(
                "nav2_autostart",
                default_value="true",
                description="Autostart the static-map Nav2 lifecycle manager.",
            ),
            DeclareLaunchArgument(
                "start_realsense",
                default_value="true",
                description="Start the RealSense camera for GPSR perception.",
            ),
            DeclareLaunchArgument(
                "enable_depth",
                default_value="true",
                description="Enable the RealSense depth stream.",
            ),
            realsense_camera,
            static_map_navigation,
        ]
    )
