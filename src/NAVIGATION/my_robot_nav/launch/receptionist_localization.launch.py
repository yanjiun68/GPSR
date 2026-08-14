import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Resolve the package share directory once so the launch file can locate default assets.
    package_share = get_package_share_directory("my_robot_nav")
    # Resolve the Nav2 bringup share directory once so the launch file can include official launch files.
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    # Expose the map path as a launch argument because the workspace does not yet define one canonical map file.
    map_yaml = LaunchConfiguration("map")
    # Expose the Nav2 params file as a launch argument so localization tuning can be overridden externally.
    params_file = LaunchConfiguration(
        "params_file",
        default=os.path.join(package_share, "config", "nav2_params.yaml"),
    )
    # Expose the use_sim_time flag so the same launch file can be used in simulation if needed.
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # Include the official Nav2 localization launch so AMCL and map_server are started in localization mode.
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "localization_launch.py")
        ),
        launch_arguments={
            "map": map_yaml,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
        }.items(),
    )

    # Include the official Nav2 navigation launch so planner, controller, and BT navigator are available.
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
        }.items(),
    )

    navigate_named_pose_server = Node(
        package="my_robot_nav",
        executable="navigate_named_pose_server.py",
        name="navigate_named_pose_server",
        parameters=[{
            "waypoints_file": os.path.join(package_share, "maps", "waypoints.yaml"),
            "action_name": "/nav/navigate_named_pose",
            "navigate_to_pose_action": "/navigate_to_pose",
        }],
        output="screen",
    )

    return LaunchDescription(
        [
            # Require the caller to provide a saved map yaml because the receptionist task should not run on SLAM.
            DeclareLaunchArgument("map", default_value="", description="Path to the saved map yaml file."),
            # Allow the caller to override the Nav2 params file when needed.
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(package_share, "config", "nav2_params.yaml"),
                description="Path to the Nav2 params file.",
            ),
            # Allow the caller to toggle simulation time.
            DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation time."),
            localization_launch,
            navigation_launch,
            navigate_named_pose_server,
        ]
    )
