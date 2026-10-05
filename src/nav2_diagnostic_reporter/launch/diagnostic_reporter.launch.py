"""Launch only the independent Nav2 diagnostic reporter proxy."""

# Section: ROS launch imports.
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Section: Standalone launch-description factory.
def generate_launch_description() -> LaunchDescription:
    """Start the reporter only; Nav2 itself must be launched separately."""

    default_config = get_package_share_directory('nav2_diagnostic_reporter') + '/config/diagnostic_reporter.yaml'
    config_argument = DeclareLaunchArgument(
        'params_file', default_value=default_config,
        description='Reporter parameter YAML file.')
    reporter_node = Node(
        package='nav2_diagnostic_reporter',
        executable='diagnostic_reporter',
        name='nav2_diagnostic_reporter',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )
    return LaunchDescription([config_argument, reporter_node])
