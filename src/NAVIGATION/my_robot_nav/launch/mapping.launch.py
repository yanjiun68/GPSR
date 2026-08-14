import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'my_robot_nav'
    pkg_share = get_package_share_directory(pkg_name)

    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'my_nav.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_realsense_imu = LaunchConfiguration('use_realsense_imu', default='false')
    scan_topic = LaunchConfiguration('scan_topic', default='/scan_filtered')

    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'real_robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_realsense_imu': use_realsense_imu,
            'enable_breadcrumb_teach_repeat': 'false',
        }.items(),
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        # Force mapping-specific overrides here so this launch file stays in
        # pure SLAM mode even if the shared params file is reused for localization.
        parameters=[slam_params_file, {
            'use_sim_time': use_sim_time,
            'mode': 'mapping',
            'scan_topic': scan_topic,
        }],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'use_realsense_imu',
            default_value='false',
            description='Enable the D435i IMU and fuse it into the EKF odom output',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan_filtered',
            description='LaserScan topic for SLAM mapping',
        ),
        robot_bringup,
        slam,
        rviz,
    ])
