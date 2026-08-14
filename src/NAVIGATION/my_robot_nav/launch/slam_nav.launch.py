import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_name = 'my_robot_nav'
    pkg_share = get_package_share_directory(pkg_name)

    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(pkg_share, 'config', 'slam_params.yaml'),
    )
    nav2_params_file = LaunchConfiguration(
        'nav2_params_file',
        default=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
    )
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'my_nav.rviz')
    nav_to_pose_bt_xml = os.path.join(
        pkg_share, 'behavior_trees', 'navigate_to_pose_no_backup.xml'
    )
    nav_through_poses_bt_xml = os.path.join(
        pkg_share, 'behavior_trees', 'navigate_through_poses_no_backup.xml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    serial_port = LaunchConfiguration(
        'serial_port',
        default='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
    )
    arduino_port = LaunchConfiguration(
        'arduino_port',
        default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
    )
    use_realsense_imu = LaunchConfiguration('use_realsense_imu', default='false')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id', default='laser')
    scan_topic = LaunchConfiguration('scan_topic', default='/scan_filtered')
    angle_scale_factor = LaunchConfiguration('angle_scale_factor', default='1.49')
    invert_z = LaunchConfiguration('invert_z', default='False')
    enc_sign_m1 = LaunchConfiguration('enc_sign_m1', default='1')
    enc_sign_m2 = LaunchConfiguration('enc_sign_m2', default='1')
    enc_sign_m3 = LaunchConfiguration('enc_sign_m3', default='1')
    enc_sign_m4 = LaunchConfiguration('enc_sign_m4', default='1')
    nav2_delay = LaunchConfiguration('nav2_delay', default='8.0')
    rviz_delay = LaunchConfiguration('rviz_delay', default='10.0')

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': nav_to_pose_bt_xml,
            'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': nav_through_poses_bt_xml,
        },
        convert_types=True,
    )

    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'real_robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_realsense_imu': use_realsense_imu,
            'serial_port': serial_port,
            'arduino_port': arduino_port,
            'lidar_frame_id': lidar_frame_id,
            'cmd_vel_topic': '/cmd_vel_safe',
            'angle_scale_factor': angle_scale_factor,
            'invert_z': invert_z,
            'enc_sign_m1': enc_sign_m1,
            'enc_sign_m2': enc_sign_m2,
            'enc_sign_m3': enc_sign_m3,
            'enc_sign_m4': enc_sign_m4,
        }.items(),
    )

    # Use synchronous scan matching so map->odom is driven directly by lidar updates.
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        # Keep this combined launch in mapping mode regardless of the shared
        # default file, which is also reused by localization launches.
        parameters=[slam_params_file, {
            'use_sim_time': use_sim_time,
            'mode': 'mapping',
            'scan_topic': scan_topic,
        }],
        output='screen',
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': configured_nav2_params,
            'autostart': autostart,
            'use_composition': 'False',
        }.items(),
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[
            configured_nav2_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    collision_monitor_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['collision_monitor'],
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz if true',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup Nav2 lifecycle nodes',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg_share, 'config', 'slam_params.yaml'),
            description='Full path to slam_toolbox params file',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            description='Full path to Nav2 params file',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
            description='Serial port for lidar',
        ),
        DeclareLaunchArgument(
            'arduino_port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            description='Serial port for Arduino',
        ),
        DeclareLaunchArgument(
            'lidar_frame_id',
            default_value='laser',
            description='Lidar frame id',
        ),
        DeclareLaunchArgument(
            'use_realsense_imu',
            default_value='false',
            description='Enable the RealSense IMU and fuse it into EKF odom',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan_filtered',
            description='LaserScan topic used by slam_toolbox',
        ),
        DeclareLaunchArgument(
            'angle_scale_factor',
            default_value='1.49',
            description='Scale factor for odometry angle calibration',
        ),
        DeclareLaunchArgument(
            'invert_z',
            default_value='False',
            description='Invert z-axis rotation in wheel odometry',
        ),
        DeclareLaunchArgument('enc_sign_m1', default_value='1'),
        DeclareLaunchArgument('enc_sign_m2', default_value='1'),
        DeclareLaunchArgument('enc_sign_m3', default_value='1'),
        DeclareLaunchArgument('enc_sign_m4', default_value='1'),
        DeclareLaunchArgument(
            'nav2_delay',
            default_value='8.0',
            description='Seconds to wait before launching Nav2',
        ),
        DeclareLaunchArgument(
            'rviz_delay',
            default_value='10.0',
            description='Seconds to wait before launching RViz',
        ),
        robot_bringup,
        slam,
        collision_monitor,
        collision_monitor_manager,
        TimerAction(
            period=nav2_delay,
            actions=[nav2],
        ),
        TimerAction(
            period=rviz_delay,
            actions=[rviz],
            condition=IfCondition(use_rviz),
        ),
    ])
