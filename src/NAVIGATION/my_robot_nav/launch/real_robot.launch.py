import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'my_robot_nav'
    pkg_share = get_package_share_directory(pkg_name)

    ekf_wheel_only_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    ekf_wheel_imu_config = os.path.join(pkg_share, 'config', 'ekf_wheel_imu.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_realsense_imu = LaunchConfiguration('use_realsense_imu', default='false')
    camera_serial_no = LaunchConfiguration('camera_serial_no', default="_022422071806")
    serial_port = LaunchConfiguration(
        'serial_port',
        default='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
    )
    arduino_port = LaunchConfiguration(
        'arduino_port',
        default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
    )
    lidar_frame_id = LaunchConfiguration('lidar_frame_id', default='laser')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic', default='/cmd_vel')
    base_linear_speed_limit = LaunchConfiguration(
        'base_linear_speed_limit',
        default='0.50',
    )
    base_angular_speed_limit = LaunchConfiguration(
        'base_angular_speed_limit',
        default='0.65',
    )
    enable_breadcrumb_teach_repeat = LaunchConfiguration(
        'enable_breadcrumb_teach_repeat',
        default='true',
    )
    breadcrumb_route_file = LaunchConfiguration(
        'breadcrumb_route_file',
        default='/tmp/my_robot_nav_breadcrumb_route.yaml',
    )
    breadcrumb_cmd_vel_topic = LaunchConfiguration(
        'breadcrumb_cmd_vel_topic',
        default='/cmd_vel',
    )
    breadcrumb_heading_mode = LaunchConfiguration(
        'breadcrumb_heading_mode',
        default='path_tangent',
    )
    breadcrumb_max_linear_speed = LaunchConfiguration(
        'breadcrumb_max_linear_speed',
        default='0.18',
    )
    breadcrumb_max_lateral_speed = LaunchConfiguration(
        'breadcrumb_max_lateral_speed',
        default='0.12',
    )
    breadcrumb_max_angular_speed = LaunchConfiguration(
        'breadcrumb_max_angular_speed',
        default='0.70',
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_description'),
                'launch',
                'base.launch.py',
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Section: hardware lidar publisher feeding /scan for the local filter.
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'channel_type': 'serial',
            'serial_port': serial_port,
            'serial_baudrate': 115200,
            'frame_id': lidar_frame_id,
            'inverted': False,
            'angle_compensate': True,
            'range_min': 0.15,
            'range_max': 12.0,
            'scan_frequency': 10.0,
        }],
        output='screen',
    )

    # Use the official RealSense launch so IMU topics and parameter names stay aligned
    # with the installed driver. IMU-only mode keeps CPU load low while improving yaw.
    realsense_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py',
            )
        ),
        condition=IfCondition(use_realsense_imu),
        launch_arguments={
            'camera_namespace': 'camera',
            'camera_name': 'camera',
            'serial_no': camera_serial_no,
            'enable_color': 'false',
            'enable_depth': 'false',
            'enable_infra': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'gyro_fps': '200',
            'accel_fps': '63',
            'publish_tf': 'false',
            'pointcloud.enable': 'false',
            'wait_for_device_timeout': '10.0',
            'reconnect_timeout': '3.0',
            'use_sim_time': use_sim_time,
            'log_level': 'warn',
            'output': 'screen',
        }.items(),
    )

    base_driver = Node(
        package='my_robot_nav',
        executable='base_driver.py',
        name='base_driver',
        output='screen',
        parameters=[{
            'port': arduino_port,
            'baud': 115200,
            'targets_timeout_sec': 0.8,
        }],
    )

    scan_self_filter = Node(
        package='my_robot_nav',
        executable='scan_self_filter.py',
        name='scan_self_filter',
        output='screen',
        parameters=[{
            'input_scan_topic': '/scan',
            'output_scan_topic': '/scan_filtered',
            'base_frame': 'base_footprint',
            # Polygon mirrors the footprint used in nav2_params.yaml so each
            # laser hit is judged against the true robot outline instead of
            # the loose circumscribed circle. This removes the ~19 cm front
            # blind zone that previously masked real obstacles.
            'footprint_polygon': [
                0.2655, -0.11,
                0.2655,  0.11,
                0.212,   0.212,
                0.11,    0.2655,
               -0.11,    0.2655,
               -0.212,   0.212,
               -0.2655,  0.11,
               -0.2655, -0.11,
               -0.212,  -0.212,
               -0.11,   -0.2655,
                0.11,   -0.2655,
                0.212,  -0.212,
            ],
            'footprint_padding': 0.03,
            'timestamp_offset_sec': 0.08,
        }],
    )

    kinematics_node = Node(
        package='my_robot_nav',
        executable='cmd_vel_to_targets.py',
        name='cmdvel_to_targets',
        output='screen',
        parameters=[{
            'cmd_vel_topic': cmd_vel_topic,
            'vx_max': base_linear_speed_limit,
            'wz_max': base_angular_speed_limit,
            'timeout': 0.8,
            'min_effective_wz': 0.08,
        }],
    )

    odometry_node = Node(
        package='my_robot_nav',
        executable='omni_odometry.py',
        name='wheel_odometry',
        parameters=[{
            'publish_tf': False,
            'odom_topic': '/wheel_odom',
            'odom_frame': 'odom',
            'base_footprint_frame': 'base_footprint',
            'angle_scale_factor': LaunchConfiguration('angle_scale_factor'),
            'invert_z': LaunchConfiguration('invert_z'),
            'enc_sign_m1': LaunchConfiguration('enc_sign_m1'),
            'enc_sign_m2': LaunchConfiguration('enc_sign_m2'),
            'enc_sign_m3': LaunchConfiguration('enc_sign_m3'),
            'enc_sign_m4': LaunchConfiguration('enc_sign_m4'),
        }],
        output='screen',
    )

    ekf_wheel_only_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        condition=UnlessCondition(use_realsense_imu),
        output='screen',
        parameters=[ekf_wheel_only_config],
        remappings=[('odometry/filtered', '/odom')],
    )

    ekf_wheel_imu_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        condition=IfCondition(use_realsense_imu),
        output='screen',
        parameters=[ekf_wheel_imu_config],
        remappings=[('odometry/filtered', '/odom')],
    )

    imu_decimator_node = Node(
        package='my_robot_nav',
        executable='imu_decimator.py',
        name='imu_decimator',
        condition=IfCondition(use_realsense_imu),
        output='screen',
    )

    breadcrumb_teach_repeat = Node(
        package='my_robot_nav',
        executable='breadcrumb_teach_repeat.py',
        name='breadcrumb_teach_repeat',
        condition=IfCondition(enable_breadcrumb_teach_repeat),
        output='screen',
        parameters=[{
            'odom_topic': '/odom',
            'scan_topic': '/scan_filtered',
            'cmd_vel_topic': breadcrumb_cmd_vel_topic,
            'initial_pose_topic': '/initialpose',
            'route_file': breadcrumb_route_file,
            'heading_mode': breadcrumb_heading_mode,
            'autoload_route': True,
            'autosave_route': True,
            'publish_start_pose_on_finish': True,
            'publish_tf': False,
            'max_linear_speed': breadcrumb_max_linear_speed,
            'max_lateral_speed': breadcrumb_max_lateral_speed,
            'max_angular_speed': breadcrumb_max_angular_speed,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (keep false on the real robot)',
        ),
        DeclareLaunchArgument(
            'use_realsense_imu',
            default_value='false',
            description='Enable the D435i IMU and fuse it into the EKF odom output',
        ),
        DeclareLaunchArgument(
            'camera_serial_no',
            default_value="_022422071806",
            description='RealSense serial number for navigation camera (empty for auto-detect)',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
            description='Serial port for the lidar',
        ),
        DeclareLaunchArgument(
            'arduino_port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            description='Serial port for the Arduino base controller',
        ),
        DeclareLaunchArgument(
            'lidar_frame_id',
            default_value='laser',
            description='Lidar frame ID (must match the URDF)',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='Final Twist topic consumed by cmd_vel_to_targets',
        ),
        DeclareLaunchArgument(
            'base_linear_speed_limit',
            default_value='0.50',
            description='Linear speed clamp for manual, breadcrumb, and navigation commands',
        ),
        DeclareLaunchArgument(
            'base_angular_speed_limit',
            default_value='0.65',
            description='Angular speed clamp for manual, breadcrumb, and navigation commands',
        ),
        DeclareLaunchArgument(
            'angle_scale_factor',
            default_value='1.49',
            description='Wheel-odom yaw scale factor. Mainly used in wheel-only mode.',
        ),
        DeclareLaunchArgument(
            'invert_z',
            default_value='False',
            description='Invert z-axis rotation for wheel odometry',
        ),
        DeclareLaunchArgument(
            'breadcrumb_max_linear_speed',
            default_value='0.18',
            description='Maximum forward speed used by breadcrumb repeat',
        ),
        DeclareLaunchArgument(
            'breadcrumb_max_lateral_speed',
            default_value='0.12',
            description='Maximum lateral speed used by breadcrumb repeat',
        ),
        DeclareLaunchArgument(
            'breadcrumb_max_angular_speed',
            default_value='0.70',
            description='Maximum angular speed used by breadcrumb repeat',
        ),
        DeclareLaunchArgument('enc_sign_m1', default_value='1', description='Encoder sign for Motor 1 (FL)'),
        DeclareLaunchArgument('enc_sign_m2', default_value='1', description='Encoder sign for Motor 2 (FR)'),
        DeclareLaunchArgument('enc_sign_m3', default_value='1', description='Encoder sign for Motor 3 (RR)'),
        DeclareLaunchArgument('enc_sign_m4', default_value='1', description='Encoder sign for Motor 4 (RL)'),
        robot_description_launch,
        lidar_node,
        scan_self_filter,
        realsense_imu_launch,
        base_driver,
        kinematics_node,
        odometry_node,
        ekf_wheel_only_node,
        ekf_wheel_imu_node,
        imu_decimator_node,
        breadcrumb_teach_repeat,
    ])
