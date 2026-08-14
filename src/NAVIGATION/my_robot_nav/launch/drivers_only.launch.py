import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Section: package paths used to include the shared real-robot driver launch.
    pkg_share = get_package_share_directory('my_robot_nav')

    # Section: launch arguments forwarded from external bringup packages.
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_realsense_imu = LaunchConfiguration('use_realsense_imu', default='false')
    serial_port = LaunchConfiguration(
        'serial_port',
        default='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
    )
    arduino_port = LaunchConfiguration(
        'arduino_port',
        default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
    )
    lidar_frame_id = LaunchConfiguration('lidar_frame_id', default='laser')
    angle_scale_factor = LaunchConfiguration('angle_scale_factor', default='1.49')
    invert_z = LaunchConfiguration('invert_z', default='False')
    enc_sign_m1 = LaunchConfiguration('enc_sign_m1', default='1')
    enc_sign_m2 = LaunchConfiguration('enc_sign_m2', default='1')
    enc_sign_m3 = LaunchConfiguration('enc_sign_m3', default='1')
    enc_sign_m4 = LaunchConfiguration('enc_sign_m4', default='1')
    base_linear_speed_limit = LaunchConfiguration(
        'base_linear_speed_limit',
        default='0.25',
    )
    base_angular_speed_limit = LaunchConfiguration(
        'base_angular_speed_limit',
        default='0.40',
    )
    breadcrumb_max_linear_speed = LaunchConfiguration(
        'breadcrumb_max_linear_speed',
        default='0.12',
    )
    breadcrumb_max_lateral_speed = LaunchConfiguration(
        'breadcrumb_max_lateral_speed',
        default='0.08',
    )
    breadcrumb_max_angular_speed = LaunchConfiguration(
        'breadcrumb_max_angular_speed',
        default='0.40',
    )

    # Section: shared real-robot driver stack without Nav2 localization/navigation.
    real_robot_launch = IncludeLaunchDescription(
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
            'base_linear_speed_limit': base_linear_speed_limit,
            'base_angular_speed_limit': base_angular_speed_limit,
            'angle_scale_factor': angle_scale_factor,
            'invert_z': invert_z,
            'enc_sign_m1': enc_sign_m1,
            'enc_sign_m2': enc_sign_m2,
            'enc_sign_m3': enc_sign_m3,
            'enc_sign_m4': enc_sign_m4,
            'breadcrumb_max_linear_speed': breadcrumb_max_linear_speed,
            'breadcrumb_max_lateral_speed': breadcrumb_max_lateral_speed,
            'breadcrumb_max_angular_speed': breadcrumb_max_angular_speed,
        }.items(),
    )

    # Section: public launch interface consumed by putrabot_bringup.
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'use_realsense_imu',
            default_value='false',
            description='Enable the RealSense IMU in the robot odom pipeline',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
            description='Serial port for Lidar',
        ),
        DeclareLaunchArgument(
            'arduino_port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            description='Serial port for Arduino',
        ),
        DeclareLaunchArgument(
            'lidar_frame_id',
            default_value='laser',
            description='Lidar frame ID',
        ),
        DeclareLaunchArgument(
            'angle_scale_factor',
            default_value='1.49',
            description='Scale factor for odometry angle calibration',
        ),
        DeclareLaunchArgument(
            'invert_z',
            default_value='False',
            description='Invert z-axis rotation',
        ),
        DeclareLaunchArgument(
            'base_linear_speed_limit',
            default_value='0.25',
            description='Linear speed clamp used by the driver stack',
        ),
        DeclareLaunchArgument(
            'base_angular_speed_limit',
            default_value='0.40',
            description='Angular speed clamp used by the driver stack',
        ),
        DeclareLaunchArgument(
            'breadcrumb_max_linear_speed',
            default_value='0.12',
            description='Maximum breadcrumb forward speed used by the driver stack',
        ),
        DeclareLaunchArgument(
            'breadcrumb_max_lateral_speed',
            default_value='0.08',
            description='Maximum breadcrumb lateral speed used by the driver stack',
        ),
        DeclareLaunchArgument(
            'breadcrumb_max_angular_speed',
            default_value='0.40',
            description='Maximum breadcrumb angular speed used by the driver stack',
        ),
        DeclareLaunchArgument('enc_sign_m1', default_value='1'),
        DeclareLaunchArgument('enc_sign_m2', default_value='1'),
        DeclareLaunchArgument('enc_sign_m3', default_value='1'),
        DeclareLaunchArgument('enc_sign_m4', default_value='1'),
        real_robot_launch,
    ])
