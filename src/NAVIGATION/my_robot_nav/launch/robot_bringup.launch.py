# launch file for lidar,camera, base,nav2 and slam and rviz
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'my_robot_nav'
    pkg_share = get_package_share_directory(pkg_name)
    rplidar_pkg_share = get_package_share_directory('rplidar_ros')

    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    twist_mux_config_path = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    default_nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params_mppi.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'person.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_port = LaunchConfiguration('lidar_port')
    arduino_port = LaunchConfiguration('arduino_port')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    angle_scale_factor = LaunchConfiguration('angle_scale_factor')
    invert_z = LaunchConfiguration('invert_z')
    enc_sign_m1 = LaunchConfiguration('enc_sign_m1')
    enc_sign_m2 = LaunchConfiguration('enc_sign_m2')
    enc_sign_m3 = LaunchConfiguration('enc_sign_m3')
    enc_sign_m4 = LaunchConfiguration('enc_sign_m4')
    enable_slam = LaunchConfiguration('enable_slam')
    enable_nav2 = LaunchConfiguration('enable_nav2')
    enable_rviz = LaunchConfiguration('enable_rviz')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    slam_and_nav2_enabled = PythonExpression(["'", enable_slam, "' == 'true' and '", enable_nav2, "' == 'true'"])
    nav2_without_slam = PythonExpression(["'", enable_slam, "' != 'true' and '", enable_nav2, "' == 'true'"])

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_description'), 'launch', 'base.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_pkg_share, 'launch', 'rplidar_a2m8_launch.py')
        ),
        launch_arguments={
            'channel_type': 'serial',
            'serial_port': lidar_port,
            'serial_baudrate': '115200',
            'frame_id': lidar_frame_id,
            'inverted': 'false',
            'angle_compensate': 'true',
            'range_min': '0.25',
            'range_max': '12.0',
            'scan_frequency': '10.0',
        }.items()
    )

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[{
            'enable_depth': True,
            'align_depth.enable': True,
            'enable_sync': True,
            'enable_infra1': False,
            'enable_color': True,
            'rgb_camera.color_profile': '640x480x30',
            'enable_infra2': False,
            'enable_gyro': True,
            'enable_accel': True,
            'unite_imu_method': 1,
            'publish_tf': False,
            'depth_module.depth_profile': '640x480x30',
            'gyro_fps': 200,
            'accel_fps': 63,
            'pointcloud.enable': True,
            'pointcloud.stream_filter': 2,
            'pointcloud.stream_index_filter': 0,
            'filters': 'spatial,decimation',
            'decimation_filter.filter_magnitude': 2,
        }],
        remappings=[
            ('/camera/camera/imu', '/camera/imu_combined')
        ],
        output='screen'
    )

    base_driver = Node(
        package='my_robot_nav',
        executable='base_driver.py',
        name='base_driver',
        output='screen',
        parameters=[{
            'port': arduino_port,
            'baud': 115200
        }]
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        remappings=[('/cmd_vel_out', '/cmd_vel')],
        parameters=[twist_mux_config_path]
    )

    kinematics_node = Node(
        package='my_robot_nav',
        executable='cmd_vel_to_targets.py',
        name='cmdvel_to_targets',
        output='screen'
    )

    odometry_node = Node(
        package='my_robot_nav',
        executable='omni_odometry.py',
        name='wheel_odometry',
        output='screen',
        parameters=[{
            'angle_scale_factor': angle_scale_factor,
            'invert_z': invert_z,
            'enc_sign_m1': enc_sign_m1,
            'enc_sign_m2': enc_sign_m2,
            'enc_sign_m3': enc_sign_m3,
            'enc_sign_m4': enc_sign_m4,
            'publish_tf': False,
            'use_sim_time': use_sim_time
        }]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('odometry/filtered', '/odom_filtered')]
    )

    slam_node = Node(
        condition=IfCondition(enable_slam),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        condition=IfCondition(slam_and_nav2_enabled),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',
            'use_composition': 'False'
        }.items()
    )

    slam_nav_group = TimerAction(
        period=3.0,
        actions=[
            slam_node,
            nav2_launch,
            LogInfo(
                condition=IfCondition(nav2_without_slam),
                msg='[robot_bringup] Nav2 requested with enable_slam:=false; Nav2 will not be started in this launch.'
            ),
        ]
    )

    rviz_node = Node(
        condition=IfCondition(enable_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (keep false for real robot)'
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=default_nav2_params_file,
            description='Nav2 parameter file path. Keep default for mapped navigation; override for MPPI experiments.'
        ),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
            description='Serial port for Lidar'
        ),
        DeclareLaunchArgument(
            'arduino_port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            description='Serial port for Arduino base controller'
        ),
        DeclareLaunchArgument(
            'lidar_frame_id',
            default_value='laser',
            description='Lidar frame ID (must match URDF)'
        ),
        DeclareLaunchArgument(
            'angle_scale_factor',
            default_value='1.49',
            description='Scale factor for odometry angle calibration'
        ),
        DeclareLaunchArgument(
            'invert_z',
            default_value='False',
            description='Invert z-axis rotation for odometry'
        ),
        DeclareLaunchArgument('enc_sign_m1', default_value='1', description='Encoder sign for Motor 1 (FL)'),
        DeclareLaunchArgument('enc_sign_m2', default_value='1', description='Encoder sign for Motor 2 (FR)'),
        DeclareLaunchArgument('enc_sign_m3', default_value='1', description='Encoder sign for Motor 3 (RR)'),
        DeclareLaunchArgument('enc_sign_m4', default_value='1', description='Encoder sign for Motor 4 (RL)'),
        DeclareLaunchArgument(
            'enable_slam',
            default_value='false',
            description='Start slam_toolbox after the hardware stack is ready'
        ),
        DeclareLaunchArgument(
            'enable_nav2',
            default_value='false',
            description='Start Nav2 after SLAM is enabled and ready'
        ),
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='true',
            description='Start RViz with the package navigation config'
        ),
        robot_description_launch,
        rplidar_launch,
        base_driver,
        twist_mux_node,
        kinematics_node,
        odometry_node,
        realsense_node,
        ekf_node,
        slam_nav_group,
        rviz_node,
    ])
