import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_default_semantic_locations_file(pkg_share, pkg_name):
    install_file = os.path.join(pkg_share, 'config', 'semantic_locations.yaml')
    workspace_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(pkg_share)))
    )
    source_file = os.path.join(
        workspace_root,
        'src',
        pkg_name,
        'config',
        'semantic_locations.yaml',
    )

    if os.path.isfile(source_file):
        return source_file

    return install_file


def generate_launch_description():
    pkg_name = 'my_robot_nav'
    pkg_share = get_package_share_directory(pkg_name)

    # Section: Use the GPSR occupancy map as the default static navigation map.
    default_map = (
        '/home/yanjiun68/workspace/WS1/PutrabotUPM-project/'
        'src/NAVIGATION/my_robot_nav/maps/gpsr_map.yaml'
    )
    default_waypoints_file = os.path.join(pkg_share, 'config', 'gpsr_waypoint.yaml')
    default_semantic_locations = resolve_default_semantic_locations_file(
        pkg_share,
        pkg_name,
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_external_drivers = LaunchConfiguration('use_external_drivers', default='false')
    map_yaml_file = LaunchConfiguration('map', default=default_map)
    waypoints_file = LaunchConfiguration('waypoints_file', default=default_waypoints_file)
    use_realsense_imu = LaunchConfiguration('use_realsense_imu', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    nav2_delay = LaunchConfiguration('nav2_delay', default='10.0')
    nav2_autostart = LaunchConfiguration('nav2_autostart', default='false')
    navigate_to_pose_server_timeout_sec = LaunchConfiguration(
        'navigate_to_pose_server_timeout_sec',
        default='30.0',
    )
    use_startup_gate = LaunchConfiguration('use_startup_gate', default='true')
    require_nav2_transform = LaunchConfiguration(
        'require_nav2_transform',
        default='true',
    )
    nav2_transform_target_frame = LaunchConfiguration(
        'nav2_transform_target_frame',
        default='map',
    )
    nav2_transform_source_frame = LaunchConfiguration(
        'nav2_transform_source_frame',
        default='base_footprint',
    )
    rviz_delay = LaunchConfiguration('rviz_delay', default='12.0')
    use_semantic_nav = LaunchConfiguration('use_semantic_nav', default='true')
    semantic_locations_file = LaunchConfiguration(
        'semantic_locations_file',
        default=default_semantic_locations,
    )
    set_initial_pose = LaunchConfiguration('set_initial_pose', default='false')
    global_localize_on_start = LaunchConfiguration('global_localize_on_start', default='false')
    global_localization_delay = LaunchConfiguration('global_localization_delay', default='2.0')
    initial_pose_x = LaunchConfiguration('initial_pose_x', default='2.82')
    initial_pose_y = LaunchConfiguration('initial_pose_y', default='-0.11')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw', default='0.37')
    initial_pose_delay = LaunchConfiguration('initial_pose_delay', default='10.0')
    base_linear_speed_limit = LaunchConfiguration(
        'base_linear_speed_limit',
        default='0.25',
    )
    base_angular_speed_limit = LaunchConfiguration(
        'base_angular_speed_limit',
        default='0.40',
    )
    nav_desired_linear_vel = LaunchConfiguration(
        'nav_desired_linear_vel',
        default='0.20',
    )
    nav_rotate_to_heading_angular_vel = LaunchConfiguration(
        'nav_rotate_to_heading_angular_vel',
        default='0.35',
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
    serial_port = LaunchConfiguration(
        'serial_port',
        default=(
            '/dev/serial/by-id/'
            'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
        ),
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

    lidar_loc_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'lidar_loc_nav.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            # Section: Forward the requested waypoint file to the named-pose server.
            'waypoints_file': waypoints_file,
            'use_realsense_imu': use_realsense_imu,
            'use_rviz': use_rviz,
            'nav2_delay': nav2_delay,
            'nav2_autostart': nav2_autostart,
            'navigate_to_pose_server_timeout_sec': navigate_to_pose_server_timeout_sec,
            'use_startup_gate': use_startup_gate,
            'require_nav2_transform': require_nav2_transform,
            'nav2_transform_target_frame': nav2_transform_target_frame,
            'nav2_transform_source_frame': nav2_transform_source_frame,
            'rviz_delay': rviz_delay,
            'set_initial_pose': set_initial_pose,
            'global_localize_on_start': global_localize_on_start,
            'global_localization_delay': global_localization_delay,
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_yaw': initial_pose_yaw,
            'initial_pose_delay': initial_pose_delay,
            'base_linear_speed_limit': base_linear_speed_limit,
            'base_angular_speed_limit': base_angular_speed_limit,
            'nav_desired_linear_vel': nav_desired_linear_vel,
            'nav_rotate_to_heading_angular_vel': nav_rotate_to_heading_angular_vel,
            'serial_port': serial_port,
            'arduino_port': arduino_port,
            'lidar_frame_id': lidar_frame_id,
            'angle_scale_factor': angle_scale_factor,
            'invert_z': invert_z,
            'enc_sign_m1': enc_sign_m1,
            'enc_sign_m2': enc_sign_m2,
            'enc_sign_m3': enc_sign_m3,
            'enc_sign_m4': enc_sign_m4,
            'breadcrumb_max_linear_speed': breadcrumb_max_linear_speed,
            'breadcrumb_max_lateral_speed': breadcrumb_max_lateral_speed,
            'breadcrumb_max_angular_speed': breadcrumb_max_angular_speed,
            'use_external_drivers': use_external_drivers
        }.items(),
    )

    semantic_navigator = Node(
        package='my_robot_nav',
        executable='semantic_navigator.py',
        name='semantic_navigator',
        condition=IfCondition(use_semantic_nav),
        parameters=[{
            'use_sim_time': use_sim_time,
            'locations_file': semantic_locations_file,
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'use_external_drivers',
            default_value='false',
            description='Expect hardware drivers to be launched by an external bringup',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map yaml file',
        ),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=default_waypoints_file,
            description='Full path to named navigation waypoints yaml',
        ),
        DeclareLaunchArgument(
            'use_realsense_imu',
            default_value='false',
            description='Enable the RealSense IMU in the robot odom pipeline',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz after localization and Nav2 settle',
        ),
        DeclareLaunchArgument(
            'nav2_delay',
            default_value='10.0',
            description='Seconds to wait before starting Nav2',
        ),
        DeclareLaunchArgument(
            'nav2_autostart',
            default_value='false',
            description=(
                'Autostart Nav2 immediately; keep false to wait for '
                '2D Pose Estimate first'
            ),
        ),
        DeclareLaunchArgument(
            'navigate_to_pose_server_timeout_sec',
            default_value='30.0',
            description='Seconds named-pose navigation waits for /navigate_to_pose to appear',
        ),
        DeclareLaunchArgument(
            'use_startup_gate',
            default_value='true',
            description='Start Nav2 only after odom and a 2D Pose Estimate are available',
        ),
        DeclareLaunchArgument(
            'require_nav2_transform',
            default_value='true',
            description='Start Nav2 only after the map-to-base TF is available',
        ),
        DeclareLaunchArgument(
            'nav2_transform_target_frame',
            default_value='map',
            description='Target frame required by the Nav2 startup gate',
        ),
        DeclareLaunchArgument(
            'nav2_transform_source_frame',
            default_value='base_footprint',
            description='Source frame required by the Nav2 startup gate',
        ),
        DeclareLaunchArgument(
            'rviz_delay',
            default_value='12.0',
            description='Seconds to wait before starting RViz',
        ),
        DeclareLaunchArgument(
            'use_semantic_nav',
            default_value='true',
            description='Start the semantic named-goal navigation helper',
        ),
        DeclareLaunchArgument(
            'semantic_locations_file',
            default_value=default_semantic_locations,
            description='Full path to semantic locations yaml file',
        ),
        DeclareLaunchArgument(
            'set_initial_pose',
            default_value='false',
            description='Publish an initial pose after startup',
        ),
        DeclareLaunchArgument(
            'global_localize_on_start',
            default_value='false',
            description='Request AMCL global localization when startup pose is unknown',
        ),
        DeclareLaunchArgument(
            'global_localization_delay',
            default_value='2.0',
            description='Seconds to wait before requesting AMCL global localization',
        ),
        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='2.82',
            description='Initial pose X in the map frame',
        ),
        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='-0.11',
            description='Initial pose Y in the map frame',
        ),
        DeclareLaunchArgument(
            'initial_pose_yaw',
            default_value='0.37',
            description='Initial pose yaw in radians',
        ),
        DeclareLaunchArgument(
            'initial_pose_delay',
            default_value='10.0',
            description='Seconds to wait before publishing the initial pose',
        ),
        DeclareLaunchArgument(
            'base_linear_speed_limit',
            default_value='0.25',
            description='Linear speed clamp used only in static-map mode',
        ),
        DeclareLaunchArgument(
            'base_angular_speed_limit',
            default_value='0.40',
            description='Angular speed clamp used only in static-map mode',
        ),
        DeclareLaunchArgument(
            'nav_desired_linear_vel',
            default_value='0.20',
            description='Desired Nav2 forward speed used only in static-map mode',
        ),
        DeclareLaunchArgument(
            'nav_rotate_to_heading_angular_vel',
            default_value='0.35',
            description='Nav2 rotate-to-heading speed used only in static-map mode',
        ),
        DeclareLaunchArgument(
            'breadcrumb_max_linear_speed',
            default_value='0.12',
            description='Maximum breadcrumb forward speed used only in static-map mode',
        ),
        DeclareLaunchArgument(
            'breadcrumb_max_lateral_speed',
            default_value='0.08',
            description='Maximum breadcrumb lateral speed used only in static-map mode',
        ),
        DeclareLaunchArgument(
            'breadcrumb_max_angular_speed',
            default_value='0.40',
            description='Maximum breadcrumb angular speed used only in static-map mode',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-'
                'if00-port0'
            ),
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
        DeclareLaunchArgument('enc_sign_m1', default_value='1'),
        DeclareLaunchArgument('enc_sign_m2', default_value='1'),
        DeclareLaunchArgument('enc_sign_m3', default_value='1'),
        DeclareLaunchArgument('enc_sign_m4', default_value='1'),
        lidar_loc_nav,
        semantic_navigator,
    ])
