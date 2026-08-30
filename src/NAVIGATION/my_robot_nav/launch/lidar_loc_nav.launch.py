import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def resolve_workspace_root(pkg_share, environment=None):
    """Special function: Resolve workspace runtime data with an environment override."""
    # Section: Prefer an explicit absolute root for nonstandard installations.
    active_environment = os.environ if environment is None else environment
    override = str(active_environment.get('PUTRABOT_WORKSPACE_ROOT', '')).strip()
    if override:
        override_path = Path(override).expanduser()
        if not override_path.is_absolute():
            raise ValueError('PUTRABOT_WORKSPACE_ROOT must be an absolute path')
        return override_path.resolve(strict=False)

    # Section: Derive the root only from install/<package>/share/<package>.
    share_path = Path(pkg_share).expanduser().resolve(strict=False)
    if (
        share_path.parent.name != 'share'
        or share_path.parent.parent.name != share_path.name
        or share_path.parent.parent.parent.name != 'install'
    ):
        raise ValueError(
            'Cannot derive workspace root from package share path; '
            'set PUTRABOT_WORKSPACE_ROOT'
        )
    return share_path.parent.parent.parent.parent


def resolve_runtime_data_paths(pkg_share, environment=None):
    """Special function: Build workspace-level map and waypoint defaults."""
    # Section: Keep mutable arena data outside package source and install space.
    workspace_root = resolve_workspace_root(pkg_share, environment)
    return (
        workspace_root / 'maps' / 'gpsr_map.yaml',
        workspace_root / 'waypoints' / 'gpsr_waypoint.yaml',
    )


def generate_launch_description():
    """Special function: Build localization/navigation with workspace data defaults."""
    # Section: Resolve package assets separately from mutable workspace runtime data.
    pkg_name = 'my_robot_nav'
    pkg_share = get_package_share_directory(pkg_name)

    default_map_path, default_waypoints_path = resolve_runtime_data_paths(pkg_share)
    default_map = str(default_map_path)
    default_nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_waypoints_file = str(default_waypoints_path)
    default_rviz_config_file = os.path.join(
        pkg_share, 'rviz', 'my_nav_minimal.rviz')
    nav_to_pose_bt_xml = os.path.join(
        pkg_share, 'behavior_trees', 'navigate_to_pose_no_backup.xml'
    )
    nav_through_poses_bt_xml = os.path.join(
        pkg_share, 'behavior_trees', 'navigate_through_poses_no_backup.xml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    start_waypoint_saver = LaunchConfiguration('start_waypoint_saver', default='true')
    waypoint_saver_service_name = LaunchConfiguration(
        'waypoint_saver_service_name',
        default='/waypoint_saver/save_current',
    )
    waypoint_saver_transform_timeout_sec = LaunchConfiguration(
        'waypoint_saver_transform_timeout_sec',
        default='2.0',
    )
    waypoint_saver_max_transform_age_sec = LaunchConfiguration(
        'waypoint_saver_max_transform_age_sec',
        default='1.0',
    )
    use_external_drivers = LaunchConfiguration('use_external_drivers', default='false')
    map_yaml_file = LaunchConfiguration('map', default=default_map)
    waypoints_file = LaunchConfiguration(
        'waypoints_file',
        default=default_waypoints_file,
    )
    # Allows the IAIG-Nav launch (or any other caller) to swap in an
    # alternative Nav2 params file — e.g. one with controller_server
    # removed from the lifecycle manager when the DRL local planner
    # is used instead.
    nav2_params_file_arg = LaunchConfiguration('nav2_params_file',
                                               default=default_nav2_params)
    use_realsense_imu = LaunchConfiguration('use_realsense_imu', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    rviz_config_file = LaunchConfiguration('rviz_config',
                                           default=default_rviz_config_file)
    nav2_delay = LaunchConfiguration('nav2_delay', default='10.0')
    nav2_autostart = LaunchConfiguration('nav2_autostart', default='false')
    navigate_to_pose_server_timeout_sec = LaunchConfiguration(
        'navigate_to_pose_server_timeout_sec',
        default='30.0',
    )
    use_nav2_stack = LaunchConfiguration('use_nav2_stack', default='true')
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
    set_initial_pose = LaunchConfiguration('set_initial_pose', default='false')
    global_localize_on_start = LaunchConfiguration('global_localize_on_start', default='false')
    global_localization_delay = LaunchConfiguration('global_localization_delay', default='2.0')
    initial_pose_x = LaunchConfiguration('initial_pose_x', default='2.82')
    initial_pose_y = LaunchConfiguration('initial_pose_y', default='-0.11')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw', default='0.37')
    initial_pose_delay = LaunchConfiguration('initial_pose_delay', default='10.0')
    base_linear_speed_limit = LaunchConfiguration(
        'base_linear_speed_limit',
        default='0.50',
    )
    base_angular_speed_limit = LaunchConfiguration(
        'base_angular_speed_limit',
        default='0.65',
    )
    nav_desired_linear_vel = LaunchConfiguration(
        'nav_desired_linear_vel',
        default='0.40',
    )
    nav_rotate_to_heading_angular_vel = LaunchConfiguration(
        'nav_rotate_to_heading_angular_vel',
        default='0.65',
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

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file_arg,
        param_rewrites={
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': (
                nav_to_pose_bt_xml
            ),
            'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': (
                nav_through_poses_bt_xml
            ),
            'controller_server.ros__parameters.FollowPath.desired_linear_vel': (
                nav_desired_linear_vel
            ),
            (
                'controller_server.ros__parameters.FollowPath.'
                'rotate_to_heading_angular_vel'
            ): nav_rotate_to_heading_angular_vel,
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
            # 'use_external_drivers': use_external_drivers,
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
        condition=UnlessCondition(use_external_drivers),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'localization_launch.py',
            )
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': configured_nav2_params,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )

    nav2_autostart_gate = Node(
        package='my_robot_nav',
        executable='nav2_autostart_gate.py',
        name='nav2_autostart_gate',
        condition=IfCondition(use_startup_gate),
        parameters=[{
            'service_name': '/lifecycle_manager_navigation/manage_nodes',
            'odom_topic': '/odom',
            'initial_pose_topic': '/initialpose',
            'require_initial_pose': True,
            'localization_topic': '/amcl_pose',
            'require_localization': False,
            'require_transform': require_nav2_transform,
            'transform_target_frame': nav2_transform_target_frame,
            'transform_source_frame': nav2_transform_source_frame,
            'covariance_x_threshold': 0.20,
            'covariance_y_threshold': 0.20,
            'covariance_yaw_threshold': 0.15,
            'startup_delay': 3.0,
        }],
        output='screen',
    )

    amcl_global_localizer = Node(
        package='my_robot_nav',
        executable='amcl_global_localizer.py',
        name='amcl_global_localizer',
        condition=IfCondition(global_localize_on_start),
        output='screen',
        parameters=[{
            'enabled': global_localize_on_start,
            'startup_delay_sec': global_localization_delay,
            'retry_interval_sec': 10.0,
            'max_retry_count': 3,
            'localization_topic': '/amcl_pose',
            'initial_pose_topic': '/initialpose',
            'covariance_x_threshold': 0.12,
            'covariance_y_threshold': 0.12,
            'covariance_yaw_threshold': 0.10,
        }],
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
            'autostart': nav2_autostart,
            'use_composition': 'False',
        }.items(),
        condition=IfCondition(use_nav2_stack),
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
        name='nav_rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    initial_pose = Node(
        package='my_robot_nav',
        executable='initial_pose_publisher.py',
        name='initial_pose_publisher',
        condition=IfCondition(set_initial_pose),
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'map',
            'topic': '/initialpose_seed',
            'x': initial_pose_x,
            'y': initial_pose_y,
            'yaw': initial_pose_yaw,
            'delay_sec': initial_pose_delay,
            'repeat_count': 5,
            'repeat_interval': 0.5,
        }],
        output='screen',
    )

    initial_pose_relay = Node(
        package='my_robot_nav',
        executable='initial_pose_relay.py',
        name='initial_pose_relay',
        parameters=[{
            'input_topic': '/initialpose_raw',
            'output_topic': '/initialpose_seed',
            'timestamp_offset_sec': 0.08,
        }],
        output='screen',
    )

    bootstrap_lidar_localizer = Node(
        package='my_robot_nav',
        executable='lidar_loc.py',
        name='bootstrap_lidar_loc',
        parameters=[{
            'map_topic': '/map',
            'scan_topic': '/scan_filtered',
            'odom_topic': '/odom',
            'initial_pose_topic': '/initialpose_seed',
            'pose_topic': '/lidar_loc_pose',
            'publish_tf': False,
            'max_refinement_time_sec': 2.0,
            'coarse_xy_window': 0.18,
            'coarse_xy_step': 0.03,
            'coarse_yaw_window': 0.20,
            'coarse_yaw_step': 0.04,
            'fine_xy_window': 0.04,
            'fine_xy_step': 0.01,
            'fine_yaw_window': 0.06,
            'fine_yaw_step': 0.015,
            'translation_penalty': 4.0,
            'rotation_penalty': 2.0,
            # Section: Bound the full session correction and reject wall-overlapping poses.
            'max_correction_distance': 0.15,
            'max_correction_yaw': 0.15,
            'min_score_improvement_per_beam': 0.01,
            'minimum_pose_clearance': 0.35,
            'treat_unknown_as_occupied': True,
        }],
        output='screen',
    )

    bootstrap_pose_promoter = Node(
        package='my_robot_nav',
        executable='bootstrap_pose_promoter.py',
        name='bootstrap_pose_promoter',
        parameters=[{
            'seed_topic': '/initialpose_seed',
            'refined_pose_topic': '/lidar_loc_pose',
            'output_topic': '/initialpose',
            'bootstrap_duration_sec': 2.0,
            'output_timestamp_offset_sec': 0.25,
            'publish_count': 2,
            'publish_interval_sec': 0.3,
            # Section: Apply the same hard bounds again before publishing to AMCL.
            'max_correction_distance': 0.15,
            'max_correction_yaw': 0.15,
            'tf_stamp_margin_sec': 0.10,
        }],
        output='screen',
    )

    navigate_named_pose_server = Node(
        package='my_robot_nav',
        executable='navigate_named_pose_server.py',
        name='navigate_named_pose_server',
        parameters=[{
            # Section: Honor the caller-provided waypoint file instead of hardcoding install space.
            'waypoints_file': waypoints_file,
            'action_name': '/nav/navigate_named_pose',
            'navigate_to_pose_action': '/navigate_to_pose',
            'navigate_to_pose_server_timeout_sec': navigate_to_pose_server_timeout_sec,
        }],
        output='screen',
    )

    # Section: Integrate the isolated saver without giving it control of navigation or hardware.
    waypoint_saver_server = Node(
        package='waypoint_saver',
        executable='waypoint_saver_server.py',
        name='waypoint_saver_server',
        condition=IfCondition(start_waypoint_saver),
        parameters=[{
            'service_name': waypoint_saver_service_name,
            'target_frame': 'map',
            'robot_frame': 'base_footprint',
            'transform_timeout_sec': waypoint_saver_transform_timeout_sec,
            'max_transform_age_sec': waypoint_saver_max_transform_age_sec,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'start_waypoint_saver',
            default_value='true',
            description='Start the read-only TF waypoint saver service',
        ),
        DeclareLaunchArgument(
            'waypoint_saver_service_name',
            default_value='/waypoint_saver/save_current',
            description='SaveWaypoint service name',
        ),
        DeclareLaunchArgument(
            'waypoint_saver_transform_timeout_sec',
            default_value='2.0',
            description='Maximum waypoint saver TF lookup wait in seconds',
        ),
        DeclareLaunchArgument(
            'waypoint_saver_max_transform_age_sec',
            default_value='1.0',
            description='Maximum accepted waypoint saver TF age in seconds',
        ),
        DeclareLaunchArgument(
            'use_external_drivers',
            default_value='false',
            description='Expect lidar/base/camera/odom drivers to be launched elsewhere',
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
            'nav2_params_file',
            default_value=default_nav2_params,
            description=('Full path to Nav2 params yaml. Override to swap '
                         'the default stack — for example to use the '
                         'IAIG-Nav DRL local planner instead of the stock '
                         'RegulatedPurePursuitController.'),
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
            'rviz_config',
            default_value=default_rviz_config_file,
            description='Full path to the RViz config file to launch',
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
            description=(
                'Seconds named-pose navigation waits for '
                '/navigate_to_pose to appear'
            ),
        ),
        DeclareLaunchArgument(
            'use_nav2_stack',
            default_value='true',
            description=('Include nav2_bringup navigation_launch.py '
                         '(controller/planner/bt_navigator + '
                         'lifecycle_manager_navigation). Set false when an '
                         'overlay launch (e.g. real_robot_iaig) spawns its '
                         'own lean subset of Nav2 nodes.'),
        ),
        DeclareLaunchArgument(
            'use_startup_gate',
            default_value='true',
            description=(
                'Start Nav2 only after odom and a 2D Pose Estimate are available'
            ),
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
            'set_initial_pose',
            default_value='false',
            description='Publish an initial pose after startup',
        ),
        DeclareLaunchArgument(
            'global_localize_on_start',
            default_value='false',
            description=(
                'Request AMCL global localization at startup when no initial '
                'pose is known'
            ),
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
            default_value='0.50',
            description='Linear speed clamp passed to the real robot drive chain',
        ),
        DeclareLaunchArgument(
            'base_angular_speed_limit',
            default_value='0.65',
            description='Angular speed clamp passed to the real robot drive chain',
        ),
        DeclareLaunchArgument(
            'nav_desired_linear_vel',
            default_value='0.40',
            description='Desired forward speed used by Nav2 in localization mode',
        ),
        DeclareLaunchArgument(
            'nav_rotate_to_heading_angular_vel',
            default_value='0.65',
            description='Rotate-to-heading speed used by Nav2 in localization mode',
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
        robot_bringup,
        localization,
        initial_pose_relay,
        bootstrap_lidar_localizer,
        bootstrap_pose_promoter,
        navigate_named_pose_server,
        waypoint_saver_server,
        amcl_global_localizer,
        TimerAction(
            period=nav2_delay,
            actions=[nav2, collision_monitor, collision_monitor_manager],
        ),
        TimerAction(
            period=PythonExpression([nav2_delay, ' + 1.0']),
            actions=[nav2_autostart_gate],
            condition=IfCondition(use_startup_gate),
        ),
        TimerAction(
            period=rviz_delay,
            actions=[rviz],
            condition=IfCondition(use_rviz),
        ),
        initial_pose,
    ])
