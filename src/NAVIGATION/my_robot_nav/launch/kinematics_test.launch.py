from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arduino_port = LaunchConfiguration(
        'arduino_port',
        default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
    )

    wheel_radius = LaunchConfiguration('wheel_radius', default='0.05')
    lx = LaunchConfiguration('lx', default='0.2051')
    ly = LaunchConfiguration('ly', default='0.2051')

    tpr_m1 = LaunchConfiguration('tpr_m1', default='3332.0')
    tpr_m2 = LaunchConfiguration('tpr_m2', default='3332.0')
    tpr_m3 = LaunchConfiguration('tpr_m3', default='3332.0')
    tpr_m4 = LaunchConfiguration('tpr_m4', default='3332.0')

    invert_m1 = LaunchConfiguration('invert_m1', default='false')
    invert_m2 = LaunchConfiguration('invert_m2', default='false')
    invert_m3 = LaunchConfiguration('invert_m3', default='false')
    invert_m4 = LaunchConfiguration('invert_m4', default='false')

    enc_sign_m1 = LaunchConfiguration('enc_sign_m1', default='1')
    enc_sign_m2 = LaunchConfiguration('enc_sign_m2', default='1')
    enc_sign_m3 = LaunchConfiguration('enc_sign_m3', default='1')
    enc_sign_m4 = LaunchConfiguration('enc_sign_m4', default='1')

    angle_scale_factor = LaunchConfiguration('angle_scale_factor', default='1.49')
    invert_z = LaunchConfiguration('invert_z', default='False')

    mode = LaunchConfiguration('mode', default='manual')
    sequence = LaunchConfiguration('sequence', default='all')
    trigger_topic = LaunchConfiguration('trigger_topic', default='/kinematics_test/trigger')
    start_delay_sec = LaunchConfiguration('start_delay_sec', default='3.0')
    move_duration_sec = LaunchConfiguration('move_duration_sec', default='2.0')
    pause_duration_sec = LaunchConfiguration('pause_duration_sec', default='1.5')
    forward_speed = LaunchConfiguration('forward_speed', default='0.08')
    strafe_speed = LaunchConfiguration('strafe_speed', default='0.08')
    angular_speed = LaunchConfiguration('angular_speed', default='0.35')

    base_driver = Node(
        package='my_robot_nav',
        executable='base_driver.py',
        name='base_driver',
        output='screen',
        parameters=[{
            'port': arduino_port,
            'baud': 115200,
        }],
    )

    cmd_vel_to_targets = Node(
        package='my_robot_nav',
        executable='cmd_vel_to_targets.py',
        name='cmdvel_to_targets',
        output='screen',
        parameters=[{
            'cmd_vel_topic': '/cmd_vel',
            'targets_topic': '/wheel_targets',
            'wheel_radius': wheel_radius,
            'lx': lx,
            'ly': ly,
            'tpr_m1': tpr_m1,
            'tpr_m2': tpr_m2,
            'tpr_m3': tpr_m3,
            'tpr_m4': tpr_m4,
            'invert_m1': invert_m1,
            'invert_m2': invert_m2,
            'invert_m3': invert_m3,
            'invert_m4': invert_m4,
        }],
    )

    odometry = Node(
        package='my_robot_nav',
        executable='omni_odometry.py',
        name='wheel_odometry',
        output='screen',
        parameters=[{
            'publish_tf': False,
            'odom_topic': '/odom',
            'wheel_radius': wheel_radius,
            'lx': lx,
            'ly': ly,
            'tpr_m1': tpr_m1,
            'tpr_m2': tpr_m2,
            'tpr_m3': tpr_m3,
            'tpr_m4': tpr_m4,
            'angle_scale_factor': angle_scale_factor,
            'invert_z': invert_z,
            'enc_sign_m1': enc_sign_m1,
            'enc_sign_m2': enc_sign_m2,
            'enc_sign_m3': enc_sign_m3,
            'enc_sign_m4': enc_sign_m4,
        }],
    )

    tester = Node(
        package='my_robot_nav',
        executable='kinematics_tester.py',
        name='kinematics_tester',
        output='screen',
        parameters=[{
            'cmd_vel_topic': '/cmd_vel',
            'targets_topic': '/wheel_targets',
            'ticks_topic': '/wheel_ticks',
            'odom_topic': '/odom',
            'trigger_topic': trigger_topic,
            'mode': mode,
            'sequence': sequence,
            'start_delay_sec': start_delay_sec,
            'move_duration_sec': move_duration_sec,
            'pause_duration_sec': pause_duration_sec,
            'forward_speed': forward_speed,
            'strafe_speed': strafe_speed,
            'angular_speed': angular_speed,
            'auto_shutdown': True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'arduino_port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            description='Serial port for the Arduino base controller',
        ),
        DeclareLaunchArgument('wheel_radius', default_value='0.05'),
        DeclareLaunchArgument('lx', default_value='0.2051'),
        DeclareLaunchArgument('ly', default_value='0.2051'),
        DeclareLaunchArgument('tpr_m1', default_value='3332.0'),
        DeclareLaunchArgument('tpr_m2', default_value='3332.0'),
        DeclareLaunchArgument('tpr_m3', default_value='3332.0'),
        DeclareLaunchArgument('tpr_m4', default_value='3332.0'),
        DeclareLaunchArgument('invert_m1', default_value='false'),
        DeclareLaunchArgument('invert_m2', default_value='false'),
        DeclareLaunchArgument('invert_m3', default_value='false'),
        DeclareLaunchArgument('invert_m4', default_value='false'),
        DeclareLaunchArgument('enc_sign_m1', default_value='1'),
        DeclareLaunchArgument('enc_sign_m2', default_value='1'),
        DeclareLaunchArgument('enc_sign_m3', default_value='1'),
        DeclareLaunchArgument('enc_sign_m4', default_value='1'),
        DeclareLaunchArgument('angle_scale_factor', default_value='1.49'),
        DeclareLaunchArgument('invert_z', default_value='False'),
        DeclareLaunchArgument('mode', default_value='manual'),
        DeclareLaunchArgument('sequence', default_value='all'),
        DeclareLaunchArgument('trigger_topic', default_value='/kinematics_test/trigger'),
        DeclareLaunchArgument('start_delay_sec', default_value='3.0'),
        DeclareLaunchArgument('move_duration_sec', default_value='2.0'),
        DeclareLaunchArgument('pause_duration_sec', default_value='1.5'),
        DeclareLaunchArgument('forward_speed', default_value='0.08'),
        DeclareLaunchArgument('strafe_speed', default_value='0.08'),
        DeclareLaunchArgument('angular_speed', default_value='0.35'),
        base_driver,
        cmd_vel_to_targets,
        odometry,
        tester,
    ])
