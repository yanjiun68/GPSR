import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ---------------- Arguments ----------------
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    arduino_port = LaunchConfiguration('arduino_port', default='/dev/ttyUSB1') 
    
    # ============================================================================
    # 以下底盘驱动部分已注释，改为由 real_robot.launch.py 启动
    # 使用时请先运行: ros2 launch my_robot_nav real_robot.launch.py
    # ============================================================================
    
    # # ---------------- Robot Parts ----------------
    # # (Based on user provided code)
    # pkg_name = 'my_robot_nav'
    # pkg_share_path = get_package_share_directory(pkg_name)
    # xacro_file = os.path.join(pkg_share_path, 'urdf', 'my_car.urdf.xacro')

    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'robot_description': Command(['xacro ', xacro_file])
    #     }]
    # )

    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )

    # # Lidar
    # lidar_node = Node(
    #     package='sllidar_ros2',
    #     executable='sllidar_node',
    #     name='sllidar_node',
    #     parameters=[{
    #         'channel_type': 'serial',
    #         'serial_port': serial_port, 
    #         'serial_baudrate': 115200, 
    #         'frame_id': 'lidar_link',
    #         'inverted': False, 
    #         'angle_compensate': True
    #     }],
    #     remappings=[('/scan', '/scan_raw')],
    #     output='screen'
    # )

    # scan_fixer = Node(
    #     package='my_robot_nav',
    #     executable='scan_fixer.py',
    #     name='scan_fixer',
    #     output='screen'
    # )

    # base_driver = Node(
    #     package='my_robot_nav',
    #     executable='base_driver.py',
    #     name='base_driver',
    #     output='screen',
    #     parameters=[{
    #         'port': arduino_port,
    #         'baud': 115200
    #     }]
    # )

    # kinematics_node = Node(
    #     package='my_robot_nav',
    #     executable='cmd_vel_to_targets.py',
    #     name='cmdvel_to_targets',
    #     output='screen'
    # )

    # odometry_node = Node(
    #     package='my_robot_nav',
    #     executable='omni_odometry.py',
    #     name='wheel_odometry',
    #     parameters=[{
    #         # Using defaults or what was provided in prompt
    #         'angle_scale_factor': 1.04,
    #         'invert_z': False,
    #     }],
    #     output='screen'
    # )

    # # ---------------- Camera ----------------
    # # Launch Realsense (已由 real_robot.launch.py 启动)
    # realsense_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('realsense2_camera'),
    #             'launch',
    #             'rs_launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'align_depth.enable': 'true',
    #         'rgb_camera.profile': '640x480x30',
    #         'depth_module.profile': '640x480x30',
    #         'pointcloud.enable': 'false',
    #     }.items()
    # )

    # ============================================================================
    # 以下为视觉跟随节点，需要 real_robot.launch.py 先启动
    # ============================================================================

    # ---------------- Vision ----------------
    vision_node = Node(
        package='vision',
        executable='vision_node.py',
        name='vision_node',
        output='screen',
        parameters=[{
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/depth/image_rect_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            # 使用 map 坐标系，让 Nav2 可以正确规划
            'global_frame': 'map', 
            'debug_mode': False,
        }]
    )

    # ---------------- Follower (Nav2 版本) ----------------
    person_follower_nav2 = Node(
        package='vision',
        executable='person_follower_nav2.py',
        name='person_follower_nav2',
        output='screen',
        parameters=[{
            'target_distance': 1.0,  # 与人保持的距离 (米)
            'goal_update_interval': 1.0,  # 目标更新间隔 (秒)
        }]
    )

    return LaunchDescription([
        # 底盘驱动已注释，由 real_robot.launch.py 启动
        # robot_state_publisher,
        # joint_state_publisher,
        # lidar_node,
        # scan_fixer,
        # base_driver,
        # kinematics_node,
        # odometry_node,
        # realsense_launch,

        # 视觉跟随逻辑
        TimerAction(
            period=3.0,  # 等待 Nav2 和传感器就绪
            actions=[vision_node]
        ),
        
        TimerAction(
            period=5.0,
            actions=[person_follower_nav2]
        )
    ])
