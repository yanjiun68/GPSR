# Copyright 2024
# RX200 Official Grasp System - Complete Launch File
#
# Uses official Interbotix perception modules for object detection and grasping.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ========================================
    # Launch Arguments
    # ========================================
    robot_model_arg = DeclareLaunchArgument(
        'robot_model', default_value='rx200',
        description='Robot model name'
    )

    hardware_type_arg = DeclareLaunchArgument(
        'hardware_type', default_value='actual',
        description='Hardware type: actual or fake'
    )

    use_moveit_arg = DeclareLaunchArgument(
        'use_moveit', default_value='true',
        description='Launch MoveIt for motion planning'
    )

    use_camera_arg = DeclareLaunchArgument(
        'use_camera', default_value='true',
        description='Launch RealSense camera'
    )

    use_pointcloud_arg = DeclareLaunchArgument(
        'use_pointcloud', default_value='true',
        description='Launch PointCloud filter pipeline'
    )

    use_apriltag_arg = DeclareLaunchArgument(
        'use_apriltag', default_value='true',
        description='Launch AprilTag + ArmTag for calibration'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz visualization'
    )

    # Launch configurations
    robot_model = LaunchConfiguration('robot_model')
    hardware_type = LaunchConfiguration('hardware_type')
    use_moveit = LaunchConfiguration('use_moveit')
    use_camera = LaunchConfiguration('use_camera')
    use_pointcloud = LaunchConfiguration('use_pointcloud')
    use_apriltag = LaunchConfiguration('use_apriltag')
    use_rviz = LaunchConfiguration('use_rviz')

    # Package paths
    grasp_pkg = FindPackageShare('rx200_official_grasp')
    xsarm_moveit_pkg = FindPackageShare('interbotix_xsarm_moveit')
    perception_pkg = FindPackageShare('interbotix_perception_modules')

    # Config files
    grasp_params = PathJoinSubstitution([grasp_pkg, 'config', 'grasp_params.yaml'])
    filter_params = PathJoinSubstitution([grasp_pkg, 'config', 'filter_params.yaml'])
    tags_config = PathJoinSubstitution([grasp_pkg, 'config', 'tags.yaml'])

    # ========================================
    # MoveIt Launch (includes robot control)
    # ========================================
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                xsarm_moveit_pkg, 'launch', 'xsarm_moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model,
            'hardware_type': hardware_type,
            'use_moveit_rviz': use_rviz,
            'show_ar_tag': 'true',
        }.items(),
        condition=IfCondition(use_moveit)
    )

    # ========================================
    # RealSense Camera
    # ========================================
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch', 'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': 'camera',
            'rgb_camera.profile': '640x480x30',
            'depth_module.profile': '640x480x30',
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
        }.items(),
        condition=IfCondition(use_camera)
    )

    # ========================================
    # Official PointCloud Filter Pipeline
    # ========================================
    pc_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                perception_pkg, 'launch', 'pc_filter.launch.py'
            ])
        ]),
        launch_arguments={
            'filter_ns': 'pc_filter',
            'filter_params': filter_params,
            'cloud_topic': '/camera/camera/depth/color/points',
            'use_pointcloud_tuner_gui': 'false',
        }.items(),
        condition=IfCondition(use_pointcloud)
    )

    # ========================================
    # AprilTag Detection (using apriltag_node directly)
    # ========================================
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        namespace='apriltag',
        remappings=[
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info'),
        ],
        parameters=[tags_config],
        output='screen',
        condition=IfCondition(use_apriltag)
    )

    # ========================================
    # Custom Nodes
    # ========================================
    perception_node = Node(
        package='rx200_official_grasp',
        executable='perception_node.py',
        name='perception_node',
        parameters=[grasp_params],
        output='screen'
    )

    grasp_executor_node = Node(
        package='rx200_official_grasp',
        executable='grasp_executor_node.py',
        name='grasp_executor',
        parameters=[grasp_params],
        output='screen'
    )

    pick_place_node = Node(
        package='rx200_official_grasp',
        executable='pick_place_node.py',
        name='pick_place_node',
        parameters=[grasp_params],
        output='screen'
    )

    # ========================================
    # Delayed Launches (for startup sequencing)
    # ========================================
    # AprilTag needs camera to be ready
    delayed_apriltag = TimerAction(
        period=3.0,
        actions=[apriltag_node]
    )

    # PointCloud filter needs camera to be ready
    delayed_pc_filter = TimerAction(
        period=4.0,
        actions=[pc_filter_launch]
    )

    # Custom nodes need perception modules ready
    delayed_custom_nodes = TimerAction(
        period=6.0,
        actions=[
            perception_node,
            grasp_executor_node,
            pick_place_node,
        ]
    )

    # ========================================
    # Launch Description
    # ========================================
    return LaunchDescription([
        # Arguments
        robot_model_arg,
        hardware_type_arg,
        use_moveit_arg,
        use_camera_arg,
        use_pointcloud_arg,
        use_apriltag_arg,
        use_rviz_arg,

        # Core systems
        moveit_launch,
        camera_launch,

        # Delayed nodes
        delayed_apriltag,
        delayed_pc_filter,
        delayed_custom_nodes,
    ])
