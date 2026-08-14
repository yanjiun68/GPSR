# Comprehensive MoveIt Launch for RX200 + Lift
#
# ARCHITECTURE:
#   1. xs_sdk - DYNAMIXEL communication
#   2. robot_state_publisher - Combined URDF (single instance)
#   3. ros2_control_node - XSHardwareInterface for real hardware
#   4. Controllers: arm_controller, gripper_controller, joint_state_broadcaster
#   5. lift_serial_driver - Lift hardware communication
#   6. joint_state_merger - Combines arm + lift + computes mimic joints
#   7. move_group + rviz - MoveIt planning
#
# KEY FIXES:
#   - Single RSP (no duplicate from xsarm_ros_control)
#   - joint_state_merger computes right_finger = -left_finger (mimic)
#   - 50 Hz update rate for smooth motion
#   - Correct startup sequencing

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def load_yaml(package_name, file_path):
    """Load a YAML file as a dict."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Warning: Could not load {absolute_file_path}: {e}")
        return {}


def generate_launch_description():
    # ========================================
    # Arguments
    # ========================================
    robot_model_arg = DeclareLaunchArgument('robot_model', default_value='rx200')
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='rx200')
    use_lift_arg = DeclareLaunchArgument(
        'use_lift',
        default_value='true',
        description='Launch the real lift serial driver when true; otherwise use a fake lift controller.',
    )
    use_external_drivers_arg = DeclareLaunchArgument(
        'use_external_drivers',
        default_value='false',
        description='Assume shared RX200 drivers are already launched elsewhere.',
    )
    startup_lift_position_arg = DeclareLaunchArgument(
        'startup_lift_position',
        default_value='0.065',
        description='Lift position in meters after startup homing completes.',
    )

    use_lift = LaunchConfiguration('use_lift')
    startup_lift_position = LaunchConfiguration('startup_lift_position')

    pkg_grasp = FindPackageShare('rx200_handbag_grasp')

    # ========================================
    # Robot Description (Combined URDF with lift + arm)
    # ========================================
    urdf_file = PathJoinSubstitution([pkg_grasp, 'urdf', 'combined.urdf.xacro'])
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_file,
        ' robot_name:=rx200',
        ' use_world_frame:=true',
        ' show_ar_tag:=true',
        ' show_gripper_bar:=true',
        ' show_gripper_fingers:=true',
        ' use_gripper:=true',
        ' hardware_type:=actual'
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # ========================================
    # Semantic Description (SRDF) - for MoveIt
    # ========================================
    srdf_file = PathJoinSubstitution([pkg_grasp, 'config', 'combined.srdf.xacro'])
    robot_description_semantic_content = Command([
        FindExecutable(name='xacro'), ' ', srdf_file,
        ' robot_name:=rx200'
    ])
    robot_description_semantic = {'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str)}

    # ========================================
    # Config files
    # ========================================
    moveit_controllers_yaml = PathJoinSubstitution([pkg_grasp, 'config', 'moveit_controllers.yaml'])
    ros2_control_config = PathJoinSubstitution([pkg_grasp, 'config', 'ros2_controllers.yaml'])

    # Load YAML configs
    ompl_planning_yaml = load_yaml('rx200_handbag_grasp', 'config/ompl_planning.yaml')
    joint_limits_yaml = load_yaml('rx200_handbag_grasp', 'config/joint_limits.yaml')

    # Load kinematics config properly - must be under robot_description_kinematics key
    kinematics_params = load_yaml('rx200_handbag_grasp', 'config/kinematics.yaml')
    kinematics_yaml = {'robot_description_kinematics': kinematics_params if kinematics_params else {}}

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters':
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    joint_limits = {'robot_description_planning': joint_limits_yaml}

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # ========================================
    # ROBOT STATE PUBLISHER (Single instance, combined URDF)
    # ========================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        remappings=[('joint_states', '/rx200/joint_states')],
    )

    # ========================================
    # XS_SDK
    # Communicates with DYNAMIXEL motors via U2D2
    # ========================================
    xs_sdk = Node(
        package='interbotix_xs_sdk',
        executable='xs_sdk',
        name='xs_sdk',
        namespace='rx200',
        parameters=[{
            'motor_configs': PathJoinSubstitution([
                pkg_grasp,
                'config',
                'rx200_xs_sdk.yaml'
            ]),
            'mode_configs': PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_ros_control'),
                'config',
                'modes.yaml'
            ]),
            'load_configs': False,
        }],
        output='screen',
    )

    # ========================================
    # ROS2_CONTROL
    # ========================================
    ros2_control_node = TimerAction(
        period=3.0,  # Wait for xs_sdk to initialize
        actions=[Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace='rx200',
            parameters=[robot_description, ros2_control_config],
            output='screen',
        )],
    )

    # ========================================
    # CONTROLLER SPAWNERS
    # ========================================
    spawn_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            namespace='rx200',
            arguments=['joint_state_broadcaster', '-c', '/rx200/controller_manager'],
            output='screen',
        )],
    )

    spawn_arm_controller = TimerAction(
        period=7.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            namespace='rx200',
            arguments=['arm_controller', '-c', '/rx200/controller_manager'],
            output='screen',
        )],
    )

    spawn_gripper_controller = TimerAction(
        period=9.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            namespace='rx200',
            arguments=['gripper_controller', '-c', '/rx200/controller_manager'],
            output='screen',
        )],
    )

    # ========================================
    # LIFT COMPONENTS
    # ========================================
    lift_driver = TimerAction(
        period=2.0,  # Start after xs_sdk has connected to its port
        actions=[Node(
            package='lift_bridge',
            executable='lift_serial_driver',
            name='lift_serial_driver',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('lift_port'),
                'baud': 115200,
                'auto_home_on_start': True,
                'joint_name': 'lift_joint',
                'joint_states_topic': '/lift/joint_state',
                'min_m': 0.0,
                'max_m': 0.275,
                'timeout_min_s': 8.0,
            }],
            condition=IfCondition(use_lift),
        )],
    )

    fake_lift_driver = TimerAction(
        period=2.0,
        actions=[Node(
            package='lift_bridge',
            executable='fake_lift_controller',
            name='fake_lift_controller',
            output='screen',
            parameters=[{
                'joint_name': 'lift_joint',
                'joint_states_topic': '/lift/joint_state',
                'action_name': '/lift_controller/follow_joint_trajectory',
                'min_m': 0.0,
                'max_m': 0.275,
            }],
            condition=UnlessCondition(use_lift),
        )],
    )

    startup_lift_to_position = TimerAction(
        period=4.0,
        actions=[Node(
            package='rx200_handbag_grasp',
            executable='startup_lift_to_position.py',
            name='startup_lift_to_position',
            output='screen',
            parameters=[{
                'target_position': startup_lift_position,
                'joint_name': 'lift_joint',
                'action_name': '/lift_controller/follow_joint_trajectory',
                'homed_topic': '/lift/is_homed',
                'wait_timeout_s': 45.0,
                'move_duration_s': 4.0,
            }],
            condition=IfCondition(use_lift),
        )],
    )

    # Joint state merger: combine arm + lift + compute mimic joints
    joint_state_merger = Node(
        package='lift_bridge',
        executable='joint_state_merger',
        name='joint_state_merger',
        output='screen',
        parameters=[{
            # Use sdk_joint_states so merged /joint_states is available even before ros2_control controllers are active.
            'arm_topic': '/rx200/sdk_joint_states',
            'lift_topic': '/lift/joint_state',
            'out_topic': '/rx200/joint_states',
        }]
    )

    # ========================================
    # MOVEIT
    # ========================================
    move_group = TimerAction(
        period=15.0,  # Increased waiting time for controllers
        actions=[Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[
                {
                    'planning_scene_monitor_options': {
                        'robot_description': 'robot_description',
                        'joint_state_topic': '/rx200/joint_states',
                    },
                    'use_sim_time': False,
                },
                robot_description,
                robot_description_semantic,
                kinematics_yaml,
                ompl_planning_pipeline_config,
                trajectory_execution,
                moveit_controllers_yaml,
                planning_scene_monitor,
                joint_limits,
            ],
        )],
    )

    rviz_config = PathJoinSubstitution([pkg_grasp, 'rviz', 'handbag_grasp.rviz'])
    rviz = TimerAction(
        period=20.0,  # Increased waiting time for move_group
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config, '-f', 'world'],
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_yaml,
                ompl_planning_pipeline_config,
                moveit_controllers_yaml,
                {'use_sim_time': False},
            ],
        )],
    )

    return LaunchDescription([
        robot_model_arg,
        robot_name_arg,
        use_lift_arg,
        use_external_drivers_arg,
        startup_lift_position_arg,

        # Single Robot State Publisher (combined URDF)
        robot_state_publisher,

        # XS_SDK
        xs_sdk,

        # Lift (start early to avoid port conflicts)
        lift_driver,
        fake_lift_driver,
        startup_lift_to_position,
        joint_state_merger,

        # ros2_control + controllers
        ros2_control_node,
        spawn_joint_state_broadcaster,
        spawn_arm_controller,
        spawn_gripper_controller,

        # MoveIt
        move_group,
        rviz,
    ])
