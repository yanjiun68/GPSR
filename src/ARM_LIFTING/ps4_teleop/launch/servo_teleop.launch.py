
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    # Arguments
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev', default_value='/dev/input/js0',
        description='Joystick device path'
    )
    launch_joy_arg = DeclareLaunchArgument(
        'launch_joy', default_value='true',
        description='Whether to launch joy_node'
    )

    # Configs - Hardcoded to prevent loading/namespace issues
    # Configs - Hardcoded & Nested correctly
    servo_params = {
        'moveit_servo': {
            'move_group_name': 'interbotix_arm',
            'planning_frame': 'lift_base_link',
            'ee_frame_name': 'rx200/ee_gripper_link',
            'robot_link_command_frame': 'rx200/base_link',
            
            'command_in_type': 'speed_units',
            'scale': {
                'linear': 0.4,
                'rotational': 0.8,
                'joint': 0.5
            },
            'publish_period': 0.034,
            'low_pass_filter_coeff': 2.0,
            'command_out_type': 'trajectory_msgs/JointTrajectory',
            'publish_joint_positions': True,
            'publish_joint_velocities': True,
            'publish_joint_accelerations': False,
            'incoming_command_timeout': 0.1,
            'num_outgoing_halt_msgs_to_publish': 4,
            'lower_singularity_threshold': 17.0,
            'hard_stop_singularity_threshold': 30.0,
            'joint_limit_margin': 0.1,
            'leaving_singularity_threshold_multiplier': 2.0,
            'check_collisions': True,
            'collision_check_rate': 10.0,
            'self_collision_checking_planning_step_size': 0.01,
            'scene_collision_proximity_threshold': 0.02,
            # Topics
            'cartesian_command_in_topic': '~/delta_twist_cmds',
            'joint_command_in_topic': '~/delta_joint_cmds',
            'joint_topic': '/joint_states',
            'status_topic': '~/status',
            'command_out_topic': '/rx200/arm_controller/joint_trajectory',
            # Extra
            'is_primary_planning_scene_monitor': True,
        }
    }


    # Robot Description & SRDF (Required for MoveIt Servo)
    pkg_grasp = FindPackageShare('rx200_handbag_grasp')
    
    # URDF
    urdf_file = PathJoinSubstitution([pkg_grasp, 'urdf', 'combined.urdf.xacro'])
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_file,
        ' robot_name:=rx200',
        ' use_world_frame:=true',
        ' show_ar_tag:=true',
        ' show_gripper_bar:=true',
        ' show_gripper_fingers:=true',
        ' use_gripper:=true',
        ' hardware_type:=actual' # Assume actual since this is for teleop
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # SRDF
    srdf_file = PathJoinSubstitution([pkg_grasp, 'config', 'combined.srdf.xacro'])
    robot_description_semantic_content = Command([
        FindExecutable(name='xacro'), ' ', srdf_file,
        ' robot_name:=rx200'
    ])
    robot_description_semantic = {'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str)}

    # Nodes
    from launch.conditions import IfCondition
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
        condition=IfCondition(LaunchConfiguration('launch_joy'))
    )

    ps4_controller = Node(
        package='ps4_teleop',
        executable='ps4_servo_controller.py',
        name='ps4_servo_teleop',
        parameters=[
            {'linear_speed': 0.2},
            {'angular_speed': 1.0},
            {'frame_id': 'lift_base_link'},
        ],
        output='screen'
    )

    # Servo Node
    # Using default node name 'servo_node' to avoid parameter namespace issues
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        # name='servo_server', # REMOVED: Let it use default name
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
        ],
        output='screen'
    )

    return LaunchDescription([
        joy_dev_arg,
        launch_joy_arg,
        joy_node,
        servo_node,
        ps4_controller,
    ])
