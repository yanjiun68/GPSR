from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_file = LaunchConfiguration('urdf_file')
    use_gui = LaunchConfiguration('use_gui')

    pkg_share = FindPackageShare('lift_description')

    default_urdf = PathJoinSubstitution([pkg_share, 'urdf', 'lift.urdf.xacro'])
    default_rviz = PathJoinSubstitution([pkg_share, 'rviz', 'display.rviz'])

    robot_description = {'robot_description': Command(['xacro ', urdf_file])}

    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf_file',
            default_value=default_urdf,
            description='Absolute path to robot xacro file'
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Use joint_state_publisher_gui'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
            condition=None  # 保持简单：你想关GUI时直接不用启动它
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', default_rviz],
        ),
    ])
