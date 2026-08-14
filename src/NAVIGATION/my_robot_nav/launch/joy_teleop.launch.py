import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_nav')
    joy_config = os.path.join(pkg_share, 'config', 'ps4_teleop.yaml')
    
    return LaunchDescription([
        # Joy 节点 - 读取手柄输入
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
            }],
            output='screen'
        ),
        
        # Teleop Twist Joy 节点 - 转换为 cmd_vel
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[joy_config],
            output='screen'
        ),
    ])
