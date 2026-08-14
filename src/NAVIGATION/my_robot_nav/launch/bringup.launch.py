import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'my_robot_nav'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Launch Configs
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sim.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    slam_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'slam_nav.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        sim_launch,
        
        # Add a delay for SLAM/Nav to ensure Sim is up
        TimerAction(
            period=5.0,
            actions=[slam_nav_launch]
        )
    ])
