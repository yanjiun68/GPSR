import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, AppendEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'my_robot_nav'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Paths
    xacro_file = os.path.join(pkg_share, 'urdf', 'my_car.urdf.xacro')
    world_file = os.path.join(pkg_share, 'world', 'static_world.sdf')
    models_path = os.path.join(pkg_share, 'world', 'models')

    # Launch Configs
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Set Env Var
    set_env = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        models_path
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', xacro_file])
        }]
    )

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Spawn Entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_car',
            '-topic', 'robot_description',
            '-z', '0.15' # Spawn closer to ground
        ],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            # RGBD camera — Gz Sim rgbd_camera advertises these four topics
            # under the sensor's <topic> prefix (see URDF camera_link block).
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        # Rename to REP-118 conventional topics so downstream vision_node works.
        remappings=[
            ('/camera/image',        '/camera/color/image_raw'),
            ('/camera/depth_image',  '/camera/depth/image_raw'),
            ('/camera/camera_info',  '/camera/color/camera_info'),
        ],
        output='screen'
    )

    return LaunchDescription([
        set_env,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
    ])
