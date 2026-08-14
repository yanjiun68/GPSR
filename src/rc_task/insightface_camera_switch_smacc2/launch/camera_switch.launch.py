from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera0_index = LaunchConfiguration('camera0_index')
    camera2_index = LaunchConfiguration('camera2_index')
    face_topic = LaunchConfiguration('face_topic')
    stage1_delay_sec = LaunchConfiguration('stage1_delay_sec')
    stage2_hold_sec = LaunchConfiguration('stage2_hold_sec')
    face_wait_timeout_sec = LaunchConfiguration('face_wait_timeout_sec')
    stream_info_period_sec = LaunchConfiguration('stream_info_period_sec')

    lifecycle_node = LifecycleNode(
        package='insightface_camera_switch_smacc2',
        executable='camera_switch_lifecycle_node',
        name='insightface_camera_switch_lifecycle',
        namespace='',
        output='screen',
        parameters=[
            {
                'camera0_index': ParameterValue(camera0_index, value_type=str),
                'camera2_index': ParameterValue(camera2_index, value_type=str),
                'face_topic': ParameterValue(face_topic, value_type=str),
                'stage1_delay_sec': ParameterValue(stage1_delay_sec, value_type=float),
                'stage2_hold_sec': ParameterValue(stage2_hold_sec, value_type=float),
                'face_wait_timeout_sec': ParameterValue(
                    face_wait_timeout_sec,
                    value_type=float,
                ),
                'stream_info_period_sec': ParameterValue(stream_info_period_sec, value_type=float),
            }
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_insightface_camera_switch',
        namespace='',
        output='screen',
        parameters=[
            {
                'autostart': True,
                'bond_timeout': 0.0,
                'node_names': ['insightface_camera_switch_lifecycle'],
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('camera0_index', default_value='0'),
            DeclareLaunchArgument('camera2_index', default_value='2'),
            DeclareLaunchArgument('face_topic', default_value='/perception/largest_face'),
            DeclareLaunchArgument('stage1_delay_sec', default_value='3.0'),
            DeclareLaunchArgument('stage2_hold_sec', default_value='10.0'),
            DeclareLaunchArgument(
                'face_wait_timeout_sec',
                default_value='0.0',
                description='0 means wait indefinitely for face detection.',
            ),
            DeclareLaunchArgument('stream_info_period_sec', default_value='1.0'),
            lifecycle_node,
            lifecycle_manager,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=lifecycle_node,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason='InsightFace camera switch lifecycle node exited.'
                            )
                        )
                    ],
                )
            ),
        ]
    )
