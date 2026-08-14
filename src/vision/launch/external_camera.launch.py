# Section: ROS 2 launch imports.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Section: Launch description entry point.
def generate_launch_description():
    # Special function: expose camera device path for USB camera selection.
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='',
        description='External camera device path, for example /dev/video0. Leave empty to use camera_index.',
    )

    # Special function: expose numeric camera index when no device path is given.
    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='OpenCV camera index used when camera_device is empty.',
    )

    # Special function: expose requested image width.
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='640',
        description='Requested camera image width.',
    )

    # Special function: expose requested image height.
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='480',
        description='Requested camera image height.',
    )

    # Special function: expose requested publishing FPS.
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30.0',
        description='Requested camera capture and publish rate.',
    )

    # Special function: expose output image topic.
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Topic used for published Image messages.',
    )

    # Special function: expose output camera info topic.
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera_info',
        description='Topic used for published CameraInfo messages.',
    )

    # Special function: expose image frame id.
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='external_camera_link',
        description='Frame id stamped into published image headers.',
    )

    # Section: External camera node process.
    external_camera_node = Node(
        package='vision',
        executable='external_camera_node.py',
        name='external_camera_node',
        output='screen',
        parameters=[{
            'camera_device': LaunchConfiguration('camera_device'),
            'camera_index': ParameterValue(LaunchConfiguration('camera_index'), value_type=int),
            'width': ParameterValue(LaunchConfiguration('width'), value_type=int),
            'height': ParameterValue(LaunchConfiguration('height'), value_type=int),
            'fps': ParameterValue(LaunchConfiguration('fps'), value_type=float),
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
    )

    # Section: Launch all declared arguments before starting the node.
    return LaunchDescription([
        camera_device_arg,
        camera_index_arg,
        width_arg,
        height_arg,
        fps_arg,
        image_topic_arg,
        camera_info_topic_arg,
        frame_id_arg,
        external_camera_node,
    ])
