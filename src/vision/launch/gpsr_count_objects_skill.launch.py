"""Section: Production launch for the ephemeral GPSR COUNT_OBJECTS stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Special function: Launch only nodes owned by the COUNT_OBJECTS skill."""
    # Section: Skill input and detector configuration.
    rgb_topic = LaunchConfiguration("rgb_topic")
    show_gui = LaunchConfiguration("show_gui")
    # Section: Use the requested workspace YOLOE segmentation weights for COUNT_OBJECTS.
    yolo_model_path = "/home/yanjiun68/workspace/WS1/PutrabotUPM-project/yoloe-26s-seg.pt"

    # Section: Temporal confirmation configuration shared with the existing test launch.
    confirmation_window_sec = LaunchConfiguration("confirmation_window_sec")
    confirmation_required_ratio = LaunchConfiguration("confirmation_required_ratio")
    confirmation_confidence_threshold = LaunchConfiguration("confirmation_confidence_threshold")
    confirmation_last_seen_timeout_sec = LaunchConfiguration("confirmation_last_seen_timeout_sec")
    confirmation_min_frames = LaunchConfiguration("confirmation_min_frames")

    # Section: COUNT_OBJECTS-owned detector consuming the persistent camera topic.
    gpsr_object_detector = Node(
        package="vision",
        executable="gpsr_object_detection_node.py",
        name="gpsr_object_detection_node",
        output="screen",
        parameters=[{
            "rgb_topic": rgb_topic,
            "detections_topic": "/perception/raw_object_detections",
            "annotated_image_topic": "/perception/object_detections_image",
            "yolo_model_path": yolo_model_path,
            "confidence_threshold": 0.35,
            "exclude_classes": ["person"],
            "publish_annotated_image": True,
            "show_gui": show_gui,
        }],
    )

    # Section: COUNT_OBJECTS-owned perception service.
    count_objects_perception_server = Node(
        package="vision",
        executable="count_objects_perception_server.py",
        name="count_objects_perception_server",
        output="screen",
        parameters=[{
            "detections_topic": "/perception/raw_object_detections",
            "service_name": "/perception/count_objects",
            "object_quantities_topic": "/perception/object_quantities",
            "raw_object_quantities_topic": "/perception/object_quantities_raw",
            "stale_timeout_sec": 5.0,
            "confirmation_window_sec": ParameterValue(confirmation_window_sec, value_type=float),
            "confirmation_required_ratio": ParameterValue(confirmation_required_ratio, value_type=float),
            "confirmation_confidence_threshold": ParameterValue(confirmation_confidence_threshold, value_type=float),
            "confirmation_last_seen_timeout_sec": ParameterValue(confirmation_last_seen_timeout_sec, value_type=float),
            "confirmation_min_frames": ParameterValue(confirmation_min_frames, value_type=int),
        }],
    )

    # Section: COUNT_OBJECTS-owned GPSR action bridge.
    count_objects_skill_server = Node(
        package="robot_yasmin_tasks",
        executable="count_objects_skill_server",
        name="count_objects_skill_server",
        output="screen",
        parameters=[{
            "action_name": "/gpsr/count_objects",
            "perception_service": "/perception/count_objects",
            "default_timeout_sec": 5.0,
        }],
    )

    # Section: Only skill-owned nodes are returned; camera and voice remain persistent.
    return LaunchDescription([
        DeclareLaunchArgument("show_gui", default_value="true", description="Show the external-camera detector debug window."),
        DeclareLaunchArgument("rgb_topic", default_value="/camera/image_raw", description="Persistent camera RGB topic."),
        DeclareLaunchArgument("confirmation_window_sec", default_value="2.0", description="Seconds of frames used for confirmation."),
        DeclareLaunchArgument("confirmation_required_ratio", default_value="0.60", description="Required detection-frame ratio."),
        DeclareLaunchArgument("confirmation_confidence_threshold", default_value="0.65", description="Minimum confidence for confirmation."),
        DeclareLaunchArgument("confirmation_last_seen_timeout_sec", default_value="1.0", description="Maximum confirmed-object age."),
        DeclareLaunchArgument("confirmation_min_frames", default_value="3", description="Minimum confirmation frames."),
        gpsr_object_detector,
        count_objects_perception_server,
        count_objects_skill_server,
    ])
