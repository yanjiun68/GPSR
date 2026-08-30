"""Section: Test launch for GPSR count-object computer vision pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Special function: Start detector, perception service, and GPSR action server together."""
    # Section: Camera and detector launch-time options.
    show_gui = LaunchConfiguration("show_gui")
    rgb_topic = LaunchConfiguration("rgb_topic")
    enable_count_report_tts = LaunchConfiguration("enable_count_report_tts")
    start_voice_tts = LaunchConfiguration("start_voice_tts")
    # Section: Use the requested workspace YOLOE segmentation weights for count-object testing.
    yolo_model_path = "/home/yanjiun68/workspace/WS1/PutrabotUPM-project/yoloe-26s-seg.pt"

    # Section: Temporal confirmation options for stable object-count decisions.
    confirmation_window_sec = LaunchConfiguration("confirmation_window_sec")
    confirmation_required_ratio = LaunchConfiguration("confirmation_required_ratio")
    confirmation_confidence_threshold = LaunchConfiguration("confirmation_confidence_threshold")
    confirmation_last_seen_timeout_sec = LaunchConfiguration("confirmation_last_seen_timeout_sec")
    confirmation_min_frames = LaunchConfiguration("confirmation_min_frames")

    # Section: Camera-facing object detector with optional OpenCV GUI.
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

    # Section: Perception service required by /gpsr/count_objects action server.
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

    # Section: GPSR-facing action server that the YASMIN skill and CLI action call use.
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

    # Section: Local TTS service used by the blackboard reporter.
    voice_tts_node = Node(
        package="voice",
        executable="tts_node.py",
        name="voice_tts",
        output="screen",
        condition=IfCondition(start_voice_tts),
    )

    # Section: Blackboard-backed TTS reporter for stable object quantity summaries.
    count_objects_blackboard_tts_node = Node(
        package="robot_yasmin_tasks",
        executable="count_objects_blackboard_tts_node",
        name="count_objects_blackboard_tts_node",
        output="screen",
        condition=IfCondition(enable_count_report_tts),
        parameters=[{
            "object_quantities_topic": "/perception/object_quantities",
            "tts_service_name": "/voice/tts/say",
            "speak_confirmed_only": True,
            "speak_empty_scene": False,
            "speak_on_change_only": True,
            "min_report_interval_sec": 5.0,
            "report_style": "combined",
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("show_gui", default_value="true", description="Open an OpenCV object-detection debug window."),
        DeclareLaunchArgument("rgb_topic", default_value="/camera/image_raw", description="RGB image topic from the Astra adapter."),
        DeclareLaunchArgument("enable_count_report_tts", default_value="true", description="Speak confirmed object quantities through the GPSR TTS service."),
        DeclareLaunchArgument("start_voice_tts", default_value="true", description="Start the local /voice/tts/say service for object count reports."),
        DeclareLaunchArgument("confirmation_window_sec", default_value="2.0", description="Seconds of recent frames used to confirm an object."),
        DeclareLaunchArgument("confirmation_required_ratio", default_value="0.60", description="Minimum fraction of frames where an object must appear."),
        DeclareLaunchArgument("confirmation_confidence_threshold", default_value="0.65", description="Minimum detection confidence required for a frame vote."),
        DeclareLaunchArgument("confirmation_last_seen_timeout_sec", default_value="1.0", description="Maximum age since the object was last seen."),
        DeclareLaunchArgument("confirmation_min_frames", default_value="3", description="Minimum frames required before confirmation can pass."),
        gpsr_object_detector,
        count_objects_perception_server,
        count_objects_skill_server,
        voice_tts_node,
        count_objects_blackboard_tts_node,
    ])
