"""Section: Launch the name-only find-object perception action server."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Special function: Start FindObject perception server and optional TTS service."""
    # Section: Launch-time interface and behavior parameters.
    detections_topic = LaunchConfiguration("detections_topic")
    action_name = LaunchConfiguration("action_name")
    tts_service_name = LaunchConfiguration("tts_service_name")
    min_confidence = LaunchConfiguration("min_confidence")
    default_timeout_sec = LaunchConfiguration("default_timeout_sec")
    speak_on_found = LaunchConfiguration("speak_on_found")
    start_voice_tts = LaunchConfiguration("start_voice_tts")
    target_object_name = LaunchConfiguration("target_object_name")
    item_found_topic = LaunchConfiguration("item_found_topic")
    item_found_cooldown_sec = LaunchConfiguration("item_found_cooldown_sec")
    start_object_detector = LaunchConfiguration("start_object_detector")
    rgb_topic = LaunchConfiguration("rgb_topic")
    show_gui = LaunchConfiguration("show_gui")
    yolo_model_path = LaunchConfiguration("yolo_model_path")
    confidence_threshold = LaunchConfiguration("confidence_threshold")
    yolo_device = LaunchConfiguration("yolo_device")

    # Section: Default installed YOLO model path for the optional OpenCV detector.
    default_yolo_model_path = PathJoinSubstitution([FindPackageShare("vision"), "models", "yolo11n.pt"])

    # Section: Optional local TTS service for found-object announcements.
    voice_tts_node = Node(
        package="voice",
        executable="tts_node.py",
        name="voice_tts",
        output="screen",
        condition=IfCondition(start_voice_tts),
        parameters=[{"service_name": tts_service_name}],
    )

    # Section: Optional YOLO detector with OpenCV GUI for real camera validation.
    gpsr_object_detector = Node(
        package="vision",
        executable="gpsr_object_detection_node.py",
        name="gpsr_object_detection_node",
        output="screen",
        condition=IfCondition(start_object_detector),
        parameters=[{
            "rgb_topic": rgb_topic,
            "detections_topic": detections_topic,
            "annotated_image_topic": "/perception/object_detections_image",
            "yolo_model_path": yolo_model_path,
            "confidence_threshold": ParameterValue(confidence_threshold, value_type=float),
            "exclude_classes": ["person"],
            "publish_annotated_image": True,
            "show_gui": ParameterValue(show_gui, value_type=bool),
            "yolo_device": yolo_device,
        }],
    )

    # Section: Perception-only action server that compares detection names with the requested object.
    find_object_perception_server = Node(
        package="vision",
        executable="find_object_perception_server.py",
        name="find_object_perception_server",
        output="screen",
        parameters=[{
            "detections_topic": detections_topic,
            "action_name": action_name,
            "tts_service_name": tts_service_name,
            "min_confidence": ParameterValue(min_confidence, value_type=float),
            "default_timeout_sec": ParameterValue(default_timeout_sec, value_type=float),
            "speak_on_found": ParameterValue(speak_on_found, value_type=bool),
            "target_object_name": target_object_name,
            "item_found_topic": item_found_topic,
            "item_found_cooldown_sec": ParameterValue(item_found_cooldown_sec, value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("detections_topic", default_value="/perception/raw_object_detections", description="Structured object detections topic."),
        DeclareLaunchArgument("action_name", default_value="/perception/find_object", description="FindObject action endpoint."),
        DeclareLaunchArgument("tts_service_name", default_value="/voice/tts/say", description="TTS SayText service endpoint."),
        DeclareLaunchArgument("min_confidence", default_value="0.35", description="Minimum matching detection confidence."),
        DeclareLaunchArgument("default_timeout_sec", default_value="6.0", description="Default search timeout when action timeout is zero."),
        DeclareLaunchArgument("speak_on_found", default_value="true", description="Speak a TTS announcement when the target object is found."),
        DeclareLaunchArgument("start_voice_tts", default_value="true", description="Start local voice TTS service."),
        DeclareLaunchArgument("target_object_name", default_value="", description="Preset target object name for passive item-found detection."),
        DeclareLaunchArgument("item_found_topic", default_value="/perception/item_found", description="Topic that publishes structured item-found JSON."),
        DeclareLaunchArgument("item_found_cooldown_sec", default_value="3.0", description="Minimum seconds between repeated item-found outputs for the same target instance."),
        DeclareLaunchArgument("start_object_detector", default_value="false", description="Start the camera-backed YOLO object detector."),
        DeclareLaunchArgument("rgb_topic", default_value="/camera/image_raw", description="RGB image topic for the optional object detector."),
        DeclareLaunchArgument("show_gui", default_value="false", description="Open the optional detector OpenCV GUI window."),
        DeclareLaunchArgument("yolo_model_path", default_value=default_yolo_model_path, description="YOLO model path for the optional object detector."),
        DeclareLaunchArgument("confidence_threshold", default_value="0.35", description="YOLO detector confidence threshold."),
        DeclareLaunchArgument("yolo_device", default_value="cuda", description="YOLO CUDA inference device for the optional object detector; CPU fallback is disabled."),
        voice_tts_node,
        gpsr_object_detector,
        find_object_perception_server,
    ])
