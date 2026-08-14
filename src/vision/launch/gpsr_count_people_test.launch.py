"""Section: Test launch for GPSR count-people computer vision pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Special function: Start detector, people perception service, and GPSR action server together."""
    # Section: Camera and detector launch-time options.
    show_gui = LaunchConfiguration("show_gui")
    rgb_topic = LaunchConfiguration("rgb_topic")
    enable_count_report_tts = LaunchConfiguration("enable_count_report_tts")
    start_voice_tts = LaunchConfiguration("start_voice_tts")
    start_pose_detector = LaunchConfiguration("start_pose_detector")
    pose_model_path = LaunchConfiguration("pose_model_path")
    yolo_model_path = PathJoinSubstitution([FindPackageShare("vision"), "models", "yolo11n.pt"])

    # Section: Temporal confirmation options for stable people-count decisions.
    confirmation_window_sec = LaunchConfiguration("confirmation_window_sec")
    confirmation_required_ratio = LaunchConfiguration("confirmation_required_ratio")
    confirmation_confidence_threshold = LaunchConfiguration("confirmation_confidence_threshold")
    confirmation_last_seen_timeout_sec = LaunchConfiguration("confirmation_last_seen_timeout_sec")
    confirmation_min_frames = LaunchConfiguration("confirmation_min_frames")

    # Section: Camera-facing human detector with optional OpenCV GUI.
    gpsr_human_detector = Node(
        package="vision",
        executable="gpsr_object_detection_node.py",
        name="gpsr_human_detection_node",
        output="screen",
        parameters=[{
            "rgb_topic": rgb_topic,
            "detections_topic": "/perception/raw_human_detections",
            "annotated_image_topic": "/perception/human_detections_image",
            "yolo_model_path": yolo_model_path,
            "confidence_threshold": 0.35,
            "include_classes": ["person"],
            "exclude_classes": ["__none__"],
            "publish_annotated_image": True,
            "show_gui": show_gui,
            "gui_window_name": "GPSR Human Detection",
        }],
    )

    # Section: Optional pose detector for skeleton links and pose-aware people sorting.
    human_skeleton_detector = Node(
        package="vision",
        executable="human_skeleton_detection_node.py",
        name="human_skeleton_detection_node",
        output="screen",
        condition=IfCondition(start_pose_detector),
        parameters=[{
            "image_topic": rgb_topic,
            "model_path": pose_model_path,
            "num_poses": 2,
            "min_detection_confidence": 0.60,
            "min_presence_confidence": 0.60,
            "min_tracking_confidence": 0.65,
            "process_every_n_frames": 1,
            "publish_debug_image": True,
            "show_gui": show_gui,
            "gui_window_name": "MediaPipe Human Pose",
            "enable_tts": False,
            "tts_service_name": "/voice/tts/say",
            "min_usable_landmarks": 12,
            "min_torso_landmark_confidence": 0.50,
            "min_pose_bbox_area_ratio": 0.02,
            "duplicate_iou_threshold": 0.35,
            "duplicate_center_distance_ratio": 0.18,
            "prefer_single_person_mode": False,
            "poses_topic": "/human_pose/poses",
            "debug_image_topic": "/human_pose/debug_image",
            "pose_summary_topic": "/perception/human_poses",
            "gestures_topic": "/human_pose/gestures",
            "legacy_skeletons_topic": "/perception/human_skeletons",
        }],
    )

    # Section: Perception service required by /gpsr/count_people action server.
    count_people_perception_server = Node(
        package="vision",
        executable="count_people_perception_server.py",
        name="count_people_perception_server",
        output="screen",
        parameters=[{
            "detections_topic": "/perception/raw_human_detections",
            "service_name": "/perception/count_people",
            "human_quantities_topic": "/perception/human_quantities",
            "raw_human_quantities_topic": "/perception/human_quantities_raw",
            "enable_pose_sorting": True,
            "skeletons_topic": "/perception/human_skeletons",
            "stale_timeout_sec": 5.0,
            "confirmation_window_sec": ParameterValue(confirmation_window_sec, value_type=float),
            "confirmation_required_ratio": ParameterValue(confirmation_required_ratio, value_type=float),
            "confirmation_confidence_threshold": ParameterValue(confirmation_confidence_threshold, value_type=float),
            "confirmation_last_seen_timeout_sec": ParameterValue(confirmation_last_seen_timeout_sec, value_type=float),
            "confirmation_min_frames": ParameterValue(confirmation_min_frames, value_type=int),
        }],
    )

    # Section: GPSR-facing action server that the YASMIN COUNT_PEOPLE skill uses.
    count_people_skill_server = Node(
        package="robot_yasmin_tasks",
        executable="count_people_skill_server",
        name="count_people_skill_server",
        output="screen",
        parameters=[{
            "action_name": "/gpsr/count_people",
            "perception_service": "/perception/count_people",
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

    # Section: Blackboard-backed TTS reporter for stable human quantity summaries.
    count_people_blackboard_tts_node = Node(
        package="robot_yasmin_tasks",
        executable="count_people_blackboard_tts_node",
        name="count_people_blackboard_tts_node",
        output="screen",
        condition=IfCondition(enable_count_report_tts),
        parameters=[{
            "human_quantities_topic": "/perception/human_quantities",
            "tts_service_name": "/voice/tts/say",
            "speak_confirmed_only": True,
            "speak_empty_scene": False,
            "speak_on_change_only": True,
            "min_report_interval_sec": 5.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("show_gui", default_value="true", description="Open an OpenCV human-detection debug window."),
        DeclareLaunchArgument("rgb_topic", default_value="/camera/image_raw", description="RGB image topic from the camera adapter."),
        DeclareLaunchArgument("enable_count_report_tts", default_value="true", description="Speak confirmed human quantities through the GPSR TTS service."),
        DeclareLaunchArgument("start_voice_tts", default_value="true", description="Start the local /voice/tts/say service for people count reports."),
        DeclareLaunchArgument("start_pose_detector", default_value="false", description="Start the YOLO pose skeleton detector for pose-aware sorting."),
        DeclareLaunchArgument("pose_model_path", default_value=PathJoinSubstitution([FindPackageShare("vision"), "models", "pose_landmarker_full.task"]), description="Path to pose_landmarker_full.task."),
        DeclareLaunchArgument("confirmation_window_sec", default_value="2.0", description="Seconds of recent frames used to confirm a human."),
        DeclareLaunchArgument("confirmation_required_ratio", default_value="0.60", description="Minimum fraction of frames where a human must appear."),
        DeclareLaunchArgument("confirmation_confidence_threshold", default_value="0.35", description="Minimum detection confidence required for a frame vote."),
        DeclareLaunchArgument("confirmation_last_seen_timeout_sec", default_value="1.0", description="Maximum age since a human was last seen."),
        DeclareLaunchArgument("confirmation_min_frames", default_value="3", description="Minimum frames required before confirmation can pass."),
        gpsr_human_detector,
        human_skeleton_detector,
        count_people_perception_server,
        count_people_skill_server,
        voice_tts_node,
        count_people_blackboard_tts_node,
    ])
