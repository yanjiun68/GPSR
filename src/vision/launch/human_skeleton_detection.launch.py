"""Section: Launch the MediaPipe human pose detector."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Special function: Start MediaPipe Pose Landmarker Full from an RGB camera topic."""
    # Section: Launch-time detector options.
    image_topic = LaunchConfiguration("image_topic")
    model_path = LaunchConfiguration("model_path")
    num_poses = LaunchConfiguration("num_poses")
    min_detection_confidence = LaunchConfiguration("min_detection_confidence")
    min_presence_confidence = LaunchConfiguration("min_presence_confidence")
    min_tracking_confidence = LaunchConfiguration("min_tracking_confidence")
    process_every_n_frames = LaunchConfiguration("process_every_n_frames")
    publish_debug_image = LaunchConfiguration("publish_debug_image")
    show_gui = LaunchConfiguration("show_gui")
    enable_tts = LaunchConfiguration("enable_tts")
    start_voice_tts = LaunchConfiguration("start_voice_tts")
    min_usable_landmarks = LaunchConfiguration("min_usable_landmarks")
    min_torso_landmark_confidence = LaunchConfiguration("min_torso_landmark_confidence")
    min_pose_bbox_area_ratio = LaunchConfiguration("min_pose_bbox_area_ratio")
    duplicate_iou_threshold = LaunchConfiguration("duplicate_iou_threshold")
    duplicate_center_distance_ratio = LaunchConfiguration("duplicate_center_distance_ratio")
    prefer_single_person_mode = LaunchConfiguration("prefer_single_person_mode")
    gesture_detection_enabled = LaunchConfiguration("gesture_detection_enabled")
    gesture_min_visibility = LaunchConfiguration("gesture_min_visibility")
    gesture_mirror_input = LaunchConfiguration("gesture_mirror_input")
    gesture_raised_margin = LaunchConfiguration("gesture_raised_margin")
    gesture_pointing_min_elbow_angle_deg = LaunchConfiguration("gesture_pointing_min_elbow_angle_deg")
    gesture_pointing_min_extension = LaunchConfiguration("gesture_pointing_min_extension")
    gesture_pointing_max_vertical_offset = LaunchConfiguration("gesture_pointing_max_vertical_offset")
    gesture_wave_window_sec = LaunchConfiguration("gesture_wave_window_sec")
    gesture_wave_min_samples = LaunchConfiguration("gesture_wave_min_samples")
    gesture_wave_min_span = LaunchConfiguration("gesture_wave_min_span")
    gesture_wave_direction_deadband = LaunchConfiguration("gesture_wave_direction_deadband")
    gesture_wave_min_direction_changes = LaunchConfiguration("gesture_wave_min_direction_changes")
    gesture_stability_frames = LaunchConfiguration("gesture_stability_frames")
    gesture_result_hold_sec = LaunchConfiguration("gesture_result_hold_sec")
    gesture_history_reset_sec = LaunchConfiguration("gesture_history_reset_sec")
    gesture_tie_margin = LaunchConfiguration("gesture_tie_margin")

    # Section: Upper-body-only raised-arm and temporal waving options.
    arm_landmark_min_visibility = LaunchConfiguration("arm_landmark_min_visibility")
    raised_arm_vertical_margin = LaunchConfiguration("raised_arm_vertical_margin")
    raised_arm_elbow_margin = LaunchConfiguration("raised_arm_elbow_margin")
    wave_window_seconds = LaunchConfiguration("wave_window_seconds")
    wave_min_horizontal_range = LaunchConfiguration("wave_min_horizontal_range")
    wave_min_direction_changes = LaunchConfiguration("wave_min_direction_changes")
    wave_max_vertical_drift = LaunchConfiguration("wave_max_vertical_drift")
    wave_history_timeout_seconds = LaunchConfiguration("wave_history_timeout_seconds")
    # Section: Lower-body-only posture scoring and temporal stability options.
    posture_landmark_min_visibility = LaunchConfiguration("posture_landmark_min_visibility")
    sitting_knee_angle_max = LaunchConfiguration("sitting_knee_angle_max")
    standing_knee_angle_min = LaunchConfiguration("standing_knee_angle_min")
    sitting_hip_angle_max = LaunchConfiguration("sitting_hip_angle_max")
    hip_knee_sitting_ratio_max = LaunchConfiguration("hip_knee_sitting_ratio_max")
    hip_knee_standing_ratio_min = LaunchConfiguration("hip_knee_standing_ratio_min")
    posture_confirm_frames = LaunchConfiguration("posture_confirm_frames")
    posture_release_frames = LaunchConfiguration("posture_release_frames")
    posture_debug = LaunchConfiguration("posture_debug")
    posture_debug_interval_sec = LaunchConfiguration("posture_debug_interval_sec")
    standing_leg_extension_min = LaunchConfiguration("standing_leg_extension_min")
    torso_vertical_ratio_min = LaunchConfiguration("torso_vertical_ratio_min")
    posture_minimum_evidence = LaunchConfiguration("posture_minimum_evidence")
    lying_torso_angle_max = LaunchConfiguration("lying_torso_angle_max")
    lying_vertical_ratio_max = LaunchConfiguration("lying_vertical_ratio_max")
    lying_body_aspect_min = LaunchConfiguration("lying_body_aspect_min")
    lying_score_threshold = LaunchConfiguration("lying_score_threshold")
    lying_confirm_frames = LaunchConfiguration("lying_confirm_frames")
    lying_release_frames = LaunchConfiguration("lying_release_frames")
    posture_min_score = LaunchConfiguration("posture_min_score")
    posture_min_margin = LaunchConfiguration("posture_min_margin")
    standing_knee_full_score_angle = LaunchConfiguration("standing_knee_full_score_angle")
    standing_knee_partial_angle = LaunchConfiguration("standing_knee_partial_angle")
    sitting_knee_full_score_angle = LaunchConfiguration("sitting_knee_full_score_angle")
    sitting_knee_partial_angle = LaunchConfiguration("sitting_knee_partial_angle")
    lying_torso_full_score_angle = LaunchConfiguration("lying_torso_full_score_angle")
    lying_torso_partial_angle = LaunchConfiguration("lying_torso_partial_angle")
    posture_history_size = LaunchConfiguration("posture_history_size")
    default_model_path = PathJoinSubstitution([FindPackageShare("vision"), "models", "pose_landmarker_full.task"])

    # Section: MediaPipe pose detector node.
    pose_detector = Node(
        package="vision",
        executable="human_skeleton_detection_node.py",
        name="human_skeleton_detection_node",
        output="screen",
        parameters=[{
            "image_topic": image_topic,
            "model_path": model_path,
            "num_poses": ParameterValue(num_poses, value_type=int),
            "min_detection_confidence": ParameterValue(min_detection_confidence, value_type=float),
            "min_presence_confidence": ParameterValue(min_presence_confidence, value_type=float),
            "min_tracking_confidence": ParameterValue(min_tracking_confidence, value_type=float),
            "process_every_n_frames": ParameterValue(process_every_n_frames, value_type=int),
            "publish_debug_image": publish_debug_image,
            "show_gui": show_gui,
            "gui_window_name": "MediaPipe Human Pose",
            "enable_tts": enable_tts,
            "tts_service_name": "/voice/tts/say",
            "min_usable_landmarks": ParameterValue(min_usable_landmarks, value_type=int),
            "min_torso_landmark_confidence": ParameterValue(min_torso_landmark_confidence, value_type=float),
            "min_pose_bbox_area_ratio": ParameterValue(min_pose_bbox_area_ratio, value_type=float),
            "duplicate_iou_threshold": ParameterValue(duplicate_iou_threshold, value_type=float),
            "duplicate_center_distance_ratio": ParameterValue(duplicate_center_distance_ratio, value_type=float),
            "prefer_single_person_mode": prefer_single_person_mode,
            "gesture_detection.enabled": gesture_detection_enabled,
            "gesture_detection.min_visibility": ParameterValue(gesture_min_visibility, value_type=float),
            "gesture_detection.mirror_input": gesture_mirror_input,
            "gesture_detection.raised_margin": ParameterValue(gesture_raised_margin, value_type=float),
            "gesture_detection.pointing_min_elbow_angle_deg": ParameterValue(gesture_pointing_min_elbow_angle_deg, value_type=float),
            "gesture_detection.pointing_min_extension": ParameterValue(gesture_pointing_min_extension, value_type=float),
            "gesture_detection.pointing_max_vertical_offset": ParameterValue(gesture_pointing_max_vertical_offset, value_type=float),
            "gesture_detection.wave_window_sec": ParameterValue(gesture_wave_window_sec, value_type=float),
            "gesture_detection.wave_min_samples": ParameterValue(gesture_wave_min_samples, value_type=int),
            "gesture_detection.wave_min_span": ParameterValue(gesture_wave_min_span, value_type=float),
            "gesture_detection.wave_direction_deadband": ParameterValue(gesture_wave_direction_deadband, value_type=float),
            "gesture_detection.wave_min_direction_changes": ParameterValue(gesture_wave_min_direction_changes, value_type=int),
            "gesture_detection.stability_frames": ParameterValue(gesture_stability_frames, value_type=int),
            "gesture_detection.result_hold_sec": ParameterValue(gesture_result_hold_sec, value_type=float),
            "gesture_detection.history_reset_sec": ParameterValue(gesture_history_reset_sec, value_type=float),
            "gesture_detection.tie_margin": ParameterValue(gesture_tie_margin, value_type=float),
            "arm_landmark_min_visibility": ParameterValue(arm_landmark_min_visibility, value_type=float),
            "raised_arm_vertical_margin": ParameterValue(raised_arm_vertical_margin, value_type=float),
            "raised_arm_elbow_margin": ParameterValue(raised_arm_elbow_margin, value_type=float),
            "wave_window_seconds": ParameterValue(wave_window_seconds, value_type=float),
            "wave_min_horizontal_range": ParameterValue(wave_min_horizontal_range, value_type=float),
            "wave_min_direction_changes": ParameterValue(wave_min_direction_changes, value_type=int),
            "wave_max_vertical_drift": ParameterValue(wave_max_vertical_drift, value_type=float),
            "wave_history_timeout_seconds": ParameterValue(wave_history_timeout_seconds, value_type=float),
            "posture_landmark_min_visibility": ParameterValue(posture_landmark_min_visibility, value_type=float),
            "sitting_knee_angle_max": ParameterValue(sitting_knee_angle_max, value_type=float),
            "standing_knee_angle_min": ParameterValue(standing_knee_angle_min, value_type=float),
            "sitting_hip_angle_max": ParameterValue(sitting_hip_angle_max, value_type=float),
            "hip_knee_sitting_ratio_max": ParameterValue(hip_knee_sitting_ratio_max, value_type=float),
            "hip_knee_standing_ratio_min": ParameterValue(hip_knee_standing_ratio_min, value_type=float),
            "posture_confirm_frames": ParameterValue(posture_confirm_frames, value_type=int),
            "posture_release_frames": ParameterValue(posture_release_frames, value_type=int),
            "posture_debug": posture_debug,
            "posture_debug_interval_sec": ParameterValue(posture_debug_interval_sec, value_type=float),
            "standing_leg_extension_min": ParameterValue(standing_leg_extension_min, value_type=float),
            "torso_vertical_ratio_min": ParameterValue(torso_vertical_ratio_min, value_type=float),
            "posture_minimum_evidence": ParameterValue(posture_minimum_evidence, value_type=int),
            "lying_torso_angle_max": ParameterValue(lying_torso_angle_max, value_type=float),
            "lying_vertical_ratio_max": ParameterValue(lying_vertical_ratio_max, value_type=float),
            "lying_body_aspect_min": ParameterValue(lying_body_aspect_min, value_type=float),
            "lying_score_threshold": ParameterValue(lying_score_threshold, value_type=int),
            "lying_confirm_frames": ParameterValue(lying_confirm_frames, value_type=int),
            "lying_release_frames": ParameterValue(lying_release_frames, value_type=int),
            "posture_min_score": ParameterValue(posture_min_score, value_type=float),
            "posture_min_margin": ParameterValue(posture_min_margin, value_type=float),
            "standing_knee_full_score_angle": ParameterValue(standing_knee_full_score_angle, value_type=float),
            "standing_knee_partial_angle": ParameterValue(standing_knee_partial_angle, value_type=float),
            "sitting_knee_full_score_angle": ParameterValue(sitting_knee_full_score_angle, value_type=float),
            "sitting_knee_partial_angle": ParameterValue(sitting_knee_partial_angle, value_type=float),
            "lying_torso_full_score_angle": ParameterValue(lying_torso_full_score_angle, value_type=float),
            "lying_torso_partial_angle": ParameterValue(lying_torso_partial_angle, value_type=float),
            "posture_history_size": ParameterValue(posture_history_size, value_type=int),
            "poses_topic": "/human_pose/poses",
            "debug_image_topic": "/human_pose/debug_image",
            "pose_summary_topic": "/perception/human_poses",
            "gestures_topic": "/human_pose/gestures",
            "legacy_skeletons_topic": "/perception/human_skeletons",
        }],
    )

    # Section: Optional local TTS service used by pose announcements.
    voice_tts_node = Node(
        package="voice",
        executable="tts_node.py",
        name="voice_tts",
        output="screen",
        condition=IfCondition(start_voice_tts),
    )

    return LaunchDescription([
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw", description="RGB image topic from the camera adapter."),
        DeclareLaunchArgument("model_path", default_value=default_model_path, description="Path to pose_landmarker_full.task."),
        DeclareLaunchArgument("num_poses", default_value="4", description="Maximum number of poses to detect."),
        DeclareLaunchArgument("min_detection_confidence", default_value="0.60", description="Minimum pose detection confidence."),
        DeclareLaunchArgument("min_presence_confidence", default_value="0.60", description="Minimum pose presence confidence."),
        DeclareLaunchArgument("min_tracking_confidence", default_value="0.65", description="Minimum pose tracking confidence."),
        DeclareLaunchArgument("process_every_n_frames", default_value="1", description="Process every Nth camera frame."),
        DeclareLaunchArgument("publish_debug_image", default_value="true", description="Publish skeleton debug images."),
        DeclareLaunchArgument("show_gui", default_value="false", description="Show a local OpenCV monitor window."),
        DeclareLaunchArgument("enable_tts", default_value="false", description="Speak detected human pose status through /voice/tts/say."),
        DeclareLaunchArgument("start_voice_tts", default_value="false", description="Start the local voice TTS service."),
        DeclareLaunchArgument("min_usable_landmarks", default_value="12", description="Minimum usable landmarks required to publish a pose."),
        DeclareLaunchArgument("min_torso_landmark_confidence", default_value="0.50", description="Minimum torso confidence required to publish a pose."),
        DeclareLaunchArgument("min_pose_bbox_area_ratio", default_value="0.02", description="Minimum normalized pose bbox area required to publish a pose."),
        DeclareLaunchArgument("duplicate_iou_threshold", default_value="0.35", description="Suppress pose candidates with bbox IoU above this threshold."),
        DeclareLaunchArgument("duplicate_center_distance_ratio", default_value="0.18", description="Suppress pose candidates whose normalized centers are too close."),
        DeclareLaunchArgument("prefer_single_person_mode", default_value="false", description="Force only the best pose candidate to be published."),
        DeclareLaunchArgument("gesture_detection_enabled", default_value="true", description="Enable RoboCup GPSR gesture classification."),
        DeclareLaunchArgument("gesture_min_visibility", default_value="0.55", description="Minimum landmark visibility required for gesture joints."),
        DeclareLaunchArgument("gesture_mirror_input", default_value="false", description="Mirror pointing direction for mirrored camera input."),
        DeclareLaunchArgument("gesture_raised_margin", default_value="0.15", description="Raised-arm wrist-above-shoulder margin in torso scales."),
        DeclareLaunchArgument("gesture_pointing_min_elbow_angle_deg", default_value="145.0", description="Minimum shoulder-elbow-wrist angle for pointing."),
        DeclareLaunchArgument("gesture_pointing_min_extension", default_value="0.55", description="Minimum horizontal extension for pointing in torso scales."),
        DeclareLaunchArgument("gesture_pointing_max_vertical_offset", default_value="0.30", description="Maximum wrist-shoulder vertical offset for pointing in torso scales."),
        DeclareLaunchArgument("gesture_wave_window_sec", default_value="1.2", description="Wrist history window for waving."),
        DeclareLaunchArgument("gesture_wave_min_samples", default_value="8", description="Minimum valid wrist samples for waving."),
        DeclareLaunchArgument("gesture_wave_min_span", default_value="0.22", description="Minimum horizontal wrist span for waving in torso scales."),
        DeclareLaunchArgument("gesture_wave_direction_deadband", default_value="0.05", description="Deadband for waving direction changes in torso scales."),
        DeclareLaunchArgument("gesture_wave_min_direction_changes", default_value="2", description="Minimum alternating wrist direction changes for waving."),
        DeclareLaunchArgument("gesture_stability_frames", default_value="3", description="Frames required before accepting a gesture."),
        DeclareLaunchArgument("gesture_result_hold_sec", default_value="0.30", description="Hold accepted gesture result after brief loss."),
        DeclareLaunchArgument("gesture_history_reset_sec", default_value="0.50", description="Clear per-person gesture history after detection gap."),
        DeclareLaunchArgument("gesture_tie_margin", default_value="0.10", description="Return none for ambiguous same-class arm scores within this margin."),
        DeclareLaunchArgument("arm_landmark_min_visibility", default_value="0.55", description="Minimum shoulder, elbow, and wrist visibility for arm actions."),
        DeclareLaunchArgument("raised_arm_vertical_margin", default_value="0.04", description="Wrist-above-shoulder margin in shoulder-width units, or normalized image units as fallback."),
        DeclareLaunchArgument("raised_arm_elbow_margin", default_value="0.08", description="Allowed elbow offset below the shoulder in shoulder-width units, or normalized image units as fallback."),
        DeclareLaunchArgument("wave_window_seconds", default_value="1.5", description="Per-person wrist motion window used for waving."),
        DeclareLaunchArgument("wave_min_horizontal_range", default_value="0.12", description="Minimum shoulder-relative horizontal wrist range for waving."),
        DeclareLaunchArgument("wave_min_direction_changes", default_value="2", description="Minimum repeated horizontal wrist direction changes for waving."),
        DeclareLaunchArgument("wave_max_vertical_drift", default_value="0.15", description="Maximum shoulder-relative vertical wrist drift during a wave."),
        DeclareLaunchArgument("wave_history_timeout_seconds", default_value="2.0", description="Clear a missing person's wrist history after this timeout."),
        DeclareLaunchArgument("posture_landmark_min_visibility", default_value="0.55", description="Minimum hip, knee, and ankle visibility for posture geometry."),
        DeclareLaunchArgument("sitting_knee_angle_max", default_value="135.0", description="Maximum hip-knee-ankle angle in degrees for sitting evidence."),
        DeclareLaunchArgument("standing_knee_angle_min", default_value="160.0", description="Minimum hip-knee-ankle angle in degrees for standing evidence."),
        DeclareLaunchArgument("sitting_hip_angle_max", default_value="145.0", description="Maximum shoulder-hip-knee angle in degrees for sitting evidence."),
        DeclareLaunchArgument("hip_knee_sitting_ratio_max", default_value="0.45", description="Maximum torso-normalized hip-to-knee vertical distance for sitting evidence."),
        DeclareLaunchArgument("hip_knee_standing_ratio_min", default_value="0.65", description="Minimum torso-normalized hip-to-knee vertical distance for standing evidence."),
        DeclareLaunchArgument("posture_confirm_frames", default_value="3", description="Consistent frames needed to confirm an initial posture."),
        DeclareLaunchArgument("posture_release_frames", default_value="4", description="Consistent opposite frames needed to switch a confirmed posture."),
        DeclareLaunchArgument("posture_debug", default_value="false", description="Enable throttled lower-body posture feature logs."),
        DeclareLaunchArgument("posture_debug_interval_sec", default_value="2.0", description="Seconds between posture debug logs."),
        DeclareLaunchArgument("standing_leg_extension_min", default_value="0.90", description="Minimum normalized hip-to-ankle extension for standing evidence."),
        DeclareLaunchArgument("torso_vertical_ratio_min", default_value="0.45", description="Minimum vertical torso fraction for upright posture geometry."),
        DeclareLaunchArgument("posture_minimum_evidence", default_value="2", description="Independent cues needed for raw sitting or standing."),
        DeclareLaunchArgument("lying_torso_angle_max", default_value="25.0", description="Maximum torso angle from image horizontal for lying evidence, in degrees."),
        DeclareLaunchArgument("lying_vertical_ratio_max", default_value="0.35", description="Maximum normalized shoulder-hip vertical separation for lying evidence."),
        DeclareLaunchArgument("lying_body_aspect_min", default_value="1.8", description="Minimum visible body width-to-height aspect for lying evidence."),
        DeclareLaunchArgument("lying_score_threshold", default_value="3", description="Independent lying cues required before a raw lying decision."),
        DeclareLaunchArgument("lying_confirm_frames", default_value="4", description="Frames required to enter lying-down posture."),
        DeclareLaunchArgument("lying_release_frames", default_value="4", description="Frames required to leave lying-down posture."),
        DeclareLaunchArgument("posture_min_score", default_value="0.60", description="Minimum winning normalized posture score."),
        DeclareLaunchArgument("posture_min_margin", default_value="0.15", description="Minimum winner margin over the second posture score."),
        DeclareLaunchArgument("standing_knee_full_score_angle", default_value="170.0", description="Knee angle in degrees for full standing evidence."),
        DeclareLaunchArgument("standing_knee_partial_angle", default_value="145.0", description="Knee angle in degrees where standing evidence begins."),
        DeclareLaunchArgument("sitting_knee_full_score_angle", default_value="100.0", description="Knee angle in degrees for full sitting evidence."),
        DeclareLaunchArgument("sitting_knee_partial_angle", default_value="145.0", description="Knee angle in degrees where sitting evidence begins."),
        DeclareLaunchArgument("lying_torso_full_score_angle", default_value="12.0", description="Torso angle from horizontal in degrees for full lying evidence."),
        DeclareLaunchArgument("lying_torso_partial_angle", default_value="30.0", description="Torso angle from horizontal in degrees where lying evidence begins."),
        DeclareLaunchArgument("posture_history_size", default_value="5", description="Reserved bounded history length for posture calibration and temporal filtering."),
        pose_detector,
        voice_tts_node,
    ])
