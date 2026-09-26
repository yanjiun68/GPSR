"""Section: MediaPipe human pose node helper tests."""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image

from putrabot_interfaces.msg import PoseLandmark


def load_pose_module():
    """Special function: Load the pose detector script as a testable Python module."""
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "human_skeleton_detection_node.py"
    spec = importlib.util.spec_from_file_location("human_skeleton_detection_node", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def make_ros_image(sec: int = 12, nanosec: int = 345_600_000) -> Image:
    """Special function: Build a minimal image message with a valid ROS timestamp."""
    msg = Image()
    msg.header.stamp = Time(sec=sec, nanosec=nanosec)
    msg.height = 480
    msg.width = 640
    return msg


def test_num_poses_capacity_validation() -> None:
    """Special function: Multi-person default range accepts four and rejects nonsense."""
    module = load_pose_module()
    assert module.validate_num_poses(4) == 4
    for invalid in (0, -1, 33, "many"):
        try:
            module.validate_num_poses(invalid)
        except ValueError:
            continue
        raise AssertionError(f"Expected invalid num_poses value to fail: {invalid}")


def make_landmark(x: float = 0.5, y: float = 0.5, z: float = 0.0, visibility: float = 0.9, presence: float = 0.9):
    """Special function: Build a MediaPipe-like landmark object for conversion tests."""
    return SimpleNamespace(x=x, y=y, z=z, visibility=visibility, presence=presence)


def make_pose_landmarks(visibility: float = 0.9, presence: float = 0.9, body_pose: str = "standing"):
    """Special function: Build one synthetic 33-landmark MediaPipe pose."""
    landmarks = [
        make_landmark(
            x=0.1 + 0.01 * index,
            y=0.2 + 0.01 * index,
            z=-0.01 * index,
            visibility=visibility,
            presence=presence,
        )
        for index in range(33)
    ]
    landmarks[11] = make_landmark(0.45, 0.20, 0.0, visibility, presence)
    landmarks[12] = make_landmark(0.55, 0.20, 0.0, visibility, presence)
    landmarks[23] = make_landmark(0.46, 0.45, 0.0, visibility, presence)
    landmarks[24] = make_landmark(0.54, 0.45, 0.0, visibility, presence)
    landmarks[25] = make_landmark(0.47, 0.65, 0.0, visibility, presence)
    landmarks[26] = make_landmark(0.53, 0.65, 0.0, visibility, presence)
    landmarks[27] = make_landmark(0.47, 0.85, 0.0, visibility, presence)
    landmarks[28] = make_landmark(0.53, 0.85, 0.0, visibility, presence)
    if body_pose == "sitting":
        landmarks[25] = make_landmark(0.35, 0.52, 0.0, visibility, presence)
        landmarks[26] = make_landmark(0.65, 0.52, 0.0, visibility, presence)
        landmarks[27] = make_landmark(0.30, 0.60, 0.0, visibility, presence)
        landmarks[28] = make_landmark(0.70, 0.60, 0.0, visibility, presence)
    if body_pose == "lying":
        landmarks[11] = make_landmark(0.20, 0.45, 0.0, visibility, presence)
        landmarks[12] = make_landmark(0.28, 0.45, 0.0, visibility, presence)
        landmarks[23] = make_landmark(0.50, 0.47, 0.0, visibility, presence)
        landmarks[24] = make_landmark(0.58, 0.47, 0.0, visibility, presence)
        landmarks[25] = make_landmark(0.72, 0.49, 0.0, visibility, presence)
        landmarks[26] = make_landmark(0.80, 0.49, 0.0, visibility, presence)
        landmarks[27] = make_landmark(0.88, 0.50, 0.0, visibility, presence)
        landmarks[28] = make_landmark(0.96, 0.50, 0.0, visibility, presence)
    return landmarks


def shifted_pose_landmarks(x_offset: float, visibility: float = 0.9, presence: float = 0.9):
    """Special function: Shift synthetic landmarks horizontally for multi-person filter tests."""
    shifted = []
    for landmark in make_pose_landmarks(visibility=visibility, presence=presence):
        shifted.append(make_landmark(
            x=min(max(landmark.x + x_offset, 0.0), 1.0),
            y=landmark.y,
            z=landmark.z,
            visibility=landmark.visibility,
            presence=landmark.presence,
        ))
    return shifted


def upper_body_only_pose_landmarks():
    """Special function: Build a chest-up pose with every hip and leg landmark unusable."""
    landmarks = make_pose_landmarks()
    for index in range(23, 33):
        landmarks[index] = make_landmark(0.5, 1.2, visibility=0.0, presence=0.0)
    return landmarks


def single_arm_only_pose_landmarks():
    """Special function: Build a pose candidate with only one complete arm chain visible."""
    landmarks = [
        make_landmark(0.5, 0.5, visibility=0.0, presence=0.0)
        for _index in range(33)
    ]
    landmarks[12] = make_landmark(0.55, 0.30)
    landmarks[14] = make_landmark(0.57, 0.21)
    landmarks[16] = make_landmark(0.59, 0.10)
    return landmarks


def make_pose_landmark_msg(visibility: float = 0.9, presence: float = 0.9) -> PoseLandmark:
    """Special function: Build one ROS pose landmark message for status tests."""
    landmark = PoseLandmark()
    landmark.x = 0.5
    landmark.y = 0.5
    landmark.z = 0.0
    landmark.visibility = visibility
    landmark.presence = presence
    return landmark


def test_ros_stamp_to_millis_uses_image_timestamp() -> None:
    """Special function: ROS image timestamps convert to MediaPipe VIDEO milliseconds."""
    module = load_pose_module()
    stamp = Time(sec=12, nanosec=345_600_000)
    assert module.ros_stamp_to_millis(stamp) == 12345


def test_landmark_conversion_preserves_confidence_and_nan() -> None:
    """Special function: Landmark conversion preserves visibility, presence, and missing coordinates."""
    module = load_pose_module()
    converted = module.landmark_to_msg(make_landmark(x=math.nan, visibility=0.42, presence=0.33))
    assert math.isnan(converted.x)
    assert converted.visibility == 0.42
    assert converted.presence == 0.33


def test_status_selection_detected_unreliable_and_not_detected() -> None:
    """Special function: Pose status follows torso confidence and landmark count rules."""
    module = load_pose_module()
    reliable = [make_pose_landmark_msg() for _ in range(33)]
    unreliable = [make_pose_landmark_msg() for _ in range(33)]
    unreliable[11].visibility = 0.10
    assert module.select_pose_status(reliable) == module.STATUS_DETECTED
    assert module.select_pose_status(unreliable) == module.STATUS_UNRELIABLE
    assert module.select_pose_status(reliable[:10]) == module.STATUS_NOT_DETECTED


def test_build_pose_array_publishes_33_landmarks() -> None:
    """Special function: MediaPipe detections convert into 33-landmark HumanPose messages."""
    module = load_pose_module()
    pose_array = module.build_pose_array(make_ros_image(), [make_pose_landmarks()])
    assert len(pose_array.poses) == 1
    assert pose_array.poses[0].person_id == 0
    assert pose_array.poses[0].status == module.STATUS_DETECTED
    assert pose_array.poses[0].pose_label == module.POSE_STANDING
    assert pose_array.poses[0].description == "Person 0 is standing."
    assert len(pose_array.poses[0].landmarks) == 33


def test_no_detection_publishes_empty_pose_array() -> None:
    """Special function: No MediaPipe detections become an empty pose array."""
    module = load_pose_module()
    pose_array = module.build_pose_array(make_ros_image(), [])
    assert pose_array.header.stamp.sec == 12
    assert pose_array.poses == []


def test_pose_summary_exposes_status_counts() -> None:
    """Special function: Compact summary reports primary status and status quantities."""
    module = load_pose_module()
    pose_array = module.build_pose_array(make_ros_image(), [make_pose_landmarks()])
    summary = module.build_pose_summary(pose_array)
    assert summary["count"] == 1
    assert summary["raw_count"] == 1
    assert summary["filtered_count"] == 1
    assert summary["primary_status"] == module.STATUS_DETECTED
    assert summary["primary_pose"] == module.POSE_STANDING
    assert summary["statuses"] == [module.STATUS_DETECTED]
    assert summary["poses"] == [module.POSE_STANDING]
    assert summary["descriptions"] == ["Person 0 is standing."]
    assert summary["status_quantities"] == {module.STATUS_DETECTED: 1}


def test_filter_keeps_one_valid_pose() -> None:
    """Special function: One valid MediaPipe pose remains published after filtering."""
    module = load_pose_module()
    raw = module.build_pose_array(make_ros_image(), [make_pose_landmarks()])
    filtered, diagnostics = module.filter_pose_candidates(raw)
    assert len(filtered.poses) == 1
    assert diagnostics["raw_count"] == 1
    assert diagnostics["filtered_count"] == 1


def test_filter_keeps_upper_body_only_pose() -> None:
    """Special function: Pose filtering does not discard a chest-up action candidate."""
    module = load_pose_module()
    raw = module.build_pose_array(make_ros_image(), [upper_body_only_pose_landmarks()])
    filtered, diagnostics = module.filter_pose_candidates(raw)
    assert raw.poses[0].status == module.STATUS_DETECTED
    assert len(filtered.poses) == 1
    assert diagnostics["rejected_low_quality"] == 0


def test_filter_keeps_single_reliable_arm_chain() -> None:
    """Special function: General pose-quality thresholds cannot discard one reliable arm."""
    module = load_pose_module()
    raw = module.build_pose_array(make_ros_image(), [single_arm_only_pose_landmarks()])
    filtered, diagnostics = module.filter_pose_candidates(raw)
    assert len(filtered.poses) == 1
    assert diagnostics["rejected_low_quality"] == 0


def test_filter_collapses_duplicate_pose_candidates() -> None:
    """Special function: Two overlapping pose candidates for one person collapse to one output."""
    module = load_pose_module()
    raw = module.build_pose_array(make_ros_image(), [make_pose_landmarks(), shifted_pose_landmarks(0.01)])
    filtered, diagnostics = module.filter_pose_candidates(raw)
    assert len(filtered.poses) == 1
    assert diagnostics["raw_count"] == 2
    assert diagnostics["suppressed_duplicates"] == 1


def test_filter_keeps_two_separated_people() -> None:
    """Special function: Two separated valid people remain as two outputs."""
    module = load_pose_module()
    raw = module.build_pose_array(make_ros_image(), [shifted_pose_landmarks(-0.20), shifted_pose_landmarks(0.20)])
    filtered, diagnostics = module.filter_pose_candidates(raw)
    assert len(filtered.poses) == 2
    assert diagnostics["filtered_count"] == 2
    assert diagnostics["suppressed_duplicates"] == 0


def test_filter_rejects_low_quality_partial_pose() -> None:
    """Special function: Low-confidence partial candidates are rejected before publishing."""
    module = load_pose_module()
    raw = module.build_pose_array(make_ros_image(), [make_pose_landmarks(visibility=0.1, presence=0.1)])
    filtered, diagnostics = module.filter_pose_candidates(raw)
    assert len(filtered.poses) == 0
    assert diagnostics["rejected_low_quality"] == 1


def test_filter_keeps_higher_quality_duplicate() -> None:
    """Special function: Duplicate suppression preserves the highest-quality candidate."""
    module = load_pose_module()
    weak = make_pose_landmarks(visibility=0.6, presence=0.6)
    strong = shifted_pose_landmarks(0.01, visibility=0.95, presence=0.95)
    raw = module.build_pose_array(make_ros_image(), [weak, strong])
    filtered, diagnostics = module.filter_pose_candidates(raw)
    assert len(filtered.poses) == 1
    assert diagnostics["suppressed_duplicates"] == 1
    assert filtered.poses[0].landmarks[11].visibility == 0.95


def test_filter_prefer_single_person_mode_keeps_best_pose() -> None:
    """Special function: Single-person preference publishes only the best candidate."""
    module = load_pose_module()
    raw = module.build_pose_array(make_ros_image(), [shifted_pose_landmarks(-0.20), shifted_pose_landmarks(0.20, visibility=0.95, presence=0.95)])
    filtered, diagnostics = module.filter_pose_candidates(raw, prefer_single_person_mode=True)
    assert len(filtered.poses) == 1
    assert diagnostics["suppressed_duplicates"] == 1


def test_pose_summary_reports_raw_and_filtered_counts() -> None:
    """Special function: Pose summary exposes raw and filtered candidate diagnostics."""
    module = load_pose_module()
    raw = module.build_pose_array(make_ros_image(), [make_pose_landmarks(), shifted_pose_landmarks(0.01)])
    filtered, diagnostics = module.filter_pose_candidates(raw)
    summary = module.build_pose_summary(filtered, diagnostics)
    assert summary["raw_count"] == 2
    assert summary["filtered_count"] == 1
    assert summary["suppressed_duplicates"] == 1


def test_body_pose_classification_uses_lower_body_for_sitting() -> None:
    """Special function: Body pose classifier labels reliable chair geometry as sitting without a lying fallback."""
    module = load_pose_module()
    sitting_array = module.build_pose_array(make_ros_image(), [make_pose_landmarks(body_pose="sitting")])
    lying_array = module.build_pose_array(make_ros_image(), [make_pose_landmarks(body_pose="lying")])
    assert sitting_array.poses[0].pose_label == module.POSE_SITTING
    assert lying_array.poses[0].pose_label == module.POSE_LYING_DOWN


def test_tts_report_describes_each_detected_human_pose() -> None:
    """Special function: TTS text reports per-person body pose descriptions."""
    module = load_pose_module()
    reliable = make_pose_landmarks(body_pose="standing")
    unreliable = make_pose_landmarks(visibility=0.9, presence=0.9, body_pose="sitting")
    unreliable[23].visibility = 0.3
    pose_array = module.build_pose_array(make_ros_image(), [reliable, unreliable])
    assert module.build_tts_report(pose_array) == "Person 0 is standing. Person 1 is sitting."


def test_tts_report_can_suppress_no_detection() -> None:
    """Special function: TTS text is empty for no detection unless explicitly enabled."""
    module = load_pose_module()
    pose_array = module.build_pose_array(make_ros_image(), [])
    assert module.build_tts_report(pose_array) == ""
    assert module.build_tts_report(pose_array, speak_no_detection=True) == "I do not detect a human pose."


def test_pose_summary_includes_gesture_fields() -> None:
    """Special function: Pose summary embeds stabilized gesture results per person."""
    module = load_pose_module()
    pose_array = module.build_pose_array(make_ros_image(), [make_pose_landmarks()])
    gesture = SimpleNamespace(
        gesture=module.GESTURE_POINTING_LEFT,
        active_arm="right",
        confidence=0.87,
        left_arm_state="NO_ARM_RAISED",
        right_arm_state="RIGHT_ARM_RAISED",
        raised_arm_state="RIGHT_ARM_RAISED",
        wave_state="NOT_WAVING",
        timestamp=12.3456,
        diagnostics={"horizontal_extension": 1.4},
    )
    summary = module.build_pose_summary(pose_array, gesture_results={0: gesture})
    assert summary["people"][0]["gesture"] == module.GESTURE_POINTING_LEFT
    assert summary["people"][0]["active_arm"] == "right"
    assert summary["people"][0]["gesture_confidence"] == 0.87
    assert summary["people"][0]["right_arm_state"] == "RIGHT_ARM_RAISED"
    assert summary["people"][0]["wave_state"] == "NOT_WAVING"
    assert summary["people"][0]["timestamp"] == 12.3456


def test_tts_report_includes_detected_gesture() -> None:
    """Special function: TTS text describes both body pose and GPSR gesture."""
    module = load_pose_module()
    pose_array = module.build_pose_array(make_ros_image(), [make_pose_landmarks()])
    gesture = SimpleNamespace(
        gesture=module.GESTURE_WAVING,
        active_arm="left",
        confidence=0.91,
        diagnostics={},
    )
    assert module.build_tts_report(pose_array, gesture_results={0: gesture}) == "Person 0 is standing and is waving with the left arm."


def test_side_facing_standing_is_not_sitting() -> None:
    """Special function: A side-facing straight leg cannot be classified as sitting from image height alone."""
    module = load_pose_module()
    landmarks = make_pose_landmarks()
    landmarks[25] = make_landmark(0.47, 0.58)
    landmarks[27] = make_landmark(0.47, 0.72)
    result = module.evaluate_body_posture([module.landmark_to_msg(item) for item in landmarks])
    assert result.raw_posture != module.POSE_SITTING


def test_slight_knee_bend_while_standing_is_not_sitting() -> None:
    """Special function: One modest knee bend lacks enough independent sitting evidence."""
    module = load_pose_module()
    landmarks = make_pose_landmarks()
    landmarks[25] = make_landmark(0.50, 0.64)
    result = module.evaluate_body_posture([module.landmark_to_msg(item) for item in landmarks])
    assert result.raw_posture != module.POSE_SITTING


def test_one_occluded_leg_uses_the_other_reliable_leg() -> None:
    """Special function: A reliable left leg still yields standing when the right leg is absent."""
    module = load_pose_module()
    landmarks = make_pose_landmarks()
    for index in (24, 26, 28):
        landmarks[index] = make_landmark(visibility=0.0, presence=0.0)
    result = module.evaluate_body_posture([module.landmark_to_msg(item) for item in landmarks])
    assert result.legs["left"]["reliable"] is True
    assert result.legs["right"]["reliable"] is False
    assert result.raw_posture == module.POSE_STANDING


def test_unavailable_lower_body_returns_unknown() -> None:
    """Special function: Missing hips knees and ankles cannot be fabricated into a sitting decision."""
    module = load_pose_module()
    landmarks = upper_body_only_pose_landmarks()
    result = module.evaluate_body_posture([module.landmark_to_msg(item) for item in landmarks])
    assert result.raw_posture in {module.POSE_UNKNOWN, module.POSE_UNRELIABLE}


def test_angle_degrees_uses_joint_middle_and_guards_degenerate_vectors() -> None:
    """Special function: Angle calculation is degree-valued and never produces NaN for overlapping joints."""
    module = load_pose_module()
    hip = module.landmark_to_msg(make_landmark(0.5, 0.4))
    knee = module.landmark_to_msg(make_landmark(0.5, 0.6))
    ankle = module.landmark_to_msg(make_landmark(0.5, 0.8))
    assert module.angle_degrees(hip, knee, ankle) == 180.0
    assert module.angle_degrees(knee, knee, ankle) is None


def test_invalid_landmark_ordering_does_not_create_standing() -> None:
    """Special function: An ankle above its knee cannot supply vertical standing evidence."""
    module = load_pose_module()
    landmarks = make_pose_landmarks()
    landmarks[27] = make_landmark(0.47, 0.55)
    landmarks[28] = make_landmark(0.53, 0.55)
    result = module.evaluate_body_posture([module.landmark_to_msg(item) for item in landmarks])
    assert result.raw_posture != module.POSE_STANDING


def test_posture_hysteresis_rejects_one_noisy_sitting_frame() -> None:
    """Special function: A confirmed standing track ignores one opposite raw posture frame."""
    module = load_pose_module()
    state = module.PostureTrackState()
    for _index in range(3):
        assert module.filter_posture_state(state, module.POSE_STANDING, 3, 4) in {module.POSE_UNKNOWN, module.POSE_STANDING}
    assert state.filtered_posture == module.POSE_STANDING
    assert module.filter_posture_state(state, module.POSE_SITTING, 3, 4) == module.POSE_STANDING


def test_missing_lower_body_does_not_block_raised_arm_detection() -> None:
    """Special function: Upper-body raised-arm detection remains independent of lower-body reliability."""
    module = load_pose_module()
    landmarks = upper_body_only_pose_landmarks()
    landmarks[11] = make_landmark(0.45, 0.30)
    landmarks[13] = make_landmark(0.45, 0.21)
    landmarks[15] = make_landmark(0.45, 0.10)
    classifier = module.GestureClassifier(module.GestureConfig())
    result = classifier.update(1.0, 1, [module.landmark_to_msg(item) for item in landmarks])
    assert result.left_arm_state == "LEFT_ARM_RAISED"


def test_posture_diagnostics_separate_standing_and_sitting_scores() -> None:
    """Special function: Standing and chair samples expose clearly different score diagnostics."""
    module = load_pose_module()
    standing = module.evaluate_body_posture([module.landmark_to_msg(item) for item in make_pose_landmarks()])
    sitting = module.evaluate_body_posture([module.landmark_to_msg(item) for item in make_pose_landmarks(body_pose="sitting")])
    assert standing.standing_score > standing.sitting_score
    assert sitting.sitting_score > sitting.standing_score


def test_lying_left_to_right_uses_multiple_torso_cues() -> None:
    """Special function: A left-to-right horizontal body reaches lying-down score from torso and aspect cues."""
    module = load_pose_module()
    result = module.evaluate_body_posture([module.landmark_to_msg(item) for item in make_pose_landmarks(body_pose="lying")])
    assert result.raw_posture == module.POSE_LYING_DOWN
    assert result.lying_score >= 0.60
    assert result.torso_angle_deg is not None
    assert result.body_aspect_ratio is not None


def test_lying_right_to_left_is_camera_symmetric() -> None:
    """Special function: Mirroring horizontal coordinates preserves lying-down classification."""
    module = load_pose_module()
    mirrored = []
    for item in make_pose_landmarks(body_pose="lying"):
        mirrored.append(make_landmark(1.0 - item.x, item.y, item.z, item.visibility, item.presence))
    result = module.evaluate_body_posture([module.landmark_to_msg(item) for item in mirrored])
    assert result.raw_posture == module.POSE_LYING_DOWN


def test_missing_legs_can_still_detect_lying_from_torso() -> None:
    """Special function: Reliable shoulders and hips are sufficient when lower-body landmarks are occluded."""
    module = load_pose_module()
    landmarks = make_pose_landmarks(body_pose="lying")
    for index in (25, 26, 27, 28):
        landmarks[index] = make_landmark(0.5, 1.2, visibility=0.0, presence=0.0)
    result = module.evaluate_body_posture([module.landmark_to_msg(item) for item in landmarks])
    assert result.raw_posture == module.POSE_LYING_DOWN


def test_missing_shoulder_or_hip_is_not_lying() -> None:
    """Special function: Missing torso anchors return unknown rather than fabricating horizontal geometry."""
    module = load_pose_module()
    landmarks = make_pose_landmarks(body_pose="lying")
    landmarks[11] = make_landmark(0.2, 0.45, visibility=0.0, presence=0.0)
    result = module.evaluate_body_posture([module.landmark_to_msg(item) for item in landmarks])
    assert result.raw_posture in {module.POSE_UNKNOWN, module.POSE_UNRELIABLE}


def test_sitting_and_standing_are_not_lying() -> None:
    """Special function: Upright standing and chair-sitting samples do not trigger lying."""
    module = load_pose_module()
    for kind in ("standing", "sitting"):
        result = module.evaluate_body_posture([module.landmark_to_msg(item) for item in make_pose_landmarks(body_pose=kind)])
        assert result.raw_posture != module.POSE_LYING_DOWN


def test_one_noisy_horizontal_frame_does_not_enter_lying() -> None:
    """Special function: Existing standing posture hysteresis ignores one horizontal raw frame."""
    module = load_pose_module()
    state = module.PostureTrackState()
    for _index in range(3):
        module.filter_posture_state(state, module.POSE_STANDING, 3, 4, 4, 4)
    assert state.filtered_posture == module.POSE_STANDING
    assert module.filter_posture_state(state, module.POSE_LYING_DOWN, 3, 4, 4, 4) == module.POSE_STANDING


def test_ambiguous_crouch_remains_unknown_from_competing_scores() -> None:
    """Special function: A partial crouch lacks the minimum score for a confident posture label."""
    module = load_pose_module()
    landmarks = make_pose_landmarks()
    landmarks[25] = make_landmark(0.40, 0.58)
    landmarks[26] = make_landmark(0.60, 0.58)
    landmarks[27] = make_landmark(0.48, 0.70)
    landmarks[28] = make_landmark(0.52, 0.70)
    result = module.evaluate_body_posture([module.landmark_to_msg(item) for item in landmarks])
    assert result.raw_posture == module.POSE_UNKNOWN
    assert max(result.raw_scores.values()) < module.PostureConfig().posture_min_score


def test_feature_extraction_is_separate_from_posture_scoring() -> None:
    """Special function: The reusable feature vector can be scored independently for future lightweight classifiers."""
    module = load_pose_module()
    landmarks = [module.landmark_to_msg(item) for item in make_pose_landmarks()]
    features, legs = module.extract_posture_features(landmarks)
    scores, margin = module.score_posture(features, legs)
    assert features["torso_angle_deg"] is not None
    assert scores[module.POSE_STANDING] > scores[module.POSE_SITTING]
    assert margin > 0.0
