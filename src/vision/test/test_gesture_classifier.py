"""Section: Pure-Python GPSR gesture classifier tests."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace


def load_gesture_module():
    """Special function: Load the gesture classifier module without ROS node startup."""
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "gesture_classifier.py"
    spec = importlib.util.spec_from_file_location("gesture_classifier", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def lm(x: float, y: float, visibility: float = 0.95):
    """Special function: Build a minimal landmark-like object."""
    return SimpleNamespace(x=x, y=y, z=0.0, visibility=visibility, presence=visibility)


def base_landmarks():
    """Special function: Build neutral standing MediaPipe landmarks for gesture tests."""
    landmarks = [lm(0.5, 0.5) for _ in range(33)]
    landmarks[11] = lm(0.45, 0.30)
    landmarks[12] = lm(0.55, 0.30)
    landmarks[13] = lm(0.43, 0.45)
    landmarks[14] = lm(0.57, 0.45)
    landmarks[15] = lm(0.42, 0.60)
    landmarks[16] = lm(0.58, 0.60)
    landmarks[23] = lm(0.46, 0.62)
    landmarks[24] = lm(0.54, 0.62)
    return landmarks


def hide_lower_body(landmarks):
    """Special function: Mark hips, knees, ankles, heels, and feet outside the usable view."""
    for index in range(23, 33):
        landmarks[index] = lm(0.5, 1.2, visibility=0.0)
    return landmarks


def stabilize(classifier, module, landmarks, person_id: int = 1, start: float = 0.0):
    """Special function: Feed enough repeated frames to pass temporal stabilization."""
    result = None
    for index in range(3):
        result = classifier.update(start + 0.1 * index, person_id, landmarks)
    assert result is not None
    return result


def test_left_arm_raised() -> None:
    """Special function: Left wrist above left shoulder detects raising_left_arm."""
    module = load_gesture_module()
    classifier = module.GestureClassifier()
    landmarks = base_landmarks()
    landmarks[13] = lm(0.43, 0.24)
    landmarks[15] = lm(0.42, 0.12)
    result = stabilize(classifier, module, landmarks)
    assert result.gesture == module.GESTURE_RAISING_LEFT_ARM
    assert result.active_arm == module.ARM_LEFT


def test_raised_hand_with_hips_outside_image() -> None:
    """Special function: A chest-up person remains eligible for raised-arm detection."""
    module = load_gesture_module()
    classifier = module.GestureClassifier()
    landmarks = hide_lower_body(base_landmarks())
    landmarks[13] = lm(0.43, 0.24)
    landmarks[15] = lm(0.42, 0.12)
    result = stabilize(classifier, module, landmarks)
    assert result.left_arm_state == module.LEFT_ARM_RAISED
    assert result.raised_arm_state == module.LEFT_ARM_RAISED


def test_right_arm_raised() -> None:
    """Special function: Right wrist above right shoulder detects raising_right_arm."""
    module = load_gesture_module()
    classifier = module.GestureClassifier()
    landmarks = base_landmarks()
    landmarks[14] = lm(0.57, 0.24)
    landmarks[16] = lm(0.58, 0.12)
    result = stabilize(classifier, module, landmarks)
    assert result.gesture == module.GESTURE_RAISING_RIGHT_ARM
    assert result.active_arm == module.ARM_RIGHT


def test_one_occluded_arm_does_not_hide_other_raised_arm() -> None:
    """Special function: One unreliable arm is independent from a reliable raised arm."""
    module = load_gesture_module()
    classifier = module.GestureClassifier()
    landmarks = hide_lower_body(base_landmarks())
    landmarks[15] = lm(0.42, 0.12, visibility=0.1)
    landmarks[14] = lm(0.57, 0.24)
    landmarks[16] = lm(0.58, 0.12)
    result = stabilize(classifier, module, landmarks)
    assert result.left_arm_state == module.ARM_POSE_UNRELIABLE
    assert result.right_arm_state == module.RIGHT_ARM_RAISED
    assert result.raised_arm_state == module.RIGHT_ARM_RAISED


def test_both_raised_arms_publish_combined_label() -> None:
    """Special function: Two reliable raised arms publish the combined raised label."""
    module = load_gesture_module()
    classifier = module.GestureClassifier()
    landmarks = hide_lower_body(base_landmarks())
    landmarks[13] = lm(0.43, 0.24)
    landmarks[15] = lm(0.42, 0.12)
    landmarks[14] = lm(0.57, 0.24)
    landmarks[16] = lm(0.58, 0.12)
    result = stabilize(classifier, module, landmarks)
    assert result.raised_arm_state == module.BOTH_ARMS_RAISED


def test_pointing_left_with_either_arm() -> None:
    """Special function: Either arm can point left when extended horizontally left."""
    module = load_gesture_module()
    for arm in (module.ARM_LEFT, module.ARM_RIGHT):
        classifier = module.GestureClassifier()
        landmarks = base_landmarks()
        shoulder_index, elbow_index, wrist_index = module.arm_indices(arm)
        shoulder = landmarks[shoulder_index]
        landmarks[elbow_index] = lm(shoulder.x - 0.22, shoulder.y)
        landmarks[wrist_index] = lm(shoulder.x - 0.48, shoulder.y)
        result = stabilize(classifier, module, landmarks)
        assert result.gesture == module.GESTURE_POINTING_LEFT
        assert result.active_arm == arm


def test_pointing_right_with_either_arm() -> None:
    """Special function: Either arm can point right when extended horizontally right."""
    module = load_gesture_module()
    for arm in (module.ARM_LEFT, module.ARM_RIGHT):
        classifier = module.GestureClassifier()
        landmarks = base_landmarks()
        shoulder_index, elbow_index, wrist_index = module.arm_indices(arm)
        shoulder = landmarks[shoulder_index]
        landmarks[elbow_index] = lm(shoulder.x + 0.22, shoulder.y)
        landmarks[wrist_index] = lm(shoulder.x + 0.48, shoulder.y)
        result = stabilize(classifier, module, landmarks)
        assert result.gesture == module.GESTURE_POINTING_RIGHT
        assert result.active_arm == arm


def test_left_hand_waving() -> None:
    """Special function: Alternating left wrist motion above shoulder detects waving."""
    module = load_gesture_module()
    classifier = module.GestureClassifier()
    result = None
    xs = [0.35, 0.55, 0.34, 0.56, 0.35, 0.55, 0.34, 0.56, 0.35, 0.55]
    for index, wrist_x in enumerate(xs):
        landmarks = base_landmarks()
        landmarks[13] = lm(0.43, 0.22)
        landmarks[15] = lm(wrist_x, 0.16)
        result = classifier.update(index * 0.1, 7, landmarks)
    assert result is not None
    assert result.gesture == module.GESTURE_WAVING
    assert result.active_arm == module.ARM_LEFT
    assert result.wave_state == module.LEFT_ARM_WAVING


def test_upper_body_only_waving() -> None:
    """Special function: Head and arm landmarks alone support temporal waving."""
    module = load_gesture_module()
    classifier = module.GestureClassifier()
    result = None
    xs = [0.35, 0.55, 0.34, 0.56, 0.35, 0.55, 0.34, 0.56, 0.35, 0.55]
    for index, wrist_x in enumerate(xs):
        landmarks = hide_lower_body(base_landmarks())
        landmarks[13] = lm(0.43, 0.22)
        landmarks[15] = lm(wrist_x, 0.16)
        result = classifier.update(index * 0.1, 70, landmarks)
    assert result is not None
    assert result.wave_state == module.LEFT_ARM_WAVING


def test_right_hand_waving() -> None:
    """Special function: Alternating right wrist motion above shoulder detects waving."""
    module = load_gesture_module()
    classifier = module.GestureClassifier()
    result = None
    xs = [0.45, 0.65, 0.44, 0.66, 0.45, 0.65, 0.44, 0.66, 0.45, 0.65]
    for index, wrist_x in enumerate(xs):
        landmarks = base_landmarks()
        landmarks[14] = lm(0.57, 0.22)
        landmarks[16] = lm(wrist_x, 0.16)
        result = classifier.update(index * 0.1, 8, landmarks)
    assert result is not None
    assert result.gesture == module.GESTURE_WAVING
    assert result.active_arm == module.ARM_RIGHT


def test_both_waving_arms_publish_combined_label() -> None:
    """Special function: Independent oscillating wrists publish the combined wave label."""
    module = load_gesture_module()
    classifier = module.GestureClassifier()
    result = None
    left_xs = [0.35, 0.55, 0.34, 0.56, 0.35, 0.55, 0.34, 0.56, 0.35, 0.55]
    right_xs = [0.45, 0.65, 0.44, 0.66, 0.45, 0.65, 0.44, 0.66, 0.45, 0.65]
    for index, (left_x, right_x) in enumerate(zip(left_xs, right_xs)):
        landmarks = hide_lower_body(base_landmarks())
        landmarks[13] = lm(0.43, 0.22)
        landmarks[15] = lm(left_x, 0.16)
        landmarks[14] = lm(0.57, 0.22)
        landmarks[16] = lm(right_x, 0.16)
        result = classifier.update(index * 0.1, 80, landmarks)
    assert result is not None
    assert result.wave_state == module.BOTH_ARMS_WAVING


def test_static_raised_hand_is_not_waving() -> None:
    """Special function: Static raised hand remains raised arm, not waving."""
    module = load_gesture_module()
    classifier = module.GestureClassifier()
    result = None
    for index in range(10):
        landmarks = base_landmarks()
        landmarks[13] = lm(0.43, 0.22)
        landmarks[15] = lm(0.42, 0.14)
        result = classifier.update(index * 0.1, 2, landmarks)
    assert result is not None
    assert result.gesture == module.GESTURE_RAISING_LEFT_ARM
    assert result.wave_state == module.NOT_WAVING


def test_small_landmark_jitter_is_not_waving() -> None:
    """Special function: Sub-deadband raised-wrist jitter does not create a wave."""
    module = load_gesture_module()
    classifier = module.GestureClassifier()
    result = None
    xs = [0.420, 0.423, 0.418, 0.422, 0.419, 0.421, 0.417, 0.422, 0.420, 0.419]
    for index, wrist_x in enumerate(xs):
        landmarks = base_landmarks()
        landmarks[13] = lm(0.43, 0.22)
        landmarks[15] = lm(wrist_x, 0.14)
        result = classifier.update(index * 0.1, 21, landmarks)
    assert result is not None
    assert result.wave_state == module.NOT_WAVING
    assert result.left_arm_state == module.LEFT_ARM_RAISED


def test_horizontal_motion_below_shoulder_is_not_waving() -> None:
    """Special function: Wrist motion below shoulder cannot count as waving."""
    module = load_gesture_module()
    classifier = module.GestureClassifier()
    result = None
    xs = [0.35, 0.55, 0.34, 0.56, 0.35, 0.55, 0.34, 0.56]
    for index, wrist_x in enumerate(xs):
        landmarks = base_landmarks()
        landmarks[15] = lm(wrist_x, 0.50)
        result = classifier.update(index * 0.1, 3, landmarks)
    assert result is not None
    assert result.gesture == module.GESTURE_NONE


def test_bent_arm_is_not_pointing() -> None:
    """Special function: Bent arm fails pointing angle threshold."""
    module = load_gesture_module()
    classifier = module.GestureClassifier()
    landmarks = base_landmarks()
    landmarks[13] = lm(0.35, 0.30)
    landmarks[15] = lm(0.35, 0.45)
    result = stabilize(classifier, module, landmarks)
    assert result.gesture == module.GESTURE_NONE


def test_low_visibility_joints_produce_none() -> None:
    """Special function: Required low-visibility arm joints reject gestures."""
    module = load_gesture_module()
    classifier = module.GestureClassifier()
    landmarks = base_landmarks()
    landmarks[15] = lm(0.42, 0.10, visibility=0.2)
    result = stabilize(classifier, module, landmarks)
    assert result.gesture == module.GESTURE_NONE
    assert result.left_arm_state == module.ARM_POSE_UNRELIABLE


def test_lower_body_loss_does_not_change_arm_classification() -> None:
    """Special function: Removing all lower-body confidence leaves arm states unchanged."""
    module = load_gesture_module()
    full_classifier = module.GestureClassifier()
    partial_classifier = module.GestureClassifier()
    full = base_landmarks()
    full[13] = lm(0.43, 0.24)
    full[15] = lm(0.42, 0.12)
    partial = hide_lower_body(base_landmarks())
    partial[13] = lm(0.43, 0.24)
    partial[15] = lm(0.42, 0.12)
    full_result = stabilize(full_classifier, module, full)
    partial_result = stabilize(partial_classifier, module, partial)
    assert partial_result.left_arm_state == full_result.left_arm_state
    assert partial_result.raised_arm_state == full_result.raised_arm_state


def test_waving_overrides_raised_arm() -> None:
    """Special function: Waving has priority over raised-arm output."""
    module = load_gesture_module()
    classifier = module.GestureClassifier()
    result = None
    xs = [0.35, 0.55, 0.34, 0.56, 0.35, 0.55, 0.34, 0.56, 0.35, 0.55]
    for index, wrist_x in enumerate(xs):
        landmarks = base_landmarks()
        landmarks[13] = lm(0.43, 0.22)
        landmarks[15] = lm(wrist_x, 0.12)
        result = classifier.update(index * 0.1, 4, landmarks)
    assert result is not None
    assert result.gesture == module.GESTURE_WAVING


def test_mirror_input_reverses_pointing_direction_only() -> None:
    """Special function: Mirroring flips pointing direction and not raised-arm side."""
    module = load_gesture_module()
    classifier = module.GestureClassifier(module.GestureConfig(mirror_input=True))
    landmarks = base_landmarks()
    landmarks[13] = lm(0.23, 0.30)
    landmarks[15] = lm(0.02, 0.30)
    result = stabilize(classifier, module, landmarks)
    assert result.gesture == module.GESTURE_POINTING_RIGHT
    landmarks = base_landmarks()
    landmarks[13] = lm(0.43, 0.24)
    landmarks[15] = lm(0.42, 0.12)
    result = stabilize(classifier, module, landmarks, person_id=2, start=1.0)
    assert result.gesture == module.GESTURE_RAISING_LEFT_ARM


def test_histories_do_not_leak_between_people() -> None:
    """Special function: Waving history is isolated by tracked person id."""
    module = load_gesture_module()
    classifier = module.GestureClassifier()
    xs = [0.35, 0.55, 0.34, 0.56, 0.35, 0.55, 0.34]
    for index, wrist_x in enumerate(xs):
        landmarks = base_landmarks()
        landmarks[15] = lm(wrist_x, 0.16)
        classifier.update(index * 0.1, 10, landmarks)
    other = base_landmarks()
    other[15] = lm(0.56, 0.16)
    result = classifier.update(0.8, 11, other)
    assert result.gesture == module.GESTURE_NONE
    assert result.wave_state == module.NOT_WAVING


def test_person_history_expires_after_timeout() -> None:
    """Special function: A missing person's wrist history is deleted after its timeout."""
    module = load_gesture_module()
    config = module.GestureConfig(wave_history_timeout_seconds=0.5)
    classifier = module.GestureClassifier(config)
    classifier.update(0.0, 42, base_landmarks())
    assert 42 in classifier.person_states
    classifier.prune_missing_people(0.51)
    assert 42 not in classifier.person_states
