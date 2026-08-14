#!/usr/bin/env python3
"""Section: Pure-Python RoboCup@Home GPSR gesture classifier."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any


# Section: MediaPipe landmark indices used by gesture rules.
LEFT_SHOULDER = 11
RIGHT_SHOULDER = 12
LEFT_ELBOW = 13
RIGHT_ELBOW = 14
LEFT_WRIST = 15
RIGHT_WRIST = 16
LEFT_HIP = 23
RIGHT_HIP = 24


# Section: Supported gesture and arm labels.
GESTURE_NONE = "none"
GESTURE_WAVING = "waving"
GESTURE_RAISING_LEFT_ARM = "raising_left_arm"
GESTURE_RAISING_RIGHT_ARM = "raising_right_arm"
GESTURE_RAISING_BOTH_ARMS = "raising_both_arms"
GESTURE_POINTING_LEFT = "pointing_left"
GESTURE_POINTING_RIGHT = "pointing_right"
ARM_LEFT = "left"
ARM_RIGHT = "right"
ARM_BOTH = "both"
ARM_UNKNOWN = "unknown"


# Section: Explicit upper-body arm-action labels published in gesture JSON.
LEFT_ARM_RAISED = "LEFT_ARM_RAISED"
RIGHT_ARM_RAISED = "RIGHT_ARM_RAISED"
BOTH_ARMS_RAISED = "BOTH_ARMS_RAISED"
NO_ARM_RAISED = "NO_ARM_RAISED"
ARM_POSE_UNRELIABLE = "ARM_POSE_UNRELIABLE"
LEFT_ARM_WAVING = "LEFT_ARM_WAVING"
RIGHT_ARM_WAVING = "RIGHT_ARM_WAVING"
BOTH_ARMS_WAVING = "BOTH_ARMS_WAVING"
NOT_WAVING = "NOT_WAVING"
WAVE_UNRELIABLE = "WAVE_UNRELIABLE"


@dataclass(slots=True)
class GestureConfig:
    """Special function: Store configurable gesture classifier thresholds."""

    # Section: Minimum required landmark visibility.
    min_visibility: float = 0.55

    # Section: Upper-body arm-action visibility threshold.
    arm_landmark_min_visibility: float = 0.55

    # Section: Flip pointing direction for mirrored camera input.
    mirror_input: bool = False

    # Section: Raised-arm geometry threshold.
    raised_margin: float = 0.15

    # Section: Upper-body raised-arm margins in shoulder-width units when available.
    raised_arm_vertical_margin: float = 0.04
    raised_arm_elbow_margin: float = 0.08

    # Section: Pointing geometry thresholds.
    pointing_min_elbow_angle_deg: float = 145.0
    pointing_min_extension: float = 0.55
    pointing_max_vertical_offset: float = 0.30

    # Section: Waving temporal thresholds.
    wave_window_sec: float = 1.2
    wave_min_samples: int = 8
    wave_min_span: float = 0.22
    wave_direction_deadband: float = 0.05
    wave_min_direction_changes: int = 2

    # Section: Upper-body temporal waving thresholds.
    wave_window_seconds: float = 1.5
    wave_min_horizontal_range: float = 0.12
    wave_max_vertical_drift: float = 0.15
    wave_history_timeout_seconds: float = 2.0

    # Section: Lightweight arm-coordinate exponential smoothing.
    landmark_smoothing_alpha: float = 0.45

    # Section: Temporal stabilization thresholds.
    stability_frames: int = 3
    result_hold_sec: float = 0.30
    history_reset_sec: float = 0.50
    tie_margin: float = 0.10


@dataclass(slots=True)
class GestureResult:
    """Special function: Return one stabilized gesture classification."""

    # Section: Detected gesture label.
    gesture: str = GESTURE_NONE

    # Section: Gesture confidence from 0.0 to 1.0.
    confidence: float = 0.0

    # Section: Arm responsible for the gesture.
    active_arm: str = ARM_UNKNOWN

    # Section: Diagnostic scores useful for tuning.
    diagnostics: dict[str, Any] = field(default_factory=dict)

    # Section: Independent left and right raised-arm states.
    left_arm_state: str = ARM_POSE_UNRELIABLE
    right_arm_state: str = ARM_POSE_UNRELIABLE

    # Section: Combined raised-arm and waving states.
    raised_arm_state: str = ARM_POSE_UNRELIABLE
    wave_state: str = WAVE_UNRELIABLE

    # Section: Independent left and right waving states.
    left_wave_state: str = WAVE_UNRELIABLE
    right_wave_state: str = WAVE_UNRELIABLE

    # Section: Source timestamp for this classification.
    timestamp: float = 0.0


@dataclass(slots=True)
class GestureCandidate:
    """Special function: Internal candidate before temporal stabilization."""

    # Section: Candidate gesture label.
    gesture: str

    # Section: Candidate confidence score.
    confidence: float

    # Section: Active arm for the candidate.
    active_arm: str

    # Section: Candidate diagnostics.
    diagnostics: dict[str, Any]


@dataclass(slots=True)
class WristSample:
    """Special function: Timestamped wrist and shoulder sample for waving."""

    # Section: Sample timestamp in seconds.
    timestamp: float

    # Section: Smoothed wrist coordinates in normalized image space.
    wrist_x: float
    wrist_y: float

    # Section: Shoulder coordinates in normalized image space.
    shoulder_x: float
    shoulder_y: float

    # Section: Weakest required arm-landmark visibility.
    visibility: float

    # Section: Shoulder-width scale, or one for normalized-image fallback.
    shoulder_scale: float

    # Section: Whether wrist is above or near shoulder.
    above_shoulder: bool


@dataclass(slots=True)
class SmoothedPoint:
    """Special function: Store one exponentially smoothed image-space point."""

    # Section: Smoothed normalized coordinates.
    x: float
    y: float

    # Section: Current landmark visibility retained for history validation.
    visibility: float


@dataclass(slots=True)
class ArmEvaluation:
    """Special function: Store one arm's current reliability and raised geometry."""

    # Section: Arm side and reliability decision.
    arm: str
    reliable: bool

    # Section: Current raised decision and confidence.
    raised: bool
    confidence: float

    # Section: Current upper-body points and normalization scale.
    shoulder: Any = None
    elbow: Any = None
    wrist: Any = None
    shoulder_scale: float = 1.0

    # Section: Per-arm diagnostics used by JSON and debug rendering.
    diagnostics: dict[str, Any] = field(default_factory=dict)


@dataclass(slots=True)
class PersonGestureState:
    """Special function: Per-person temporal gesture state."""

    # Section: Per-arm wrist motion history.
    wrist_history: dict[str, deque[WristSample]] = field(default_factory=lambda: {
        ARM_LEFT: deque(),
        ARM_RIGHT: deque(),
    })

    # Section: Last frame timestamp observed for this person.
    last_seen_sec: float | None = None

    # Section: Per-arm wrist and elbow smoothing state.
    smoothed_points: dict[str, dict[str, SmoothedPoint]] = field(default_factory=lambda: {
        ARM_LEFT: {},
        ARM_RIGHT: {},
    })

    # Section: Current unstable candidate key.
    candidate_key: tuple[str, str] = (GESTURE_NONE, ARM_UNKNOWN)

    # Section: Number of consecutive frames with the same candidate.
    candidate_frames: int = 0

    # Section: Last accepted stable result.
    accepted_result: GestureResult = field(default_factory=GestureResult)

    # Section: Timestamp when accepted result was updated.
    accepted_time_sec: float = 0.0


class GestureClassifier:
    """Special function: Classify and stabilize GPSR gestures from MediaPipe landmarks."""

    def __init__(self, config: GestureConfig | None = None) -> None:
        """Special function: Initialize classifier thresholds and person histories."""
        self.config = config or GestureConfig()
        self.person_states: dict[int, PersonGestureState] = {}

    def update(self, timestamp_sec: float, person_id: int, landmarks: list[Any]) -> GestureResult:
        """Special function: Update one person's gesture classification for one frame."""
        self.prune_missing_people(timestamp_sec)
        state = self.person_states.setdefault(int(person_id), PersonGestureState())

        # Section: Preserve legacy candidate reset behavior without discarding valid wave history early.
        if state.last_seen_sec is not None and (timestamp_sec - state.last_seen_sec) > self.config.history_reset_sec:
            state.candidate_key = (GESTURE_NONE, ARM_UNKNOWN)
            state.candidate_frames = 0
            state.accepted_result = GestureResult()
        state.last_seen_sec = float(timestamp_sec)

        # Section: Evaluate upper-body arm reliability and geometry without lower-body landmarks.
        arm_evaluations = {
            ARM_LEFT: self.evaluate_arm_pose(state, landmarks, ARM_LEFT),
            ARM_RIGHT: self.evaluate_arm_pose(state, landmarks, ARM_RIGHT),
        }

        # Section: Update independent per-person wrist histories before evaluating waving.
        self.update_wrist_history(state, timestamp_sec, arm_evaluations[ARM_LEFT])
        self.update_wrist_history(state, timestamp_sec, arm_evaluations[ARM_RIGHT])
        wave_candidates = {
            ARM_LEFT: self.evaluate_waving(state, ARM_LEFT, arm_evaluations[ARM_LEFT]),
            ARM_RIGHT: self.evaluate_waving(state, ARM_RIGHT, arm_evaluations[ARM_RIGHT]),
        }

        # Section: Preserve legacy gesture priority while adding explicit arm-action states.
        torso_scale = compute_torso_scale(landmarks)
        candidate = self.select_candidate(timestamp_sec, landmarks, torso_scale, arm_evaluations, wave_candidates)
        legacy_result = self.stabilize_candidate(timestamp_sec, state, candidate)
        return self.compose_action_result(timestamp_sec, legacy_result, arm_evaluations, wave_candidates)

    def evaluate_arm_pose(self, state: PersonGestureState, landmarks: list[Any], arm: str) -> ArmEvaluation:
        """Special function: Evaluate one raised arm from shoulder, elbow, and wrist only."""
        shoulder_index, elbow_index, wrist_index = arm_indices(arm)
        shoulder = get_landmark(landmarks, shoulder_index)
        elbow = get_landmark(landmarks, elbow_index)
        wrist = get_landmark(landmarks, wrist_index)
        required = {
            "shoulder": shoulder,
            "elbow": elbow,
            "wrist": wrist,
        }
        unreliable = [
            name for name, point in required.items()
            if not valid_landmark(point, self.config.arm_landmark_min_visibility)
        ]
        diagnostics: dict[str, Any] = {
            "arm": arm,
            "reliable": not unreliable,
            "unreliable_landmarks": unreliable,
        }
        if unreliable:
            return ArmEvaluation(arm, False, False, 0.0, diagnostics=diagnostics)

        # Section: Smooth wrist and elbow coordinates so camera jitter cannot mimic motion.
        smoothed_elbow = self.smooth_point(state, arm, "elbow", elbow)
        smoothed_wrist = self.smooth_point(state, arm, "wrist", wrist)
        shoulder_scale = compute_shoulder_scale(landmarks, self.config.arm_landmark_min_visibility)
        vertical_margin = self.config.raised_arm_vertical_margin * shoulder_scale
        elbow_margin = self.config.raised_arm_elbow_margin * shoulder_scale
        wrist_above_shoulder = smoothed_wrist.y < float(shoulder.y) - vertical_margin
        elbow_supports_raise = smoothed_elbow.y < float(shoulder.y) + elbow_margin
        arm_extends_upward = smoothed_wrist.y < smoothed_elbow.y - vertical_margin
        raised = wrist_above_shoulder and (elbow_supports_raise or arm_extends_upward)

        # Section: Blend geometry and visibility into a bounded per-arm confidence.
        vertical_delta = (float(shoulder.y) - smoothed_wrist.y) / shoulder_scale
        weakest_visibility = min(float(point.visibility) for point in required.values())
        visibility_score = clamp01(
            (weakest_visibility - self.config.arm_landmark_min_visibility)
            / max(1e-6, 1.0 - self.config.arm_landmark_min_visibility)
        )
        vertical_score = clamp01(
            vertical_delta / max(1e-6, self.config.raised_arm_vertical_margin * 2.0)
        )
        support_score = 1.0 if elbow_supports_raise else (0.75 if arm_extends_upward else 0.0)
        confidence = clamp01(0.50 * vertical_score + 0.30 * support_score + 0.20 * visibility_score) if raised else 0.0
        diagnostics.update({
            "wrist_above_shoulder": vertical_delta,
            "elbow_offset_from_shoulder": (smoothed_elbow.y - float(shoulder.y)) / shoulder_scale,
            "elbow_supports_raise": int(elbow_supports_raise),
            "arm_extends_upward": int(arm_extends_upward),
            "shoulder_scale": shoulder_scale,
            "score": confidence,
        })
        return ArmEvaluation(
            arm,
            True,
            raised,
            confidence,
            shoulder,
            smoothed_elbow,
            smoothed_wrist,
            shoulder_scale,
            diagnostics,
        )

    def smooth_point(self, state: PersonGestureState, arm: str, joint: str, point: Any) -> SmoothedPoint:
        """Special function: Apply lightweight exponential smoothing to one arm joint."""
        previous = state.smoothed_points[arm].get(joint)
        alpha = clamp01(self.config.landmark_smoothing_alpha)
        if previous is None:
            smoothed = SmoothedPoint(float(point.x), float(point.y), float(point.visibility))
        else:
            smoothed = SmoothedPoint(
                alpha * float(point.x) + (1.0 - alpha) * previous.x,
                alpha * float(point.y) + (1.0 - alpha) * previous.y,
                float(point.visibility),
            )
        state.smoothed_points[arm][joint] = smoothed
        return smoothed

    def update_wrist_history(self, state: PersonGestureState, timestamp_sec: float, arm_evaluation: ArmEvaluation) -> None:
        """Special function: Append one reliable wrist-and-shoulder sample for waving."""
        history = state.wrist_history[arm_evaluation.arm]
        wave_window_seconds = prefer_new_config_value(
            self.config.wave_window_seconds,
            1.5,
            self.config.wave_window_sec,
            1.2,
        )
        while history and (timestamp_sec - history[0].timestamp) > wave_window_seconds:
            history.popleft()
        if not arm_evaluation.reliable:
            return
        weakest_visibility = min(
            float(arm_evaluation.shoulder.visibility),
            float(getattr(arm_evaluation.elbow, "visibility", 1.0)),
            float(getattr(arm_evaluation.wrist, "visibility", 1.0)),
        )
        elbow_margin = self.config.raised_arm_elbow_margin * arm_evaluation.shoulder_scale
        above_shoulder = arm_evaluation.wrist.y <= float(arm_evaluation.shoulder.y) + elbow_margin
        history.append(WristSample(
            float(timestamp_sec),
            float(arm_evaluation.wrist.x),
            float(arm_evaluation.wrist.y),
            float(arm_evaluation.shoulder.x),
            float(arm_evaluation.shoulder.y),
            weakest_visibility,
            float(arm_evaluation.shoulder_scale),
            above_shoulder,
        ))

    def select_candidate(
        self,
        timestamp_sec: float,
        landmarks: list[Any],
        torso_scale: float,
        arm_evaluations: dict[str, ArmEvaluation],
        wave_candidates: dict[str, GestureCandidate],
    ) -> GestureCandidate:
        """Special function: Select one raw candidate using waving, pointing, raised-arm priority."""
        waving = self.best_waving_candidate(wave_candidates)
        if waving.gesture != GESTURE_NONE:
            return waving

        pointing = self.best_pointing_candidate(landmarks, torso_scale)
        if pointing.gesture != GESTURE_NONE:
            return pointing

        raised = self.best_raised_arm_candidate(arm_evaluations)
        if raised.gesture != GESTURE_NONE:
            return raised

        return GestureCandidate(GESTURE_NONE, 0.0, ARM_UNKNOWN, {"timestamp_sec": float(timestamp_sec)})

    def best_waving_candidate(self, candidates_by_arm: dict[str, GestureCandidate]) -> GestureCandidate:
        """Special function: Return the strongest waving candidate across both arms."""
        left = candidates_by_arm[ARM_LEFT]
        right = candidates_by_arm[ARM_RIGHT]
        if left.gesture == GESTURE_WAVING and right.gesture == GESTURE_WAVING:
            confidence = min(left.confidence, right.confidence)
            return GestureCandidate(
                GESTURE_WAVING,
                confidence,
                ARM_BOTH,
                {"arm": ARM_BOTH, "score": confidence},
            )
        return left if left.gesture == GESTURE_WAVING else right

    def evaluate_waving(self, state: PersonGestureState, arm: str, arm_evaluation: ArmEvaluation) -> GestureCandidate:
        """Special function: Detect waving from horizontal wrist oscillation above the shoulder."""
        history = [
            sample for sample in state.wrist_history[arm]
            if sample.above_shoulder and sample.visibility >= self.config.arm_landmark_min_visibility
        ]
        diagnostics: dict[str, Any] = {
            "arm": arm,
            "wave_samples": len(history),
            "reliable": arm_evaluation.reliable,
        }
        if not arm_evaluation.reliable:
            diagnostics["unreliable_landmarks"] = arm_evaluation.diagnostics.get("unreliable_landmarks", [])
            return GestureCandidate(GESTURE_NONE, 0.0, ARM_UNKNOWN, diagnostics)
        if not arm_evaluation.raised:
            return GestureCandidate(GESTURE_NONE, 0.0, ARM_UNKNOWN, diagnostics)
        if len(history) < self.config.wave_min_samples:
            return GestureCandidate(GESTURE_NONE, 0.0, ARM_UNKNOWN, diagnostics)
        xs = smooth_values([
            (sample.wrist_x - sample.shoulder_x) / max(1e-6, sample.shoulder_scale)
            for sample in history
        ])
        ys = smooth_values([
            (sample.wrist_y - sample.shoulder_y) / max(1e-6, sample.shoulder_scale)
            for sample in history
        ])
        span = max(xs) - min(xs)
        vertical_drift = max(ys) - min(ys)
        direction_changes = count_direction_changes(xs, self.config.wave_direction_deadband)
        duration = history[-1].timestamp - history[0].timestamp
        min_horizontal_range = prefer_new_config_value(
            self.config.wave_min_horizontal_range,
            0.12,
            self.config.wave_min_span,
            0.22,
        )
        wave_window_seconds = prefer_new_config_value(
            self.config.wave_window_seconds,
            1.5,
            self.config.wave_window_sec,
            1.2,
        )
        span_score = clamp01(span / max(1e-6, min_horizontal_range))
        change_score = clamp01(direction_changes / max(1, self.config.wave_min_direction_changes))
        drift_score = clamp01(1.0 - vertical_drift / max(1e-6, self.config.wave_max_vertical_drift))
        confidence = 0.45 * span_score + 0.40 * change_score + 0.15 * drift_score
        diagnostics.update({
            "wave_span": span,
            "wave_vertical_drift": vertical_drift,
            "wave_direction_changes": direction_changes,
            "wave_duration": duration,
            "score": confidence,
        })
        if duration < min(0.25, wave_window_seconds / 3.0):
            return GestureCandidate(GESTURE_NONE, confidence, ARM_UNKNOWN, diagnostics)
        if span < min_horizontal_range:
            return GestureCandidate(GESTURE_NONE, confidence, ARM_UNKNOWN, diagnostics)
        if direction_changes < self.config.wave_min_direction_changes:
            return GestureCandidate(GESTURE_NONE, confidence, ARM_UNKNOWN, diagnostics)
        if vertical_drift > self.config.wave_max_vertical_drift:
            return GestureCandidate(GESTURE_NONE, confidence, ARM_UNKNOWN, diagnostics)
        return GestureCandidate(GESTURE_WAVING, clamp01(confidence), arm, diagnostics)

    def best_pointing_candidate(self, landmarks: list[Any], torso_scale: float) -> GestureCandidate:
        """Special function: Return strongest pointing candidate across both arms."""
        candidates = [
            self.evaluate_pointing(landmarks, torso_scale, ARM_LEFT),
            self.evaluate_pointing(landmarks, torso_scale, ARM_RIGHT),
        ]
        return resolve_candidates(candidates, self.config.tie_margin)

    def evaluate_pointing(self, landmarks: list[Any], torso_scale: float, arm: str) -> GestureCandidate:
        """Special function: Evaluate one arm for pointing_left or pointing_right."""
        shoulder_index, elbow_index, wrist_index = arm_indices(arm)
        shoulder = get_landmark(landmarks, shoulder_index)
        elbow = get_landmark(landmarks, elbow_index)
        wrist = get_landmark(landmarks, wrist_index)
        diagnostics: dict[str, float | int | str] = {"arm": arm}
        if not all(valid_landmark(point, self.config.min_visibility) for point in (shoulder, elbow, wrist)):
            return GestureCandidate(GESTURE_NONE, 0.0, ARM_UNKNOWN, diagnostics)
        angle_deg = elbow_angle_deg(shoulder, elbow, wrist)
        vertical_offset = abs(float(wrist.y) - float(shoulder.y)) / torso_scale
        horizontal_delta = (float(wrist.x) - float(shoulder.x)) / torso_scale
        extension = abs(horizontal_delta)
        diagnostics.update({
            "elbow_angle_deg": angle_deg,
            "vertical_offset": vertical_offset,
            "horizontal_extension": extension,
        })
        if angle_deg < self.config.pointing_min_elbow_angle_deg:
            return GestureCandidate(GESTURE_NONE, 0.0, ARM_UNKNOWN, diagnostics)
        if vertical_offset > self.config.pointing_max_vertical_offset:
            return GestureCandidate(GESTURE_NONE, 0.0, ARM_UNKNOWN, diagnostics)
        if extension < self.config.pointing_min_extension:
            return GestureCandidate(GESTURE_NONE, 0.0, ARM_UNKNOWN, diagnostics)
        points_left = horizontal_delta < 0.0
        if self.config.mirror_input:
            points_left = not points_left
        gesture = GESTURE_POINTING_LEFT if points_left else GESTURE_POINTING_RIGHT
        angle_score = clamp01((angle_deg - self.config.pointing_min_elbow_angle_deg) / max(1.0, 180.0 - self.config.pointing_min_elbow_angle_deg))
        extension_score = clamp01(extension / max(1e-6, self.config.pointing_min_extension))
        vertical_score = clamp01(1.0 - vertical_offset / max(1e-6, self.config.pointing_max_vertical_offset))
        confidence = 0.35 * angle_score + 0.45 * extension_score + 0.20 * vertical_score
        diagnostics["score"] = confidence
        return GestureCandidate(gesture, clamp01(confidence), arm, diagnostics)

    def best_raised_arm_candidate(self, arm_evaluations: dict[str, ArmEvaluation]) -> GestureCandidate:
        """Special function: Return strongest raised-arm candidate across both arms."""
        left = arm_evaluations[ARM_LEFT]
        right = arm_evaluations[ARM_RIGHT]
        if left.raised and right.raised:
            confidence = min(left.confidence, right.confidence)
            return GestureCandidate(
                GESTURE_RAISING_BOTH_ARMS,
                confidence,
                ARM_BOTH,
                {"arm": ARM_BOTH, "score": confidence},
            )
        if left.raised:
            return GestureCandidate(GESTURE_RAISING_LEFT_ARM, left.confidence, ARM_LEFT, left.diagnostics)
        if right.raised:
            return GestureCandidate(GESTURE_RAISING_RIGHT_ARM, right.confidence, ARM_RIGHT, right.diagnostics)
        return GestureCandidate(GESTURE_NONE, 0.0, ARM_UNKNOWN, {})

    def evaluate_raised_arm(self, landmarks: list[Any], torso_scale: float, arm: str) -> GestureCandidate:
        """Special function: Preserve the direct raised-arm helper for external callers."""
        temporary_state = PersonGestureState()
        evaluation = self.evaluate_arm_pose(temporary_state, landmarks, arm)
        if not evaluation.raised:
            return GestureCandidate(GESTURE_NONE, 0.0, ARM_UNKNOWN, evaluation.diagnostics)
        gesture = GESTURE_RAISING_LEFT_ARM if arm == ARM_LEFT else GESTURE_RAISING_RIGHT_ARM
        return GestureCandidate(gesture, evaluation.confidence, arm, evaluation.diagnostics)

    def compose_action_result(
        self,
        timestamp_sec: float,
        legacy_result: GestureResult,
        arm_evaluations: dict[str, ArmEvaluation],
        wave_candidates: dict[str, GestureCandidate],
    ) -> GestureResult:
        """Special function: Merge legacy gesture output with independent arm-action states."""
        left = arm_evaluations[ARM_LEFT]
        right = arm_evaluations[ARM_RIGHT]
        left_wave = wave_candidates[ARM_LEFT].gesture == GESTURE_WAVING
        right_wave = wave_candidates[ARM_RIGHT].gesture == GESTURE_WAVING
        left_arm_state = LEFT_ARM_RAISED if left.raised else (NO_ARM_RAISED if left.reliable else ARM_POSE_UNRELIABLE)
        right_arm_state = RIGHT_ARM_RAISED if right.raised else (NO_ARM_RAISED if right.reliable else ARM_POSE_UNRELIABLE)
        if left.raised and right.raised:
            raised_arm_state = BOTH_ARMS_RAISED
        elif left.raised:
            raised_arm_state = LEFT_ARM_RAISED
        elif right.raised:
            raised_arm_state = RIGHT_ARM_RAISED
        elif left.reliable or right.reliable:
            raised_arm_state = NO_ARM_RAISED
        else:
            raised_arm_state = ARM_POSE_UNRELIABLE

        # Section: Resolve waving independently so one occluded arm cannot hide the other.
        left_wave_state = LEFT_ARM_WAVING if left_wave else (NOT_WAVING if left.reliable else WAVE_UNRELIABLE)
        right_wave_state = RIGHT_ARM_WAVING if right_wave else (NOT_WAVING if right.reliable else WAVE_UNRELIABLE)
        if left_wave and right_wave:
            wave_state = BOTH_ARMS_WAVING
        elif left_wave:
            wave_state = LEFT_ARM_WAVING
        elif right_wave:
            wave_state = RIGHT_ARM_WAVING
        elif left.reliable or right.reliable:
            wave_state = NOT_WAVING
        else:
            wave_state = WAVE_UNRELIABLE

        # Section: Do not hold a legacy result whose required current arm is unreliable.
        gesture = legacy_result.gesture
        active_arm = legacy_result.active_arm
        confidence = legacy_result.confidence
        active_unreliable = (
            (active_arm == ARM_LEFT and not left.reliable)
            or (active_arm == ARM_RIGHT and not right.reliable)
            or (active_arm == ARM_BOTH and (not left.reliable or not right.reliable))
        )
        if active_unreliable:
            gesture = GESTURE_NONE
            active_arm = ARM_UNKNOWN
            confidence = 0.0
        confidence = max(
            confidence,
            left.confidence,
            right.confidence,
            wave_candidates[ARM_LEFT].confidence if left_wave else 0.0,
            wave_candidates[ARM_RIGHT].confidence if right_wave else 0.0,
        )
        diagnostics = dict(legacy_result.diagnostics)
        diagnostics.update({
            "left_arm": left.diagnostics,
            "right_arm": right.diagnostics,
            "left_wave": wave_candidates[ARM_LEFT].diagnostics,
            "right_wave": wave_candidates[ARM_RIGHT].diagnostics,
        })
        return GestureResult(
            gesture=gesture,
            confidence=clamp01(confidence),
            active_arm=active_arm,
            diagnostics=diagnostics,
            left_arm_state=left_arm_state,
            right_arm_state=right_arm_state,
            raised_arm_state=raised_arm_state,
            wave_state=wave_state,
            left_wave_state=left_wave_state,
            right_wave_state=right_wave_state,
            timestamp=float(timestamp_sec),
        )

    def stabilize_candidate(self, timestamp_sec: float, state: PersonGestureState, candidate: GestureCandidate) -> GestureResult:
        """Special function: Require persistent candidates and hold accepted results briefly."""
        candidate_key = (candidate.gesture, candidate.active_arm)
        if candidate_key == state.candidate_key:
            state.candidate_frames += 1
        else:
            state.candidate_key = candidate_key
            state.candidate_frames = 1
        if candidate.gesture != GESTURE_NONE and state.candidate_frames >= self.config.stability_frames:
            result = GestureResult(candidate.gesture, candidate.confidence, candidate.active_arm, candidate.diagnostics)
            state.accepted_result = result
            state.accepted_time_sec = float(timestamp_sec)
            return result
        if state.accepted_result.gesture != GESTURE_NONE and (timestamp_sec - state.accepted_time_sec) <= self.config.result_hold_sec:
            return state.accepted_result
        if candidate.gesture == GESTURE_NONE:
            state.accepted_result = GestureResult()
        return GestureResult(GESTURE_NONE, 0.0, ARM_UNKNOWN, candidate.diagnostics)

    def prune_missing_people(self, timestamp_sec: float) -> None:
        """Special function: Remove person histories that have timed out."""
        stale_ids = [
            person_id for person_id, state in self.person_states.items()
            if state.last_seen_sec is not None
            and (timestamp_sec - state.last_seen_sec) > self.config.wave_history_timeout_seconds
        ]
        for person_id in stale_ids:
            del self.person_states[person_id]

    def wrist_trajectory(self, person_id: int, arm: str) -> list[WristSample]:
        """Special function: Return one tracked wrist trajectory for debug drawing."""
        state = self.person_states.get(int(person_id))
        if state is None:
            return []
        return list(state.wrist_history.get(arm, ()))


def resolve_candidates(candidates: list[GestureCandidate], tie_margin: float) -> GestureCandidate:
    """Special function: Select the best candidate or none when top scores are ambiguous."""
    valid = [candidate for candidate in candidates if candidate.gesture != GESTURE_NONE]
    if not valid:
        return GestureCandidate(GESTURE_NONE, 0.0, ARM_UNKNOWN, {})
    valid.sort(key=lambda candidate: candidate.confidence, reverse=True)
    if len(valid) >= 2 and abs(valid[0].confidence - valid[1].confidence) <= tie_margin and valid[0].gesture == valid[1].gesture:
        return GestureCandidate(GESTURE_NONE, 0.0, ARM_UNKNOWN, {"ambiguous": valid[0].gesture})
    return valid[0]


def prefer_new_config_value(
    new_value: float,
    new_default: float,
    legacy_value: float,
    legacy_default: float,
) -> float:
    """Special function: Honor a customized legacy threshold unless the new alias is set."""
    if float(new_value) != float(new_default) or float(legacy_value) == float(legacy_default):
        return float(new_value)
    return float(legacy_value)


def compute_torso_scale(landmarks: list[Any]) -> float:
    """Special function: Compute normalized torso scale from shoulders and hips."""
    left_shoulder = get_landmark(landmarks, LEFT_SHOULDER)
    right_shoulder = get_landmark(landmarks, RIGHT_SHOULDER)
    left_hip = get_landmark(landmarks, LEFT_HIP)
    right_hip = get_landmark(landmarks, RIGHT_HIP)
    shoulder_width = distance(left_shoulder, right_shoulder)
    torso_height = 0.5 * (distance(left_shoulder, left_hip) + distance(right_shoulder, right_hip))
    return max(shoulder_width, torso_height, 1e-6)


def compute_shoulder_scale(landmarks: list[Any], min_visibility: float) -> float:
    """Special function: Use shoulder width or normalized image coordinates for arm scale."""
    left_shoulder = get_landmark(landmarks, LEFT_SHOULDER)
    right_shoulder = get_landmark(landmarks, RIGHT_SHOULDER)
    if valid_landmark(left_shoulder, min_visibility) and valid_landmark(right_shoulder, min_visibility):
        shoulder_width = distance(left_shoulder, right_shoulder)
        if shoulder_width > 1e-6:
            return shoulder_width
    return 1.0


def arm_indices(arm: str) -> tuple[int, int, int]:
    """Special function: Return shoulder, elbow, wrist landmark indices for one arm."""
    if arm == ARM_LEFT:
        return LEFT_SHOULDER, LEFT_ELBOW, LEFT_WRIST
    return RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST


def get_landmark(landmarks: list[Any], index: int) -> Any:
    """Special function: Safely return a landmark-like object by MediaPipe index."""
    return landmarks[index] if 0 <= index < len(landmarks) else None


def valid_landmark(landmark: Any, min_visibility: float) -> bool:
    """Special function: Validate landmark geometry and visibility."""
    if landmark is None:
        return False
    values = [getattr(landmark, "x", math.nan), getattr(landmark, "y", math.nan)]
    if not all(math.isfinite(float(value)) for value in values):
        return False
    return float(getattr(landmark, "visibility", 0.0)) >= min_visibility


def distance(first: Any, second: Any) -> float:
    """Special function: Return 2D normalized distance between two landmarks."""
    if first is None or second is None:
        return 0.0
    return math.hypot(float(first.x) - float(second.x), float(first.y) - float(second.y))


def elbow_angle_deg(shoulder: Any, elbow: Any, wrist: Any) -> float:
    """Special function: Compute shoulder-elbow-wrist angle in degrees."""
    first = (float(shoulder.x) - float(elbow.x), float(shoulder.y) - float(elbow.y))
    second = (float(wrist.x) - float(elbow.x), float(wrist.y) - float(elbow.y))
    norm_first = math.hypot(first[0], first[1])
    norm_second = math.hypot(second[0], second[1])
    if norm_first <= 1e-6 or norm_second <= 1e-6:
        return 0.0
    cosine = (first[0] * second[0] + first[1] * second[1]) / (norm_first * norm_second)
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def smooth_values(values: list[float]) -> list[float]:
    """Special function: Smooth wrist x positions with a small moving average."""
    if len(values) < 3:
        return values
    smoothed = [values[0]]
    for index in range(1, len(values) - 1):
        smoothed.append((values[index - 1] + values[index] + values[index + 1]) / 3.0)
    smoothed.append(values[-1])
    return smoothed


def count_direction_changes(values: list[float], deadband: float) -> int:
    """Special function: Count alternating horizontal wrist direction changes."""
    directions: list[int] = []
    for previous, current in zip(values, values[1:]):
        delta = current - previous
        if abs(delta) < deadband:
            continue
        direction = 1 if delta > 0.0 else -1
        if not directions or direction != directions[-1]:
            directions.append(direction)
    return max(0, len(directions) - 1)


def clamp01(value: float) -> float:
    """Special function: Clamp a numeric score into the confidence range."""
    return max(0.0, min(1.0, float(value)))
