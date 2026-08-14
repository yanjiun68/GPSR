#!/usr/bin/env python3
"""Section: MediaPipe human pose landmarker node for ROS 2."""

from __future__ import annotations

import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from cv_bridge import CvBridge
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from voice_interfaces.srv import SayText

from putrabot_interfaces.msg import (
    HumanPose,
    HumanPoseArray,
    HumanSkeleton,
    HumanSkeletonKeypoint,
    HumanSkeletonLink,
    HumanSkeletons,
    PoseLandmark,
)


# Section: Import the pure-Python gesture classifier from the source or installed script directory.
try:
    from gesture_classifier import (
        ARM_LEFT,
        ARM_RIGHT,
        ARM_UNKNOWN,
        GESTURE_NONE,
        GESTURE_POINTING_LEFT,
        GESTURE_WAVING,
        GestureClassifier,
        GestureConfig,
    )
except ImportError:
    sys.path.append(str(Path(__file__).resolve().parent))
    from gesture_classifier import (
        ARM_LEFT,
        ARM_RIGHT,
        ARM_UNKNOWN,
        GESTURE_NONE,
        GESTURE_POINTING_LEFT,
        GESTURE_WAVING,
        GestureClassifier,
        GestureConfig,
    )


# Section: MediaPipe task imports are optional so the node can log a clear runtime error.
try:
    import mediapipe as mp
    from mediapipe.tasks import python
    from mediapipe.tasks.python import vision as mp_vision
    MEDIAPIPE_AVAILABLE = True
except ImportError:
    mp = None
    python = None
    mp_vision = None
    MEDIAPIPE_AVAILABLE = False


# Section: MediaPipe landmark names in canonical 33-landmark order.
MEDIAPIPE_LANDMARK_NAMES = [
    "nose",
    "left_eye_inner",
    "left_eye",
    "left_eye_outer",
    "right_eye_inner",
    "right_eye",
    "right_eye_outer",
    "left_ear",
    "right_ear",
    "mouth_left",
    "mouth_right",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
    "left_pinky",
    "right_pinky",
    "left_index",
    "right_index",
    "left_thumb",
    "right_thumb",
    "left_hip",
    "right_hip",
    "left_knee",
    "right_knee",
    "left_ankle",
    "right_ankle",
    "left_heel",
    "right_heel",
    "left_foot_index",
    "right_foot_index",
]


# Section: MediaPipe pose connections drawn on debug images.
MEDIAPIPE_CONNECTIONS = [
    (11, 12),
    (11, 13),
    (13, 15),
    (12, 14),
    (14, 16),
    (11, 23),
    (12, 24),
    (23, 24),
    (23, 25),
    (25, 27),
    (24, 26),
    (26, 28),
    (27, 29),
    (29, 31),
    (28, 30),
    (30, 32),
    (0, 11),
    (0, 12),
]


# Section: Map MediaPipe landmarks into the existing 17-keypoint skeleton message for compatibility.
LEGACY_COCO_FROM_MEDIAPIPE = [
    ("nose", 0),
    ("left_eye", 2),
    ("right_eye", 5),
    ("left_ear", 7),
    ("right_ear", 8),
    ("left_shoulder", 11),
    ("right_shoulder", 12),
    ("left_elbow", 13),
    ("right_elbow", 14),
    ("left_wrist", 15),
    ("right_wrist", 16),
    ("left_hip", 23),
    ("right_hip", 24),
    ("left_knee", 25),
    ("right_knee", 26),
    ("left_ankle", 27),
    ("right_ankle", 28),
]


# Section: Torso landmarks retained for body-pose and diagnostic compatibility.
REQUIRED_TORSO_INDICES = [11, 12, 23, 24]


# Section: Upper-body anchors used for whole-pose eligibility and tracking.
REQUIRED_UPPER_BODY_INDICES = [11, 12]


# Section: Per-arm landmarks that can independently preserve an action candidate.
ARM_ACTION_INDICES = [
    (11, 13, 15),
    (12, 14, 16),
]


# Section: Supported pose status labels.
STATUS_DETECTED = "DETECTED"
STATUS_UNRELIABLE = "UNRELIABLE"
STATUS_NOT_DETECTED = "NOT_DETECTED"


# Section: Supported per-human body pose labels derived from landmark geometry.
POSE_STANDING = "standing"
POSE_SITTING = "sitting"
POSE_LYING_DOWN = "lying_down"
POSE_UNKNOWN = "unknown"
POSE_UNRELIABLE = "unreliable"


@dataclass(frozen=True, slots=True)
class PostureConfig:
    """Special function: Tunable lower-body-only posture thresholds in degrees and body scales."""

    # Section: Landmark confidence required before lower-body geometry is trusted.
    landmark_min_visibility: float = 0.55
    # Section: Knee-angle boundary for bent-leg sitting evidence, in degrees.
    sitting_knee_angle_max: float = 135.0
    # Section: Knee-angle boundary for straight-leg standing evidence, in degrees.
    standing_knee_angle_min: float = 160.0
    # Section: Hip-angle boundary for seated hip-flexion evidence, in degrees.
    sitting_hip_angle_max: float = 145.0
    # Section: Hip-to-knee vertical distance boundary, normalized by torso length.
    hip_knee_sitting_ratio_max: float = 0.45
    # Section: Hip-to-knee vertical distance boundary for upright-leg evidence.
    hip_knee_standing_ratio_min: float = 0.65
    # Section: Minimum hip-to-ankle extension ratio for standing evidence.
    standing_leg_extension_min: float = 0.90
    # Section: Minimum image-vertical torso fraction for upright posture geometry.
    torso_vertical_ratio_min: float = 0.45
    # Section: Minimum independent sitting/standing cues needed for a raw decision.
    minimum_evidence: int = 2
    # Section: Maximum torso angle from image horizontal for lying evidence, in degrees.
    lying_torso_angle_max: float = 25.0
    # Section: Maximum normalized shoulder-center to hip-center vertical separation.
    lying_vertical_ratio_max: float = 0.35
    # Section: Minimum visible body width-to-height aspect ratio for lying evidence.
    lying_body_aspect_min: float = 1.8
    # Section: Independent lying cues needed for a raw lying decision.
    lying_score_threshold: int = 3
    # Section: Minimum normalized winning class score required for a raw posture.
    posture_min_score: float = 0.60
    # Section: Minimum winning-score margin over the second class.
    posture_min_margin: float = 0.15
    # Section: Knee angles, in degrees, for full and partial standing evidence.
    standing_knee_full_score_angle: float = 170.0
    standing_knee_partial_angle: float = 145.0
    # Section: Knee angles, in degrees, for full and partial sitting evidence.
    sitting_knee_full_score_angle: float = 100.0
    sitting_knee_partial_angle: float = 145.0
    # Section: Torso angles from horizontal, in degrees, for full and partial lying evidence.
    lying_torso_full_score_angle: float = 12.0
    lying_torso_partial_angle: float = 30.0


@dataclass(slots=True)
class PostureEvaluation:
    """Special function: Raw posture result and normalized diagnostic features."""

    # Section: Unfiltered posture decision.
    raw_posture: str
    # Section: Accumulated independent sitting cues.
    sitting_score: int
    # Section: Accumulated independent standing cues.
    standing_score: int
    # Section: Accumulated torso/body cues supporting lying down.
    lying_score: int
    # Section: Torso angle from horizontal, in degrees, when measurable.
    torso_angle_deg: float | None
    # Section: Normalized shoulder-center to hip-center vertical separation.
    torso_vertical_ratio: float | None
    # Section: Visible body width-to-height aspect ratio.
    body_aspect_ratio: float | None
    # Section: Per-leg feature dictionary used for debug and tests.
    legs: dict[str, dict[str, float | bool | None]]
    # Section: Feature dictionary retained for future lightweight classifiers.
    features: dict[str, float | bool | None]
    # Section: Raw class scores retained for summaries and calibration logs.
    raw_scores: dict[str, float]
    # Section: Difference between the highest and second-highest class score.
    score_margin: float


@dataclass(slots=True)
class PostureTrackState:
    """Special function: Per-person hysteresis state for stable posture output."""

    # Section: Last confirmed posture.
    filtered_posture: str = POSE_UNKNOWN
    # Section: Candidate waiting for temporal confirmation.
    candidate_posture: str = POSE_UNKNOWN
    # Section: Consecutive candidate-frame count.
    candidate_frames: int = 0
    # Section: Bounded raw posture history retained for temporal diagnostics.
    raw_history: list[str] = field(default_factory=list)


@dataclass(slots=True)
class PoseCandidate:
    """Special function: Internal pose candidate metadata used for quality filtering."""

    # Section: Original pose message candidate.
    pose: HumanPose

    # Section: Normalized bbox as x1, y1, x2, y2.
    bbox: tuple[float, float, float, float]

    # Section: Overall quality score used to keep the best duplicate.
    quality: float

    # Section: Number of usable landmarks in this candidate.
    usable_landmarks: int

    # Section: Minimum torso confidence for this candidate.
    torso_confidence: float

    # Section: Normalized bbox area ratio.
    bbox_area_ratio: float


@dataclass(slots=True)
class TrackState:
    """Special function: Minimal nearest-centroid track state for gesture histories."""

    # Section: Stable track identifier.
    track_id: int

    # Section: Last normalized body centroid.
    centroid: tuple[float, float]

    # Section: Last observation timestamp in seconds.
    last_seen_sec: float


class HumanSkeletonDetectionNode(LifecycleNode):
    """Special function: Detect MediaPipe human poses and publish normalized landmarks."""

    def __init__(self) -> None:
        """Special function: Initialize ROS interfaces and load the MediaPipe Full model once."""
        super().__init__("human_skeleton_detection_node")

        # Section: Declare image, model, inference, and output parameters.
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("model_path", self.default_model_path())
        self.declare_parameter("num_poses", 2)
        self.declare_parameter("min_detection_confidence", 0.60)
        self.declare_parameter("min_presence_confidence", 0.60)
        self.declare_parameter("min_tracking_confidence", 0.65)
        self.declare_parameter("process_every_n_frames", 1)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("show_gui", False)
        self.declare_parameter("gui_window_name", "MediaPipe Human Pose")
        self.declare_parameter("enable_tts", False)
        self.declare_parameter("tts_service_name", "/voice/tts/say")
        self.declare_parameter("tts_min_interval_sec", 5.0)
        self.declare_parameter("tts_on_change_only", True)
        self.declare_parameter("tts_speak_unreliable", True)
        self.declare_parameter("tts_speak_no_detection", False)
        self.declare_parameter("poses_topic", "/human_pose/poses")
        self.declare_parameter("debug_image_topic", "/human_pose/debug_image")
        self.declare_parameter("pose_summary_topic", "/perception/human_poses")
        self.declare_parameter("legacy_skeletons_topic", "/perception/human_skeletons")
        self.declare_parameter("torso_visibility_threshold", 0.50)
        self.declare_parameter("min_usable_landmarks", 12)
        self.declare_parameter("min_torso_landmark_confidence", 0.50)
        self.declare_parameter("min_pose_bbox_area_ratio", 0.02)
        self.declare_parameter("duplicate_iou_threshold", 0.35)
        self.declare_parameter("duplicate_center_distance_ratio", 0.18)
        self.declare_parameter("prefer_single_person_mode", False)
        self.declare_parameter("gestures_topic", "/human_pose/gestures")
        self.declare_parameter("tracker_timeout_sec", 0.75)
        self.declare_parameter("tracker_max_distance", 0.35)
        self.declare_parameter("gesture_detection.enabled", True)
        self.declare_parameter("gesture_detection.min_visibility", 0.55)
        self.declare_parameter("gesture_detection.mirror_input", False)
        self.declare_parameter("gesture_detection.raised_margin", 0.15)
        self.declare_parameter("gesture_detection.pointing_min_elbow_angle_deg", 145.0)
        self.declare_parameter("gesture_detection.pointing_min_extension", 0.55)
        self.declare_parameter("gesture_detection.pointing_max_vertical_offset", 0.30)
        self.declare_parameter("gesture_detection.wave_window_sec", 1.2)
        self.declare_parameter("gesture_detection.wave_min_samples", 8)
        self.declare_parameter("gesture_detection.wave_min_span", 0.22)
        self.declare_parameter("gesture_detection.wave_direction_deadband", 0.05)
        self.declare_parameter("gesture_detection.wave_min_direction_changes", 2)
        self.declare_parameter("gesture_detection.stability_frames", 3)
        self.declare_parameter("gesture_detection.result_hold_sec", 0.30)
        self.declare_parameter("gesture_detection.history_reset_sec", 0.50)
        self.declare_parameter("gesture_detection.tie_margin", 0.10)

        # Section: Upper-body-only arm-action thresholds requested by the action interface.
        self.declare_parameter("arm_landmark_min_visibility", 0.55)
        self.declare_parameter("raised_arm_vertical_margin", 0.04)
        self.declare_parameter("raised_arm_elbow_margin", 0.08)
        self.declare_parameter("wave_window_seconds", 1.5)
        self.declare_parameter("wave_min_horizontal_range", 0.12)
        self.declare_parameter("wave_min_direction_changes", 2)
        self.declare_parameter("wave_max_vertical_drift", 0.15)
        self.declare_parameter("wave_history_timeout_seconds", 2.0)
        self.declare_parameter("gesture_detection.landmark_smoothing_alpha", 0.45)
        self.declare_parameter("log_stats_interval_sec", 5.0)
        # Section: Declare lower-body-only posture scoring and temporal stability parameters.
        self.declare_parameter("posture_landmark_min_visibility", 0.55)
        self.declare_parameter("sitting_knee_angle_max", 135.0)
        self.declare_parameter("standing_knee_angle_min", 160.0)
        self.declare_parameter("sitting_hip_angle_max", 145.0)
        self.declare_parameter("hip_knee_sitting_ratio_max", 0.45)
        self.declare_parameter("hip_knee_standing_ratio_min", 0.65)
        self.declare_parameter("posture_confirm_frames", 3)
        self.declare_parameter("posture_release_frames", 4)
        self.declare_parameter("posture_debug", False)
        self.declare_parameter("posture_debug_interval_sec", 2.0)
        self.declare_parameter("standing_leg_extension_min", 0.90)
        self.declare_parameter("torso_vertical_ratio_min", 0.45)
        self.declare_parameter("posture_minimum_evidence", 2)
        self.declare_parameter("lying_torso_angle_max", 25.0)
        self.declare_parameter("lying_vertical_ratio_max", 0.35)
        self.declare_parameter("lying_body_aspect_min", 1.8)
        self.declare_parameter("lying_score_threshold", 3)
        self.declare_parameter("lying_confirm_frames", 4)
        self.declare_parameter("lying_release_frames", 4)
        self.declare_parameter("posture_min_score", 0.60)
        self.declare_parameter("posture_min_margin", 0.15)
        self.declare_parameter("standing_knee_full_score_angle", 170.0)
        self.declare_parameter("standing_knee_partial_angle", 145.0)
        self.declare_parameter("sitting_knee_full_score_angle", 100.0)
        self.declare_parameter("sitting_knee_partial_angle", 145.0)
        self.declare_parameter("lying_torso_full_score_angle", 12.0)
        self.declare_parameter("lying_torso_partial_angle", 30.0)
        self.declare_parameter("posture_history_size", 5)

        # Section: Store runtime parameters used by callbacks.
        self.model_path = str(self.get_parameter("model_path").value)
        self.num_poses = int(self.get_parameter("num_poses").value)
        self.min_detection_confidence = float(self.get_parameter("min_detection_confidence").value)
        self.min_presence_confidence = float(self.get_parameter("min_presence_confidence").value)
        self.min_tracking_confidence = float(self.get_parameter("min_tracking_confidence").value)
        self.process_every_n_frames = max(1, int(self.get_parameter("process_every_n_frames").value))
        self.publish_debug_image_enabled = bool(self.get_parameter("publish_debug_image").value)
        self.show_gui = bool(self.get_parameter("show_gui").value)
        self.gui_window_name = str(self.get_parameter("gui_window_name").value)
        self.enable_tts = bool(self.get_parameter("enable_tts").value)
        self.tts_min_interval_sec = float(self.get_parameter("tts_min_interval_sec").value)
        self.tts_on_change_only = bool(self.get_parameter("tts_on_change_only").value)
        self.tts_speak_unreliable = bool(self.get_parameter("tts_speak_unreliable").value)
        self.tts_speak_no_detection = bool(self.get_parameter("tts_speak_no_detection").value)
        self.torso_visibility_threshold = float(self.get_parameter("torso_visibility_threshold").value)
        self.min_usable_landmarks = int(self.get_parameter("min_usable_landmarks").value)
        self.min_torso_landmark_confidence = float(self.get_parameter("min_torso_landmark_confidence").value)
        self.min_pose_bbox_area_ratio = float(self.get_parameter("min_pose_bbox_area_ratio").value)
        self.duplicate_iou_threshold = float(self.get_parameter("duplicate_iou_threshold").value)
        self.duplicate_center_distance_ratio = float(self.get_parameter("duplicate_center_distance_ratio").value)
        self.prefer_single_person_mode = bool(self.get_parameter("prefer_single_person_mode").value)
        self.gesture_detection_enabled = bool(self.get_parameter("gesture_detection.enabled").value)
        self.arm_landmark_min_visibility = float(self.get_parameter("arm_landmark_min_visibility").value)
        self.tracker_timeout_sec = float(self.get_parameter("tracker_timeout_sec").value)
        self.tracker_max_distance = float(self.get_parameter("tracker_max_distance").value)
        self.log_stats_interval_sec = float(self.get_parameter("log_stats_interval_sec").value)
        # Section: Build one shared lower-body-only scoring configuration from ROS parameters.
        self.posture_config = PostureConfig(
            landmark_min_visibility=float(self.get_parameter("posture_landmark_min_visibility").value),
            sitting_knee_angle_max=float(self.get_parameter("sitting_knee_angle_max").value),
            standing_knee_angle_min=float(self.get_parameter("standing_knee_angle_min").value),
            sitting_hip_angle_max=float(self.get_parameter("sitting_hip_angle_max").value),
            hip_knee_sitting_ratio_max=float(self.get_parameter("hip_knee_sitting_ratio_max").value),
            hip_knee_standing_ratio_min=float(self.get_parameter("hip_knee_standing_ratio_min").value),
            standing_leg_extension_min=float(self.get_parameter("standing_leg_extension_min").value),
            torso_vertical_ratio_min=float(self.get_parameter("torso_vertical_ratio_min").value),
            minimum_evidence=max(1, int(self.get_parameter("posture_minimum_evidence").value)),
            lying_torso_angle_max=float(self.get_parameter("lying_torso_angle_max").value),
            lying_vertical_ratio_max=float(self.get_parameter("lying_vertical_ratio_max").value),
            lying_body_aspect_min=float(self.get_parameter("lying_body_aspect_min").value),
            lying_score_threshold=max(1, int(self.get_parameter("lying_score_threshold").value)),
            posture_min_score=float(self.get_parameter("posture_min_score").value),
            posture_min_margin=float(self.get_parameter("posture_min_margin").value),
            standing_knee_full_score_angle=float(self.get_parameter("standing_knee_full_score_angle").value),
            standing_knee_partial_angle=float(self.get_parameter("standing_knee_partial_angle").value),
            sitting_knee_full_score_angle=float(self.get_parameter("sitting_knee_full_score_angle").value),
            sitting_knee_partial_angle=float(self.get_parameter("sitting_knee_partial_angle").value),
            lying_torso_full_score_angle=float(self.get_parameter("lying_torso_full_score_angle").value),
            lying_torso_partial_angle=float(self.get_parameter("lying_torso_partial_angle").value),
        )
        self.posture_confirm_frames = max(1, int(self.get_parameter("posture_confirm_frames").value))
        self.posture_release_frames = max(1, int(self.get_parameter("posture_release_frames").value))
        self.posture_debug = bool(self.get_parameter("posture_debug").value)
        self.posture_debug_interval_sec = max(0.1, float(self.get_parameter("posture_debug_interval_sec").value))
        if self.prefer_single_person_mode:
            self.num_poses = 1

        # Section: Configure pure-Python gesture classification separately from MediaPipe inference.
        wave_min_direction_changes = int(self.get_parameter("wave_min_direction_changes").value)
        legacy_wave_min_direction_changes = int(
            self.get_parameter("gesture_detection.wave_min_direction_changes").value
        )
        if wave_min_direction_changes == 2 and legacy_wave_min_direction_changes != 2:
            wave_min_direction_changes = legacy_wave_min_direction_changes
        self.gesture_classifier = GestureClassifier(GestureConfig(
            min_visibility=float(self.get_parameter("gesture_detection.min_visibility").value),
            arm_landmark_min_visibility=self.arm_landmark_min_visibility,
            mirror_input=bool(self.get_parameter("gesture_detection.mirror_input").value),
            raised_margin=float(self.get_parameter("gesture_detection.raised_margin").value),
            raised_arm_vertical_margin=float(self.get_parameter("raised_arm_vertical_margin").value),
            raised_arm_elbow_margin=float(self.get_parameter("raised_arm_elbow_margin").value),
            pointing_min_elbow_angle_deg=float(self.get_parameter("gesture_detection.pointing_min_elbow_angle_deg").value),
            pointing_min_extension=float(self.get_parameter("gesture_detection.pointing_min_extension").value),
            pointing_max_vertical_offset=float(self.get_parameter("gesture_detection.pointing_max_vertical_offset").value),
            wave_window_sec=float(self.get_parameter("gesture_detection.wave_window_sec").value),
            wave_min_samples=int(self.get_parameter("gesture_detection.wave_min_samples").value),
            wave_min_span=float(self.get_parameter("gesture_detection.wave_min_span").value),
            wave_direction_deadband=float(self.get_parameter("gesture_detection.wave_direction_deadband").value),
            wave_min_direction_changes=wave_min_direction_changes,
            wave_window_seconds=float(self.get_parameter("wave_window_seconds").value),
            wave_min_horizontal_range=float(self.get_parameter("wave_min_horizontal_range").value),
            wave_max_vertical_drift=float(self.get_parameter("wave_max_vertical_drift").value),
            wave_history_timeout_seconds=float(self.get_parameter("wave_history_timeout_seconds").value),
            landmark_smoothing_alpha=float(self.get_parameter("gesture_detection.landmark_smoothing_alpha").value),
            stability_frames=int(self.get_parameter("gesture_detection.stability_frames").value),
            result_hold_sec=float(self.get_parameter("gesture_detection.result_hold_sec").value),
            history_reset_sec=float(self.get_parameter("gesture_detection.history_reset_sec").value),
            tie_margin=float(self.get_parameter("gesture_detection.tie_margin").value),
        ))

        # Section: Initialize frame ordering and performance counters.
        self.frame_count = 0
        self.last_mediapipe_timestamp_ms = -1
        self.processed_frames = 0
        self.total_inference_time_sec = 0.0
        self.stats_window_start_sec = time.monotonic()
        self.stats_window_frames = 0
        self.last_warning_time_by_key: dict[str, float] = {}
        self.last_tts_report_time_sec = 0.0
        self.last_tts_fingerprint = ""
        self.active_tts_future: Any | None = None
        self.last_filter_diagnostics = default_filter_diagnostics(0)
        self.next_track_id = 1
        self.tracks: dict[int, TrackState] = {}
        self.last_gesture_results: dict[int, Any] = {}
        # Section: Maintain independent posture filters and diagnostics per tracked person.
        self.posture_tracks: dict[int, PostureTrackState] = {}
        self.posture_diagnostics: dict[int, PostureEvaluation] = {}
        self.last_posture_debug_sec = 0.0

        # Section: Enable local OpenCV GUI only when requested and display support exists.
        self.gui_available = self.show_gui and bool(os.environ.get("DISPLAY", "").strip())
        if self.show_gui and not self.gui_available:
            self.get_logger().warn("show_gui is true, but DISPLAY is empty; OpenCV GUI window will not open.")

        # Section: Heavy inference and ROS resources remain empty until CONFIGURE.
        self.bridge = None
        self.landmarker = None
        self.image_subscription = None
        self.poses_pub = None
        self.debug_image_pub = None
        self.pose_summary_pub = None
        self.gestures_pub = None
        self.legacy_skeletons_pub = None
        self.tts_client = None
        self.skill_active = False
        self.get_logger().info("MediaPipe pose lifecycle node is UNCONFIGURED.")
    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Load MediaPipe and allocate pose ROS interfaces."""
        try:
            # Section: Heavy model ownership begins only during CONFIGURE.
            self.bridge = CvBridge()
            self.landmarker = self.load_landmarker(self.model_path)
            if self.landmarker is None:
                raise RuntimeError("MediaPipe pose landmarker could not be loaded")

            # Section: Allocate camera and lifecycle publisher interfaces while INACTIVE.
            sensor_qos = QoSProfile(depth=10)
            sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            self.image_subscription = self.create_subscription(
                Image,
                str(self.get_parameter("image_topic").value),
                self.image_callback,
                sensor_qos,
            )
            self.poses_pub = self.create_lifecycle_publisher(
                HumanPoseArray,
                str(self.get_parameter("poses_topic").value),
                10,
            )
            self.debug_image_pub = self.create_lifecycle_publisher(
                Image,
                str(self.get_parameter("debug_image_topic").value),
                10,
            )
            self.pose_summary_pub = self.create_lifecycle_publisher(
                String,
                str(self.get_parameter("pose_summary_topic").value),
                10,
            )
            self.gestures_pub = self.create_lifecycle_publisher(
                String,
                str(self.get_parameter("gestures_topic").value),
                10,
            )
            self.legacy_skeletons_pub = self.create_lifecycle_publisher(
                HumanSkeletons,
                str(self.get_parameter("legacy_skeletons_topic").value),
                10,
            )
            if self.enable_tts:
                self.tts_client = self.create_client(
                    SayText,
                    str(self.get_parameter("tts_service_name").value),
                )
            self.reset_runtime_state()
            self.get_logger().info("[GPSR_LIFECYCLE] MediaPipe pose: configured")
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.get_logger().error(f"MediaPipe pose configure failed: {exc}")
            self.release_resources()
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Special function: Activate pose publishers and permit inference."""
        result = super().on_activate(state)
        if result == TransitionCallbackReturn.SUCCESS:
            self.skill_active = True
        return result

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Special function: Stop pose inference before publisher deactivation."""
        self.skill_active = False
        if self.gui_available:
            cv2.destroyWindow(self.gui_window_name)
        return super().on_deactivate(state)

    def on_cleanup(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Unload MediaPipe and destroy configured interfaces."""
        self.skill_active = False
        self.release_resources()
        self.reset_runtime_state()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Release pose resources during lifecycle shutdown."""
        self.skill_active = False
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Fail closed and restore a clean runtime baseline."""
        self.skill_active = False
        self.release_resources()
        self.reset_runtime_state()
        return TransitionCallbackReturn.SUCCESS

    def release_resources(self) -> None:
        """Special function: Destroy resources allocated during CONFIGURE."""
        if self.image_subscription is not None:
            self.destroy_subscription(self.image_subscription)
            self.image_subscription = None
        for attribute in (
            "poses_pub",
            "debug_image_pub",
            "pose_summary_pub",
            "gestures_pub",
            "legacy_skeletons_pub",
        ):
            publisher = getattr(self, attribute, None)
            if publisher is not None:
                self.destroy_publisher(publisher)
                setattr(self, attribute, None)
        if self.tts_client is not None:
            self.destroy_client(self.tts_client)
            self.tts_client = None
        if self.landmarker is not None:
            self.landmarker.close()
            self.landmarker = None
        self.bridge = None
        if self.gui_available:
            cv2.destroyAllWindows()

    def reset_runtime_state(self) -> None:
        """Special function: Clear inference, tracking, gesture, and posture state."""
        self.frame_count = 0
        self.last_mediapipe_timestamp_ms = -1
        self.processed_frames = 0
        self.total_inference_time_sec = 0.0
        self.stats_window_start_sec = time.monotonic()
        self.stats_window_frames = 0
        self.last_warning_time_by_key.clear()
        self.last_tts_report_time_sec = 0.0
        self.last_tts_fingerprint = ""
        self.active_tts_future = None
        self.next_track_id = 1
        self.tracks.clear()
        self.last_gesture_results.clear()
        self.posture_tracks.clear()
        self.posture_diagnostics.clear()

    def default_model_path(self) -> str:
        """Special function: Return the package-local default Full landmarker model path."""
        try:
            return str(Path(get_package_share_directory("vision")) / "models" / "pose_landmarker_full.task")
        except PackageNotFoundError:
            return str(Path(__file__).resolve().parents[1] / "models" / "pose_landmarker_full.task")

    def load_landmarker(self, model_path: str) -> Any | None:
        """Special function: Load the MediaPipe Pose Landmarker in VIDEO mode."""
        if not MEDIAPIPE_AVAILABLE:
            self.get_logger().error("mediapipe is not installed; human poses will be empty.")
            return None
        if not Path(model_path).is_file():
            self.get_logger().error(f"MediaPipe pose model not found: {model_path}")
            return None
        try:
            base_options = python.BaseOptions(model_asset_path=model_path)
            options = mp_vision.PoseLandmarkerOptions(
                base_options=base_options,
                running_mode=mp_vision.RunningMode.VIDEO,
                num_poses=self.num_poses,
                min_pose_detection_confidence=self.min_detection_confidence,
                min_pose_presence_confidence=self.min_presence_confidence,
                min_tracking_confidence=self.min_tracking_confidence,
                output_segmentation_masks=False,
            )
            return mp_vision.PoseLandmarker.create_from_options(options)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to load MediaPipe Pose Landmarker: {exc}")
            return None

    def image_callback(self, msg: Image) -> None:
        """Special function: Convert one ROS image and run MediaPipe pose inference when eligible."""
        if not self.skill_active:
            return
        self.frame_count += 1
        if (self.frame_count - 1) % self.process_every_n_frames != 0:
            return

        # Section: Convert ROS timestamp to MediaPipe video timestamp and reject stale ordering.
        timestamp_ms = ros_stamp_to_millis(msg.header.stamp)
        if timestamp_ms <= self.last_mediapipe_timestamp_ms:
            self.throttled_warn("timestamp_order", "Skipping duplicate or out-of-order image timestamp.")
            return
        self.last_mediapipe_timestamp_ms = timestamp_ms

        # Section: Publish empty arrays instead of crashing when the model is unavailable.
        if self.landmarker is None:
            raw_pose_array = build_pose_array(msg, [])
            pose_array, diagnostics = filter_pose_candidates(raw_pose_array)
            self.publish_outputs(msg, pose_array, None, diagnostics)
            return

        # Section: Convert ROS image to RGB NumPy safely.
        try:
            rgb_image = self.ros_image_to_rgb(msg)
        except Exception as exc:  # noqa: BLE001
            self.throttled_warn("image_conversion", f"Failed to convert image for MediaPipe pose: {exc}")
            return

        # Section: Run MediaPipe inference and catch transient runtime failures.
        try:
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)
            start_sec = time.perf_counter()
            result = self.landmarker.detect_for_video(mp_image, timestamp_ms)
            inference_time_sec = time.perf_counter() - start_sec
        except Exception as exc:  # noqa: BLE001
            self.throttled_warn("inference", f"MediaPipe pose inference failed: {exc}")
            return

        # Section: Convert MediaPipe landmarks into ROS messages and publish all outputs.
        raw_pose_array = build_pose_array(msg, result.pose_landmarks, self.torso_visibility_threshold)
        pose_array, diagnostics = filter_pose_candidates(
            raw_pose_array,
            min_usable_landmarks=self.min_usable_landmarks,
            min_torso_landmark_confidence=self.min_torso_landmark_confidence,
            min_pose_bbox_area_ratio=self.min_pose_bbox_area_ratio,
            duplicate_iou_threshold=self.duplicate_iou_threshold,
            duplicate_center_distance_ratio=self.duplicate_center_distance_ratio,
            prefer_single_person_mode=self.prefer_single_person_mode,
            arm_landmark_min_visibility=self.arm_landmark_min_visibility,
        )
        self.publish_outputs(msg, pose_array, rgb_image, diagnostics)
        self.update_performance_stats(inference_time_sec)

    def ros_image_to_rgb(self, msg: Image) -> np.ndarray:
        """Special function: Convert a ROS Image into an RGB NumPy array without fabricating pixels."""
        if msg.encoding.lower() in {"rgb8", "rgba8"}:
            rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        else:
            bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        return np.ascontiguousarray(rgb_image)

    def publish_outputs(self, source_msg: Image, pose_array: HumanPoseArray, rgb_image: np.ndarray | None, diagnostics: dict[str, int]) -> None:
        """Special function: Publish pose array, pose summary, legacy skeletons, and debug image."""
        self.last_filter_diagnostics = diagnostics
        timestamp_sec = ros_stamp_to_seconds(source_msg.header.stamp)
        self.assign_track_ids(pose_array, timestamp_sec)
        # Section: Filter lower-body posture separately after person tracking; arm actions remain independent.
        self.classify_postures(pose_array)
        gesture_results = self.classify_gestures(timestamp_sec, pose_array)
        self.poses_pub.publish(pose_array)
        self.publish_pose_summary(pose_array, diagnostics, gesture_results)
        self.publish_gesture_summary(source_msg, pose_array, gesture_results)
        self.legacy_skeletons_pub.publish(build_legacy_skeletons(pose_array, source_msg.width, source_msg.height))
        self.maybe_speak_pose_report(pose_array)
        if (self.publish_debug_image_enabled or self.gui_available) and rgb_image is not None:
            self.publish_debug_image(source_msg, rgb_image, pose_array, gesture_results)

    def publish_pose_summary(self, pose_array: HumanPoseArray, diagnostics: dict[str, int], gesture_results: dict[int, Any] | None = None) -> None:
        """Special function: Publish detected pose and gesture statuses as compact JSON text."""
        summary_msg = String()
        summary_msg.data = json.dumps(build_pose_summary(pose_array, diagnostics, gesture_results))
        self.pose_summary_pub.publish(summary_msg)

    def publish_debug_image(self, source_msg: Image, rgb_image: np.ndarray, pose_array: HumanPoseArray, gesture_results: dict[int, Any] | None = None) -> None:
        """Special function: Draw skeletons, arm states, trajectories, reversals, and reliability."""
        active_gestures = gesture_results or {}
        debug_rgb = rgb_image.copy()
        height, width = debug_rgb.shape[:2]
        for pose in pose_array.poses:
            points = normalized_landmarks_to_pixels(pose.landmarks, width, height)
            color = status_color(pose.status)
            for start_index, end_index in MEDIAPIPE_CONNECTIONS:
                start_point = points.get(start_index)
                end_point = points.get(end_index)
                if start_point is None or end_point is None:
                    continue
                cv2.line(debug_rgb, start_point, end_point, color, 2)
            for index, point in points.items():
                landmark = pose.landmarks[index]
                radius = 6 if index in {11, 12, 13, 14, 15, 16} else 3
                cv2.circle(debug_rgb, point, radius, color, -1)
                cv2.putText(debug_rgb, f"{landmark.visibility:.2f}", (point[0] + 4, point[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.30, color, 1, cv2.LINE_AA)

            # Section: Draw per-track wrist trajectories before the text overlay.
            self.draw_wrist_trajectory(debug_rgb, int(pose.person_id), ARM_LEFT, width, height, (80, 220, 255))
            self.draw_wrist_trajectory(debug_rgb, int(pose.person_id), ARM_RIGHT, width, height, (255, 120, 220))
            label_point = first_visible_point(points)
            if label_point is not None:
                gesture = active_gestures.get(int(pose.person_id))
                gesture_text = ""
                if gesture is not None and gesture.gesture != GESTURE_NONE:
                    gesture_text = f" {gesture.gesture}/{gesture.active_arm} {gesture.confidence:.2f}"
                cv2.putText(debug_rgb, f"id={pose.person_id} {pose.pose_label} {pose.status}{gesture_text}", label_point, cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
                if gesture is not None:
                    diagnostics = getattr(gesture, "diagnostics", {}) or {}
                    left_wave = diagnostics.get("left_wave", {})
                    right_wave = diagnostics.get("right_wave", {})
                    left_changes = left_wave.get("wave_direction_changes", 0)
                    right_changes = right_wave.get("wave_direction_changes", 0)
                    wave_confidence = max(float(left_wave.get("score", 0.0)), float(right_wave.get("score", 0.0)))
                    arm_text = f"L={gesture.left_arm_state} R={gesture.right_arm_state}"
                    wave_text = f"wave={gesture.wave_state} changes L/R={left_changes}/{right_changes} confidence={wave_confidence:.2f}"
                    cv2.putText(debug_rgb, arm_text, (label_point[0], label_point[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
                    cv2.putText(debug_rgb, wave_text, (label_point[0], label_point[1] + 38), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
                    self.draw_unreliable_arm_landmarks(debug_rgb, points, diagnostics, label_point)
        debug_bgr = cv2.cvtColor(debug_rgb, cv2.COLOR_RGB2BGR)
        if self.publish_debug_image_enabled:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_bgr, encoding="bgr8")
            debug_msg.header = source_msg.header
            self.debug_image_pub.publish(debug_msg)
        if self.gui_available:
            cv2.imshow(self.gui_window_name, debug_bgr)
            cv2.waitKey(1)

    def draw_wrist_trajectory(
        self,
        image: np.ndarray,
        person_id: int,
        arm: str,
        width: int,
        height: int,
        color: tuple[int, int, int],
    ) -> None:
        """Special function: Draw one person's recent wrist trajectory on the debug image."""
        samples = self.gesture_classifier.wrist_trajectory(person_id, arm)
        trajectory = [
            (
                int(min(max(sample.wrist_x, 0.0), 1.0) * max(1, width - 1)),
                int(min(max(sample.wrist_y, 0.0), 1.0) * max(1, height - 1)),
            )
            for sample in samples
        ]
        for start, end in zip(trajectory, trajectory[1:]):
            cv2.line(image, start, end, color, 2)
        if trajectory:
            cv2.circle(image, trajectory[-1], 7, color, 2)

    def draw_unreliable_arm_landmarks(
        self,
        image: np.ndarray,
        points: dict[int, tuple[int, int]],
        diagnostics: dict[str, Any],
        label_point: tuple[int, int],
    ) -> None:
        """Special function: Mark visible but unreliable shoulder, elbow, or wrist landmarks."""
        joint_indices = {
            ARM_LEFT: {"shoulder": 11, "elbow": 13, "wrist": 15},
            ARM_RIGHT: {"shoulder": 12, "elbow": 14, "wrist": 16},
        }
        unreliable_labels = []
        for arm in (ARM_LEFT, ARM_RIGHT):
            arm_diagnostics = diagnostics.get(f"{arm}_arm", {})
            for joint in arm_diagnostics.get("unreliable_landmarks", []):
                unreliable_labels.append(f"{arm}:{joint}")
                point = points.get(joint_indices[arm][joint])
                if point is None:
                    continue
                cv2.line(image, (point[0] - 7, point[1] - 7), (point[0] + 7, point[1] + 7), (255, 0, 0), 2)
                cv2.line(image, (point[0] + 7, point[1] - 7), (point[0] - 7, point[1] + 7), (255, 0, 0), 2)
        if unreliable_labels:
            text = "unreliable=" + ",".join(unreliable_labels)
            cv2.putText(image, text, (label_point[0], label_point[1] + 56), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 0, 0), 1, cv2.LINE_AA)

    def assign_track_ids(self, pose_array: HumanPoseArray, timestamp_sec: float) -> None:
        """Special function: Assign stable nearest-centroid track IDs to filtered poses."""
        self.prune_tracks(timestamp_sec)
        used_tracks: set[int] = set()
        for pose in pose_array.poses:
            centroid = pose_centroid(pose.landmarks)
            if centroid is None:
                continue
            track_id = self.find_nearest_track(centroid, used_tracks)
            if track_id is None:
                track_id = self.next_track_id
                self.next_track_id += 1
            self.tracks[track_id] = TrackState(track_id, centroid, timestamp_sec)
            used_tracks.add(track_id)
            pose.person_id = int(track_id)
            pose.description = describe_human_pose(pose.person_id, pose.status, pose.pose_label)

    def find_nearest_track(self, centroid: tuple[float, float], used_tracks: set[int]) -> int | None:
        """Special function: Find the nearest unused existing track to one pose centroid."""
        best_track_id = None
        best_distance = float("inf")
        for track_id, track in self.tracks.items():
            if track_id in used_tracks:
                continue
            distance = math.hypot(centroid[0] - track.centroid[0], centroid[1] - track.centroid[1])
            if distance <= self.tracker_max_distance and distance < best_distance:
                best_track_id = track_id
                best_distance = distance
        return best_track_id

    def prune_tracks(self, timestamp_sec: float) -> None:
        """Special function: Drop tracks that have not been observed recently."""
        stale_ids = [
            track_id for track_id, track in self.tracks.items()
            if (timestamp_sec - track.last_seen_sec) > self.tracker_timeout_sec
        ]
        for track_id in stale_ids:
            del self.tracks[track_id]
            self.last_gesture_results.pop(track_id, None)
            self.posture_tracks.pop(track_id, None)
            self.posture_diagnostics.pop(track_id, None)

    def classify_postures(self, pose_array: HumanPoseArray) -> None:
        """Special function: Score and temporally filter lower-body posture without touching arm classification."""
        # Section: Re-evaluate each filtered pose with the configured visibility and angle thresholds.
        for pose in pose_array.poses:
            person_id = int(pose.person_id)
            evaluation = evaluate_body_posture(pose.landmarks, self.posture_config)
            state = self.posture_tracks.setdefault(person_id, PostureTrackState())
            pose.pose_label = filter_posture_state(
                state, evaluation.raw_posture, self.posture_confirm_frames, self.posture_release_frames,
                int(self.get_parameter("lying_confirm_frames").value), int(self.get_parameter("lying_release_frames").value),
                int(self.get_parameter("posture_history_size").value)
            )
            pose.description = describe_human_pose(person_id, pose.status, pose.pose_label)
            self.posture_diagnostics[person_id] = evaluation
        # Section: Emit concise feature diagnostics only when explicitly enabled and throttled.
        now_sec = time.monotonic()
        if self.posture_debug and (now_sec - self.last_posture_debug_sec) >= self.posture_debug_interval_sec:
            for person_id, evaluation in self.posture_diagnostics.items():
                self.get_logger().info(f"posture id={person_id} torso_angle={evaluation.torso_angle_deg} torso_vertical_ratio={evaluation.torso_vertical_ratio} body_aspect={evaluation.body_aspect_ratio} lying_score={evaluation.lying_score:.2f} sitting_score={evaluation.sitting_score:.2f} standing_score={evaluation.standing_score:.2f} margin={evaluation.score_margin:.2f} raw={evaluation.raw_posture} filtered={self.posture_tracks[person_id].filtered_posture} legs={evaluation.legs}")
            self.last_posture_debug_sec = now_sec

    def classify_gestures(self, timestamp_sec: float, pose_array: HumanPoseArray) -> dict[int, Any]:
        """Special function: Classify gestures for each tracked person in a pose array."""
        if not self.gesture_detection_enabled:
            self.last_gesture_results = {}
            return {}
        self.gesture_classifier.prune_missing_people(timestamp_sec)
        results: dict[int, Any] = {}
        for pose in pose_array.poses:
            result = self.gesture_classifier.update(timestamp_sec, int(pose.person_id), pose.landmarks)
            results[int(pose.person_id)] = result
        self.last_gesture_results = results
        return results

    def publish_gesture_summary(self, source_msg: Image, pose_array: HumanPoseArray, gesture_results: dict[int, Any]) -> None:
        """Special function: Publish one JSON gesture object per frame for GPSR."""
        payload = build_gesture_summary(source_msg, pose_array, gesture_results)
        msg = String()
        msg.data = json.dumps(payload)
        self.gestures_pub.publish(msg)

    def maybe_speak_pose_report(self, pose_array: HumanPoseArray) -> None:
        """Special function: Speak detected human pose status through the existing TTS service."""
        if not self.enable_tts or self.tts_client is None:
            return
        report_text = build_tts_report(
            pose_array,
            gesture_results=self.last_gesture_results,
            speak_unreliable=self.tts_speak_unreliable,
            speak_no_detection=self.tts_speak_no_detection,
        )
        if not report_text:
            return
        fingerprint = make_tts_fingerprint(pose_array, self.last_gesture_results)
        now_sec = time.monotonic()
        if self.tts_on_change_only and fingerprint == self.last_tts_fingerprint:
            return
        if (now_sec - self.last_tts_report_time_sec) < self.tts_min_interval_sec:
            return
        if self.active_tts_future is not None and not self.active_tts_future.done():
            return
        if not self.tts_client.service_is_ready():
            self.throttled_warn("tts_not_ready", "TTS service is not ready; pose announcement skipped.")
            return
        request = SayText.Request()
        request.text = report_text
        self.active_tts_future = self.tts_client.call_async(request)
        self.last_tts_fingerprint = fingerprint
        self.last_tts_report_time_sec = now_sec

    def update_performance_stats(self, inference_time_sec: float) -> None:
        """Special function: Log average inference time and effective FPS periodically."""
        self.processed_frames += 1
        self.stats_window_frames += 1
        self.total_inference_time_sec += inference_time_sec
        elapsed_sec = time.monotonic() - self.stats_window_start_sec
        if elapsed_sec < self.log_stats_interval_sec:
            return
        avg_ms = 1000.0 * self.total_inference_time_sec / max(1, self.processed_frames)
        fps = self.stats_window_frames / max(0.001, elapsed_sec)
        self.get_logger().info(f"MediaPipe pose avg inference={avg_ms:.1f} ms, effective_fps={fps:.2f}")
        self.stats_window_start_sec = time.monotonic()
        self.stats_window_frames = 0

    def throttled_warn(self, key: str, message: str, period_sec: float = 5.0) -> None:
        """Special function: Log repeated warnings no more than once per throttle period."""
        now = time.monotonic()
        if now - self.last_warning_time_by_key.get(key, 0.0) >= period_sec:
            self.last_warning_time_by_key[key] = now
            self.get_logger().warn(message)


def ros_stamp_to_millis(stamp: Any) -> int:
    """Special function: Convert a ROS builtin_interfaces/Time stamp to milliseconds."""
    return int(stamp.sec) * 1000 + int(stamp.nanosec) // 1_000_000


def ros_stamp_to_seconds(stamp: Any) -> float:
    """Special function: Convert a ROS builtin_interfaces/Time stamp to floating seconds."""
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def build_pose_array(source_msg: Image, pose_landmarks: list[Any], torso_visibility_threshold: float = 0.50) -> HumanPoseArray:
    """Special function: Convert MediaPipe landmark lists into a HumanPoseArray message."""
    pose_array = HumanPoseArray()
    pose_array.header = source_msg.header
    for person_id, landmarks in enumerate(pose_landmarks or []):
        pose = HumanPose()
        pose.person_id = int(person_id)
        pose.landmarks = [landmark_to_msg(landmark) for landmark in landmarks]
        pose.status = select_pose_status(pose.landmarks, torso_visibility_threshold)
        pose.pose_label = classify_body_pose(pose.landmarks)
        pose.description = describe_human_pose(pose.person_id, pose.status, pose.pose_label)
        pose_array.poses.append(pose)
    return pose_array


def default_filter_diagnostics(raw_count: int) -> dict[str, int]:
    """Special function: Build default raw-vs-filtered diagnostics for pose summaries."""
    return {
        "raw_count": int(raw_count),
        "filtered_count": int(raw_count),
        "suppressed_duplicates": 0,
        "rejected_low_quality": 0,
    }


def filter_pose_candidates(
    raw_pose_array: HumanPoseArray,
    min_usable_landmarks: int = 12,
    min_torso_landmark_confidence: float = 0.50,
    min_pose_bbox_area_ratio: float = 0.02,
    duplicate_iou_threshold: float = 0.35,
    duplicate_center_distance_ratio: float = 0.18,
    prefer_single_person_mode: bool = False,
    arm_landmark_min_visibility: float = 0.55,
) -> tuple[HumanPoseArray, dict[str, int]]:
    """Special function: Reject weak MediaPipe candidates and suppress duplicate humans."""
    diagnostics = default_filter_diagnostics(len(raw_pose_array.poses))
    filtered_pose_array = HumanPoseArray()
    filtered_pose_array.header = raw_pose_array.header

    # Section: Score every raw pose and reject low-quality candidates before duplicate checks.
    candidates: list[PoseCandidate] = []
    for pose in raw_pose_array.poses:
        candidate = build_pose_candidate(pose)
        if candidate is None:
            diagnostics["rejected_low_quality"] += 1
            continue
        arm_action_eligible = has_reliable_arm_action_landmarks(
            pose.landmarks,
            arm_landmark_min_visibility,
        )
        normal_pose_quality = (
            candidate.usable_landmarks >= min_usable_landmarks
            and candidate.torso_confidence >= min_torso_landmark_confidence
            and candidate.bbox_area_ratio >= min_pose_bbox_area_ratio
        )
        if not normal_pose_quality and not arm_action_eligible:
            diagnostics["rejected_low_quality"] += 1
            continue
        candidates.append(candidate)

    # Section: Keep the highest-quality candidates first so duplicate suppression preserves the best pose.
    candidates.sort(key=lambda candidate: candidate.quality, reverse=True)
    accepted: list[PoseCandidate] = []
    for candidate in candidates:
        if prefer_single_person_mode and accepted:
            diagnostics["suppressed_duplicates"] += 1
            continue
        if any(poses_are_duplicates(candidate, accepted_candidate, duplicate_iou_threshold, duplicate_center_distance_ratio) for accepted_candidate in accepted):
            diagnostics["suppressed_duplicates"] += 1
            continue
        accepted.append(candidate)

    # Section: Restore stable left-to-right person IDs after filtering.
    accepted.sort(key=lambda candidate: bbox_center(candidate.bbox)[0])
    for person_id, candidate in enumerate(accepted):
        pose = candidate.pose
        pose.person_id = int(person_id)
        pose.description = describe_human_pose(pose.person_id, pose.status, pose.pose_label)
        filtered_pose_array.poses.append(pose)
    diagnostics["filtered_count"] = len(filtered_pose_array.poses)
    return filtered_pose_array, diagnostics


def build_pose_candidate(pose: HumanPose) -> PoseCandidate | None:
    """Special function: Compute bbox and quality metadata for one pose candidate."""
    bbox = normalized_bbox_from_landmarks(pose.landmarks)
    if bbox is None:
        return None
    usable_count = count_usable_landmarks(pose.landmarks)
    torso_confidence = min_torso_confidence(pose.landmarks)
    area_ratio = bbox_area(bbox)
    quality = float(torso_confidence) * 0.55 + min(1.0, usable_count / 33.0) * 0.30 + min(1.0, area_ratio / 0.20) * 0.15
    return PoseCandidate(pose, bbox, quality, usable_count, torso_confidence, area_ratio)


def pose_centroid(landmarks: list[PoseLandmark]) -> tuple[float, float] | None:
    """Special function: Compute an upper-body-first centroid for nearest-centroid tracking."""
    shoulder_points = [
        landmarks[index] for index in REQUIRED_UPPER_BODY_INDICES
        if index < len(landmarks) and usable_landmark(landmarks[index])
    ]
    points = shoulder_points
    if not points:
        points = [
            landmarks[index] for index in REQUIRED_TORSO_INDICES
            if index < len(landmarks) and usable_landmark(landmarks[index])
        ]
    if not points:
        return None
    return (
        sum(float(point.x) for point in points) / len(points),
        sum(float(point.y) for point in points) / len(points),
    )


def normalized_bbox_from_landmarks(landmarks: list[PoseLandmark]) -> tuple[float, float, float, float] | None:
    """Special function: Build a normalized bbox around finite usable landmarks."""
    points = [
        (float(landmark.x), float(landmark.y))
        for landmark in landmarks
        if usable_landmark(landmark)
    ]
    if not points:
        return None
    xs = [min(max(point[0], 0.0), 1.0) for point in points]
    ys = [min(max(point[1], 0.0), 1.0) for point in points]
    x1, y1, x2, y2 = min(xs), min(ys), max(xs), max(ys)
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


def count_usable_landmarks(landmarks: list[PoseLandmark]) -> int:
    """Special function: Count landmarks that have finite geometry and usable confidence."""
    return sum(1 for landmark in landmarks if usable_landmark(landmark))


def has_reliable_arm_action_landmarks(
    landmarks: list[PoseLandmark],
    min_visibility: float,
) -> bool:
    """Special function: Keep a pose when either shoulder-elbow-wrist chain is reliable."""
    for indices in ARM_ACTION_INDICES:
        if all(
            index < len(landmarks)
            and finite_landmark(landmarks[index])
            and float(landmarks[index].visibility) >= min_visibility
            for index in indices
        ):
            return True
    return False


def bbox_area(bbox: tuple[float, float, float, float]) -> float:
    """Special function: Return normalized bbox area."""
    x1, y1, x2, y2 = bbox
    return max(0.0, x2 - x1) * max(0.0, y2 - y1)


def bbox_center(bbox: tuple[float, float, float, float]) -> tuple[float, float]:
    """Special function: Return normalized bbox center."""
    x1, y1, x2, y2 = bbox
    return (x1 + x2) / 2.0, (y1 + y2) / 2.0


def bbox_iou(first: tuple[float, float, float, float], second: tuple[float, float, float, float]) -> float:
    """Special function: Compute normalized bbox intersection-over-union."""
    first_x1, first_y1, first_x2, first_y2 = first
    second_x1, second_y1, second_x2, second_y2 = second
    inter_x1 = max(first_x1, second_x1)
    inter_y1 = max(first_y1, second_y1)
    inter_x2 = min(first_x2, second_x2)
    inter_y2 = min(first_y2, second_y2)
    intersection = max(0.0, inter_x2 - inter_x1) * max(0.0, inter_y2 - inter_y1)
    union = bbox_area(first) + bbox_area(second) - intersection
    return intersection / union if union > 0.0 else 0.0


def poses_are_duplicates(first: PoseCandidate, second: PoseCandidate, iou_threshold: float, center_distance_ratio: float) -> bool:
    """Special function: Decide whether two pose candidates describe the same person."""
    if bbox_iou(first.bbox, second.bbox) >= iou_threshold:
        return True
    first_center = bbox_center(first.bbox)
    second_center = bbox_center(second.bbox)
    center_distance = math.hypot(first_center[0] - second_center[0], first_center[1] - second_center[1])
    return center_distance <= center_distance_ratio


def landmark_to_msg(landmark: Any) -> PoseLandmark:
    """Special function: Preserve one MediaPipe landmark without replacing missing values."""
    msg = PoseLandmark()
    msg.x = float(getattr(landmark, "x", math.nan))
    msg.y = float(getattr(landmark, "y", math.nan))
    msg.z = float(getattr(landmark, "z", math.nan))
    msg.visibility = float(getattr(landmark, "visibility", 0.0))
    msg.presence = float(getattr(landmark, "presence", 0.0))
    return msg


def select_pose_status(landmarks: list[PoseLandmark], torso_visibility_threshold: float = 0.50) -> str:
    """Special function: Choose pose status without requiring hips or lower-body landmarks."""
    if len(landmarks) != 33:
        return STATUS_NOT_DETECTED
    upper_body_anchors = [landmarks[index] for index in REQUIRED_UPPER_BODY_INDICES]
    if any(not finite_landmark(landmark) for landmark in upper_body_anchors):
        return STATUS_UNRELIABLE
    if any(float(landmark.visibility) < torso_visibility_threshold for landmark in upper_body_anchors):
        return STATUS_UNRELIABLE
    if any(float(landmark.presence) < torso_visibility_threshold for landmark in upper_body_anchors):
        return STATUS_UNRELIABLE
    return STATUS_DETECTED


def angle_degrees(first: PoseLandmark, middle: PoseLandmark, last: PoseLandmark) -> float | None:
    """Special function: Return a guarded three-point angle in degrees, with the joint as middle."""
    # Section: Reject non-finite landmarks before vector arithmetic.
    if any(not finite_landmark(landmark) for landmark in (first, middle, last)):
        return None
    # Section: Form the two vectors that meet at the requested middle joint.
    first_vector = (float(first.x) - float(middle.x), float(first.y) - float(middle.y), float(first.z) - float(middle.z))
    last_vector = (float(last.x) - float(middle.x), float(last.y) - float(middle.y), float(last.z) - float(middle.z))
    first_length = math.sqrt(sum(component * component for component in first_vector))
    last_length = math.sqrt(sum(component * component for component in last_vector))
    # Section: Avoid division by zero for overlapping or degenerate landmarks.
    if first_length <= 1e-6 or last_length <= 1e-6:
        return None
    cosine = sum(a * b for a, b in zip(first_vector, last_vector)) / (first_length * last_length)
    # Section: Clamp floating-point roundoff before acos.
    return math.degrees(math.acos(min(1.0, max(-1.0, cosine))))


def torso_length(landmarks: list[PoseLandmark], minimum_visibility: float) -> float | None:
    """Special function: Return a confidence-gated shoulder-to-hip body scale without pixel thresholds."""
    # Section: Use each visible same-side shoulder-to-hip pair independently.
    lengths = []
    for shoulder_index, hip_index in ((11, 23), (12, 24)):
        shoulder, hip = landmarks[shoulder_index], landmarks[hip_index]
        if usable_landmark(shoulder, minimum_visibility) and usable_landmark(hip, minimum_visibility):
            length = math.dist((shoulder.x, shoulder.y, shoulder.z), (hip.x, hip.y, hip.z))
            if length > 1e-6:
                lengths.append(length)
    return sum(lengths) / len(lengths) if lengths else None


def clamp01(value: float) -> float:
    """Special function: Clamp a normalized posture evidence value to the inclusive unit interval."""
    return min(1.0, max(0.0, float(value)))


def graded_score(value: float | None, partial: float, full: float, higher_is_better: bool = True) -> float | None:
    """Special function: Convert one finite feature into conservative partial-to-full evidence."""
    # Section: Reject unavailable or degenerate calibration intervals.
    if value is None or not math.isfinite(float(value)) or math.isclose(partial, full):
        return None
    # Section: Interpolate in the appropriate direction without reversing descending thresholds.
    if higher_is_better:
        normalized = (float(value) - partial) / (full - partial)
    else:
        normalized = (partial - float(value)) / (partial - full)
    return clamp01(normalized)


def mean_available(values: list[float | None]) -> float | None:
    """Special function: Average only finite evidence values without fabricating missing landmarks."""
    usable = [float(value) for value in values if value is not None and math.isfinite(float(value))]
    return sum(usable) / len(usable) if usable else None


def extract_posture_features(landmarks: list[PoseLandmark], config: PostureConfig | None = None) -> tuple[dict[str, float | bool | None], dict[str, dict[str, float | bool | None]]]:
    """Special function: Extract reusable confidence-gated normalized posture features without classification."""
    # Section: Return empty features for malformed MediaPipe landmark lists.
    active_config = config or PostureConfig()
    features: dict[str, float | bool | None] = {"torso_reliable": False, "lying_torso_reliable": False}
    legs: dict[str, dict[str, float | bool | None]] = {}
    if len(landmarks) != 33:
        return features, legs
    # Section: Build torso centers from any reliable same-side pair for upright classification.
    torso_pairs = [(11, 23), (12, 24)]
    valid_pairs = [(landmarks[shoulder], landmarks[hip]) for shoulder, hip in torso_pairs if usable_landmark(landmarks[shoulder], active_config.landmark_min_visibility) and usable_landmark(landmarks[hip], active_config.landmark_min_visibility)]
    both_shoulders = all(usable_landmark(landmarks[index], active_config.landmark_min_visibility) for index in (11, 12))
    both_hips = all(usable_landmark(landmarks[index], active_config.landmark_min_visibility) for index in (23, 24))
    if not valid_pairs:
        return features, legs
    shoulder_center = (sum(float(pair[0].x) for pair in valid_pairs) / len(valid_pairs), sum(float(pair[0].y) for pair in valid_pairs) / len(valid_pairs))
    hip_center = (sum(float(pair[1].x) for pair in valid_pairs) / len(valid_pairs), sum(float(pair[1].y) for pair in valid_pairs) / len(valid_pairs))
    torso_scale = mean_available([math.dist((shoulder.x, shoulder.y, shoulder.z), (hip.x, hip.y, hip.z)) for shoulder, hip in valid_pairs])
    if torso_scale is None or torso_scale <= 1e-6:
        return features, legs
    horizontal_delta = abs(hip_center[0] - shoulder_center[0])
    vertical_delta = abs(hip_center[1] - shoulder_center[1])
    torso_angle = math.degrees(math.atan2(vertical_delta, max(horizontal_delta, 1e-6)))
    features.update({"torso_reliable": True, "lying_torso_reliable": both_shoulders and both_hips, "torso_scale": torso_scale, "torso_angle_deg": torso_angle, "torso_vertical_ratio": vertical_delta / torso_scale, "torso_vertical_score": graded_score(torso_angle, 50.0, 75.0)})
    # Section: Measure body span using only reliable torso and lower-body points.
    body_points = [(float(landmarks[index].x), float(landmarks[index].y)) for index in (11, 12, 23, 24, 25, 26, 27, 28) if usable_landmark(landmarks[index], active_config.landmark_min_visibility)]
    if len(body_points) >= 4:
        width = max(point[0] for point in body_points) - min(point[0] for point in body_points)
        height = max(point[1] for point in body_points) - min(point[1] for point in body_points)
        features.update({"body_width_ratio": width / torso_scale, "body_height_ratio": height / torso_scale, "body_aspect_ratio": width / max(height, 1e-6)})
    # Section: Extract each leg independently so one occlusion cannot invalidate the other.
    leg_alignment_values: list[float] = []
    for side, shoulder_index, hip_index, knee_index, ankle_index in (("left", 11, 23, 25, 27), ("right", 12, 24, 26, 28)):
        hip, knee, ankle = landmarks[hip_index], landmarks[knee_index], landmarks[ankle_index]
        reliable = all(usable_landmark(point, active_config.landmark_min_visibility) for point in (hip, knee, ankle))
        leg: dict[str, float | bool | None] = {"reliable": reliable, "knee_angle_deg": None, "hip_angle_deg": None, "hip_knee_ratio": None, "leg_extension": None, "vertical_order": False, "hip_visibility": float(hip.visibility), "knee_visibility": float(knee.visibility), "ankle_visibility": float(ankle.visibility)}
        legs[side] = leg
        if not reliable:
            continue
        knee_angle = angle_degrees(hip, knee, ankle)
        shoulder = landmarks[shoulder_index]
        leg["knee_angle_deg"] = knee_angle
        leg["hip_angle_deg"] = angle_degrees(shoulder, hip, knee) if usable_landmark(shoulder, active_config.landmark_min_visibility) else None
        leg["hip_knee_ratio"] = abs(float(knee.y) - float(hip.y)) / torso_scale
        chain_length = math.dist((hip.x, hip.y, hip.z), (knee.x, knee.y, knee.z)) + math.dist((knee.x, knee.y, knee.z), (ankle.x, ankle.y, ankle.z))
        leg["leg_extension"] = math.dist((hip.x, hip.y, hip.z), (ankle.x, ankle.y, ankle.z)) / chain_length if chain_length > 1e-6 else None
        leg["vertical_order"] = float(hip.y) < float(knee.y) < float(ankle.y)
        leg_alignment_values.extend([abs(float(knee.y) - hip_center[1]) / torso_scale, abs(float(ankle.y) - hip_center[1]) / torso_scale])
    features["legs_horizontal_alignment"] = mean_available(leg_alignment_values)
    return features, legs


def score_posture(features: dict[str, float | bool | None], legs: dict[str, dict[str, float | bool | None]], config: PostureConfig | None = None) -> tuple[dict[str, float], float]:
    """Special function: Score independent standing, sitting, and lying evidence from extracted features."""
    active_config = config or PostureConfig()
    # Section: Score per-leg evidence with graded angles and normalized geometry.
    standing_parts: list[float | None] = [features.get("torso_vertical_score") if bool(features.get("torso_reliable")) else None]
    sitting_parts: list[float | None] = [features.get("torso_vertical_score") if bool(features.get("torso_reliable")) else None]
    knee_extension: list[float | None] = []
    knee_bend: list[float | None] = []
    hip_closeness: list[float | None] = []
    hip_flexion: list[float | None] = []
    for leg in legs.values():
        if not bool(leg.get("reliable")):
            continue
        extension = graded_score(leg.get("knee_angle_deg"), active_config.standing_knee_partial_angle, active_config.standing_knee_full_score_angle)
        bend = graded_score(leg.get("knee_angle_deg"), active_config.sitting_knee_partial_angle, active_config.sitting_knee_full_score_angle, higher_is_better=False)
        knee_extension.append(extension)
        knee_bend.append(bend)
        hip_closeness.append(graded_score(leg.get("hip_knee_ratio"), active_config.hip_knee_standing_ratio_min, active_config.hip_knee_sitting_ratio_max, higher_is_better=False))
        hip_flexion.append(graded_score(leg.get("hip_angle_deg"), active_config.sitting_hip_angle_max, 95.0, higher_is_better=False))
        standing_parts.extend([extension, graded_score(leg.get("leg_extension"), 0.75, active_config.standing_leg_extension_min), 1.0 if bool(leg.get("vertical_order")) else 0.0])
        sitting_parts.extend([bend, graded_score(leg.get("hip_angle_deg"), active_config.sitting_hip_angle_max, 95.0, higher_is_better=False), graded_score(leg.get("hip_knee_ratio"), active_config.hip_knee_standing_ratio_min, active_config.hip_knee_sitting_ratio_max, higher_is_better=False)])
    standing_score = mean_available(standing_parts) or 0.0
    # Section: Preserve strong seated evidence when hip flexion and hip-knee closeness agree.
    sitting_score = max(mean_available(sitting_parts) or 0.0, mean_available([features.get("torso_vertical_score"), mean_available(hip_flexion), mean_available(hip_closeness)]) or 0.0)
    # Section: Lying relies on torso anchors; legs may only add optional support.
    lying_parts: list[float | None] = []
    if bool(features.get("lying_torso_reliable")):
        lying_parts = [graded_score(features.get("torso_angle_deg"), active_config.lying_torso_partial_angle, active_config.lying_torso_full_score_angle, higher_is_better=False), graded_score(features.get("torso_vertical_ratio"), active_config.lying_vertical_ratio_max, 0.10, higher_is_better=False), graded_score(features.get("body_aspect_ratio"), active_config.lying_body_aspect_min, 3.0)]
        alignment = features.get("legs_horizontal_alignment")
        if alignment is not None:
            lying_parts.append(graded_score(alignment, active_config.lying_vertical_ratio_max, 0.10, higher_is_better=False))
    lying_score = mean_available(lying_parts) or 0.0
    # Section: Apply small incompatible-class penalties only for strong opposing evidence.
    horizontal_strength = graded_score(features.get("torso_angle_deg"), active_config.lying_torso_partial_angle, active_config.lying_torso_full_score_angle, higher_is_better=False) or 0.0
    standing_score *= 1.0 - 0.75 * horizontal_strength
    sitting_score *= 1.0 - 0.75 * horizontal_strength
    standing_score *= 1.0 - 0.35 * (mean_available(knee_bend) or 0.0)
    standing_score *= 1.0 - 0.75 * (mean_available(hip_closeness) or 0.0)
    sitting_score *= 1.0 - 0.35 * (mean_available(knee_extension) or 0.0)
    scores = {POSE_STANDING: clamp01(standing_score), POSE_SITTING: clamp01(sitting_score), POSE_LYING_DOWN: clamp01(lying_score)}
    ordered = sorted(scores.values(), reverse=True)
    return scores, ordered[0] - ordered[1]


def evaluate_body_posture(landmarks: list[PoseLandmark], config: PostureConfig | None = None) -> PostureEvaluation:
    """Special function: Produce raw posture and calibration diagnostics from features and competing scores."""
    active_config = config or PostureConfig()
    features, legs = extract_posture_features(landmarks, active_config)
    if not bool(features.get("torso_reliable")):
        return PostureEvaluation(POSE_UNRELIABLE if len(landmarks) != 33 else POSE_UNKNOWN, 0.0, 0.0, 0.0, None, None, None, legs, features, {}, 0.0)
    scores, margin = score_posture(features, legs, active_config)
    best_posture, best_score = max(scores.items(), key=lambda item: item[1])
    raw_posture = best_posture if best_score >= active_config.posture_min_score and margin >= active_config.posture_min_margin else POSE_UNKNOWN
    return PostureEvaluation(raw_posture, scores[POSE_SITTING], scores[POSE_STANDING], scores[POSE_LYING_DOWN], features.get("torso_angle_deg"), features.get("torso_vertical_ratio"), features.get("body_aspect_ratio"), legs, features, scores, margin)


def classify_body_pose(landmarks: list[PoseLandmark], config: PostureConfig | None = None) -> str:
    """Special function: Return the raw scored posture label for compatibility callers."""
    return evaluate_body_posture(landmarks, config).raw_posture

def filter_posture_state(state: PostureTrackState, raw_posture: str, confirm_frames: int, release_frames: int, lying_confirm_frames: int | None = None, lying_release_frames: int | None = None, history_size: int = 5) -> str:
    """Special function: Apply per-person confirmation and release hysteresis to posture changes."""
    # Section: Retain bounded raw history for diagnosis without letting unknown erase stable output.
    state.raw_history.append(raw_posture)
    del state.raw_history[:-max(1, history_size)]
    # Section: Unknown evidence does not erase a confirmed posture on a single occluded frame.
    if raw_posture not in (POSE_SITTING, POSE_STANDING, POSE_LYING_DOWN):
        state.candidate_posture = raw_posture
        state.candidate_frames = 0
        return state.filtered_posture
    entering_or_leaving_lying = raw_posture == POSE_LYING_DOWN or state.filtered_posture == POSE_LYING_DOWN
    initial_frames = lying_confirm_frames if entering_or_leaving_lying and lying_confirm_frames is not None else confirm_frames
    release_count = lying_release_frames if entering_or_leaving_lying and lying_release_frames is not None else release_frames
    required_frames = max(1, initial_frames if state.filtered_posture in (POSE_UNKNOWN, POSE_UNRELIABLE) else release_count)
    if raw_posture == state.filtered_posture:
        state.candidate_posture = raw_posture
        state.candidate_frames = 0
        return state.filtered_posture
    if raw_posture != state.candidate_posture:
        state.candidate_posture = raw_posture
        state.candidate_frames = 1
    else:
        state.candidate_frames += 1
    if state.candidate_frames >= required_frames:
        state.filtered_posture = raw_posture
        state.candidate_frames = 0
    return state.filtered_posture

def usable_landmark(landmark: PoseLandmark, min_confidence: float = 0.20) -> bool:
    """Special function: Return true when a landmark has usable geometry and confidence."""
    return finite_landmark(landmark) and float(landmark.visibility) >= min_confidence and float(landmark.presence) >= min_confidence


def average_coordinate(landmarks: list[PoseLandmark], coordinate: str) -> float:
    """Special function: Average one coordinate across a landmark group."""
    return sum(float(getattr(landmark, coordinate)) for landmark in landmarks) / max(1, len(landmarks))


def describe_human_pose(person_id: int, status: str, pose_label: str) -> str:
    """Special function: Build a human-readable description for one detected pose."""
    prefix = f"Person {int(person_id)}"
    if status == STATUS_NOT_DETECTED:
        return f"{prefix} has no detected pose."
    if status == STATUS_UNRELIABLE:
        return f"{prefix} appears to be {pose_label}, but the pose is unreliable."
    return f"{prefix} is {pose_label}."


def finite_landmark(landmark: PoseLandmark) -> bool:
    """Special function: Return true when normalized coordinates are numeric finite values."""
    return math.isfinite(float(landmark.x)) and math.isfinite(float(landmark.y)) and math.isfinite(float(landmark.z))


def build_pose_summary(pose_array: HumanPoseArray, diagnostics: dict[str, int] | None = None, gesture_results: dict[int, Any] | None = None) -> dict[str, Any]:
    """Special function: Build compact JSON pose and gesture status output for ros2 topic echo."""
    active_diagnostics = diagnostics or default_filter_diagnostics(len(pose_array.poses))
    active_gestures = gesture_results or {}
    status_quantities: dict[str, int] = {}
    people = []
    for pose in pose_array.poses:
        status_quantities[pose.status] = int(status_quantities.get(pose.status, 0)) + 1
        gesture = active_gestures.get(int(pose.person_id))
        gesture_payload = gesture_to_payload(gesture)
        people.append({
            "person_id": int(pose.person_id),
            "status": str(pose.status),
            "pose_label": str(pose.pose_label),
            "gesture": gesture_payload["gesture"],
            "active_arm": gesture_payload["active_arm"],
            "gesture_confidence": gesture_payload["confidence"],
            "left_arm_state": gesture_payload["left_arm_state"],
            "right_arm_state": gesture_payload["right_arm_state"],
            "raised_arm_state": gesture_payload["raised_arm_state"],
            "wave_state": gesture_payload["wave_state"],
            "timestamp": gesture_payload["timestamp"],
            "description": str(pose.description),
            "landmarks": int(len(pose.landmarks)),
            "torso_visibility": torso_visibility_values(pose.landmarks),
        })
    return {
        "stamp": {
            "sec": int(pose_array.header.stamp.sec),
            "nanosec": int(pose_array.header.stamp.nanosec),
        },
        "count": int(len(pose_array.poses)),
        "raw_count": int(active_diagnostics.get("raw_count", len(pose_array.poses))),
        "filtered_count": int(active_diagnostics.get("filtered_count", len(pose_array.poses))),
        "suppressed_duplicates": int(active_diagnostics.get("suppressed_duplicates", 0)),
        "rejected_low_quality": int(active_diagnostics.get("rejected_low_quality", 0)),
        "primary_status": pose_array.poses[0].status if pose_array.poses else STATUS_NOT_DETECTED,
        "primary_pose": pose_array.poses[0].pose_label if pose_array.poses else POSE_UNKNOWN,
        "statuses": [pose.status for pose in pose_array.poses],
        "poses": [pose.pose_label for pose in pose_array.poses],
        "descriptions": [pose.description for pose in pose_array.poses],
        "status_quantities": status_quantities,
        "people": people,
    }


def build_gesture_summary(source_msg: Image, pose_array: HumanPoseArray, gesture_results: dict[int, Any]) -> dict[str, Any]:
    """Special function: Build compact per-frame gesture JSON for GPSR."""
    people = []
    for pose in pose_array.poses:
        result = gesture_results.get(int(pose.person_id))
        payload = gesture_to_payload(result)
        payload["person_id"] = int(pose.person_id)
        payload["timestamp"] = ros_stamp_to_seconds(source_msg.header.stamp)
        people.append(payload)
    return {
        "stamp": ros_stamp_to_seconds(source_msg.header.stamp),
        "people": people,
    }


def gesture_to_payload(gesture_result: Any | None) -> dict[str, Any]:
    """Special function: Convert an optional gesture result into JSON-safe fields."""
    if gesture_result is None:
        return {
            "gesture": GESTURE_NONE,
            "active_arm": ARM_UNKNOWN,
            "confidence": 0.0,
            "left_arm_state": "ARM_POSE_UNRELIABLE",
            "right_arm_state": "ARM_POSE_UNRELIABLE",
            "raised_arm_state": "ARM_POSE_UNRELIABLE",
            "left_wave_state": "WAVE_UNRELIABLE",
            "right_wave_state": "WAVE_UNRELIABLE",
            "wave_state": "WAVE_UNRELIABLE",
            "timestamp": 0.0,
            "diagnostics": {},
        }
    return {
        "gesture": str(getattr(gesture_result, "gesture", GESTURE_NONE)),
        "active_arm": str(getattr(gesture_result, "active_arm", ARM_UNKNOWN)),
        "confidence": float(getattr(gesture_result, "confidence", 0.0)),
        "left_arm_state": str(getattr(gesture_result, "left_arm_state", "ARM_POSE_UNRELIABLE")),
        "right_arm_state": str(getattr(gesture_result, "right_arm_state", "ARM_POSE_UNRELIABLE")),
        "raised_arm_state": str(getattr(gesture_result, "raised_arm_state", "ARM_POSE_UNRELIABLE")),
        "left_wave_state": str(getattr(gesture_result, "left_wave_state", "WAVE_UNRELIABLE")),
        "right_wave_state": str(getattr(gesture_result, "right_wave_state", "WAVE_UNRELIABLE")),
        "wave_state": str(getattr(gesture_result, "wave_state", "WAVE_UNRELIABLE")),
        "timestamp": float(getattr(gesture_result, "timestamp", 0.0)),
        "diagnostics": getattr(gesture_result, "diagnostics", {}) or {},
    }


def make_tts_fingerprint(pose_array: HumanPoseArray, gesture_results: dict[int, Any] | None = None) -> str:
    """Special function: Build a stable speech fingerprint from pose and gesture labels."""
    active_gestures = gesture_results or {}
    people = []
    for pose in pose_array.poses:
        gesture_payload = gesture_to_payload(active_gestures.get(int(pose.person_id)))
        people.append({
            "status": str(pose.status),
            "pose_label": str(pose.pose_label),
            "gesture": gesture_payload["gesture"],
            "active_arm": gesture_payload["active_arm"],
        })
    return json.dumps({"count": len(people), "people": people}, sort_keys=True)


def build_tts_report(
    pose_array: HumanPoseArray,
    gesture_results: dict[int, Any] | None = None,
    speak_unreliable: bool = True,
    speak_no_detection: bool = False,
) -> str:
    """Special function: Build TTS text that describes each detected human pose and gesture."""
    active_gestures = gesture_results or {}
    descriptions = []
    for pose in pose_array.poses:
        if pose.status != STATUS_DETECTED and not (pose.status == STATUS_UNRELIABLE and speak_unreliable):
            continue
        gesture_payload = gesture_to_payload(active_gestures.get(int(pose.person_id)))
        phrase = str(pose.description).rstrip(".")
        if gesture_payload["gesture"] != GESTURE_NONE:
            phrase += f" and is {gesture_payload['gesture'].replace('_', ' ')}"
            if gesture_payload["active_arm"] != ARM_UNKNOWN:
                phrase += f" with the {gesture_payload['active_arm']} arm"
        descriptions.append(f"{phrase}.")
    if descriptions:
        return " ".join(descriptions)
    if speak_no_detection:
        return "I do not detect a human pose."
    return ""


def human_count_phrase(count: int) -> str:
    """Special function: Return a short spoken phrase for a human pose count."""
    words = {
        1: "one",
        2: "two",
        3: "three",
        4: "four",
        5: "five",
    }
    return words.get(int(count), str(int(count)))


def torso_visibility_values(landmarks: list[PoseLandmark]) -> dict[str, float]:
    """Special function: Return torso visibility values for reliability diagnosis."""
    if len(landmarks) != 33:
        return {}
    return {
        MEDIAPIPE_LANDMARK_NAMES[index]: float(landmarks[index].visibility)
        for index in REQUIRED_TORSO_INDICES
    }


def normalized_landmarks_to_pixels(landmarks: list[PoseLandmark], width: int, height: int) -> dict[int, tuple[int, int]]:
    """Special function: Convert normalized visible landmarks to image pixel positions."""
    points: dict[int, tuple[int, int]] = {}
    for index, landmark in enumerate(landmarks):
        if not finite_landmark(landmark) or float(landmark.visibility) <= 0.0:
            continue
        x = int(min(max(float(landmark.x), 0.0), 1.0) * max(1, width - 1))
        y = int(min(max(float(landmark.y), 0.0), 1.0) * max(1, height - 1))
        points[index] = (x, y)
    return points


def first_visible_point(points: dict[int, tuple[int, int]]) -> tuple[int, int] | None:
    """Special function: Choose one visible point for debug image labels."""
    return points.get(0) or next(iter(points.values()), None)


def status_color(status: str) -> tuple[int, int, int]:
    """Special function: Return RGB draw color for one pose status."""
    if status == STATUS_DETECTED:
        return 0, 220, 0
    if status == STATUS_UNRELIABLE:
        return 255, 180, 0
    return 255, 0, 0


def build_legacy_skeletons(pose_array: HumanPoseArray, width: int, height: int) -> HumanSkeletons:
    """Special function: Convert 33-landmark MediaPipe poses into legacy 17-keypoint skeletons."""
    skeletons_msg = HumanSkeletons()
    skeletons_msg.stamp = pose_array.header.stamp
    for pose in pose_array.poses:
        keypoints = build_legacy_keypoints(pose.landmarks, width, height)
        bbox = bbox_from_keypoints(keypoints)
        skeleton = HumanSkeleton()
        skeleton.stamp = pose_array.header.stamp
        skeleton.skeleton_id = f"person_{pose.person_id}"
        skeleton.pose = pose.pose_label or POSE_UNKNOWN
        skeleton.confidence = min_torso_confidence(pose.landmarks)
        skeleton.x1, skeleton.y1, skeleton.x2, skeleton.y2 = bbox
        skeleton.center_u = float((bbox[0] + bbox[2]) / 2.0)
        skeleton.center_v = float((bbox[1] + bbox[3]) / 2.0)
        skeleton.keypoints = keypoints
        skeleton.links = build_legacy_links(keypoints)
        skeletons_msg.skeletons.append(skeleton)
    return skeletons_msg


def build_legacy_keypoints(landmarks: list[PoseLandmark], width: int, height: int) -> list[HumanSkeletonKeypoint]:
    """Special function: Build 17 legacy keypoints from MediaPipe landmarks for compatibility."""
    keypoints: list[HumanSkeletonKeypoint] = []
    for output_index, (name, mediapipe_index) in enumerate(LEGACY_COCO_FROM_MEDIAPIPE):
        landmark = landmarks[mediapipe_index] if mediapipe_index < len(landmarks) else None
        keypoint = HumanSkeletonKeypoint()
        keypoint.name = name
        keypoint.index = int(output_index)
        if landmark is None or not finite_landmark(landmark):
            keypoint.x = math.nan
            keypoint.y = math.nan
            keypoint.confidence = 0.0
            keypoint.visible = False
        else:
            keypoint.x = float(landmark.x) * float(width)
            keypoint.y = float(landmark.y) * float(height)
            keypoint.confidence = min(float(landmark.visibility), float(landmark.presence))
            keypoint.visible = keypoint.confidence > 0.0
        keypoints.append(keypoint)
    return keypoints


def bbox_from_keypoints(keypoints: list[HumanSkeletonKeypoint]) -> tuple[int, int, int, int]:
    """Special function: Compute a bbox around finite visible legacy keypoints."""
    visible = [keypoint for keypoint in keypoints if keypoint.visible and math.isfinite(keypoint.x) and math.isfinite(keypoint.y)]
    if not visible:
        return 0, 0, 0, 0
    xs = [float(keypoint.x) for keypoint in visible]
    ys = [float(keypoint.y) for keypoint in visible]
    return int(min(xs)), int(min(ys)), int(max(xs)), int(max(ys))


def build_legacy_links(keypoints: list[HumanSkeletonKeypoint]) -> list[HumanSkeletonLink]:
    """Special function: Build legacy skeleton links from visible 17-keypoint pairs."""
    legacy_pairs = [(5, 6), (5, 7), (7, 9), (6, 8), (8, 10), (5, 11), (6, 12), (11, 12), (11, 13), (13, 15), (12, 14), (14, 16)]
    links: list[HumanSkeletonLink] = []
    for start_index, end_index in legacy_pairs:
        start = keypoints[start_index]
        end = keypoints[end_index]
        if not start.visible or not end.visible:
            continue
        link = HumanSkeletonLink()
        link.start_name = start.name
        link.end_name = end.name
        link.start_index = int(start.index)
        link.end_index = int(end.index)
        link.confidence = min(float(start.confidence), float(end.confidence))
        links.append(link)
    return links


def min_torso_confidence(landmarks: list[PoseLandmark]) -> float:
    """Special function: Return weakest upper-body anchor confidence for pose eligibility."""
    if len(landmarks) != 33:
        return 0.0
    return min(
        min(float(landmarks[index].visibility), float(landmarks[index].presence))
        for index in REQUIRED_UPPER_BODY_INDICES
    )


def main(args: list[str] | None = None) -> None:
    """Special function: Start the MediaPipe human pose node."""
    rclpy.init(args=args)
    node = HumanSkeletonDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        if getattr(node, "landmarker", None) is not None:
            node.landmarker.close()
        if getattr(node, "gui_available", False):
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
