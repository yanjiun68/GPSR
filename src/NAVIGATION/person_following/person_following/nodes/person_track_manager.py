#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from builtin_interfaces.msg import Duration as DurationMsg
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32
from visualization_msgs.msg import Marker, MarkerArray

from putrabot_interfaces.msg import CamDetections, TargetLockDebug, TrackedPose, TrackedPoseArray
from person_following.utils.reid_debug import (
    canonical_reid_reason,
    format_reid_log_fail,
    format_reid_log_ok,
    format_reid_preview,
    should_emit_reid_log,
)
from person_following.utils.runtime_config import resolve_onnx_providers

# Optional Hungarian assignment
try:
    from scipy.optimize import linear_sum_assignment
    SCIPY_AVAILABLE = True
except Exception:
    SCIPY_AVAILABLE = False

# Optional OpenCV
try:
    import cv2
except Exception:
    cv2 = None

# Optional ONNX ReID
try:
    import onnxruntime as ort
    ONNX_AVAILABLE = True
except Exception:
    ort = None
    ONNX_AVAILABLE = False


class TrackState(str, Enum):
    TENTATIVE = 'tentative'
    CONFIRMED = 'confirmed'
    OCCLUDED = 'occluded'
    LOST = 'lost'
    DELETED = 'deleted'


class TargetLockState(str, Enum):
    NONE = 'none'
    LOCKED = 'locked'
    TEMP_OCCLUDED = 'temp_occluded'
    TEMP_LOST = 'temp_lost'
    REACQUIRE = 'reacquire'


@dataclass
class FusedMeasurement:
    stamp_sec: float
    frame_id: str
    pos_xy: np.ndarray          # [x, y] (final used measurement)
    camera_xy: np.ndarray       # [x, y] from camera
    lidar_xy: Optional[np.ndarray]
    camera_confidence: float
    cam_lidar_error: Optional[float]
    embedding: Optional[np.ndarray]
    source: str                 # camera_only | cam_lidar | lidar_only
    depth_valid_ratio: float = 0.0
    bbox_area_px: float = 0.0
    range_m: float = 0.0
    bearing_rad: float = 0.0
    center_uv: Optional[np.ndarray] = None   # [u, v]
    bbox_xyxy: Optional[Tuple[int, int, int, int]] = None
    bbox_w_px: float = 0.0
    bbox_h_px: float = 0.0
    facing01: float = 0.0
    head_rpy_deg: Optional[np.ndarray] = None  # [yaw, pitch, roll] degrees
    torso_hist: Optional[np.ndarray] = None
    upper_body_hist: Optional[np.ndarray] = None
    lower_body_hist: Optional[np.ndarray] = None
    face_embedding: Optional[np.ndarray] = None
    body_reid_confidence: float = 0.0
    torso_hist_confidence: float = 0.0
    upper_body_hist_confidence: float = 0.0
    lower_body_hist_confidence: float = 0.0
    face_embedding_confidence: float = 0.0
    detection_quality: float = 0.0
    identity_quality: float = 0.0
    continuity_only: bool = False
    image_center_vel: Optional[np.ndarray] = None
    image_center_confidence: float = 0.0
    reid_debug_reason: Optional[str] = None


@dataclass
class Track:
    track_id: int
    x: np.ndarray               # [px, py, vx, vy]
    P: np.ndarray               # 4x4 covariance
    state: TrackState
    frame_id: str
    last_update_sec: float
    created_sec: float
    hits: int = 1
    misses: int = 0
    age_frames: int = 1
    last_meas_source: str = 'camera_only'
    ema_embedding: Optional[np.ndarray] = None
    last_embedding: Optional[np.ndarray] = None
    last_match_cost: float = 0.0
    last_facing01: float = 0.0
    last_head_rpy_deg: Optional[np.ndarray] = None  # [yaw, pitch, roll] (degrees)
    imm_xs: Optional[np.ndarray] = None             # (3,4) model states [cv, turn, stop]
    imm_Ps: Optional[np.ndarray] = None             # (3,4,4) model covariances
    imm_mu: Optional[np.ndarray] = None             # (3,) model probabilities
    track_score: float = 0.2
    conf_app: float = 0.0
    conf_motion: float = 0.0
    embedding_gallery: Optional[List[np.ndarray]] = None
    torso_hist_ref: Optional[np.ndarray] = None
    upper_body_hist_ref: Optional[np.ndarray] = None
    lower_body_hist_ref: Optional[np.ndarray] = None
    face_embedding_ref: Optional[np.ndarray] = None

    def pos(self) -> np.ndarray:
        return self.x[:2].copy()


class ReIDExtractor:
    """Optional ONNX-based ReID feature extractor. If unavailable -> returns None embeddings."""

    def __init__(self, model_path: str, providers, input_w: int, input_h: int, logger, timing_cb=None):
        self.logger = logger
        self.input_w = int(input_w)
        self.input_h = int(input_h)
        self.providers = list(providers or [])
        self.timing_cb = timing_cb

        self.session = None
        self.input_name = None
        self.output_name = None

        if not model_path:
            self.logger.info('[TrackManager] ReID disabled: empty model path')
            return
        if not ONNX_AVAILABLE:
            self.logger.warn('[TrackManager] onnxruntime not available -> ReID disabled')
            return
        if cv2 is None:
            self.logger.warn('[TrackManager] OpenCV not available -> ReID disabled')
            return

        try:
            self.session = ort.InferenceSession(model_path, providers=self.providers)
            self.input_name = self.session.get_inputs()[0].name
            self.output_name = self.session.get_outputs()[0].name
            self.logger.info(f'[TrackManager] ReID ONNX loaded: {model_path} providers={self.providers}')
        except Exception as e:
            self.logger.warn(f'[TrackManager] Failed to load ReID model ({model_path}): {e}')
            self.session = None

    @property
    def enabled(self) -> bool:
        return self.session is not None

    def _extract_from_crop(self, crop: np.ndarray) -> Optional[np.ndarray]:
        if crop is None or crop.size == 0:
            return None
        if crop.shape[0] < 8 or crop.shape[1] < 8:
            return None
        try:
            rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
            resized = cv2.resize(rgb, (self.input_w, self.input_h), interpolation=cv2.INTER_LINEAR)
            x = resized.astype(np.float32) / 255.0
            x = np.transpose(x, (2, 0, 1))[None, ...]  # BCHW

            t0 = time.perf_counter()
            feat = self.session.run([self.output_name], {self.input_name: x})[0]
            if self.timing_cb is not None:
                self.timing_cb('reid', time.perf_counter() - t0)
            feat = np.asarray(feat).reshape(-1).astype(np.float32)

            n = float(np.linalg.norm(feat))
            if n < 1e-6:
                return None
            return feat / n
        except Exception as e:
            self.logger.warn(f'[TrackManager] ReID inference failed: {e}')
            return None

    def extract_from_bbox(self, bgr_img: np.ndarray, x1: int, y1: int, x2: int, y2: int) -> Optional[np.ndarray]:
        if not self.enabled or bgr_img is None or bgr_img.size == 0:
            return None
        h, w = bgr_img.shape[:2]
        x1i = max(0, min(w - 1, int(x1)))
        x2i = max(0, min(w, int(x2)))
        y1i = max(0, min(h - 1, int(y1)))
        y2i = max(0, min(h, int(y2)))
        if x2i <= x1i + 2 or y2i <= y1i + 2:
            return None
        crop = bgr_img[y1i:y2i, x1i:x2i]
        return self._extract_from_crop(crop)

    def extract(self, bgr_img: np.ndarray, center_u: float, center_v: float) -> Optional[np.ndarray]:
        if not self.enabled or bgr_img is None or bgr_img.size == 0:
            return None

        h, w = bgr_img.shape[:2]

        # Approx crop around center (since bbox not available)
        crop_w = max(24, int(w * 0.12))
        crop_h = max(48, int(h * 0.32))
        cx = int(round(center_u))
        cy = int(round(center_v))

        x0 = max(0, cx - crop_w // 2)
        x1 = min(w, cx + crop_w // 2)
        y0 = max(0, cy - int(crop_h * 0.55))
        y1 = min(h, cy + int(crop_h * 0.45))

        if (x1 - x0) < 8 or (y1 - y0) < 8:
            return None

        crop = bgr_img[y0:y1, x0:x1]
        return self._extract_from_crop(crop)


class PersonTrackManager(Node):
    def __init__(self):
        super().__init__('person_track_manager')

        def _param_bool(v) -> bool:
            if isinstance(v, bool):
                return v
            if isinstance(v, (int, float)):
                return bool(v)
            if isinstance(v, str):
                s = v.strip().lower()
                if s in ('1', 'true', 't', 'yes', 'y', 'on'):
                    return True
                if s in ('0', 'false', 'f', 'no', 'n', 'off'):
                    return False
            return bool(v)

        # Topics / frame
        self.declare_parameter('cam_detections_topic', '/cam_detections')
        self.declare_parameter('people_candidates_topic', '/people_candidates')
        self.declare_parameter('rgb_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('track_frame', 'base_footprint')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('target_facing_topic', '/person_target_facing_robot')
        self.declare_parameter('enable_target_lock_publish', True)
        self.declare_parameter('crowd_mode', True)

        # Fusion / association params
        self.declare_parameter('cam_lidar_gate_m', 0.4)
        self.declare_parameter('crowd_cam_lidar_gate_m', 0.5)
        self.declare_parameter('track_position_gate_m', 0.8)
        self.declare_parameter('allow_camera_only_updates', True)
        self.declare_parameter('allow_lidar_only_updates', True)
        self.declare_parameter('lidar_only_recovery_mode', True)
        self.declare_parameter('recovery_camera_lost_sec', 1.0)
        self.declare_parameter('allow_lidar_only_for_selected', True)
        self.declare_parameter('selected_lidar_bridge_sec', 1.0)
        self.declare_parameter('bridge_start_delay_sec', 0.2)
        self.declare_parameter('selected_track_stale_max_sec', 0.6)
        self.declare_parameter('selected_lidar_gate_m', 0.35)
        self.declare_parameter('selected_lidar_bearing_gate_deg', 30.0)
        self.declare_parameter('selected_lidar_max_speed_mps', 1.8)
        self.declare_parameter('selected_lidar_min_x_m', 0.1)
        self.declare_parameter('selected_lidar_confirm_hits', 2)
        self.declare_parameter('candidate_same_gate_m', 0.15)
        self.declare_parameter('selected_lidar_ambiguity_ratio', 1.25)
        self.declare_parameter('bridge_score_w_dist', 0.6)
        self.declare_parameter('bridge_score_w_bearing', 0.3)
        self.declare_parameter('bridge_score_w_speed', 0.1)
        self.declare_parameter('max_msg_age_sec', 0.5)
        self.declare_parameter('min_cam_confidence', 0.5)
        self.declare_parameter('appearance_gate', 0.6)
        self.declare_parameter('occluded_app_gate', 0.35)
        self.declare_parameter('association_gate_mode', 'mahalanobis')
        self.declare_parameter('maha_gate_thresh', 9.21)
        self.declare_parameter('occluded_maha_gate_thresh', 7.0)

        # Track lifecycle
        self.declare_parameter('min_hits_confirm', 1)
        self.declare_parameter('max_missed_frames', 20)
        self.declare_parameter('max_occluded_frames', 45)
        self.declare_parameter('max_lost_frames', 20)
        self.declare_parameter('confirm_score_thresh', 0.5)
        self.declare_parameter('delete_score_thresh', 0.20)
        self.declare_parameter('score_hit_gain', 0.12)
        self.declare_parameter('score_miss_penalty', 0.04)
        self.declare_parameter('target_lock_lost_timeout_sec', 1.6)
        self.declare_parameter('temp_occluded_hold_sec', 0.6)
        self.declare_parameter('selected_track_internal_stale_sec', 2.0)
        self.declare_parameter('selected_hard_hold_sec', 0.25)
        self.declare_parameter('selected_reacquire_window_sec', 1.20)
        self.declare_parameter('selected_no_switch_during_hard_hold', True)
        self.declare_parameter('switch_confirm_frames', 6)
        self.declare_parameter('reacquire_confirm_frames', 4)
        self.declare_parameter('front_occluder_range_margin_m', 0.40)
        self.declare_parameter('bbox_size_change_gate', 0.50)
        self.declare_parameter('selected_bearing_gate_deg', 35.0)
        self.declare_parameter('selected_match_min_score', 0.60)
        self.declare_parameter('selected_match_clean_update_score', 0.82)
        self.declare_parameter('selected_match_ambiguous_margin', 0.18)
        self.declare_parameter('relock_cooldown_sec', 0.8)
        self.declare_parameter('selected_center_corridor_px', 160.0)
        self.declare_parameter('selected_history_min_count', 3)
        self.declare_parameter('selected_bbox_overlap_min_iou', 0.20)
        self.declare_parameter('auto_select_nearest_on_start', True)
        self.declare_parameter('post_relock_protection_sec', 1.2)
        self.declare_parameter('post_relock_switch_margin', 0.24)
        self.declare_parameter('post_relock_min_score', 0.70)
        self.declare_parameter('crossover_enter_range_delta_m', 0.45)
        self.declare_parameter('crossover_enter_bearing_delta_deg', 10.0)
        self.declare_parameter('crossover_enter_center_delta_px', 50.0)
        self.declare_parameter('crossover_enter_bbox_ratio_delta', 0.20)
        self.declare_parameter('crossover_exit_stable_sec', 0.8)
        self.declare_parameter('crossover_confirm_frames', 8)
        self.declare_parameter('crossover_ambiguity_margin', 0.22)
        self.declare_parameter('crossover_min_relock_score', 0.80)
        self.declare_parameter('single_strong_cue_min_score', 0.82)
        self.declare_parameter('contradiction_threshold', 0.40)
        self.declare_parameter('contradictory_candidate_margin', 0.12)
        self.declare_parameter('color_hist_alpha', 0.25)
        self.declare_parameter('predicted_target_max_age_sec', 0.22)
        self.declare_parameter('predicted_target_temp_lost_max_age_sec', 0.10)
        self.declare_parameter('predicted_target_max_covariance', 0.15)
        self.declare_parameter('predicted_target_max_speed_m_s', 0.50)
        self.declare_parameter('predicted_target_max_horizon_sec', 0.25)
        self.declare_parameter('gallery_max_size', 6)
        self.declare_parameter('gallery_min_diversity_distance', 0.10)
        self.declare_parameter('stable_anchor_min_confidence', 0.80)
        self.declare_parameter('stable_anchor_max_age_sec', 8.0)
        self.declare_parameter('identity_freeze_after_relock_sec', 1.0)
        self.declare_parameter('weak_appearance_penalty', 0.18)
        self.declare_parameter('lidar_only_selected_penalty', 0.22)

        # IMM / Kalman noise params
        self.declare_parameter('process_noise_pos', 0.10)
        self.declare_parameter('process_noise_vel', 0.50)
        self.declare_parameter('meas_noise_lidar', 0.08)
        self.declare_parameter('meas_noise_camera', 0.20)
        self.declare_parameter('imm_transition_stay_prob', 0.90)
        self.declare_parameter('imm_turn_rate_rad_s', 0.90)
        self.declare_parameter('imm_stop_tau_sec', 0.60)
        self.declare_parameter('imm_q_vel_turn_scale', 1.40)
        self.declare_parameter('imm_q_vel_stop_scale', 0.30)
        self.declare_parameter('imm_init_mu_cv', 0.60)
        self.declare_parameter('imm_init_mu_turn', 0.25)
        self.declare_parameter('imm_init_mu_stop', 0.15)

        # Cost weights
        self.declare_parameter('w_pos', 0.55)
        self.declare_parameter('w_app', 0.35)
        self.declare_parameter('w_motion', 0.10)
        self.declare_parameter('crowd_w_pos', 0.35)
        self.declare_parameter('crowd_w_app', 0.55)
        self.declare_parameter('crowd_w_motion', 0.10)
        self.declare_parameter('invalid_cost', 1e6)

        # ReID params (optional)
        self.declare_parameter('reid_model_path', '')
        self.declare_parameter('reid_providers', 'CUDAExecutionProvider,CPUExecutionProvider')
        self.declare_parameter('reid_input_w', 128)
        self.declare_parameter('reid_input_h', 256)
        self.declare_parameter('reid_debug_enabled', True)
        self.declare_parameter('reid_debug_log_period_sec', 0.5)
        self.declare_parameter('reid_debug_preview_n', 4)
        self.declare_parameter('embedding_ema_alpha', 0.3)
        self.declare_parameter('embedding_gallery_size', 10)
        self.declare_parameter('embedding_update_min_conf', 0.7)
        self.declare_parameter('embedding_update_max_assoc_cost', 0.45)
        self.declare_parameter('reid_min_bbox_area_px', 2500.0)
        self.declare_parameter('reid_update_max_abs_yaw_deg', 45.0)
        self.declare_parameter('reid_update_max_abs_pitch_deg', 30.0)

        # Marker params
        self.declare_parameter('publish_debug_markers', True)
        self.declare_parameter('track_marker_scale_m', 0.20)
        self.declare_parameter('target_marker_scale_m', 0.24)
        self.declare_parameter('marker_lifetime_sec', 0.25)
        self.declare_parameter('update_period_sec', 0.02)

        # Camera measurement quality to noise mapping
        self.declare_parameter('camera_meas_noise_min', 0.18)
        self.declare_parameter('camera_meas_noise_max', 0.70)
        self.declare_parameter('depth_quality_weight', 0.55)
        self.declare_parameter('bbox_area_weight', 0.25)
        self.declare_parameter('runtime_timing_debug', False)

        # ---------------- Read params ----------------
        self.cam_topic = str(self.get_parameter('cam_detections_topic').value)
        self.lidar_topic = str(self.get_parameter('people_candidates_topic').value)
        self.rgb_topic = str(self.get_parameter('rgb_topic').value)
        self.track_frame = str(self.get_parameter('track_frame').value)
        self.target_facing_topic = str(self.get_parameter('target_facing_topic').value)
        self.enable_target_lock_publish = _param_bool(self.get_parameter('enable_target_lock_publish').value)
        self.crowd_mode = _param_bool(self.get_parameter('crowd_mode').value)

        self.cam_lidar_gate_m = float(self.get_parameter('cam_lidar_gate_m').value)
        self.crowd_cam_lidar_gate_m = float(self.get_parameter('crowd_cam_lidar_gate_m').value)
        self.track_position_gate_m = float(self.get_parameter('track_position_gate_m').value)
        self.allow_camera_only_updates = _param_bool(self.get_parameter('allow_camera_only_updates').value)
        self.allow_lidar_only_updates = _param_bool(self.get_parameter('allow_lidar_only_updates').value)
        self.lidar_only_recovery_mode = _param_bool(self.get_parameter('lidar_only_recovery_mode').value)
        self.recovery_camera_lost_sec = float(self.get_parameter('recovery_camera_lost_sec').value)
        self.allow_lidar_only_for_selected = _param_bool(self.get_parameter('allow_lidar_only_for_selected').value)
        self.selected_lidar_bridge_sec = max(0.0, float(self.get_parameter('selected_lidar_bridge_sec').value))
        self.bridge_start_delay_sec = max(0.0, float(self.get_parameter('bridge_start_delay_sec').value))
        self.selected_track_stale_max_sec = max(0.0, float(self.get_parameter('selected_track_stale_max_sec').value))
        self.selected_lidar_gate_m = max(0.05, float(self.get_parameter('selected_lidar_gate_m').value))
        self.selected_lidar_bearing_gate_deg = max(1.0, float(self.get_parameter('selected_lidar_bearing_gate_deg').value))
        self.selected_lidar_max_speed_mps = max(0.1, float(self.get_parameter('selected_lidar_max_speed_mps').value))
        self.selected_lidar_min_x_m = float(self.get_parameter('selected_lidar_min_x_m').value)
        self.selected_lidar_confirm_hits = max(1, int(self.get_parameter('selected_lidar_confirm_hits').value))
        self.candidate_same_gate_m = max(0.01, float(self.get_parameter('candidate_same_gate_m').value))
        self.selected_lidar_ambiguity_ratio = max(1.0, float(self.get_parameter('selected_lidar_ambiguity_ratio').value))
        self.bridge_score_w_dist = max(0.0, float(self.get_parameter('bridge_score_w_dist').value))
        self.bridge_score_w_bearing = max(0.0, float(self.get_parameter('bridge_score_w_bearing').value))
        self.bridge_score_w_speed = max(0.0, float(self.get_parameter('bridge_score_w_speed').value))
        self.max_msg_age_sec = float(self.get_parameter('max_msg_age_sec').value)
        self.min_cam_conf = float(self.get_parameter('min_cam_confidence').value)
        self.appearance_gate = float(self.get_parameter('appearance_gate').value)
        self.occluded_app_gate = float(self.get_parameter('occluded_app_gate').value)
        self.association_gate_mode = str(self.get_parameter('association_gate_mode').value).strip().lower()
        self.maha_gate_thresh = float(self.get_parameter('maha_gate_thresh').value)
        self.occluded_maha_gate_thresh = float(self.get_parameter('occluded_maha_gate_thresh').value)

        self.min_hits_confirm = int(self.get_parameter('min_hits_confirm').value)
        self.max_missed_frames = int(self.get_parameter('max_missed_frames').value)
        self.max_occluded_frames = int(self.get_parameter('max_occluded_frames').value)
        self.max_lost_frames = int(self.get_parameter('max_lost_frames').value)
        self.confirm_score_thresh = float(self.get_parameter('confirm_score_thresh').value)
        self.delete_score_thresh = float(self.get_parameter('delete_score_thresh').value)
        self.score_hit_gain = float(self.get_parameter('score_hit_gain').value)
        self.score_miss_penalty = float(self.get_parameter('score_miss_penalty').value)
        self.target_lock_lost_timeout_sec = float(self.get_parameter('target_lock_lost_timeout_sec').value)
        self.temp_occluded_hold_sec = max(0.0, float(self.get_parameter('temp_occluded_hold_sec').value))
        self.selected_track_internal_stale_sec = max(0.0, float(self.get_parameter('selected_track_internal_stale_sec').value))
        self.selected_hard_hold_sec = max(0.0, float(self.get_parameter('selected_hard_hold_sec').value))
        self.selected_reacquire_window_sec = max(
            self.selected_hard_hold_sec,
            float(self.get_parameter('selected_reacquire_window_sec').value),
        )
        self.selected_no_switch_during_hard_hold = _param_bool(
            self.get_parameter('selected_no_switch_during_hard_hold').value
        )
        self.switch_confirm_frames = max(1, int(self.get_parameter('switch_confirm_frames').value))
        self.reacquire_confirm_frames = max(1, int(self.get_parameter('reacquire_confirm_frames').value))
        self.front_occluder_range_margin_m = max(0.0, float(self.get_parameter('front_occluder_range_margin_m').value))
        self.bbox_size_change_gate = max(0.0, float(self.get_parameter('bbox_size_change_gate').value))
        self.selected_bearing_gate_deg = max(1.0, float(self.get_parameter('selected_bearing_gate_deg').value))
        self.selected_match_min_score = float(self.get_parameter('selected_match_min_score').value)
        self.selected_match_clean_update_score = float(self.get_parameter('selected_match_clean_update_score').value)
        self.selected_match_ambiguous_margin = max(0.0, float(self.get_parameter('selected_match_ambiguous_margin').value))
        self.relock_cooldown_sec = max(0.0, float(self.get_parameter('relock_cooldown_sec').value))
        self.selected_center_corridor_px = max(1.0, float(self.get_parameter('selected_center_corridor_px').value))
        self.selected_history_min_count = max(1, int(self.get_parameter('selected_history_min_count').value))
        self.selected_bbox_overlap_min_iou = max(0.0, float(self.get_parameter('selected_bbox_overlap_min_iou').value))
        self.auto_select_nearest_on_start = _param_bool(self.get_parameter('auto_select_nearest_on_start').value)
        self.post_relock_protection_sec = max(0.0, float(self.get_parameter('post_relock_protection_sec').value))
        self.post_relock_switch_margin = max(0.0, float(self.get_parameter('post_relock_switch_margin').value))
        self.post_relock_min_score = float(self.get_parameter('post_relock_min_score').value)
        self.crossover_enter_range_delta_m = max(0.0, float(self.get_parameter('crossover_enter_range_delta_m').value))
        self.crossover_enter_bearing_delta_deg = max(0.0, float(self.get_parameter('crossover_enter_bearing_delta_deg').value))
        self.crossover_enter_center_delta_px = max(0.0, float(self.get_parameter('crossover_enter_center_delta_px').value))
        self.crossover_enter_bbox_ratio_delta = max(0.0, float(self.get_parameter('crossover_enter_bbox_ratio_delta').value))
        self.crossover_exit_stable_sec = max(0.0, float(self.get_parameter('crossover_exit_stable_sec').value))
        self.crossover_confirm_frames = max(1, int(self.get_parameter('crossover_confirm_frames').value))
        self.crossover_ambiguity_margin = max(0.0, float(self.get_parameter('crossover_ambiguity_margin').value))
        self.crossover_min_relock_score = float(self.get_parameter('crossover_min_relock_score').value)
        self.single_strong_cue_min_score = float(self.get_parameter('single_strong_cue_min_score').value)
        self.contradiction_threshold = max(0.0, float(self.get_parameter('contradiction_threshold').value))
        self.contradictory_candidate_margin = max(0.0, float(self.get_parameter('contradictory_candidate_margin').value))
        self.color_hist_alpha = min(1.0, max(0.0, float(self.get_parameter('color_hist_alpha').value)))
        self.predicted_target_max_age_sec = max(0.0, float(self.get_parameter('predicted_target_max_age_sec').value))
        self.predicted_target_temp_lost_max_age_sec = max(0.0, float(self.get_parameter('predicted_target_temp_lost_max_age_sec').value))
        self.predicted_target_max_covariance = max(0.0, float(self.get_parameter('predicted_target_max_covariance').value))
        self.predicted_target_max_speed_m_s = max(0.0, float(self.get_parameter('predicted_target_max_speed_m_s').value))
        self.predicted_target_max_horizon_sec = max(0.0, float(self.get_parameter('predicted_target_max_horizon_sec').value))
        self.gallery_max_size = max(1, int(self.get_parameter('gallery_max_size').value))
        self.gallery_min_diversity_distance = max(0.0, float(self.get_parameter('gallery_min_diversity_distance').value))
        self.stable_anchor_min_confidence = self._clamp01(float(self.get_parameter('stable_anchor_min_confidence').value))
        self.stable_anchor_max_age_sec = max(0.0, float(self.get_parameter('stable_anchor_max_age_sec').value))
        self.identity_freeze_after_relock_sec = max(0.0, float(self.get_parameter('identity_freeze_after_relock_sec').value))
        self.weak_appearance_penalty = max(0.0, float(self.get_parameter('weak_appearance_penalty').value))
        self.lidar_only_selected_penalty = max(0.0, float(self.get_parameter('lidar_only_selected_penalty').value))

        self.q_pos = float(self.get_parameter('process_noise_pos').value)
        self.q_vel = float(self.get_parameter('process_noise_vel').value)
        self.r_lidar = float(self.get_parameter('meas_noise_lidar').value)
        self.r_camera = float(self.get_parameter('meas_noise_camera').value)
        self.imm_transition_stay_prob = float(self.get_parameter('imm_transition_stay_prob').value)
        self.imm_turn_rate_rad_s = float(self.get_parameter('imm_turn_rate_rad_s').value)
        self.imm_stop_tau_sec = float(self.get_parameter('imm_stop_tau_sec').value)
        self.imm_q_vel_turn_scale = float(self.get_parameter('imm_q_vel_turn_scale').value)
        self.imm_q_vel_stop_scale = float(self.get_parameter('imm_q_vel_stop_scale').value)
        self.imm_init_mu = np.array([
            float(self.get_parameter('imm_init_mu_cv').value),
            float(self.get_parameter('imm_init_mu_turn').value),
            float(self.get_parameter('imm_init_mu_stop').value),
        ], dtype=np.float64)

        self.w_pos = float(self.get_parameter('w_pos').value)
        self.w_app = float(self.get_parameter('w_app').value)
        self.w_motion = float(self.get_parameter('w_motion').value)
        self.crowd_w_pos = float(self.get_parameter('crowd_w_pos').value)
        self.crowd_w_app = float(self.get_parameter('crowd_w_app').value)
        self.crowd_w_motion = float(self.get_parameter('crowd_w_motion').value)
        self.invalid_cost = float(self.get_parameter('invalid_cost').value)
        self.imm_init_mu = np.maximum(self.imm_init_mu, 1e-6)
        self.imm_init_mu = self.imm_init_mu / float(np.sum(self.imm_init_mu))

        p_stay = min(0.995, max(0.70, self.imm_transition_stay_prob))
        p_switch = (1.0 - p_stay) / 2.0
        self.imm_pi = np.array([
            [p_stay, p_switch, p_switch],
            [p_switch, p_stay, p_switch],
            [p_switch, p_switch, p_stay],
        ], dtype=np.float64)

        self.emb_ema_alpha = float(self.get_parameter('embedding_ema_alpha').value)
        self.embedding_gallery_size = max(1, int(self.get_parameter('embedding_gallery_size').value))
        self.embedding_update_min_conf = float(self.get_parameter('embedding_update_min_conf').value)
        self.embedding_update_max_assoc_cost = max(0.0, float(self.get_parameter('embedding_update_max_assoc_cost').value))
        self.reid_min_bbox_area_px = max(0.0, float(self.get_parameter('reid_min_bbox_area_px').value))
        self.reid_update_max_abs_yaw_deg = max(0.0, float(self.get_parameter('reid_update_max_abs_yaw_deg').value))
        self.reid_update_max_abs_pitch_deg = max(0.0, float(self.get_parameter('reid_update_max_abs_pitch_deg').value))
        self.camera_meas_noise_min = float(self.get_parameter('camera_meas_noise_min').value)
        self.camera_meas_noise_max = float(self.get_parameter('camera_meas_noise_max').value)
        self.depth_quality_weight = float(self.get_parameter('depth_quality_weight').value)
        self.bbox_area_weight = float(self.get_parameter('bbox_area_weight').value)
        self.reid_providers_request = self.get_parameter('reid_providers').value
        self.runtime_timing_debug = _param_bool(self.get_parameter('runtime_timing_debug').value)
        self.reid_debug_enabled = _param_bool(self.get_parameter('reid_debug_enabled').value)
        self.reid_debug_log_period_sec = float(self.get_parameter('reid_debug_log_period_sec').value)
        self.reid_debug_preview_n = max(1, int(self.get_parameter('reid_debug_preview_n').value))
        self._timing_last_log_sec = {'reid': 0.0}

        wsum = self.bridge_score_w_dist + self.bridge_score_w_bearing + self.bridge_score_w_speed
        if wsum <= 1e-6:
            self.bridge_score_w_dist, self.bridge_score_w_bearing, self.bridge_score_w_speed = 0.6, 0.3, 0.1
        else:
            self.bridge_score_w_dist /= wsum
            self.bridge_score_w_bearing /= wsum
            self.bridge_score_w_speed /= wsum

        if self.crowd_mode:
            self.w_pos = self.crowd_w_pos
            self.w_app = self.crowd_w_app
            self.w_motion = self.crowd_w_motion
            self.cam_lidar_gate_m = self.crowd_cam_lidar_gate_m
            self.allow_lidar_only_updates = False

        self.publish_debug_markers = _param_bool(self.get_parameter('publish_debug_markers').value)
        self.track_marker_scale = float(self.get_parameter('track_marker_scale_m').value)
        self.target_marker_scale = float(self.get_parameter('target_marker_scale_m').value)
        self.marker_lifetime_sec = float(self.get_parameter('marker_lifetime_sec').value)
        self.update_period_sec = max(0.01, float(self.get_parameter('update_period_sec').value))

        reid_model_path = str(self.get_parameter('reid_model_path').value)
        reid_w = int(self.get_parameter('reid_input_w').value)
        reid_h = int(self.get_parameter('reid_input_h').value)
        self.reid_resolution = resolve_onnx_providers(self.reid_providers_request, ort)
        self.resolved_reid_providers = self.reid_resolution.resolved
        self.reid_runtime_enabled = bool(reid_model_path) and self.reid_resolution.enabled

        # ---------------- Utils ----------------
        self.bridge = CvBridge()
        self._log_onnx_runtime('ReID', self.reid_resolution, reid_model_path)
        self.reid = ReIDExtractor(
            reid_model_path,
            self.resolved_reid_providers,
            reid_w,
            reid_h,
            self.get_logger(),
            timing_cb=self._maybe_log_timing,
        ) if self.reid_runtime_enabled else ReIDExtractor('', [], reid_w, reid_h, self.get_logger())
        self.reid_runtime_enabled = bool(self.reid.enabled)

        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = ReliabilityPolicy.BEST_EFFORT

        # Subscriptions
        self.sub_cam = self.create_subscription(CamDetections, self.cam_topic, self.cam_cb, qos_sensor)
        self.sub_lidar = self.create_subscription(PoseArray, self.lidar_topic, self.lidar_cb, qos_sensor)
        self.sub_rgb = self.create_subscription(Image, self.rgb_topic, self.rgb_cb, qos_sensor)
        self.sub_select = self.create_subscription(Int32, '/person_target_select_id', self.select_cb, 10)

        # Publishers
        self.pub_tracks_markers = self.create_publisher(MarkerArray, '/person_tracks', 10)
        self.pub_tracks_poses = self.create_publisher(PoseArray, '/person_tracks_poses', 10)
        self.pub_tracks_state = self.create_publisher(TrackedPoseArray, '/person_tracks_state', 10)
        self.pub_target = self.create_publisher(TrackedPose, '/person_target_locked', 10)
        self.pub_target_marker = self.create_publisher(Marker, '/person_target_locked_marker', 10)
        self.pub_debug = self.create_publisher(MarkerArray, '/person_track_debug', 10)
        self.pub_target_lock_debug = self.create_publisher(TargetLockDebug, '/person_target_lock_debug', 10)
        self.pub_target_facing = self.create_publisher(Bool, self.target_facing_topic, 10)

        # Timers
        self.timer = self.create_timer(self.update_period_sec, self.timer_cb)
        self.heartbeat_timer = self.create_timer(1.0, self.heartbeat_cb)

        # Latest buffers
        self.latest_cam_msg: Optional[CamDetections] = None
        self.latest_cam_t: float = 0.0

        self.latest_lidar_msg: Optional[PoseArray] = None
        self.latest_lidar_t: float = 0.0

        self.latest_rgb_img: Optional[np.ndarray] = None
        self.latest_rgb_stamp_sec: float = 0.0

        # Tracking
        self.tracks: Dict[int, Track] = {}
        self.next_track_id = 1

        self.selected_track_id: Optional[int] = None
        self.selected_last_seen_sec: float = 0.0
        self.selected_last_clean_seen_sec: float = 0.0
        self.target_lock_state = TargetLockState.NONE
        self.target_lock_hold_until_sec: float = 0.0
        self.target_reacquire_count: int = 0
        self.target_switch_count: int = 0
        self.selected_embedding_frozen: bool = False
        self.selected_relock_cooldown_until_sec: float = 0.0
        self._selected_candidate_track_id: Optional[int] = None
        self._selected_clean_match_track_id: Optional[int] = None
        self._selected_last_decision_reason: str = ''
        self._selected_last_decision_code: int = int(TargetLockDebug.DECISION_NONE)
        self._selected_log_last_sec: Dict[str, float] = {}
        self._selected_last_best_score: float = -1.0
        self._selected_last_second_score: float = -1.0
        self._selected_last_identity_score: float = -1.0
        self._reid_debug_last_emit_sec: Dict[str, float] = {}
        self._reid_debug_last_status: Dict[str, str] = {}
        self.selected_range_hist: Deque[float] = deque(maxlen=8)
        self.selected_bearing_hist: Deque[float] = deque(maxlen=8)
        self.selected_bbox_h_hist: Deque[float] = deque(maxlen=8)
        self.selected_bbox_w_hist: Deque[float] = deque(maxlen=8)
        self.selected_center_hist: Deque[np.ndarray] = deque(maxlen=8)
        self.selected_torso_hist_ref: Optional[np.ndarray] = None
        self.selected_upper_body_hist_ref: Optional[np.ndarray] = None
        self.selected_lower_body_hist_ref: Optional[np.ndarray] = None
        self.selected_face_embedding_ref: Optional[np.ndarray] = None
        self.selected_body_embedding_ref: Optional[np.ndarray] = None
        self.selected_identity_gallery: List[np.ndarray] = []
        self.selected_stable_anchor_embedding: Optional[np.ndarray] = None
        self.selected_stable_anchor_set_sec: float = 0.0
        self.post_relock_protection_until_sec: float = 0.0
        self.crossover_active: bool = False
        self.crossover_stable_since_sec: float = 0.0

        # Debug state
        self.last_fused_measurements: List[FusedMeasurement] = []
        self.last_assignments: List[Tuple[int, int, float]] = []
        self._bridge_last_lp: Optional[np.ndarray] = None
        self._bridge_hits: int = 0
        self._bridge_log_last_sec: Dict[str, float] = {}

        # Timing for dt
        self._last_tick_sec = self._now_sec()

        self.get_logger().info('[TrackManager] Started')
        self.get_logger().info(f'  cam_topic   : {self.cam_topic}')
        self.get_logger().info(f'  lidar_topic : {self.lidar_topic}')
        self.get_logger().info(f'  rgb_topic   : {self.rgb_topic}')
        self.get_logger().info(f'  frame       : {self.track_frame}')
        self.get_logger().info(f'  facing_topic: {self.target_facing_topic}')
        self.get_logger().info(f'  scipy_hungarian={SCIPY_AVAILABLE}, reid={self.reid.enabled}')
        self.get_logger().info(
            f'[TrackManager] Runtime summary: ReID={"enabled" if self.reid_runtime_enabled else "disabled"} '
            f'on {self.resolved_reid_providers if self.resolved_reid_providers else "n/a"}'
        )
        self.get_logger().info(
            f'[TrackManager] ReID debug enabled={self.reid_debug_enabled} '
            f'log_period={self.reid_debug_log_period_sec:.2f}s preview_n={self.reid_debug_preview_n}'
        )

    # ---------------------------- callbacks ----------------------------
    def cam_cb(self, msg: CamDetections):
        self.latest_cam_msg = msg
        self.latest_cam_t = self._time_msg_to_sec(msg.stamp)

    def lidar_cb(self, msg: PoseArray):
        self.latest_lidar_msg = msg
        self.latest_lidar_t = self._stamp_to_sec(msg.header.stamp)

    def rgb_cb(self, msg: Image):
        try:
            self.latest_rgb_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_rgb_stamp_sec = self._stamp_to_sec(msg.header.stamp)
        except Exception as e:
            self.get_logger().warn(f'[TrackManager] RGB cv_bridge failed: {e}')

    def select_cb(self, msg: Int32):
        tid = int(msg.data)
        if tid <= 0:
            self.selected_track_id = None
            self._reset_selected_lock_state()
            self.get_logger().info('[TrackManager] Cleared selected target')
            return
        self.selected_track_id = tid
        self.selected_last_seen_sec = self._now_sec()
        self.selected_last_clean_seen_sec = self.selected_last_seen_sec
        self.target_lock_state = TargetLockState.LOCKED
        self.target_lock_hold_until_sec = 0.0
        self.target_reacquire_count = 0
        self.target_switch_count = 0
        self.selected_relock_cooldown_until_sec = 0.0
        self._clear_selected_histories()
        self.get_logger().info(f'[TrackManager] Selected track_id={tid}')

    def heartbeat_cb(self):
        n_conf = sum(1 for t in self.tracks.values() if t.state == TrackState.CONFIRMED)
        n_tent = sum(1 for t in self.tracks.values() if t.state == TrackState.TENTATIVE)
        n_occ = sum(1 for t in self.tracks.values() if t.state == TrackState.OCCLUDED)
        n_lost = sum(1 for t in self.tracks.values() if t.state == TrackState.LOST)
        sel_mode = 'n/a'
        if self.selected_track_id is not None and self.selected_track_id in self.tracks:
            mu = self.tracks[self.selected_track_id].imm_mu
            if mu is not None and mu.size == 3:
                sel_mode = f'cv={mu[0]:.2f},turn={mu[1]:.2f},stop={mu[2]:.2f}'
        self.get_logger().info(
            f'heartbeat: tracks={len(self.tracks)} (conf={n_conf}, tent={n_tent}, occ={n_occ}, lost={n_lost}), '
            f'fused={len(self.last_fused_measurements)}, selected={self.selected_track_id}, mu={sel_mode}'
        )

    def _log_onnx_runtime(self, label: str, resolution, model_path: str):
        if not model_path:
            self.get_logger().info(f'[TrackManager] {label} requested={resolution.requested} resolved=disabled reason="model path empty"')
            return
        self.get_logger().info(
            f'[TrackManager] {label} requested={resolution.requested} '
            f'available={resolution.available} '
            f'resolved={resolution.resolved} '
            f'reason="{resolution.reason}" model="{model_path}"'
        )

    def _maybe_log_timing(self, key: str, elapsed_sec: float):
        if not self.runtime_timing_debug:
            return
        now = self._now_sec()
        last = self._timing_last_log_sec.get(key, 0.0)
        if (now - last) < 2.0:
            return
        self._timing_last_log_sec[key] = now
        self.get_logger().info(f'[TrackManager][timing] {key}={elapsed_sec * 1000.0:.1f}ms')

    def timer_cb(self):
        now = self._now_sec()
        dt = max(1e-3, now - self._last_tick_sec)
        self._last_tick_sec = now

        fused = self._build_fused_measurements(now)
        self.last_fused_measurements = fused
        self._update_crossover_state(now, fused)

        # Predict all tracks
        for tr in list(self.tracks.values()):
            if tr.state == TrackState.DELETED:
                continue
            self._predict_track(tr, dt)
            tr.age_frames += 1

        assignments, unmatched_track_ids, unmatched_meas_idxs = self._associate(fused)
        self.last_assignments = assignments

        # Update matched tracks
        for tid, midx, total_cost in assignments:
            tr = self.tracks.get(tid)
            if tr is None:
                continue
            meas = fused[midx]

            allow_clean_selected = (tid == self._selected_clean_match_track_id) and (not self.selected_embedding_frozen)
            self._update_track(
                tr,
                meas,
                assoc_cost=total_cost,
                allow_appearance_update=(tid != self.selected_track_id) or allow_clean_selected,
                allow_history_update=(tid != self.selected_track_id) or allow_clean_selected,
            )
            tr.last_match_cost = float(total_cost)
            tr.misses = 0
            tr.hits += 1
            tr.last_update_sec = now
            tr.last_meas_source = meas.source
            tr.track_score = min(1.0, tr.track_score + self.score_hit_gain * max(0.25, meas.camera_confidence))
            tr.conf_motion = max(0.0, 1.0 - min(1.0, float(total_cost)))

            if tr.state == TrackState.TENTATIVE and (tr.hits >= self.min_hits_confirm or tr.track_score >= self.confirm_score_thresh):
                tr.state = TrackState.CONFIRMED
            elif tr.state in (TrackState.LOST, TrackState.OCCLUDED):
                tr.state = TrackState.CONFIRMED

        # Mark unmatched tracks
        for tid in unmatched_track_ids:
            tr = self.tracks.get(tid)
            if tr is None:
                continue
            tr.misses += 1
            tr.track_score = max(0.0, tr.track_score - self.score_miss_penalty)
            if tr.state == TrackState.TENTATIVE:
                tr.state = TrackState.LOST
            elif tr.state == TrackState.CONFIRMED:
                tr.state = TrackState.OCCLUDED
            elif tr.state == TrackState.OCCLUDED and tr.misses > self.max_occluded_frames:
                tr.state = TrackState.LOST

            if tr.state == TrackState.LOST and tr.misses > self.max_lost_frames:
                tr.state = TrackState.DELETED
            if tr.misses > self.max_missed_frames and tr.track_score <= self.delete_score_thresh:
                tr.state = TrackState.DELETED

        # Spawn new tracks from unmatched measurements
        for midx in unmatched_meas_idxs:
            meas = fused[midx]
            if meas.source != 'lidar_only' and meas.camera_confidence < self.min_cam_conf:
                continue
            if meas.source == 'camera_only' and not self.allow_camera_only_updates:
                continue
            if meas.source == 'lidar_only':
                if self.crowd_mode:
                    continue
                if not self.allow_lidar_only_updates:
                    continue
                if self.lidar_only_recovery_mode and (now - self.latest_cam_t) <= self.recovery_camera_lost_sec:
                    continue
            if meas.source != 'lidar_only' and meas.depth_valid_ratio < 0.15:
                continue
            self._spawn_track(meas, now)

        # Delete tracks
        to_del = [tid for tid, tr in self.tracks.items() if tr.state == TrackState.DELETED]
        for tid in to_del:
            del self.tracks[tid]
            if self.selected_track_id == tid:
                self.selected_track_id = None
                self._reset_selected_lock_state()

        self._auto_select_if_needed(now)
        self._publish_outputs(now)

    # ---------------------- measurement fusion ----------------------
    def _build_fused_measurements(self, now_sec: float) -> List[FusedMeasurement]:
        cam_msg = self.latest_cam_msg
        lidar_msg = self.latest_lidar_msg

        cam_is_fresh = False
        if cam_msg is not None:
            cam_age = now_sec - self.latest_cam_t
            cam_is_fresh = cam_age <= self.max_msg_age_sec

        # LiDAR points
        lidar_pts: List[np.ndarray] = []
        lidar_frame = self.track_frame
        lidar_is_fresh = False
        if lidar_msg is not None:
            lidar_age = now_sec - self.latest_lidar_t
            lidar_is_fresh = lidar_age <= self.max_msg_age_sec
            if lidar_is_fresh and lidar_msg.header.frame_id == self.track_frame:
                lidar_frame = lidar_msg.header.frame_id
                for p in lidar_msg.poses:
                    lidar_pts.append(np.array([float(p.position.x), float(p.position.y)], dtype=np.float64))

        fused: List[FusedMeasurement] = []
        matched_lidar_idxs = set()

        if not cam_is_fresh:
            camera_stale = max(0.0, now_sec - self.latest_cam_t) if self.latest_cam_t > 0.0 else 1e9
            selected_bridge = self._build_selected_lidar_bridge_measurement(
                now_sec=now_sec,
                lidar_pts=lidar_pts,
                lidar_frame=lidar_frame,
                camera_stale=camera_stale,
            )
            if selected_bridge is not None:
                fused.append(selected_bridge)
            can_use_lidar_only = self.allow_lidar_only_updates
            if self.lidar_only_recovery_mode:
                can_use_lidar_only = can_use_lidar_only and ((now_sec - self.latest_cam_t) > self.recovery_camera_lost_sec)
            if can_use_lidar_only and lidar_is_fresh and lidar_pts and (not self.crowd_mode):
                for lp in lidar_pts:
                    fused.append(FusedMeasurement(
                        stamp_sec=self.latest_lidar_t,
                        frame_id=lidar_frame,
                        pos_xy=lp.copy(),
                        camera_xy=lp.copy(),
                        lidar_xy=lp.copy(),
                    camera_confidence=0.0,
                    cam_lidar_error=None,
                    embedding=None,
                    source='lidar_only',
                    range_m=float(np.linalg.norm(lp)),
                    bearing_rad=math.atan2(float(lp[1]), float(lp[0]))
                    ))
            return fused

        cam_stamp_sec = self._time_msg_to_sec(cam_msg.stamp)
        candidate_dets = []

        for det in cam_msg.detections:
            conf = float(det.confidence)
            if conf < self.min_cam_conf:
                continue

            cam_xy = np.array([float(det.x), float(det.y)], dtype=np.float64)
            if not np.all(np.isfinite(cam_xy)):
                continue
            depth_valid_ratio = float(getattr(det, 'depth_valid_ratio', 0.0))
            bbox_w = max(0.0, float(getattr(det, 'x2', 0) - getattr(det, 'x1', 0)))
            bbox_h = max(0.0, float(getattr(det, 'y2', 0) - getattr(det, 'y1', 0)))
            bbox_area_px = bbox_w * bbox_h

            facing01 = float(getattr(det, 'facing01', 0.0))
            head_rpy = None
            if all(hasattr(det, f) for f in ('head_yaw_deg', 'head_pitch_deg', 'head_roll_deg')):
                head_rpy = np.array(
                    [float(det.head_yaw_deg), float(det.head_pitch_deg), float(det.head_roll_deg)],
                    dtype=np.float64,
                )
            elif hasattr(det, 'embedding') and len(det.embedding) >= 7:
                try:
                    head_rpy = np.array([float(det.embedding[3]), float(det.embedding[4]), float(det.embedding[5])], dtype=np.float64)
                    facing01 = float(det.embedding[6])
                except Exception:
                    facing01 = 0.0
                    head_rpy = None

            center_u = float(getattr(det, 'center_u', 0.0)) if hasattr(det, 'center_u') else None
            center_v = float(getattr(det, 'center_v', 0.0)) if hasattr(det, 'center_v') else None
            if (center_u is None or center_v is None) and hasattr(det, 'embedding') and len(det.embedding) >= 2:
                center_u = float(det.embedding[0])
                center_v = float(det.embedding[1])
            bbox_xyxy = (
                int(getattr(det, 'x1', 0)),
                int(getattr(det, 'y1', 0)),
                int(getattr(det, 'x2', 0)),
                int(getattr(det, 'y2', 0)),
            )
            center_uv = None
            if center_u is not None and center_v is not None:
                center_uv = np.array([center_u, center_v], dtype=np.float64)
            meas_range = float(np.linalg.norm(cam_xy))
            meas_bearing = math.atan2(float(cam_xy[1]), float(cam_xy[0]))

            emb = self._det_array(det, 'body_reid_embedding')
            reid_debug_reason = None
            if emb is None:
                emb, reid_debug_reason = self._extract_reid_embedding(center_u, center_v, bbox_xyxy)
            torso_hist = self._det_array(det, 'torso_hist')
            upper_body_hist = self._det_array(det, 'upper_body_hist')
            lower_body_hist = self._det_array(det, 'lower_body_hist')
            face_embedding = self._det_array(det, 'face_embedding')
            body_reid_conf = float(getattr(det, 'body_reid_confidence', 1.0 if emb is not None else 0.0))
            torso_hist_conf = float(getattr(det, 'torso_hist_confidence', 0.0))
            upper_body_hist_conf = float(getattr(det, 'upper_body_hist_confidence', 0.0))
            lower_body_hist_conf = float(getattr(det, 'lower_body_hist_confidence', 0.0))
            face_embedding_conf = float(getattr(det, 'face_embedding_confidence', 0.0))
            detection_quality = float(getattr(det, 'detection_quality', conf))
            identity_quality = float(getattr(det, 'identity_quality', body_reid_conf))
            continuity_only = bool(getattr(det, 'continuity_only', False))
            image_center_vel = None
            if hasattr(det, 'image_center_vel_u') and hasattr(det, 'image_center_vel_v'):
                image_center_vel = np.array(
                    [float(det.image_center_vel_u), float(det.image_center_vel_v)],
                    dtype=np.float64,
                )
            image_center_conf = float(getattr(det, 'image_center_confidence', 0.0))
            candidate_dets.append({
                'cam_xy': cam_xy,
                'conf': conf,
                'depth_valid_ratio': depth_valid_ratio,
                'bbox_area_px': bbox_area_px,
                'range_m': meas_range,
                'bearing_rad': meas_bearing,
                'center_uv': center_uv,
                'bbox_xyxy': bbox_xyxy,
                'bbox_w_px': bbox_w,
                'bbox_h_px': bbox_h,
                'facing01': float(facing01),
                'head_rpy_deg': head_rpy,
                'torso_hist': torso_hist,
                'upper_body_hist': upper_body_hist,
                'lower_body_hist': lower_body_hist,
                'face_embedding': face_embedding,
                'body_reid_confidence': body_reid_conf,
                'torso_hist_confidence': torso_hist_conf,
                'upper_body_hist_confidence': upper_body_hist_conf,
                'lower_body_hist_confidence': lower_body_hist_conf,
                'face_embedding_confidence': face_embedding_conf,
                'detection_quality': detection_quality,
                'identity_quality': identity_quality,
                'continuity_only': continuity_only,
                'image_center_vel': image_center_vel,
                'image_center_confidence': image_center_conf,
                'embedding': emb,
                'reid_debug_reason': reid_debug_reason,
            })

        matched_cam_to_lidar: Dict[int, int] = {}
        if candidate_dets and lidar_pts:
            if SCIPY_AVAILABLE:
                cost = np.full((len(candidate_dets), len(lidar_pts)), self.invalid_cost, dtype=np.float64)
                for cidx, det in enumerate(candidate_dets):
                    for lidx, lp in enumerate(lidar_pts):
                        d = float(np.linalg.norm(lp - det['cam_xy']))
                        if d <= self.cam_lidar_gate_m:
                            cost[cidx, lidx] = d
                rows, cols = linear_sum_assignment(cost)
                for row, col in zip(rows.tolist(), cols.tolist()):
                    if cost[row, col] >= self.invalid_cost:
                        continue
                    matched_cam_to_lidar[row] = col
                    matched_lidar_idxs.add(col)
            else:
                pairs = []
                for cidx, det in enumerate(candidate_dets):
                    for lidx, lp in enumerate(lidar_pts):
                        d = float(np.linalg.norm(lp - det['cam_xy']))
                        if d <= self.cam_lidar_gate_m:
                            pairs.append((d, cidx, lidx))
                pairs.sort(key=lambda item: (item[0], item[1], item[2]))
                used_cam = set()
                for _, cidx, lidx in pairs:
                    if cidx in used_cam or lidx in matched_lidar_idxs:
                        continue
                    used_cam.add(cidx)
                    matched_cam_to_lidar[cidx] = lidx
                    matched_lidar_idxs.add(lidx)

        for cidx, det in enumerate(candidate_dets):
            best_i = matched_cam_to_lidar.get(cidx)
            if best_i is not None:
                lp = lidar_pts[best_i]
                best_d = float(np.linalg.norm(lp - det['cam_xy']))
                fused.append(FusedMeasurement(
                    stamp_sec=cam_stamp_sec,
                    frame_id=lidar_frame,
                    pos_xy=lp.copy(),         # fused uses LiDAR position
                    camera_xy=det['cam_xy'].copy(),
                    lidar_xy=lp.copy(),
                    camera_confidence=det['conf'],
                    cam_lidar_error=float(best_d),
                    embedding=det['embedding'],
                    source='cam_lidar',
                    depth_valid_ratio=det['depth_valid_ratio'],
                    bbox_area_px=det['bbox_area_px'],
                    range_m=det['range_m'],
                    bearing_rad=det['bearing_rad'],
                    center_uv=det['center_uv'],
                    bbox_xyxy=det['bbox_xyxy'],
                    bbox_w_px=det['bbox_w_px'],
                    bbox_h_px=det['bbox_h_px'],
                    facing01=det['facing01'],
                    head_rpy_deg=det['head_rpy_deg'],
                    torso_hist=det['torso_hist'],
                    upper_body_hist=det['upper_body_hist'],
                    lower_body_hist=det['lower_body_hist'],
                    face_embedding=det['face_embedding'],
                    body_reid_confidence=det['body_reid_confidence'],
                    torso_hist_confidence=det['torso_hist_confidence'],
                    upper_body_hist_confidence=det['upper_body_hist_confidence'],
                    lower_body_hist_confidence=det['lower_body_hist_confidence'],
                    face_embedding_confidence=det['face_embedding_confidence'],
                    detection_quality=det['detection_quality'],
                    identity_quality=det['identity_quality'],
                    continuity_only=det['continuity_only'],
                    image_center_vel=det['image_center_vel'],
                    image_center_confidence=det['image_center_confidence'],
                    reid_debug_reason=det['reid_debug_reason'],
                ))
            elif self.allow_camera_only_updates:
                fused.append(FusedMeasurement(
                    stamp_sec=cam_stamp_sec,
                    frame_id=self.track_frame,
                    pos_xy=det['cam_xy'].copy(),      # camera-only uses camera position
                    camera_xy=det['cam_xy'].copy(),
                    lidar_xy=None,
                    camera_confidence=det['conf'],
                    cam_lidar_error=None,
                    embedding=det['embedding'],
                    source='camera_only',
                    depth_valid_ratio=det['depth_valid_ratio'],
                    bbox_area_px=det['bbox_area_px'],
                    range_m=det['range_m'],
                    bearing_rad=det['bearing_rad'],
                    center_uv=det['center_uv'],
                    bbox_xyxy=det['bbox_xyxy'],
                    bbox_w_px=det['bbox_w_px'],
                    bbox_h_px=det['bbox_h_px'],
                    facing01=det['facing01'],
                    head_rpy_deg=det['head_rpy_deg'],
                    torso_hist=det['torso_hist'],
                    upper_body_hist=det['upper_body_hist'],
                    lower_body_hist=det['lower_body_hist'],
                    face_embedding=det['face_embedding'],
                    body_reid_confidence=det['body_reid_confidence'],
                    torso_hist_confidence=det['torso_hist_confidence'],
                    upper_body_hist_confidence=det['upper_body_hist_confidence'],
                    lower_body_hist_confidence=det['lower_body_hist_confidence'],
                    face_embedding_confidence=det['face_embedding_confidence'],
                    detection_quality=det['detection_quality'],
                    identity_quality=det['identity_quality'],
                    continuity_only=det['continuity_only'],
                    image_center_vel=det['image_center_vel'],
                    image_center_confidence=det['image_center_confidence'],
                    reid_debug_reason=det['reid_debug_reason'],
                ))

        can_use_lidar_only = self.allow_lidar_only_updates and (not self.crowd_mode)
        if self.lidar_only_recovery_mode:
            can_use_lidar_only = can_use_lidar_only and ((now_sec - self.latest_cam_t) > self.recovery_camera_lost_sec)
        if can_use_lidar_only:
            for i, lp in enumerate(lidar_pts):
                if i in matched_lidar_idxs:
                    continue
                fused.append(FusedMeasurement(
                    stamp_sec=self.latest_lidar_t,
                    frame_id=lidar_frame,
                    pos_xy=lp.copy(),
                    camera_xy=lp.copy(),
                    lidar_xy=lp.copy(),
                    camera_confidence=0.0,
                    cam_lidar_error=None,
                    embedding=None,
                    source='lidar_only',
                    range_m=float(np.linalg.norm(lp)),
                    bearing_rad=math.atan2(float(lp[1]), float(lp[0]))
                ))

        return fused

    def _tracker_reid_log_failure(self, track_id: int, reason: Optional[str], now_sec: float):
        if not self.reid_debug_enabled:
            return
        key = f'tracker:{int(track_id)}:failure'
        status = canonical_reid_reason(reason)
        if not should_emit_reid_log(
            self._reid_debug_last_emit_sec,
            self._reid_debug_last_status,
            key,
            status,
            now_sec,
            self.reid_debug_log_period_sec,
        ):
            return
        self.get_logger().warn(format_reid_log_fail('tracker', track_id, status))

    def _tracker_reid_log_success(self, track_id: int, embedding: np.ndarray, now_sec: float):
        if not self.reid_debug_enabled:
            return
        key = f'tracker:{int(track_id)}:success'
        if not should_emit_reid_log(
            self._reid_debug_last_emit_sec,
            self._reid_debug_last_status,
            key,
            'ok',
            now_sec,
            self.reid_debug_log_period_sec,
        ):
            return
        norm = float(np.linalg.norm(embedding))
        preview_n = min(self.reid_debug_preview_n, int(np.asarray(embedding).reshape(-1).size))
        self.get_logger().info(
            format_reid_log_ok(
                'tracker',
                track_id,
                [
                    ('norm', f'{norm:.3f}'),
                    (f'emb[:{preview_n}]', format_reid_preview(embedding, preview_n)),
                ],
            )
        )

    def _extract_reid_embedding(
        self,
        center_u: Optional[float],
        center_v: Optional[float],
        bbox_xyxy: Optional[Tuple[int, int, int, int]] = None
    ) -> Tuple[Optional[np.ndarray], Optional[str]]:
        if self.latest_rgb_img is None:
            return None, 'rgb_unavailable'
        # freshness gate: RGB timestamp close to camera detections timestamp
        if abs(self.latest_rgb_stamp_sec - self.latest_cam_t) > 0.20:
            return None, 'rgb_stale'
        if bbox_xyxy is not None:
            x1, y1, x2, y2 = bbox_xyxy
            if x2 <= x1 or y2 <= y1:
                return None, 'invalid_bbox'
            if (x2 - x1) < 8 or (y2 - y1) < 8:
                return None, 'crop_too_small'
            if x2 > x1 + 2 and y2 > y1 + 2:
                emb = self.reid.extract_from_bbox(self.latest_rgb_img, x1, y1, x2, y2)
                if emb is not None:
                    return emb, None
        if center_u is None or center_v is None:
            return None, 'extractor_returned_none'
        emb = self.reid.extract(self.latest_rgb_img, center_u, center_v)
        return emb, None if emb is not None else 'extractor_returned_none'

    @staticmethod
    def _det_array(det, field_name: str) -> Optional[np.ndarray]:
        if not hasattr(det, field_name):
            return None
        raw = getattr(det, field_name)
        if raw is None or len(raw) == 0:
            return None
        try:
            arr = np.asarray(raw, dtype=np.float32).reshape(-1)
        except Exception:
            return None
        if arr.size == 0 or not np.all(np.isfinite(arr)):
            return None
        return arr

    def _build_selected_lidar_bridge_measurement(
        self,
        now_sec: float,
        lidar_pts: List[np.ndarray],
        lidar_frame: str,
        camera_stale: float,
    ) -> Optional[FusedMeasurement]:
        if not self.crowd_mode or (not self.allow_lidar_only_for_selected):
            self._bridge_last_lp = None
            self._bridge_hits = 0
            return None
        if self.selected_track_id is None:
            self._bridge_last_lp = None
            self._bridge_hits = 0
            return None
        tr = self.tracks.get(self.selected_track_id)
        if tr is None or tr.state not in (TrackState.CONFIRMED, TrackState.OCCLUDED):
            self._bridge_last_lp = None
            self._bridge_hits = 0
            return None
        if not (self.bridge_start_delay_sec <= camera_stale <= self.selected_lidar_bridge_sec):
            self._bridge_log_throttled('camera_window_fail', f'camera_stale={camera_stale:.2f}s not in bridge window')
            self._bridge_last_lp = None
            self._bridge_hits = 0
            return None

        track_stale = max(0.0, now_sec - tr.last_update_sec)
        if track_stale > self.selected_track_stale_max_sec:
            self._bridge_log_throttled('track_stale', f'track_stale={track_stale:.2f}s > {self.selected_track_stale_max_sec:.2f}s')
            self._bridge_last_lp = None
            self._bridge_hits = 0
            return None

        pred_xy = tr.x[:2].copy()
        pred_vel = tr.x[2:4].copy()
        pred_speed = float(np.linalg.norm(pred_vel))
        dt = max(1e-2, track_stale)

        bearing_gate_rad = math.radians(self.selected_lidar_bearing_gate_deg)
        pred_bearing = math.atan2(float(pred_xy[1]), float(pred_xy[0]))
        candidates = []

        for lp in lidar_pts:
            if float(lp[0]) <= self.selected_lidar_min_x_m:
                continue
            dist = float(np.linalg.norm(lp - pred_xy))
            if dist > self.selected_lidar_gate_m:
                continue
            cand_bearing = math.atan2(float(lp[1]), float(lp[0]))
            bdiff = abs(self._angle_diff(cand_bearing, pred_bearing))
            if bdiff > bearing_gate_rad:
                continue
            jump_speed = float(np.linalg.norm(lp - pred_xy) / dt)
            if jump_speed > self.selected_lidar_max_speed_mps:
                continue

            dist_norm = dist / max(self.selected_lidar_gate_m, 1e-3)
            bearing_norm = bdiff / max(bearing_gate_rad, 1e-6)
            speed_err = max(0.0, jump_speed - pred_speed)
            speed_norm = min(1.0, speed_err / max(self.selected_lidar_max_speed_mps, 1e-3))
            score = (
                self.bridge_score_w_dist * dist_norm
                + self.bridge_score_w_bearing * bearing_norm
                + self.bridge_score_w_speed * speed_norm
            )
            candidates.append((score, lp, dist))

        if not candidates:
            self._bridge_log_throttled('hard_gate_fail', 'no lidar candidate passed hard gates')
            self._bridge_last_lp = None
            self._bridge_hits = 0
            return None

        candidates.sort(key=lambda x: x[0])
        best_score, best_lp, _ = candidates[0]
        if len(candidates) >= 2:
            second_score = candidates[1][0]
            if second_score < (self.selected_lidar_ambiguity_ratio * max(best_score, 1e-6)):
                self._bridge_log_throttled('ambiguous_choice', f'ambiguous bridge choice best={best_score:.3f} second={second_score:.3f}')
                self._bridge_last_lp = None
                self._bridge_hits = 0
                return None

        if self._bridge_last_lp is None:
            self._bridge_last_lp = best_lp.copy()
            self._bridge_hits = 1
            self._bridge_log_throttled('debounce_pending', f'bridge candidate pending (hits={self._bridge_hits}/{self.selected_lidar_confirm_hits})')
            return None

        if float(np.linalg.norm(best_lp - self._bridge_last_lp)) > self.candidate_same_gate_m:
            self._bridge_last_lp = best_lp.copy()
            self._bridge_hits = 1
            self._bridge_log_throttled('continuity_reset', 'bridge candidate changed, resetting debounce')
            return None

        self._bridge_hits += 1
        self._bridge_last_lp = best_lp.copy()
        if self._bridge_hits < self.selected_lidar_confirm_hits:
            self._bridge_log_throttled('debounce_pending', f'bridge candidate pending (hits={self._bridge_hits}/{self.selected_lidar_confirm_hits})')
            return None

        self._bridge_log_throttled('bridge_accept', f'selected lidar bridge accepted (score={best_score:.3f})')
        return FusedMeasurement(
            stamp_sec=self.latest_lidar_t,
            frame_id=lidar_frame,
            pos_xy=best_lp.copy(),
            camera_xy=best_lp.copy(),
            lidar_xy=best_lp.copy(),
            camera_confidence=0.0,
            cam_lidar_error=None,
            embedding=None,
            source='lidar_only',
            range_m=float(np.linalg.norm(best_lp)),
            bearing_rad=math.atan2(float(best_lp[1]), float(best_lp[0]))
        )

    def _bridge_log_throttled(self, key: str, msg: str):
        now = self._now_sec()
        last = self._bridge_log_last_sec.get(key, 0.0)
        if (now - last) < 0.6:
            return
        self._bridge_log_last_sec[key] = now
        self.get_logger().info(f'[TrackManager][bridge] {key}: {msg}')

    # ---------------------- IMM tracking ----------------------
    def _spawn_track(self, meas: FusedMeasurement, now_sec: float):
        x = np.array([meas.pos_xy[0], meas.pos_xy[1], 0.0, 0.0], dtype=np.float64)
        P = np.diag([0.25, 0.25, 1.0, 1.0]).astype(np.float64)

        tid = self.next_track_id
        self.next_track_id += 1

        tr = Track(
            track_id=tid,
            x=x,
            P=P,
            state=TrackState.TENTATIVE,
            frame_id=self.track_frame,
            last_update_sec=now_sec,
            created_sec=now_sec,
            hits=1,
            misses=0,
            age_frames=1,
            last_meas_source=meas.source,
            track_score=0.2,
            embedding_gallery=[]
        )

        if meas.embedding is not None:
            tr.last_embedding = meas.embedding.copy()
            tr.ema_embedding = meas.embedding.copy()
            tr.embedding_gallery.append(meas.embedding.copy())

        self._init_track_imm(tr)
        self.tracks[tid] = tr

    def _init_track_imm(self, tr: Track):
        tr.imm_xs = np.repeat(tr.x.reshape(1, 4), 3, axis=0).astype(np.float64)
        tr.imm_Ps = np.repeat(tr.P.reshape(1, 4, 4), 3, axis=0).astype(np.float64)
        tr.imm_mu = self.imm_init_mu.copy()

    def _model_F_Q(self, model_idx: int, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        dt = max(1e-3, float(dt))
        F = np.array([
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ], dtype=np.float64)

        qvel_scale = 1.0
        if model_idx == 1:  # turning model
            wd = self.imm_turn_rate_rad_s * dt
            c = math.cos(wd)
            s = math.sin(wd)
            F[2, 2] = c
            F[2, 3] = -s
            F[3, 2] = s
            F[3, 3] = c
            qvel_scale = self.imm_q_vel_turn_scale
        elif model_idx == 2:  # slow/stop model
            tau = max(0.05, self.imm_stop_tau_sec)
            alpha = math.exp(-dt / tau)
            F[2, 2] = alpha
            F[3, 3] = alpha
            qvel_scale = self.imm_q_vel_stop_scale

        qpos2 = self.q_pos ** 2
        qvel2 = (self.q_vel * qvel_scale) ** 2
        Q = np.diag([qpos2 * dt, qpos2 * dt, qvel2 * dt, qvel2 * dt]).astype(np.float64)
        return F, Q

    @staticmethod
    def _merge_models(xs: np.ndarray, Ps: np.ndarray, mu: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        x = np.sum(mu[:, None] * xs, axis=0)
        P = np.zeros((4, 4), dtype=np.float64)
        for i in range(xs.shape[0]):
            dx = (xs[i] - x).reshape(4, 1)
            P = P + mu[i] * (Ps[i] + dx @ dx.T)
        return x, P

    def _predict_track(self, tr: Track, dt: float):
        if tr.imm_xs is None or tr.imm_Ps is None or tr.imm_mu is None:
            self._init_track_imm(tr)

        xs = tr.imm_xs
        Ps = tr.imm_Ps
        mu_prev = np.maximum(tr.imm_mu, 1e-12)
        mu_prev = mu_prev / float(np.sum(mu_prev))

        cbar = np.maximum(mu_prev @ self.imm_pi, 1e-12)
        mix = np.zeros((3, 3), dtype=np.float64)
        for i in range(3):
            for j in range(3):
                mix[i, j] = (self.imm_pi[i, j] * mu_prev[i]) / cbar[j]

        mixed_xs = np.zeros((3, 4), dtype=np.float64)
        mixed_Ps = np.zeros((3, 4, 4), dtype=np.float64)
        for j in range(3):
            x0 = np.sum(mix[:, j][:, None] * xs, axis=0)
            P0 = np.zeros((4, 4), dtype=np.float64)
            for i in range(3):
                dx = (xs[i] - x0).reshape(4, 1)
                P0 = P0 + mix[i, j] * (Ps[i] + dx @ dx.T)
            mixed_xs[j] = x0
            mixed_Ps[j] = P0

        pred_xs = np.zeros_like(mixed_xs)
        pred_Ps = np.zeros_like(mixed_Ps)
        for j in range(3):
            F, Q = self._model_F_Q(j, dt)
            pred_xs[j] = F @ mixed_xs[j]
            pred_Ps[j] = F @ mixed_Ps[j] @ F.T + Q

        tr.imm_xs = pred_xs
        tr.imm_Ps = pred_Ps
        tr.imm_mu = cbar / float(np.sum(cbar))
        tr.x, tr.P = self._merge_models(tr.imm_xs, tr.imm_Ps, tr.imm_mu)

    def _update_track(
        self,
        tr: Track,
        meas: FusedMeasurement,
        assoc_cost: Optional[float] = None,
        allow_appearance_update: bool = True,
        allow_history_update: bool = True,
    ):
        if tr.imm_xs is None or tr.imm_Ps is None or tr.imm_mu is None:
            self._init_track_imm(tr)

        H = np.array([[1.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0]], dtype=np.float64)

        r = self._measurement_noise_for(meas)
        R = np.diag([r * r, r * r]).astype(np.float64)
        z = meas.pos_xy.reshape(2, 1)
        prior_mu = np.maximum(tr.imm_mu, 1e-12)
        prior_mu = prior_mu / float(np.sum(prior_mu))

        xs_upd = np.zeros_like(tr.imm_xs)
        Ps_upd = np.zeros_like(tr.imm_Ps)
        likelihood = np.zeros((3,), dtype=np.float64)

        for j in range(3):
            x = tr.imm_xs[j].reshape(4, 1)
            P = tr.imm_Ps[j]

            y = z - (H @ x)
            S = H @ P @ H.T + R
            S = 0.5 * (S + S.T)

            try:
                Sinv = np.linalg.inv(S)
                detS = float(np.linalg.det(S))
            except np.linalg.LinAlgError:
                Sinv = np.linalg.pinv(S)
                detS = max(1e-12, float(np.linalg.det(S + 1e-6 * np.eye(2))))

            detS = max(1e-12, detS)
            expo = -0.5 * float((y.T @ Sinv @ y).reshape(-1)[0])
            norm = 1.0 / (2.0 * math.pi * math.sqrt(detS))
            likelihood[j] = max(1e-12, norm * math.exp(expo))

            K = P @ H.T @ Sinv
            x_new = x + K @ y
            I = np.eye(4, dtype=np.float64)
            P_new = (I - K @ H) @ P

            xs_upd[j] = x_new.reshape(-1)
            Ps_upd[j] = 0.5 * (P_new + P_new.T)

        post = np.maximum(prior_mu * likelihood, 1e-12)
        post = post / float(np.sum(post))

        tr.imm_xs = xs_upd
        tr.imm_Ps = Ps_upd
        tr.imm_mu = post
        tr.x, tr.P = self._merge_models(tr.imm_xs, tr.imm_Ps, tr.imm_mu)

        if meas.embedding is not None:
            tr.last_embedding = meas.embedding.copy()
            head_ok = True
            if meas.head_rpy_deg is not None:
                head_ok = (
                    abs(float(meas.head_rpy_deg[0])) <= self.reid_update_max_abs_yaw_deg
                    and abs(float(meas.head_rpy_deg[1])) <= self.reid_update_max_abs_pitch_deg
                )
            can_update_embedding = (
                (assoc_cost is None or assoc_cost <= self.embedding_update_max_assoc_cost)
                and (meas.camera_confidence >= self.embedding_update_min_conf)
                and (meas.depth_valid_ratio >= 0.35)
                and (meas.bbox_area_px >= self.reid_min_bbox_area_px)
                and head_ok
            )
            if can_update_embedding and allow_appearance_update:
                if tr.ema_embedding is None:
                    tr.ema_embedding = meas.embedding.copy()
                else:
                    tr.ema_embedding = self.emb_ema_alpha * meas.embedding + (1.0 - self.emb_ema_alpha) * tr.ema_embedding
                    n = float(np.linalg.norm(tr.ema_embedding))
                    if n > 1e-6:
                        tr.ema_embedding = tr.ema_embedding / n
            if tr.embedding_gallery is None:
                tr.embedding_gallery = []
            if can_update_embedding and allow_appearance_update:
                tr.embedding_gallery.append(meas.embedding.copy())
                if len(tr.embedding_gallery) > self.embedding_gallery_size:
                    tr.embedding_gallery = tr.embedding_gallery[-self.embedding_gallery_size:]
            app_cost = self._appearance_cost(tr, meas.embedding)
            if app_cost is not None:
                tr.conf_app = max(0.0, 1.0 - min(1.0, app_cost))
            if allow_appearance_update and meas.body_reid_confidence > 0.0:
                self.selected_body_embedding_ref = meas.embedding.copy() if tr.track_id == self.selected_track_id else self.selected_body_embedding_ref

        tr.torso_hist_ref = self._blend_hist_ref(
            tr.torso_hist_ref, meas.torso_hist, meas.torso_hist_confidence, allow_appearance_update
        )
        tr.upper_body_hist_ref = self._blend_hist_ref(
            tr.upper_body_hist_ref, meas.upper_body_hist, meas.upper_body_hist_confidence, allow_appearance_update
        )
        tr.lower_body_hist_ref = self._blend_hist_ref(
            tr.lower_body_hist_ref, meas.lower_body_hist, meas.lower_body_hist_confidence, allow_appearance_update
        )
        if allow_appearance_update and meas.face_embedding is not None and meas.face_embedding_confidence > 0.0:
            tr.face_embedding_ref = meas.face_embedding.copy()

        # Update facing/head pose if present (best-effort, optional fields)
        facing01 = getattr(meas, 'facing01', None)
        if facing01 is not None and math.isfinite(float(facing01)):
            tr.last_facing01 = float(facing01)
        head_rpy = getattr(meas, 'head_rpy_deg', None)
        if head_rpy is not None:
            try:
                v = np.asarray(head_rpy, dtype=np.float64).reshape(-1)
                if v.size == 3 and np.all(np.isfinite(v)):
                    tr.last_head_rpy_deg = v.astype(np.float64)
            except Exception:
                pass

        if allow_history_update and tr.track_id == self.selected_track_id:
            self._update_selected_history(meas)

    # ---------------------- Association ----------------------
    def _associate(self, fused: List[FusedMeasurement]) -> Tuple[List[Tuple[int, int, float]], List[int], List[int]]:
        track_ids = [tid for tid, tr in self.tracks.items() if tr.state != TrackState.DELETED]

        if not track_ids and not fused:
            return [], [], []
        if not track_ids:
            return [], [], list(range(len(fused)))
        if not fused:
            return [], track_ids, []

        assignments: List[Tuple[int, int, float]] = []
        unmatched_track_ids = set(track_ids)
        unmatched_meas_idxs = set(range(len(fused)))

        selected_assign, selected_unmatched_tracks, selected_unmatched_meas = self._associate_selected_first(fused)
        assignments.extend(selected_assign)
        unmatched_track_ids = set(selected_unmatched_tracks)
        unmatched_meas_idxs = set(selected_unmatched_meas)

        # Stage A: strict association (quality + appearance required)
        strict_meas = [
            midx for midx in sorted(unmatched_meas_idxs)
            if fused[midx].camera_confidence >= self.embedding_update_min_conf
            and fused[midx].depth_valid_ratio >= 0.35
            and fused[midx].embedding is not None
        ]
        a1, _, _ = self._associate_stage(sorted(unmatched_track_ids), strict_meas, fused, strict=True)
        assignments.extend(a1)
        for tid, midx, _ in a1:
            unmatched_track_ids.discard(tid)
            unmatched_meas_idxs.discard(midx)

        # Stage B: relaxed association for remaining
        if unmatched_track_ids and unmatched_meas_idxs:
            a2, u_tracks, u_meas = self._associate_stage(
                sorted(unmatched_track_ids), sorted(unmatched_meas_idxs), fused, strict=False
            )
            assignments.extend(a2)
            unmatched_track_ids = set(u_tracks)
            unmatched_meas_idxs = set(u_meas)

        return assignments, sorted(unmatched_track_ids), sorted(unmatched_meas_idxs)

    def _associate_selected_first(
        self,
        fused: List[FusedMeasurement],
    ) -> Tuple[List[Tuple[int, int, float]], List[int], List[int]]:
        track_ids = [tid for tid, tr in self.tracks.items() if tr.state != TrackState.DELETED]
        unmatched_track_ids = set(track_ids)
        unmatched_meas_idxs = set(range(len(fused)))
        now_sec = self._now_sec()

        if self.target_lock_state == TargetLockState.REACQUIRE:
            return self._associate_selected_reacquire(
                fused, now_sec, sorted(unmatched_track_ids), sorted(unmatched_meas_idxs)
            )

        if not self._selected_track_is_valid(now_sec):
            if not self._selected_identity_protected(now_sec):
                if self.selected_track_id is not None:
                    self._selected_reason('selected_invalid_track')
                return [], sorted(unmatched_track_ids), sorted(unmatched_meas_idxs)
            self._selected_reason('selected_invalid_track')
            return self._associate_selected_reacquire(fused, now_sec, sorted(unmatched_track_ids), sorted(unmatched_meas_idxs))

        tr = self.tracks[self.selected_track_id]
        candidate_scores: List[Tuple[float, float, int, Dict[str, float]]] = []
        for midx, meas in enumerate(fused):
            if not self._selected_hard_gate_pass(tr, meas):
                continue
            score, details = self._selected_match_score(tr, meas)
            candidate_scores.append((score, details.get('motion', 0.0), midx, details))

        if not candidate_scores:
            self._handle_selected_no_match(now_sec)
            return [], sorted(unmatched_track_ids), sorted(unmatched_meas_idxs)

        candidate_scores.sort(key=lambda item: item[0], reverse=True)
        best_score, _, best_midx, best_details = candidate_scores[0]
        second_score = candidate_scores[1][0] if len(candidate_scores) > 1 else -1.0
        best_meas = fused[best_midx]
        is_clean = best_score >= self.selected_match_clean_update_score
        is_acceptable = best_score >= self.selected_match_min_score
        is_ambiguous = (second_score >= 0.0) and ((best_score - second_score) < self.selected_match_ambiguous_margin)
        is_front_occluder = self._is_front_occluder(tr, best_meas)

        if is_front_occluder:
            self._clear_selected_uncertainty_memory(clear_clean=True)
            self._selected_reason('front_occluder_reject')
            self._handle_selected_no_match(now_sec, hold=True)
            return [], sorted(unmatched_track_ids), sorted(unmatched_meas_idxs)

        if is_clean and not is_ambiguous:
            self._accept_selected_match(tr, best_midx, best_score, now_sec, clean=True)
            unmatched_track_ids.discard(tr.track_id)
            unmatched_meas_idxs.discard(best_midx)
            return [(tr.track_id, best_midx, float(max(0.0, 1.0 - best_score)))], sorted(unmatched_track_ids), sorted(unmatched_meas_idxs)

        if is_acceptable and not is_ambiguous:
            self._accept_selected_match(tr, best_midx, best_score, now_sec, clean=False)
            unmatched_track_ids.discard(tr.track_id)
            unmatched_meas_idxs.discard(best_midx)
            return [(tr.track_id, best_midx, float(max(0.0, 1.0 - best_score)))], sorted(unmatched_track_ids), sorted(unmatched_meas_idxs)

        self._clear_selected_uncertainty_memory(clear_clean=True)
        if is_ambiguous:
            self._selected_reason('selected_ambiguous_match')
        self._handle_selected_no_match(now_sec)
        return [], sorted(unmatched_track_ids), sorted(unmatched_meas_idxs)

    def _associate_selected_reacquire(
        self,
        fused: List[FusedMeasurement],
        now_sec: float,
        track_ids: List[int],
        meas_indices: List[int],
    ) -> Tuple[List[Tuple[int, int, float]], List[int], List[int]]:
        if not self._selected_identity_protected(now_sec):
            return [], track_ids, meas_indices

        self.target_lock_state = TargetLockState.REACQUIRE
        self.selected_embedding_frozen = True
        candidate_scores: List[Tuple[float, int, Dict[str, float]]] = []
        for midx in meas_indices:
            meas = fused[midx]
            if not self._selected_reacquire_gate_pass(meas):
                continue
            score, details = self._score_reacquire_candidate(meas)
            candidate_scores.append((score, midx, details))

        if not candidate_scores:
            self._clear_selected_uncertainty_memory(clear_clean=True)
            self._handle_selected_no_match(now_sec)
            return [], track_ids, meas_indices

        candidate_scores.sort(key=lambda item: item[0], reverse=True)
        best_score, best_midx, best_details = candidate_scores[0]
        second_score = candidate_scores[1][0] if len(candidate_scores) > 1 else -1.0
        self._selected_last_best_score = float(best_score)
        self._selected_last_second_score = float(second_score)
        self._selected_last_identity_score = float(best_details.get('identity', best_details.get('appearance', 0.0)))

        min_relock_score = self.crossover_min_relock_score if self.crossover_active else self.selected_match_min_score
        if self._post_relock_protection_active(now_sec):
            min_relock_score = max(min_relock_score, self.post_relock_min_score)
        if best_score < min_relock_score:
            self._clear_selected_uncertainty_memory(clear_clean=True)
            self._handle_selected_no_match(now_sec)
            return [], track_ids, meas_indices

        candidate_meas = fused[best_midx]
        candidate_track_id = self._find_track_for_measurement(candidate_meas)
        if candidate_track_id is None:
            self._clear_selected_uncertainty_memory(clear_clean=True)
            self._selected_reason('reacquire_unresolved_track')
            self._handle_selected_no_match(now_sec)
            return [], track_ids, meas_indices

        is_cross_track = (self.selected_track_id is not None) and (candidate_track_id != self.selected_track_id)
        if is_cross_track:
            allowed, reject_reason = self._selected_cross_track_reacquire_allowed(now_sec)
            if not allowed:
                self._clear_selected_uncertainty_memory(clear_clean=True)
                if reject_reason is not None:
                    self._selected_reason(reject_reason)
                return [], track_ids, meas_indices
            appearance_score = float(best_details.get('appearance', 0.0))
            identity_score = float(best_details.get('identity', appearance_score))
            contradictory = bool(best_details.get('contradictory', 0.0) > 0.5)
            if candidate_meas.embedding is None:
                self._clear_selected_uncertainty_memory(clear_clean=True)
                self._selected_reason('reacquire_reject_missing_app')
                return [], track_ids, meas_indices
            if contradictory:
                self._clear_selected_uncertainty_memory(clear_clean=True)
                self._selected_reason('reacquire_reject_contradictory')
                return [], track_ids, meas_indices
            if appearance_score < 0.60:
                self._clear_selected_uncertainty_memory(clear_clean=True)
                self._selected_reason('reacquire_reject_weak_app')
                return [], track_ids, meas_indices
            if identity_score < 0.60:
                self._clear_selected_uncertainty_memory(clear_clean=True)
                self._selected_reason('reacquire_reject_identity')
                return [], track_ids, meas_indices
            if best_score < min_relock_score:
                self._clear_selected_uncertainty_memory(clear_clean=True)
                self._selected_reason('reacquire_reject_low_score')
                return [], track_ids, meas_indices
            min_margin = self.crossover_ambiguity_margin if self.crossover_active else self.selected_match_ambiguous_margin
            if self._post_relock_protection_active(now_sec):
                min_margin = max(min_margin, self.post_relock_switch_margin)
            if second_score >= 0.0 and (best_score - second_score) < min_margin:
                self._clear_selected_uncertainty_memory(clear_clean=True)
                self._selected_reason('reacquire_reject_ambiguous')
                return [], track_ids, meas_indices
            if self.crossover_active and best_score < self.single_strong_cue_min_score:
                self._clear_selected_uncertainty_memory(clear_clean=True)
                self._selected_reason('reacquire_reject_crossover_single_cue')
                return [], track_ids, meas_indices

        if self._selected_candidate_track_id != candidate_track_id:
            self._selected_clean_match_track_id = None
            self._selected_candidate_track_id = candidate_track_id
            self.target_reacquire_count = 1
        else:
            self.target_reacquire_count += 1

        self._selected_reason('reacquire_candidate')
        required_confirm_frames = self.crossover_confirm_frames if self.crossover_active else self.reacquire_confirm_frames
        if self.target_reacquire_count < required_confirm_frames:
            return [], track_ids, meas_indices

        old_tid = self.selected_track_id
        self._enter_locked_from_reacquire(candidate_track_id, candidate_meas, now_sec)
        self._selected_reason('relock_confirmed')

        unmatched_track_ids = set(track_ids)
        unmatched_meas_idxs = set(meas_indices)
        unmatched_track_ids.discard(candidate_track_id)
        unmatched_meas_idxs.discard(best_midx)
        if old_tid is not None and old_tid in unmatched_track_ids and old_tid != candidate_track_id:
            unmatched_track_ids.discard(old_tid)
        return [(candidate_track_id, best_midx, float(max(0.0, 1.0 - best_score)))], sorted(unmatched_track_ids), sorted(unmatched_meas_idxs)

    def _associate_stage(
        self,
        track_ids: List[int],
        meas_indices: List[int],
        fused: List[FusedMeasurement],
        strict: bool,
    ) -> Tuple[List[Tuple[int, int, float]], List[int], List[int]]:
        if not track_ids or not meas_indices:
            return [], track_ids, meas_indices

        cost = np.full((len(track_ids), len(meas_indices)), self.invalid_cost, dtype=np.float64)
        valid = np.zeros_like(cost, dtype=bool)

        for i, tid in enumerate(track_ids):
            tr = self.tracks[tid]
            pred = tr.pos()

            for jj, midx in enumerate(meas_indices):
                meas = fused[midx]
                dxy = float(np.linalg.norm(pred - meas.pos_xy))
                if dxy > self.track_position_gate_m:
                    continue

                if strict and meas.embedding is None:
                    continue

                maha = self._mahalanobis_distance(tr, meas)
                gate = self.occluded_maha_gate_thresh if tr.state == TrackState.OCCLUDED else self.maha_gate_thresh
                if self.association_gate_mode == 'mahalanobis' and maha > gate:
                    continue

                app_cost = self._appearance_cost(tr, meas.embedding)
                app_gate = self.occluded_app_gate if tr.state == TrackState.OCCLUDED else self.appearance_gate
                if tr.state == TrackState.OCCLUDED and app_cost is None:
                    continue
                if strict and app_cost is None:
                    continue
                if app_cost is not None and app_cost > app_gate:
                    continue

                motion_cost = min(maha / 10.0, 1.0)
                pos_cost = dxy / max(self.track_position_gate_m, 1e-3)
                app_cost_v = float(app_cost) if app_cost is not None else 0.0
                total = self.w_pos * pos_cost + self.w_app * app_cost_v + self.w_motion * motion_cost

                cost[i, jj] = total
                valid[i, jj] = True

        assignments: List[Tuple[int, int, float]] = []
        unmatched_track_ids = set(track_ids)
        unmatched_meas_idxs = set(meas_indices)
        if not np.any(valid):
            return assignments, sorted(unmatched_track_ids), sorted(unmatched_meas_idxs)

        if SCIPY_AVAILABLE:
            rows, cols = linear_sum_assignment(cost)
            for r, c in zip(rows.tolist(), cols.tolist()):
                if not valid[r, c] or cost[r, c] >= self.invalid_cost:
                    continue
                tid = track_ids[r]
                midx = meas_indices[c]
                assignments.append((tid, midx, float(cost[r, c])))
                unmatched_track_ids.discard(tid)
                unmatched_meas_idxs.discard(midx)
        else:
            candidates = []
            for i in range(cost.shape[0]):
                for j in range(cost.shape[1]):
                    if valid[i, j]:
                        candidates.append((float(cost[i, j]), i, j))
            candidates.sort(key=lambda x: x[0])

            used_rows = set()
            used_cols = set()
            for cst, i, j in candidates:
                if i in used_rows or j in used_cols:
                    continue
                tid = track_ids[i]
                midx = meas_indices[j]
                assignments.append((tid, midx, cst))
                used_rows.add(i)
                used_cols.add(j)
                unmatched_track_ids.discard(tid)
                unmatched_meas_idxs.discard(midx)

        return assignments, sorted(unmatched_track_ids), sorted(unmatched_meas_idxs)

    def _appearance_cost(self, tr: Track, embedding: Optional[np.ndarray]) -> Optional[float]:
        if embedding is None:
            return None
        cands: List[float] = []
        if tr.ema_embedding is not None:
            cands.append(1.0 - max(-1.0, min(1.0, float(np.dot(tr.ema_embedding, embedding)))))
        if tr.embedding_gallery:
            for e in tr.embedding_gallery:
                cands.append(1.0 - max(-1.0, min(1.0, float(np.dot(e, embedding)))))
        if not cands:
            return None
        return float(min(cands))

    def _measurement_noise_for(self, meas: FusedMeasurement) -> float:
        if meas.source in ('cam_lidar', 'lidar_only'):
            return self.r_lidar

        q_depth = max(0.0, min(1.0, float(meas.depth_valid_ratio)))
        area = max(0.0, float(meas.bbox_area_px))
        q_area = min(1.0, area / 15000.0)
        quality = self.depth_quality_weight * q_depth + self.bbox_area_weight * q_area
        quality = max(0.0, min(1.0, quality))
        # better quality -> lower noise
        return float(self.camera_meas_noise_max - (self.camera_meas_noise_max - self.camera_meas_noise_min) * quality)

    def _mahalanobis_distance(self, tr: Track, meas: FusedMeasurement) -> float:
        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]], dtype=np.float64)
        r = self._measurement_noise_for(meas)
        R = np.diag([r * r, r * r]).astype(np.float64)
        x = tr.x.reshape(4, 1)
        z = meas.pos_xy.reshape(2, 1)
        y = z - H @ x
        S = H @ tr.P @ H.T + R
        try:
            return float((y.T @ np.linalg.inv(S) @ y).reshape(-1)[0])
        except np.linalg.LinAlgError:
            return float((y.T @ np.linalg.pinv(S) @ y).reshape(-1)[0])

    def _innovation_cost(self, tr: Track, meas: FusedMeasurement) -> float:
        return min(self._mahalanobis_distance(tr, meas) / 10.0, 1.0)

    def _selected_track_is_valid(self, now_sec: float) -> bool:
        if self.selected_track_id is None:
            return False
        tr = self.tracks.get(self.selected_track_id)
        if tr is None or tr.state == TrackState.DELETED:
            return False
        if (now_sec - tr.last_update_sec) > self.selected_track_internal_stale_sec:
            return False
        return True

    def _selected_identity_protected(self, now_sec: float) -> bool:
        if self.selected_track_id is None:
            return False
        if self.target_lock_state == TargetLockState.REACQUIRE:
            return True
        if (now_sec - self.selected_last_seen_sec) <= self.target_lock_lost_timeout_sec:
            return True
        if now_sec <= self.target_lock_hold_until_sec:
            return True
        return now_sec <= self.selected_relock_cooldown_until_sec

    def _selected_trusted_evidence_age(self, now_sec: float) -> float:
        if self.selected_last_clean_seen_sec > 0.0:
            return max(0.0, now_sec - self.selected_last_clean_seen_sec)
        if self.selected_last_seen_sec > 0.0:
            return max(0.0, now_sec - self.selected_last_seen_sec)
        return 1e9

    def _selected_cross_track_reacquire_allowed(self, now_sec: float) -> tuple[bool, Optional[str]]:
        trusted_age = self._selected_trusted_evidence_age(now_sec)
        if self.selected_no_switch_during_hard_hold and trusted_age <= self.selected_hard_hold_sec:
            return False, 'reacquire_reject_hard_hold'
        if trusted_age > self.selected_reacquire_window_sec:
            return False, 'reacquire_reject_outside_window'
        return True, None

    def _selected_hard_gate_pass(self, tr: Track, meas: FusedMeasurement) -> bool:
        pred_xy = tr.pos()
        dxy = float(np.linalg.norm(pred_xy - meas.pos_xy))
        if dxy > self.track_position_gate_m:
            return False

        ref_bearing = self._hist_angle_mean(self.selected_bearing_hist, math.atan2(float(pred_xy[1]), float(pred_xy[0])))
        bearing_err = abs(self._angle_diff(float(meas.bearing_rad), ref_bearing))
        if bearing_err > math.radians(self.selected_bearing_gate_deg):
            return False

        if self._history_ready() and meas.center_uv is not None and self.selected_center_hist:
            ref_center = np.mean(np.stack(list(self.selected_center_hist), axis=0), axis=0)
            if float(np.linalg.norm(meas.center_uv - ref_center)) > self.selected_center_corridor_px:
                return False
        return True

    def _selected_reacquire_gate_pass(self, meas: FusedMeasurement) -> bool:
        if not self._history_ready():
            return False
        bearing_ref = self._hist_angle_mean(self.selected_bearing_hist, float(meas.bearing_rad))
        if abs(self._angle_diff(float(meas.bearing_rad), bearing_ref)) > math.radians(self.selected_bearing_gate_deg * 1.5):
            return False
        range_ref = self._hist_mean(self.selected_range_hist, float(meas.range_m))
        if abs(float(meas.range_m) - range_ref) > max(self.track_position_gate_m * 1.5, 0.8):
            return False
        if meas.center_uv is not None and self.selected_center_hist:
            ref_center = np.mean(np.stack(list(self.selected_center_hist), axis=0), axis=0)
            if float(np.linalg.norm(meas.center_uv - ref_center)) > (self.selected_center_corridor_px * 1.5):
                return False
        return True

    def _selected_match_score(self, tr: Track, meas: FusedMeasurement) -> Tuple[float, Dict[str, float]]:
        pred_xy = tr.pos()
        pred_range = float(np.linalg.norm(pred_xy))
        pred_bearing = math.atan2(float(pred_xy[1]), float(pred_xy[0]))

        app_cost = self._appearance_cost(tr, meas.embedding)
        appearance_score = 0.5 if app_cost is None else max(0.0, 1.0 - min(1.0, float(app_cost)))
        if meas.embedding is None or meas.body_reid_confidence <= 0.0:
            appearance_score = max(0.0, appearance_score - self.weak_appearance_penalty)
        if meas.source == 'lidar_only':
            appearance_score = max(0.0, appearance_score - self.lidar_only_selected_penalty)
        if meas.continuity_only:
            appearance_score *= 0.65
        appearance_score = min(appearance_score, self._clamp01(meas.identity_quality if meas.identity_quality > 0.0 else appearance_score))

        ref_range = self._hist_mean(self.selected_range_hist, pred_range)
        range_err = abs(float(meas.range_m) - ref_range)
        range_score = max(0.0, 1.0 - (range_err / max(0.35, self.front_occluder_range_margin_m * 2.0)))

        ref_bearing = self._hist_angle_mean(self.selected_bearing_hist, pred_bearing)
        bearing_err = abs(self._angle_diff(float(meas.bearing_rad), ref_bearing))
        bearing_gate = math.radians(self.selected_bearing_gate_deg)
        bearing_score = max(0.0, 1.0 - (bearing_err / max(1e-6, bearing_gate)))

        ref_w = self._hist_mean(self.selected_bbox_w_hist, max(1.0, float(meas.bbox_w_px)))
        ref_h = self._hist_mean(self.selected_bbox_h_hist, max(1.0, float(meas.bbox_h_px)))
        w_ratio = abs(float(meas.bbox_w_px) - ref_w) / max(ref_w, 1.0)
        h_ratio = abs(float(meas.bbox_h_px) - ref_h) / max(ref_h, 1.0)
        bbox_delta = 0.5 * (w_ratio + h_ratio)
        bbox_size_score = max(0.0, 1.0 - (bbox_delta / max(1e-3, self.bbox_size_change_gate)))

        motion_cost = min(float(np.linalg.norm(pred_xy - meas.pos_xy)) / max(1e-3, self.track_position_gate_m), 1.0)
        motion_score = max(0.0, 1.0 - motion_cost)

        score = (
            0.35 * appearance_score
            + 0.25 * range_score
            + 0.20 * bearing_score
            + 0.10 * bbox_size_score
            + 0.10 * motion_score
        )
        return score, {
            'appearance': appearance_score,
            'range': range_score,
            'bearing': bearing_score,
            'bbox': bbox_size_score,
            'motion': motion_score,
        }

    def _is_front_occluder(self, tr: Track, meas: FusedMeasurement) -> bool:
        if self.target_lock_state not in (TargetLockState.LOCKED, TargetLockState.TEMP_OCCLUDED):
            return False
        if self.selected_track_id != tr.track_id:
            return False
        if not self._history_ready():
            return False

        pred_xy = tr.pos()
        expected_range = self._hist_mean(self.selected_range_hist, float(np.linalg.norm(pred_xy)))
        if meas.range_m >= (expected_range - self.front_occluder_range_margin_m):
            return False

        expected_bearing = self._hist_angle_mean(self.selected_bearing_hist, math.atan2(float(pred_xy[1]), float(pred_xy[0])))
        bearing_err = abs(self._angle_diff(float(meas.bearing_rad), expected_bearing))
        if bearing_err > math.radians(self.selected_bearing_gate_deg):
            return False

        center_near = False
        if meas.center_uv is not None and self.selected_center_hist:
            ref_center = np.mean(np.stack(list(self.selected_center_hist), axis=0), axis=0)
            center_near = float(np.linalg.norm(meas.center_uv - ref_center)) <= self.selected_center_corridor_px

        overlap_ok = False
        if meas.bbox_xyxy is not None and self.selected_bbox_w_hist and self.selected_bbox_h_hist and self.selected_center_hist:
            ref_center = np.mean(np.stack(list(self.selected_center_hist), axis=0), axis=0)
            ref_w = self._hist_mean(self.selected_bbox_w_hist, float(meas.bbox_w_px))
            ref_h = self._hist_mean(self.selected_bbox_h_hist, float(meas.bbox_h_px))
            ref_bbox = (
                int(ref_center[0] - ref_w * 0.5),
                int(ref_center[1] - ref_h * 0.5),
                int(ref_center[0] + ref_w * 0.5),
                int(ref_center[1] + ref_h * 0.5),
            )
            overlap_ok = self._bbox_iou(ref_bbox, meas.bbox_xyxy) >= self.selected_bbox_overlap_min_iou

        short_window = self._now_sec() <= max(self.target_lock_hold_until_sec, tr.last_update_sec + self.temp_occluded_hold_sec)
        return short_window and (center_near or overlap_ok)

    def _accept_selected_match(self, tr: Track, midx: int, score: float, now_sec: float, clean: bool):
        self._selected_last_best_score = float(score)
        self._selected_last_second_score = -1.0
        self._selected_last_identity_score = float(score)
        meas = self.last_fused_measurements[midx]
        if meas.embedding is not None and meas.body_reid_confidence > 0.0:
            self._tracker_reid_log_success(tr.track_id, meas.embedding, now_sec)
        else:
            self._tracker_reid_log_failure(tr.track_id, meas.reid_debug_reason, now_sec)
        self._enter_locked_from_selected_match(tr.track_id, meas, now_sec, clean)
        if clean:
            self._selected_reason('selected_clean_match')
        else:
            self._selected_reason('selected_acceptable_match')

    def _handle_selected_no_match(self, now_sec: float, hold: bool = False):
        if self.selected_track_id is None:
            return
        since_seen = now_sec - self.selected_last_seen_sec
        self.selected_embedding_frozen = True

        if self.target_lock_state == TargetLockState.LOCKED:
            self.target_lock_state = TargetLockState.TEMP_OCCLUDED
            if self.target_lock_hold_until_sec <= now_sec:
                self.target_lock_hold_until_sec = now_sec + self.temp_occluded_hold_sec
            self._selected_reason('temp_occluded_hold')
            return

        if self.target_lock_state == TargetLockState.TEMP_OCCLUDED:
            if hold or now_sec <= self.target_lock_hold_until_sec:
                self._selected_reason('temp_occluded_hold')
                return
            self.target_lock_state = TargetLockState.TEMP_LOST

        if self.target_lock_state == TargetLockState.TEMP_LOST:
            if since_seen <= self.target_lock_lost_timeout_sec:
                self._selected_reason('temp_lost_no_match')
                return

        if self.target_lock_state != TargetLockState.REACQUIRE:
            self.target_lock_state = TargetLockState.REACQUIRE
            self.target_reacquire_count = 0
            self.target_switch_count = 0
            self._selected_candidate_track_id = None
            self._selected_clean_match_track_id = None
            self._selected_last_decision_reason = ''

        self._selected_reason('reacquire_candidate')

    def _enter_locked_from_selected_match(self, track_id: int, meas: FusedMeasurement, now_sec: float, clean: bool):
        self.selected_track_id = track_id
        self.selected_last_seen_sec = now_sec
        self.selected_last_clean_seen_sec = now_sec
        self.target_lock_state = TargetLockState.LOCKED
        self.target_lock_hold_until_sec = 0.0
        self.target_reacquire_count = 0
        self._selected_candidate_track_id = None
        self._selected_clean_match_track_id = track_id if clean else None
        if clean:
            self.target_switch_count = 0
            self.selected_embedding_frozen = False
        else:
            self.target_switch_count = min(self.switch_confirm_frames, self.target_switch_count + 1)
            self.selected_embedding_frozen = True

    def _enter_locked_from_reacquire(self, track_id: int, meas: FusedMeasurement, now_sec: float):
        old_selected_track_id = self.selected_track_id
        if meas.embedding is not None and meas.body_reid_confidence > 0.0:
            self._tracker_reid_log_success(track_id, meas.embedding, now_sec)
        else:
            self._tracker_reid_log_failure(track_id, meas.reid_debug_reason, now_sec)
        self.selected_track_id = track_id
        self.selected_last_seen_sec = now_sec
        if old_selected_track_id == track_id:
            self.selected_last_clean_seen_sec = now_sec
        self.target_lock_state = TargetLockState.LOCKED
        self.target_lock_hold_until_sec = 0.0
        self.selected_relock_cooldown_until_sec = now_sec + self.relock_cooldown_sec
        self.post_relock_protection_until_sec = now_sec + self.post_relock_protection_sec
        self._selected_candidate_track_id = None
        self._selected_clean_match_track_id = None
        self.target_reacquire_count = 0
        self.target_switch_count = 0
        self.selected_embedding_frozen = True
        self._selected_last_decision_reason = ''
        self._clear_selected_histories()
        self._update_selected_history(meas)

    def _post_relock_protection_active(self, now_sec: float) -> bool:
        return now_sec <= self.post_relock_protection_until_sec

    def _update_crossover_state(self, now_sec: float, fused: List[FusedMeasurement]):
        if self.selected_track_id is None or self.selected_track_id not in self.tracks or not self._history_ready():
            self.crossover_active = False
            self.crossover_stable_since_sec = 0.0
            return
        selected = self.tracks[self.selected_track_id]
        if selected.state == TrackState.DELETED:
            self.crossover_active = False
            self.crossover_stable_since_sec = 0.0
            return
        selected_xy = selected.pos()
        ref_center = np.mean(np.stack(list(self.selected_center_hist), axis=0), axis=0) if self.selected_center_hist else None
        ref_range = self._hist_mean(self.selected_range_hist, float(np.linalg.norm(selected_xy)))
        ref_bearing = self._hist_angle_mean(self.selected_bearing_hist, math.atan2(float(selected_xy[1]), float(selected_xy[0])))
        ref_w = self._hist_mean(self.selected_bbox_w_hist, 1.0)
        ref_h = self._hist_mean(self.selected_bbox_h_hist, 1.0)

        ambiguous = False
        selected_ref_bbox = None
        if ref_center is not None:
            selected_ref_bbox = (
                int(ref_center[0] - ref_w * 0.5),
                int(ref_center[1] - ref_h * 0.5),
                int(ref_center[0] + ref_w * 0.5),
                int(ref_center[1] + ref_h * 0.5),
            )

        for meas in fused:
            if meas.range_m <= 0.0 or meas.bearing_rad == 0.0 and meas.pos_xy is None:
                continue
            if abs(float(meas.range_m) - ref_range) > self.crossover_enter_range_delta_m:
                continue
            if abs(self._angle_diff(float(meas.bearing_rad), ref_bearing)) > math.radians(self.crossover_enter_bearing_delta_deg):
                continue

            center_ok = ref_center is None
            if ref_center is not None and meas.center_uv is not None:
                center_ok = float(np.linalg.norm(meas.center_uv - ref_center)) <= self.crossover_enter_center_delta_px
            if not center_ok:
                continue

            spatial_delta = float(np.linalg.norm(meas.pos_xy - selected_xy))
            if spatial_delta > self.track_position_gate_m:
                continue

            bbox_ratio_delta = 1.0
            bbox_iou = 0.0
            if meas.bbox_xyxy is not None and selected_ref_bbox is not None:
                bbox_iou = self._bbox_iou(selected_ref_bbox, meas.bbox_xyxy)
                bw = max(1.0, float(ref_w))
                bh = max(1.0, float(ref_h))
                mw = max(1.0, float(meas.bbox_w_px))
                mh = max(1.0, float(meas.bbox_h_px))
                bbox_ratio_delta = max(abs(mw - bw) / bw, abs(mh - bh) / bh)
            if bbox_iou <= 0.0 and selected_ref_bbox is not None and meas.bbox_xyxy is not None:
                continue
            if bbox_ratio_delta > self.crossover_enter_bbox_ratio_delta:
                continue
            # Ignore the selected target's own measurement. Without this,
            # a single clean detection can be misread as a crossover rival.
            if self._is_selected_self_measurement(meas, selected_xy, ref_center, ref_range, ref_bearing, bbox_ratio_delta, bbox_iou):
                continue
            ambiguous = True
            break

        if ambiguous:
            self.crossover_active = True
            self.crossover_stable_since_sec = 0.0
            return

        if not self.crossover_active:
            return
        if self.crossover_stable_since_sec <= 0.0:
            self.crossover_stable_since_sec = now_sec
        if (now_sec - self.crossover_stable_since_sec) >= self.crossover_exit_stable_sec:
            self.crossover_active = False
            self.crossover_stable_since_sec = 0.0

    def _update_selected_history(self, meas: FusedMeasurement):
        allow_learning = self._selected_learning_allowed(meas)
        self.selected_range_hist.append(float(meas.range_m))
        self.selected_bearing_hist.append(float(meas.bearing_rad))
        self.selected_bbox_w_hist.append(float(meas.bbox_w_px))
        self.selected_bbox_h_hist.append(float(meas.bbox_h_px))
        if meas.center_uv is not None:
            self.selected_center_hist.append(meas.center_uv.copy())
        if allow_learning and meas.embedding is not None and meas.body_reid_confidence > 0.0:
            self.selected_body_embedding_ref = meas.embedding.copy()
            self._update_selected_identity_gallery(meas.embedding.copy())
            if (
                self.selected_stable_anchor_embedding is None
                and self._clamp01(meas.body_reid_confidence) >= self.stable_anchor_min_confidence
            ):
                self.selected_stable_anchor_embedding = meas.embedding.copy()
                self.selected_stable_anchor_set_sec = self._now_sec()
        self.selected_torso_hist_ref = self._blend_hist_ref(
            self.selected_torso_hist_ref, meas.torso_hist, meas.torso_hist_confidence, allow_learning
        )
        self.selected_upper_body_hist_ref = self._blend_hist_ref(
            self.selected_upper_body_hist_ref, meas.upper_body_hist, meas.upper_body_hist_confidence, allow_learning
        )
        self.selected_lower_body_hist_ref = self._blend_hist_ref(
            self.selected_lower_body_hist_ref, meas.lower_body_hist, meas.lower_body_hist_confidence, allow_learning
        )
        if allow_learning and meas.face_embedding is not None and meas.face_embedding_confidence > 0.0:
            self.selected_face_embedding_ref = meas.face_embedding.copy()

    def _selected_reason(self, tag: str):
        self._selected_last_decision_reason = tag
        self._selected_last_decision_code = int(self._decision_code_for_reason(tag))
        last = self._selected_log_last_sec.get(tag, 0.0)
        now = self._now_sec()
        if (now - last) < 0.25:
            return
        self._selected_log_last_sec[tag] = now
        self.get_logger().info(f'[TrackManager][selected] {tag}')

    def _reset_selected_lock_state(self):
        self.target_lock_state = TargetLockState.NONE
        self.target_lock_hold_until_sec = 0.0
        self.selected_last_clean_seen_sec = 0.0
        self.target_reacquire_count = 0
        self.target_switch_count = 0
        self.selected_embedding_frozen = False
        self.selected_relock_cooldown_until_sec = 0.0
        self.post_relock_protection_until_sec = 0.0
        self._selected_candidate_track_id = None
        self._selected_clean_match_track_id = None
        self._selected_last_decision_reason = ''
        self._selected_last_decision_code = int(TargetLockDebug.DECISION_NONE)
        self._selected_last_best_score = -1.0
        self._selected_last_second_score = -1.0
        self._selected_last_identity_score = -1.0
        self.crossover_active = False
        self.crossover_stable_since_sec = 0.0
        self._clear_selected_histories()

    def _clear_selected_uncertainty_memory(self, clear_clean: bool = False):
        self.selected_embedding_frozen = True
        self.target_reacquire_count = 0
        self.target_switch_count = 0
        self._selected_candidate_track_id = None
        if clear_clean:
            self._selected_clean_match_track_id = None

    def _clear_selected_histories(self):
        self.selected_range_hist.clear()
        self.selected_bearing_hist.clear()
        self.selected_bbox_h_hist.clear()
        self.selected_bbox_w_hist.clear()
        self.selected_center_hist.clear()
        self.selected_torso_hist_ref = None
        self.selected_upper_body_hist_ref = None
        self.selected_lower_body_hist_ref = None
        self.selected_face_embedding_ref = None
        self.selected_body_embedding_ref = None
        self.selected_identity_gallery = []
        self.selected_stable_anchor_embedding = None
        self.selected_stable_anchor_set_sec = 0.0

    def _is_selected_self_measurement(
        self,
        meas: FusedMeasurement,
        selected_xy: np.ndarray,
        ref_center: Optional[np.ndarray],
        ref_range: float,
        ref_bearing: float,
        bbox_ratio_delta: float,
        bbox_iou: float,
    ) -> bool:
        spatial_delta = float(np.linalg.norm(meas.pos_xy - selected_xy))
        if spatial_delta > 0.12:
            return False

        range_delta = abs(float(meas.range_m) - ref_range)
        if range_delta > 0.12:
            return False

        bearing_delta = abs(self._angle_diff(float(meas.bearing_rad), ref_bearing))
        if bearing_delta > math.radians(4.0):
            return False

        if ref_center is not None and meas.center_uv is not None:
            if float(np.linalg.norm(meas.center_uv - ref_center)) > 18.0:
                return False

        if meas.bbox_xyxy is not None and bbox_iou > 0.0:
            return bbox_iou >= 0.55 and bbox_ratio_delta <= 0.12

        return True

    def _blend_hist_ref(
        self,
        current: Optional[np.ndarray],
        new_value: Optional[np.ndarray],
        confidence: float,
        allow_update: bool,
    ) -> Optional[np.ndarray]:
        if not allow_update or new_value is None or confidence <= 0.0:
            return current
        vec = np.asarray(new_value, dtype=np.float32).reshape(-1)
        if vec.size == 0 or not np.all(np.isfinite(vec)):
            return current
        s = float(np.sum(vec))
        if s <= 1e-6:
            return current
        vec = vec / s
        if current is None or current.shape != vec.shape:
            return vec.copy()
        out = (1.0 - self.color_hist_alpha) * current + self.color_hist_alpha * vec
        so = float(np.sum(out))
        return out / so if so > 1e-6 else vec.copy()

    def _history_ready(self) -> bool:
        return (
            len(self.selected_range_hist) >= self.selected_history_min_count
            and len(self.selected_bearing_hist) >= self.selected_history_min_count
            and len(self.selected_bbox_w_hist) >= self.selected_history_min_count
            and len(self.selected_bbox_h_hist) >= self.selected_history_min_count
            and len(self.selected_center_hist) >= self.selected_history_min_count
        )

    @staticmethod
    def _hist_mean(hist: Deque[float], fallback: float) -> float:
        if not hist:
            return fallback
        return float(sum(hist) / len(hist))

    def _hist_angle_mean(self, hist: Deque[float], fallback: float) -> float:
        if not hist:
            return fallback
        s = sum(math.sin(a) for a in hist)
        c = sum(math.cos(a) for a in hist)
        if abs(s) < 1e-6 and abs(c) < 1e-6:
            return fallback
        return math.atan2(s, c)

    @staticmethod
    def _bbox_iou(a: Tuple[int, int, int, int], b: Tuple[int, int, int, int]) -> float:
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
        ix1 = max(ax1, bx1)
        iy1 = max(ay1, by1)
        ix2 = min(ax2, bx2)
        iy2 = min(ay2, by2)
        iw = max(0, ix2 - ix1)
        ih = max(0, iy2 - iy1)
        inter = float(iw * ih)
        area_a = float(max(0, ax2 - ax1) * max(0, ay2 - ay1))
        area_b = float(max(0, bx2 - bx1) * max(0, by2 - by1))
        union = area_a + area_b - inter
        if union <= 1e-6:
            return 0.0
        return inter / union

    def _selected_target_publishable(self, now_sec: float, tr: Track) -> bool:
        if tr.track_id != self.selected_track_id:
            return False
        if tr.state == TrackState.DELETED:
            return False
        if self.target_lock_state == TargetLockState.REACQUIRE:
            return False
        if self._selected_contradictory_candidate_active(now_sec, tr):
            return False

        staleness = max(0.0, now_sec - tr.last_update_sec)
        if self.target_lock_state == TargetLockState.LOCKED:
            return staleness <= self.predicted_target_max_age_sec
        if self.target_lock_state == TargetLockState.TEMP_OCCLUDED:
            return self._selected_prediction_allowed(now_sec, tr, self.predicted_target_max_age_sec)
        if self.target_lock_state == TargetLockState.TEMP_LOST:
            return self._selected_prediction_allowed(now_sec, tr, self.predicted_target_temp_lost_max_age_sec)
        return False

    def _suppression_code_for_track(self, now_sec: float, tr: Track) -> int:
        if tr.track_id != self.selected_track_id:
            return int(TrackedPose.PUB_OK)
        if tr.state == TrackState.DELETED:
            return int(TrackedPose.SUPPRESS_INVALID_TARGET)
        if self.target_lock_state == TargetLockState.REACQUIRE:
            return int(TrackedPose.SUPPRESS_REACQUIRE)
        if self._selected_contradictory_candidate_active(now_sec, tr):
            return int(TrackedPose.SUPPRESS_CONTRADICTION)

        staleness = max(0.0, now_sec - tr.last_update_sec)
        if self.target_lock_state in (TargetLockState.TEMP_OCCLUDED, TargetLockState.TEMP_LOST):
            if not self._selected_prediction_allowed(
                now_sec,
                tr,
                self.predicted_target_max_age_sec if self.target_lock_state == TargetLockState.TEMP_OCCLUDED
                else self.predicted_target_temp_lost_max_age_sec,
            ):
                return int(TrackedPose.SUPPRESS_PREDICTED_ONLY)
        elif staleness > self.predicted_target_max_age_sec:
            return int(TrackedPose.SUPPRESS_STALE)
        return int(TrackedPose.PUB_OK)

    def _camera_visible_for_track(self, now_sec: float, tr: Track) -> bool:
        if tr.last_meas_source == 'lidar_only':
            return False
        staleness = max(0.0, now_sec - tr.last_update_sec)
        return staleness <= self.max_msg_age_sec

    @staticmethod
    def _lidar_support_for_track(tr: Track) -> bool:
        return tr.last_meas_source in ('cam_lidar', 'lidar_only')

    @staticmethod
    def _decision_code_for_reason(tag: str) -> int:
        mapping = {
            'selected_clean_match': int(TargetLockDebug.DECISION_CLEAN_MATCH),
            'selected_acceptable_match': int(TargetLockDebug.DECISION_ACCEPTABLE_MATCH),
            'front_occluder_reject': int(TargetLockDebug.DECISION_OCCLUDED_REJECT),
            'temp_occluded_hold': int(TargetLockDebug.DECISION_OCCLUDED_REJECT),
            'selected_ambiguous_match': int(TargetLockDebug.DECISION_AMBIGUOUS_REJECT),
            'reacquire_reject_hard_hold': int(TargetLockDebug.DECISION_OCCLUDED_REJECT),
            'reacquire_reject_outside_window': int(TargetLockDebug.DECISION_OCCLUDED_REJECT),
            'reacquire_reject_ambiguous': int(TargetLockDebug.DECISION_AMBIGUOUS_REJECT),
            'reacquire_reject_contradictory': int(TargetLockDebug.DECISION_AMBIGUOUS_REJECT),
            'reacquire_reject_identity': int(TargetLockDebug.DECISION_AMBIGUOUS_REJECT),
            'reacquire_candidate': int(TargetLockDebug.DECISION_REACQUIRE_CANDIDATE),
            'relock_confirmed': int(TargetLockDebug.DECISION_RELOCK_CONFIRMED),
        }
        return mapping.get(tag, int(TargetLockDebug.DECISION_NONE))

    def _selected_prediction_allowed(self, now_sec: float, tr: Track, max_age_sec: float) -> bool:
        staleness = max(0.0, now_sec - tr.last_update_sec)
        if staleness > max_age_sec:
            return False
        if staleness > self.predicted_target_max_horizon_sec:
            return False
        cov = float(max(tr.P[0, 0], tr.P[1, 1])) if tr.P.shape[0] >= 2 else 1e9
        if cov > self.predicted_target_max_covariance:
            return False
        speed = float(np.linalg.norm(tr.x[2:4])) if tr.x.shape[0] >= 4 else 0.0
        if speed > self.predicted_target_max_speed_m_s:
            return False
        return not self._selected_contradictory_candidate_active(now_sec, tr)

    def _selected_is_predicted(self, now_sec: float, tr: Track) -> bool:
        if tr.track_id != self.selected_track_id:
            return False
        return self.target_lock_state in (TargetLockState.TEMP_OCCLUDED, TargetLockState.TEMP_LOST) and self._selected_target_publishable(now_sec, tr)

    def _track_detection_confidence(self, tr: Track) -> float:
        return self._clamp01(tr.track_score)

    def _track_geometry_confidence(self, camera_visible: bool, lidar_support: bool, predicted: bool) -> float:
        if lidar_support and camera_visible:
            return 1.0
        if camera_visible:
            return 0.75
        if lidar_support:
            return 0.60
        return 0.25 if predicted else 0.1

    def _track_visibility_confidence(self, camera_visible: bool, predicted: bool) -> float:
        if camera_visible and not predicted:
            return 1.0
        if camera_visible:
            return 0.7
        if predicted:
            return 0.25
        return 0.0

    def _track_freshness_confidence(self, staleness_sec: float) -> float:
        if staleness_sec <= 0.0:
            return 1.0
        return self._clamp01(1.0 - (staleness_sec / max(self.target_lock_lost_timeout_sec, 1e-3)))

    def _track_combined_confidence(
        self,
        lock_confidence: float,
        identity_confidence: float,
        motion_confidence: float,
        detection_confidence: float,
        geometry_confidence: float,
        visibility_confidence: float,
        freshness_confidence: float,
        contradictory_candidate: bool,
    ) -> float:
        penalty = 0.20 if contradictory_candidate else 0.0
        terms = [
            float(lock_confidence),
            float(identity_confidence),
            float(motion_confidence),
            float(detection_confidence),
            float(geometry_confidence),
            float(visibility_confidence),
            float(freshness_confidence),
        ]
        return self._clamp01((sum(terms) / len(terms)) - penalty)

    @staticmethod
    def _track_valid(pose_x: float, pose_y: float, tr: Track) -> bool:
        return bool(
            tr.state != TrackState.DELETED
            and math.isfinite(float(pose_x))
            and math.isfinite(float(pose_y))
        )

    def _selected_lock_confidence(self, now_sec: float, tr: Track) -> float:
        staleness = max(0.0, now_sec - tr.last_update_sec)
        freshness = max(0.0, 1.0 - staleness / max(self.predicted_target_max_age_sec, 1e-3))
        base = 0.45 * self._clamp01(tr.track_score) + 0.30 * self._selected_identity_confidence(tr) + 0.25 * self._selected_motion_confidence(now_sec, tr)
        if self._selected_is_predicted(now_sec, tr):
            base *= 0.75
        if self._selected_contradictory_candidate_active(now_sec, tr):
            base *= 0.5
        if self.target_lock_state == TargetLockState.REACQUIRE:
            base *= 0.35
        return self._clamp01(base * max(0.4, freshness))

    def _selected_identity_confidence(self, tr: Track) -> float:
        values: List[float] = []
        if tr.conf_app > 0.0:
            values.append(self._clamp01(tr.conf_app))
        if self.selected_body_embedding_ref is not None and tr.last_embedding is not None:
            sim = self._cosine_similarity(self.selected_body_embedding_ref, tr.last_embedding)
            if sim is not None:
                values.append(self._clamp01(0.5 * (sim + 1.0)))
        if self.selected_stable_anchor_embedding is not None and tr.last_embedding is not None:
            sim = self._cosine_similarity(self.selected_stable_anchor_embedding, tr.last_embedding)
            if sim is not None:
                values.append(self._clamp01(0.5 * (sim + 1.0)))
        for emb in self.selected_identity_gallery:
            sim = self._cosine_similarity(emb, tr.last_embedding)
            if sim is not None:
                values.append(self._clamp01(0.5 * (sim + 1.0)))
        if not values:
            return 0.0
        score = float(sum(values) / len(values))
        if tr.last_meas_source == 'lidar_only':
            score -= self.lidar_only_selected_penalty
        if tr.last_embedding is None:
            score -= self.weak_appearance_penalty
        if self.crossover_active:
            score *= 0.8
        return self._clamp01(score)

    def _selected_motion_confidence(self, now_sec: float, tr: Track) -> float:
        staleness = max(0.0, now_sec - tr.last_update_sec)
        freshness = max(0.0, 1.0 - staleness / max(self.predicted_target_max_age_sec, 1e-3))
        cov = float(max(tr.P[0, 0], tr.P[1, 1])) if tr.P.shape[0] >= 2 else self.predicted_target_max_covariance
        cov_score = max(0.0, 1.0 - cov / max(self.predicted_target_max_covariance, 1e-3))
        speed = float(np.linalg.norm(tr.x[2:4])) if tr.x.shape[0] >= 4 else 0.0
        speed_score = max(0.0, 1.0 - speed / max(self.predicted_target_max_speed_m_s, 1e-3))
        score = 0.45 * self._clamp01(tr.conf_motion) + 0.35 * freshness + 0.20 * max(cov_score, speed_score)
        if self._selected_is_predicted(now_sec, tr):
            score *= 0.85
        return self._clamp01(score)

    def _selected_contradictory_candidate_active(self, now_sec: float, tr: Track) -> bool:
        del now_sec
        if tr.track_id != self.selected_track_id:
            return False
        if self.crossover_active:
            return True
        second = self._selected_last_second_score
        best = self._selected_last_best_score
        if second < 0.0 or best < 0.0:
            return False
        return (best - second) < self.contradictory_candidate_margin

    @staticmethod
    def _track_state_to_msg(state: TrackState) -> int:
        mapping = {
            TrackState.TENTATIVE: 0,
            TrackState.CONFIRMED: 1,
            TrackState.OCCLUDED: 2,
            TrackState.LOST: 3,
            TrackState.DELETED: 4,
        }
        return mapping.get(state, 4)

    @staticmethod
    def _lock_state_to_msg(state: TargetLockState) -> int:
        mapping = {
            TargetLockState.NONE: 0,
            TargetLockState.LOCKED: 1,
            TargetLockState.TEMP_OCCLUDED: 2,
            TargetLockState.TEMP_LOST: 3,
            TargetLockState.REACQUIRE: 4,
        }
        return mapping.get(state, 0)

    @staticmethod
    def _clamp01(value: float) -> float:
        return max(0.0, min(1.0, value))

    @staticmethod
    def _cosine_similarity(a: Optional[np.ndarray], b: Optional[np.ndarray]) -> Optional[float]:
        if a is None or b is None:
            return None
        va = np.asarray(a, dtype=np.float32).reshape(-1)
        vb = np.asarray(b, dtype=np.float32).reshape(-1)
        if va.size == 0 or vb.size == 0 or va.shape != vb.shape:
            return None
        na = float(np.linalg.norm(va))
        nb = float(np.linalg.norm(vb))
        if na <= 1e-6 or nb <= 1e-6:
            return None
        return max(-1.0, min(1.0, float(np.dot(va, vb) / (na * nb))))

    @staticmethod
    def _hist_similarity(a: Optional[np.ndarray], b: Optional[np.ndarray]) -> Optional[float]:
        if a is None or b is None:
            return None
        va = np.asarray(a, dtype=np.float32).reshape(-1)
        vb = np.asarray(b, dtype=np.float32).reshape(-1)
        if va.size == 0 or vb.size == 0 or va.shape != vb.shape:
            return None
        va = np.clip(va, 0.0, None)
        vb = np.clip(vb, 0.0, None)
        sa = float(np.sum(va))
        sb = float(np.sum(vb))
        if sa <= 1e-6 or sb <= 1e-6:
            return None
        va /= sa
        vb /= sb
        return float(np.sum(np.minimum(va, vb)))

    def _score_reacquire_candidate(self, meas: FusedMeasurement) -> Tuple[float, Dict[str, float]]:
        app_cost = None
        if self.selected_track_id is not None:
            tr = self.tracks.get(self.selected_track_id)
            if tr is not None:
                app_cost = self._appearance_cost(tr, meas.embedding)
        appearance_score = 0.5 if app_cost is None else max(0.0, 1.0 - min(1.0, float(app_cost)))
        if meas.embedding is None or meas.body_reid_confidence <= 0.0:
            appearance_score = max(0.0, appearance_score - self.weak_appearance_penalty)
        if meas.source == 'lidar_only':
            appearance_score = max(0.0, appearance_score - self.lidar_only_selected_penalty)
        if meas.continuity_only:
            appearance_score *= 0.65
        appearance_score = min(appearance_score, self._clamp01(meas.identity_quality if meas.identity_quality > 0.0 else appearance_score))
        if self.selected_body_embedding_ref is not None and meas.embedding is not None:
            cos = self._cosine_similarity(self.selected_body_embedding_ref, meas.embedding)
            if cos is not None:
                appearance_score = max(appearance_score, 0.5 * (cos + 1.0))

        range_ref = self._hist_mean(self.selected_range_hist, float(meas.range_m))
        range_score = max(0.0, 1.0 - abs(float(meas.range_m) - range_ref) / max(self.track_position_gate_m, 1e-3))

        bearing_ref = self._hist_angle_mean(self.selected_bearing_hist, float(meas.bearing_rad))
        bearing_score = max(
            0.0,
            1.0 - abs(self._angle_diff(float(meas.bearing_rad), bearing_ref)) / max(math.radians(self.selected_bearing_gate_deg), 1e-6),
        )

        ref_w = self._hist_mean(self.selected_bbox_w_hist, max(float(meas.bbox_w_px), 1.0))
        ref_h = self._hist_mean(self.selected_bbox_h_hist, max(float(meas.bbox_h_px), 1.0))
        bbox_score = max(
            0.0,
            1.0 - 0.5 * (
                abs(float(meas.bbox_w_px) - ref_w) / max(ref_w, 1.0)
                + abs(float(meas.bbox_h_px) - ref_h) / max(ref_h, 1.0)
            ) / max(self.bbox_size_change_gate, 1e-3)
        )

        center_score = 0.5
        if meas.center_uv is not None and self.selected_center_hist:
            ref_center = np.mean(np.stack(list(self.selected_center_hist), axis=0), axis=0)
            center_score = max(0.0, 1.0 - float(np.linalg.norm(meas.center_uv - ref_center)) / max(self.selected_center_corridor_px, 1.0))

        torso_score = self._hist_similarity(self.selected_torso_hist_ref, meas.torso_hist)
        upper_score = self._hist_similarity(self.selected_upper_body_hist_ref, meas.upper_body_hist)
        lower_score = self._hist_similarity(self.selected_lower_body_hist_ref, meas.lower_body_hist)
        face_score = None
        if self.selected_face_embedding_ref is not None and meas.face_embedding is not None:
            cos = self._cosine_similarity(self.selected_face_embedding_ref, meas.face_embedding)
            if cos is not None:
                face_score = 0.5 * (cos + 1.0)

        valid_identity_scores: List[float] = [appearance_score]
        if torso_score is not None:
            valid_identity_scores.append(torso_score)
        if upper_score is not None:
            valid_identity_scores.append(upper_score)
        if lower_score is not None:
            valid_identity_scores.append(lower_score)
        if face_score is not None:
            valid_identity_scores.append(face_score)

        identity_score = float(sum(valid_identity_scores) / len(valid_identity_scores))
        contradiction = False
        if valid_identity_scores:
            contradiction = (max(valid_identity_scores) - min(valid_identity_scores)) > self.contradiction_threshold

        score = (
            0.25 * appearance_score
            + 0.25 * range_score
            + 0.20 * bearing_score
            + 0.10 * bbox_score
            + 0.10 * center_score
            + 0.10 * identity_score
        )
        return score, {
            'appearance': appearance_score,
            'range': range_score,
            'bearing': bearing_score,
            'bbox': bbox_score,
            'center': center_score,
            'torso': torso_score if torso_score is not None else -1.0,
            'upper': upper_score if upper_score is not None else -1.0,
            'lower': lower_score if lower_score is not None else -1.0,
            'face': face_score if face_score is not None else -1.0,
            'identity': identity_score,
            'contradictory': 1.0 if contradiction else 0.0,
        }

    def _find_track_for_measurement(self, meas: FusedMeasurement, exclude_tid: Optional[int] = None) -> Optional[int]:
        candidates: List[Tuple[float, int]] = []
        for tid, tr in self.tracks.items():
            if tid == exclude_tid or tr.state == TrackState.DELETED:
                continue
            if tr.state not in (TrackState.CONFIRMED, TrackState.OCCLUDED, TrackState.LOST):
                continue
            dxy = float(np.linalg.norm(tr.pos() - meas.pos_xy))
            if dxy > self.track_position_gate_m:
                continue
            score = dxy / max(self.track_position_gate_m, 1e-6)
            app_cost = self._appearance_cost(tr, meas.embedding)
            if app_cost is not None:
                score -= 0.15 * max(0.0, 1.0 - min(1.0, app_cost))
            if self.selected_track_id is not None and tid == self.selected_track_id and dxy <= (0.25 * self.track_position_gate_m):
                score -= 0.05
            candidates.append((score, tid))
        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0])
        return candidates[0][1]

    def _selected_learning_allowed(self, meas: FusedMeasurement) -> bool:
        if self.target_lock_state != TargetLockState.LOCKED:
            return False
        if self.crossover_active:
            return False
        if self.selected_embedding_frozen:
            return False
        tr = self.tracks.get(self.selected_track_id) if self.selected_track_id is not None else None
        if tr is not None and self._selected_contradictory_candidate_active(self._now_sec(), tr):
            return False
        if (self._now_sec() - self.selected_last_seen_sec) < self.identity_freeze_after_relock_sec and self.post_relock_protection_until_sec > 0.0:
            return False
        if meas.source == 'lidar_only' or meas.embedding is None:
            return False
        if meas.body_reid_confidence < self.embedding_update_min_conf:
            return False
        return True

    def _update_selected_identity_gallery(self, embedding: np.ndarray):
        if embedding is None:
            return
        if self.selected_identity_gallery:
            for emb in self.selected_identity_gallery:
                sim = self._cosine_similarity(emb, embedding)
                if sim is not None and (1.0 - (0.5 * (sim + 1.0))) < self.gallery_min_diversity_distance:
                    return
        self.selected_identity_gallery.append(embedding)
        if len(self.selected_identity_gallery) > self.gallery_max_size:
            self.selected_identity_gallery = self.selected_identity_gallery[-self.gallery_max_size:]

    # ---------------------- Target selection + outputs ----------------------
    def _auto_select_if_needed(self, now_sec: float):
        if self.selected_track_id is not None:
            tr = self.tracks.get(self.selected_track_id)
            if tr is not None and tr.state in (TrackState.CONFIRMED, TrackState.OCCLUDED, TrackState.LOST):
                return
            self.selected_track_id = None
            self._reset_selected_lock_state()

        if not self.auto_select_nearest_on_start or self.selected_track_id is not None:
            return

        candidates = [t for t in self.tracks.values() if t.state == TrackState.CONFIRMED]
        if not candidates:
            return

        best = min(candidates, key=lambda t: float(np.linalg.norm(t.pos())))
        self.selected_track_id = best.track_id
        self.selected_last_seen_sec = now_sec
        self.selected_last_clean_seen_sec = now_sec
        self.target_lock_state = TargetLockState.LOCKED
        self._clear_selected_histories()
        self.get_logger().info(f'[TrackManager] Auto-selected track_id={best.track_id}')

    def _publish_outputs(self, now_sec: float):
        self.pub_tracks_poses.publish(self._make_tracks_pose_array())
        self.pub_tracks_state.publish(self._make_tracks_state(now_sec))
        self.pub_tracks_markers.publish(self._make_tracks_markers())
        self.pub_target_lock_debug.publish(self._make_target_lock_debug(now_sec))

        if self.publish_debug_markers:
            self.pub_debug.publish(self._make_debug_markers())

        if not self.enable_target_lock_publish:
            return

        self.pub_target_marker.publish(self._make_target_marker(now_sec))

        # publish facing bool for selected target (default false)
        facing_msg = Bool()
        facing_msg.data = False

        # publish selected target pose
        if self.selected_track_id is None:
            self.pub_target_facing.publish(facing_msg)
            return
        tr = self.tracks.get(self.selected_track_id)
        if tr is None:
            self.pub_target_facing.publish(facing_msg)
            return
        if tr.state not in (TrackState.CONFIRMED, TrackState.OCCLUDED, TrackState.LOST):
            self.pub_target_facing.publish(facing_msg)
            return
        if self._selected_target_publishable(now_sec, tr):
            facing_msg.data = bool(self._track_facing_robot(tr))
            self.pub_target_facing.publish(facing_msg)
        else:
            self.pub_target_facing.publish(facing_msg)
        self.pub_target.publish(self._make_selected_target_msg(now_sec, tr))

    @staticmethod
    def _track_facing01(tr: Track) -> float:
        return max(0.0, min(1.0, float(tr.last_facing01)))

    def _track_facing_robot(self, tr: Track) -> bool:
        return bool(self._track_facing01(tr) >= 0.5)

    def _make_target_lock_debug(self, now_sec: float) -> TargetLockDebug:
        msg = TargetLockDebug()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.track_frame
        msg.target_lock_state = str(self.target_lock_state.value)
        msg.selected_track_id = int(self.selected_track_id or 0)
        msg.relock_candidate_track_id = int(self._selected_candidate_track_id or 0)
        msg.reacquire_confirm_count = int(self.target_reacquire_count)
        msg.crossover_active = bool(self.crossover_active)
        msg.post_relock_protection_active = bool(self._post_relock_protection_active(now_sec))
        selected = self.tracks.get(self.selected_track_id) if self.selected_track_id is not None else None
        msg.publishable = bool(selected is not None and self._selected_target_publishable(now_sec, selected))
        msg.contradictory_candidate = bool(selected is not None and self._selected_contradictory_candidate_active(now_sec, selected))
        msg.rejection_reason_code = str(self._selected_last_decision_reason)
        msg.decision_code = int(self._selected_last_decision_code)
        msg.suppression_code = int(self._suppression_code_for_track(now_sec, selected)) if selected is not None else int(TrackedPose.SUPPRESS_INVALID_TARGET)
        msg.best_score = float(self._selected_last_best_score)
        msg.second_score = float(self._selected_last_second_score)
        msg.identity_agreement_score = float(self._selected_last_identity_score)
        return msg

    # ---------------------- PoseArray & Markers ----------------------
    def _make_tracks_pose_array(self) -> PoseArray:
        pa = PoseArray()
        pa.header.frame_id = self.track_frame
        pa.header.stamp = self.get_clock().now().to_msg()

        for tr in self._iter_visible_tracks():
            p = Pose()
            p.position.x = float(tr.x[0])
            p.position.y = float(tr.x[1])
            p.position.z = 0.0
            p.orientation.w = 1.0
            pa.poses.append(p)
        return pa

    def _make_tracks_state(self, now_sec: float) -> TrackedPoseArray:
        msg = TrackedPoseArray()
        msg.header.frame_id = self.track_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        for tr in self._iter_visible_tracks():
            tp = TrackedPose()
            tp.header.frame_id = self.track_frame
            tp.header.stamp = msg.header.stamp
            pose_x = float(tr.x[0])
            pose_y = float(tr.x[1])
            staleness_sec = float(max(0.0, now_sec - tr.last_update_sec))
            predicted = bool(self._selected_is_predicted(now_sec, tr))
            contradictory_candidate = bool(self._selected_contradictory_candidate_active(now_sec, tr))
            camera_visible = bool(self._camera_visible_for_track(now_sec, tr))
            lidar_support = bool(self._lidar_support_for_track(tr))
            lock_confidence = float(self._selected_lock_confidence(now_sec, tr))
            identity_confidence = float(self._selected_identity_confidence(tr))
            motion_confidence = float(self._selected_motion_confidence(now_sec, tr))
            detection_confidence = float(self._track_detection_confidence(tr))
            geometry_confidence = float(self._track_geometry_confidence(camera_visible, lidar_support, predicted))
            visibility_confidence = float(self._track_visibility_confidence(camera_visible, predicted))
            freshness_confidence = float(self._track_freshness_confidence(staleness_sec))
            combined_confidence = float(self._track_combined_confidence(
                lock_confidence,
                identity_confidence,
                motion_confidence,
                detection_confidence,
                geometry_confidence,
                visibility_confidence,
                freshness_confidence,
                contradictory_candidate,
            ))

            tp.track_id = int(tr.track_id)
            tp.pose.position.x = pose_x
            tp.pose.position.y = pose_y
            tp.pose.position.z = 0.0
            tp.pose.orientation.w = 1.0
            tp.track_state = int(self._track_state_to_msg(tr.state))
            tp.track_score = float(tr.track_score)
            tp.conf_app = float(tr.conf_app)
            tp.conf_motion = float(tr.conf_motion)
            tp.lock_confidence = lock_confidence
            tp.detection_confidence = detection_confidence
            tp.geometry_confidence = geometry_confidence
            tp.identity_confidence = identity_confidence
            tp.motion_confidence = motion_confidence
            tp.visibility_confidence = visibility_confidence
            tp.freshness_confidence = freshness_confidence
            tp.combined_confidence = combined_confidence
            tp.staleness_sec = staleness_sec
            tp.predicted = predicted
            tp.reacquire_active = bool(tr.track_id == self.selected_track_id and self.target_lock_state == TargetLockState.REACQUIRE)
            tp.contradictory_candidate = contradictory_candidate
            tp.crossover_active = bool(tr.track_id == self.selected_track_id and self.crossover_active)
            tp.selected = bool(tr.track_id == self.selected_track_id)
            tp.valid = bool(self._track_valid(pose_x, pose_y, tr))
            tp.publishable = bool(tp.selected and self._selected_target_publishable(now_sec, tr))
            tp.camera_visible = camera_visible
            tp.lidar_support = lidar_support
            tp.facing01 = float(self._track_facing01(tr))
            tp.facing_robot = bool(self._track_facing_robot(tr))
            tp.lock_state = int(self._lock_state_to_msg(self.target_lock_state if tr.track_id == self.selected_track_id else TargetLockState.NONE))
            tp.suppression_code = int(self._suppression_code_for_track(now_sec, tr))
            msg.tracks.append(tp)
        return msg

    def _make_selected_target_msg(self, now_sec: float, tr: Track) -> TrackedPose:
        msg = TrackedPose()
        msg.header.frame_id = self.track_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        pose_x = float(tr.x[0])
        pose_y = float(tr.x[1])
        staleness_sec = float(max(0.0, now_sec - tr.last_update_sec))
        predicted = bool(self._selected_is_predicted(now_sec, tr))
        contradictory_candidate = bool(self._selected_contradictory_candidate_active(now_sec, tr))
        camera_visible = bool(self._camera_visible_for_track(now_sec, tr))
        lidar_support = bool(self._lidar_support_for_track(tr))
        lock_confidence = float(self._selected_lock_confidence(now_sec, tr))
        identity_confidence = float(self._selected_identity_confidence(tr))
        motion_confidence = float(self._selected_motion_confidence(now_sec, tr))
        detection_confidence = float(self._track_detection_confidence(tr))
        geometry_confidence = float(self._track_geometry_confidence(camera_visible, lidar_support, predicted))
        visibility_confidence = float(self._track_visibility_confidence(camera_visible, predicted))
        freshness_confidence = float(self._track_freshness_confidence(staleness_sec))
        combined_confidence = float(self._track_combined_confidence(
            lock_confidence,
            identity_confidence,
            motion_confidence,
            detection_confidence,
            geometry_confidence,
            visibility_confidence,
            freshness_confidence,
            contradictory_candidate,
        ))
        msg.track_id = int(tr.track_id)
        msg.pose.position.x = pose_x
        msg.pose.position.y = pose_y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        msg.track_state = int(self._track_state_to_msg(tr.state))
        msg.track_score = float(tr.track_score)
        msg.conf_app = float(tr.conf_app)
        msg.conf_motion = float(tr.conf_motion)
        msg.lock_confidence = lock_confidence
        msg.detection_confidence = detection_confidence
        msg.geometry_confidence = geometry_confidence
        msg.identity_confidence = identity_confidence
        msg.motion_confidence = motion_confidence
        msg.visibility_confidence = visibility_confidence
        msg.freshness_confidence = freshness_confidence
        msg.combined_confidence = combined_confidence
        msg.staleness_sec = staleness_sec
        msg.predicted = predicted
        msg.reacquire_active = bool(self.target_lock_state == TargetLockState.REACQUIRE)
        msg.contradictory_candidate = contradictory_candidate
        msg.crossover_active = bool(self.crossover_active)
        msg.selected = True
        msg.valid = bool(self._track_valid(pose_x, pose_y, tr))
        msg.publishable = bool(self._selected_target_publishable(now_sec, tr))
        msg.camera_visible = camera_visible
        msg.lidar_support = lidar_support
        msg.facing01 = float(self._track_facing01(tr))
        msg.facing_robot = bool(self._track_facing_robot(tr))
        msg.lock_state = int(self._lock_state_to_msg(self.target_lock_state))
        msg.suppression_code = int(self._suppression_code_for_track(now_sec, tr))
        return msg

    def _make_tracks_markers(self) -> MarkerArray:
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        lifetime = self._lifetime_msg(self.marker_lifetime_sec)
        idx = 0

        for tr in self._iter_visible_tracks():
            r, g, b = self._track_color(tr)

            # sphere
            m = Marker()
            m.header.frame_id = self.track_frame
            m.header.stamp = stamp
            m.ns = 'person_tracks'
            m.id = idx; idx += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(tr.x[0])
            m.pose.position.y = float(tr.x[1])
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = self.track_marker_scale
            m.scale.y = self.track_marker_scale
            m.scale.z = self.track_marker_scale
            m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.95
            m.lifetime = lifetime
            ma.markers.append(m)

            # velocity arrow (points mode)
            a = Marker()
            a.header.frame_id = self.track_frame
            a.header.stamp = stamp
            a.ns = 'person_tracks_vel'
            a.id = idx; idx += 1
            a.type = Marker.ARROW
            a.action = Marker.ADD
            a.pose.orientation.w = 1.0
            a.scale.x = 0.03
            a.scale.y = 0.06
            a.scale.z = 0.08
            a.color.r = r; a.color.g = g; a.color.b = b; a.color.a = 0.9

            p0 = Pose().position
            p1 = Pose().position
            p0.x = float(tr.x[0]); p0.y = float(tr.x[1]); p0.z = 0.08
            p1.x = float(tr.x[0] + tr.x[2] * 0.35)
            p1.y = float(tr.x[1] + tr.x[3] * 0.35)
            p1.z = 0.08
            a.points = [p0, p1]
            a.lifetime = lifetime
            ma.markers.append(a)

            # text
            t = Marker()
            t.header.frame_id = self.track_frame
            t.header.stamp = stamp
            t.ns = 'person_tracks_text'
            t.id = idx; idx += 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(tr.x[0])
            t.pose.position.y = float(tr.x[1])
            t.pose.position.z = 0.25
            t.pose.orientation.w = 1.0
            t.scale.z = 0.14
            t.color.r = 1.0; t.color.g = 1.0; t.color.b = 1.0; t.color.a = 1.0
            t.text = f'id={tr.track_id} {tr.state.value} s={tr.track_score:.2f} h{tr.hits}/m{tr.misses}'
            t.lifetime = lifetime
            ma.markers.append(t)

        return ma

    def _make_target_marker(self, now_sec: float) -> Marker:
        m = Marker()
        m.header.frame_id = self.track_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'person_target_locked'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.DELETE

        if self.selected_track_id is None:
            return m
        tr = self.tracks.get(self.selected_track_id)
        if tr is None:
            return m
        if tr.state not in (TrackState.CONFIRMED, TrackState.OCCLUDED, TrackState.LOST):
            return m

        m.action = Marker.ADD
        m.pose.position.x = float(tr.x[0])
        m.pose.position.y = float(tr.x[1])
        m.pose.position.z = 0.10
        m.pose.orientation.w = 1.0
        m.scale.x = self.target_marker_scale
        m.scale.y = self.target_marker_scale
        m.scale.z = self.target_marker_scale
        m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.2; m.color.a = 1.0
        m.lifetime = self._lifetime_msg(self.marker_lifetime_sec)
        return m

    def _make_debug_markers(self) -> MarkerArray:
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        lifetime = self._lifetime_msg(self.marker_lifetime_sec)
        idx = 0

        # measurement cubes
        for meas in self.last_fused_measurements:
            m = Marker()
            m.header.frame_id = self.track_frame
            m.header.stamp = stamp
            m.ns = 'person_track_fused_meas'
            m.id = idx; idx += 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(meas.pos_xy[0])
            m.pose.position.y = float(meas.pos_xy[1])
            m.pose.position.z = 0.03
            m.pose.orientation.w = 1.0
            m.scale.x = 0.10
            m.scale.y = 0.10
            m.scale.z = 0.06
            if meas.source == 'cam_lidar':
                m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.1
            else:
                m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0
            m.color.a = 0.8
            m.lifetime = lifetime
            ma.markers.append(m)

            # cam-lidar link (if available)
            if meas.lidar_xy is not None:
                line = Marker()
                line.header.frame_id = self.track_frame
                line.header.stamp = stamp
                line.ns = 'person_track_cam_lidar_links'
                line.id = idx; idx += 1
                line.type = Marker.LINE_LIST
                line.action = Marker.ADD
                line.scale.x = 0.015
                line.color.r = 1.0; line.color.g = 1.0; line.color.b = 0.0; line.color.a = 0.7

                p0 = Pose().position
                p1 = Pose().position
                p0.x = float(meas.camera_xy[0]); p0.y = float(meas.camera_xy[1]); p0.z = 0.02
                p1.x = float(meas.lidar_xy[0]);  p1.y = float(meas.lidar_xy[1]);  p1.z = 0.02
                line.points = [p0, p1]
                line.lifetime = lifetime
                ma.markers.append(line)

        # assignment links track -> measurement
        for tid, midx, _ in self.last_assignments:
            tr = self.tracks.get(tid)
            if tr is None:
                continue
            if not (0 <= midx < len(self.last_fused_measurements)):
                continue
            meas = self.last_fused_measurements[midx]

            line = Marker()
            line.header.frame_id = self.track_frame
            line.header.stamp = stamp
            line.ns = 'person_track_assignments'
            line.id = idx; idx += 1
            line.type = Marker.LINE_LIST
            line.action = Marker.ADD
            line.scale.x = 0.012
            line.color.r = 0.0; line.color.g = 1.0; line.color.b = 1.0; line.color.a = 0.8

            p0 = Pose().position
            p1 = Pose().position
            p0.x = float(tr.x[0]);       p0.y = float(tr.x[1]);       p0.z = 0.15
            p1.x = float(meas.pos_xy[0]); p1.y = float(meas.pos_xy[1]); p1.z = 0.05
            line.points = [p0, p1]
            line.lifetime = lifetime
            ma.markers.append(line)

        return ma

    def _iter_visible_tracks(self) -> List[Track]:
        now = self._now_sec()
        out: List[Track] = []
        for tr in self.tracks.values():
            if tr.state == TrackState.DELETED:
                continue
            if tr.track_id == self.selected_track_id:
                out.append(tr)
                continue
            if tr.state in (TrackState.OCCLUDED, TrackState.LOST) and (now - tr.last_update_sec) > self.target_lock_lost_timeout_sec:
                continue
            out.append(tr)
        return sorted(out, key=lambda t: t.track_id)

    # ---------------------- helpers ----------------------
    def _track_color(self, tr: Track) -> Tuple[float, float, float]:
        if tr.track_id == self.selected_track_id:
            return (1.0, 0.2, 0.2)
        if tr.state == TrackState.CONFIRMED:
            return (0.1, 0.9, 0.2)
        if tr.state == TrackState.TENTATIVE:
            return (1.0, 0.8, 0.1)
        if tr.state == TrackState.OCCLUDED:
            return (0.2, 0.6, 1.0)
        if tr.state == TrackState.LOST:
            return (0.6, 0.6, 0.6)
        return (0.3, 0.3, 0.3)

    @staticmethod
    def _lifetime_msg(sec_f: float) -> DurationMsg:
        d = DurationMsg()
        d.sec = int(sec_f)
        d.nanosec = int(max(0.0, sec_f - int(sec_f)) * 1e9)
        return d

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_to_sec(stamp_msg) -> float:
        return float(stamp_msg.sec) + float(stamp_msg.nanosec) * 1e-9

    @staticmethod
    def _time_msg_to_sec(time_msg) -> float:
        return float(time_msg.sec) + float(time_msg.nanosec) * 1e-9

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d


def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
