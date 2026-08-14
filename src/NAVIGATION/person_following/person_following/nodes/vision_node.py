#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import math
import time
import numpy as np
from typing import Optional

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

from putrabot_interfaces.msg import CamDetection, CamDetections, TrackedPose

from cv_bridge import CvBridge
import cv2

import tf2_ros

# message_filters for RGB+Depth sync
from message_filters import Subscriber, ApproximateTimeSynchronizer

# RViz markers (optional)
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

# Optional: ONNXRuntime (for RetinaFace)
try:
    import onnxruntime as ort
    ONNX_AVAILABLE = True
except Exception:
    ort = None
    ONNX_AVAILABLE = False

from person_following.utils.reid_debug import (
    canonical_reid_reason,
    format_reid_log_fail,
    format_reid_log_ok,
    overlay_reid_reason,
    should_emit_reid_log,
)
from person_following.utils.runtime_config import resolve_onnx_providers, resolve_yolo_device
from person_following.utils.yolo_runtime import resolve_yolo_import


def _resolve_model_path(path_value: str) -> str:
    path_str = str(path_value or '').strip()
    if not path_str:
        return ''
    if os.path.isabs(path_str):
        return path_str
    try:
        pkg_share = get_package_share_directory('person_following')
    except Exception:
        pkg_share = ''
    if path_str.startswith('models/') and pkg_share:
        return os.path.join(pkg_share, path_str)
    if 'NAVIGATION/person_following/models/' in path_str and pkg_share:
        return os.path.join(pkg_share, os.path.basename(path_str))
    return path_str


def _nms_xyxy(boxes: np.ndarray, scores: np.ndarray, iou_thresh: float) -> np.ndarray:
    """
    boxes: (N,4) xyxy
    scores: (N,)
    returns keep indices (np.int64)
    """
    if boxes.size == 0:
        return np.empty((0,), dtype=np.int64)

    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]

    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break

        rest = order[1:]
        xx1 = np.maximum(x1[i], x1[rest])
        yy1 = np.maximum(y1[i], y1[rest])
        xx2 = np.minimum(x2[i], x2[rest])
        yy2 = np.minimum(y2[i], y2[rest])

        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h
        union = areas[i] + areas[rest] - inter + 1e-9
        iou = inter / union

        order = rest[iou <= float(iou_thresh)]

    return np.asarray(keep, dtype=np.int64)


class RetinaFaceOnnx:
    """
    Minimal RetinaFace (loc/conf/landms) ONNX runner + decode + NMS.

    Supports common output layout:
      loc:   (1, N, 4)
      conf:  (1, N, 2) or (1, N, 1)
      landm: (1, N, 10)

    If your ONNX exports decoded boxes already, set output-name params to match,
    or adjust this adapter accordingly.
    """

    def __init__(
        self,
        onnx_path: str,
        providers,
        input_w: int,
        input_h: int,
        conf_thresh: float,
        nms_iou: float,
        out_scores: str = '',
        out_boxes: str = '',
        out_landmarks: str = '',
        logger=None,
    ):
        self.logger = logger
        self.onnx_path = str(onnx_path)
        self.conf_thresh = float(conf_thresh)
        self.nms_iou = float(nms_iou)
        self.out_scores = str(out_scores)
        self.out_boxes = str(out_boxes)
        self.out_landmarks = str(out_landmarks)
        self.providers = list(providers or [])

        self.session = None
        self.input_name = None
        self.input_w = int(input_w)
        self.input_h = int(input_h)
        self.input_layout = 'nchw'

        if not ONNX_AVAILABLE:
            if self.logger:
                self.logger.warn('[VisionNode] onnxruntime not available -> RetinaFace disabled')
            return

        try:
            self.session = ort.InferenceSession(self.onnx_path, providers=self.providers)
            self.input_name = self.session.get_inputs()[0].name

            shp = self.session.get_inputs()[0].shape
            # Common layouts:
            #   NCHW -> [1, 3, H, W]
            #   NHWC -> [1, H, W, 3]
            try:
                if len(shp) == 4:
                    if isinstance(shp[1], int) and shp[1] == 3 and isinstance(shp[2], int) and isinstance(shp[3], int):
                        self.input_layout = 'nchw'
                        self.input_h = int(shp[2])
                        self.input_w = int(shp[3])
                    elif isinstance(shp[-1], int) and shp[-1] == 3 and isinstance(shp[1], int) and isinstance(shp[2], int):
                        self.input_layout = 'nhwc'
                        self.input_h = int(shp[1])
                        self.input_w = int(shp[2])
            except Exception:
                pass

            if self.logger:
                self.logger.info(
                    f'[VisionNode] RetinaFace ONNX loaded: {self.onnx_path} input={self.input_w}x{self.input_h} layout={self.input_layout} providers={self.providers}'
                )
        except Exception as e:
            if self.logger:
                self.logger.warn(f'[VisionNode] Failed to load RetinaFace ONNX ({self.onnx_path}): {e}')
            self.session = None

    @property
    def enabled(self) -> bool:
        return self.session is not None and self.input_name is not None

    def _prior_boxes(self, img_w: int, img_h: int) -> np.ndarray:
        # Standard RetinaFace priors
        min_sizes = [[16, 32], [64, 128], [256, 512]]
        steps = [8, 16, 32]
        priors = []
        for k, step in enumerate(steps):
            f_w = int(math.ceil(img_w / step))
            f_h = int(math.ceil(img_h / step))
            for i in range(f_h):
                for j in range(f_w):
                    cx = (j + 0.5) * step / img_w
                    cy = (i + 0.5) * step / img_h
                    for ms in min_sizes[k]:
                        s_kx = ms / img_w
                        s_ky = ms / img_h
                        priors.append([cx, cy, s_kx, s_ky])
        return np.asarray(priors, dtype=np.float32)

    @staticmethod
    def _decode_boxes(loc: np.ndarray, priors: np.ndarray) -> np.ndarray:
        # loc: (N,4), priors: (N,4) both in [cx,cy,w,h] normalized
        variances = (0.1, 0.2)
        boxes = np.concatenate(
            (
                priors[:, :2] + loc[:, :2] * variances[0] * priors[:, 2:],
                priors[:, 2:] * np.exp(loc[:, 2:] * variances[1]),
            ),
            axis=1,
        )
        # (cx,cy,w,h) -> (x1,y1,x2,y2)
        boxes[:, :2] -= boxes[:, 2:] / 2.0
        boxes[:, 2:] += boxes[:, :2]
        return boxes

    @staticmethod
    def _decode_landmarks(landm: np.ndarray, priors: np.ndarray) -> np.ndarray:
        # landm: (N,10) normalized offsets
        variances = (0.1, 0.2)
        out = np.zeros((landm.shape[0], 10), dtype=np.float32)
        for i in range(5):
            out[:, 2 * i:2 * i + 2] = priors[:, :2] + landm[:, 2 * i:2 * i + 2] * variances[0] * priors[:, 2:]
        return out

    def _select_outputs(self, outputs: list) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        # Try name-based selection first
        if self.out_scores or self.out_boxes or self.out_landmarks:
            name_to_arr = {o.name: arr for o, arr in zip(self.session.get_outputs(), outputs)}
            try:
                conf = name_to_arr[self.out_scores] if self.out_scores else None
                loc = name_to_arr[self.out_boxes] if self.out_boxes else None
                landm = name_to_arr[self.out_landmarks] if self.out_landmarks else None
                if conf is not None and loc is not None and landm is not None:
                    return loc, conf, landm
            except Exception:
                pass

        # Shape-based selection (common loc/conf/landm)
        loc = conf = landm = None
        for arr in outputs:
            if not isinstance(arr, np.ndarray):
                continue
            if arr.ndim == 3 and arr.shape[-1] == 4:
                loc = arr
            elif arr.ndim == 3 and arr.shape[-1] == 10:
                landm = arr
            elif arr.ndim == 3 and (arr.shape[-1] == 2 or arr.shape[-1] == 1):
                conf = arr

        if loc is None or conf is None or landm is None:
            raise RuntimeError('Unable to infer RetinaFace outputs (loc/conf/landm)')
        return loc, conf, landm

    def detect(self, bgr_img: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Returns:
          boxes_xyxy: (M,4) in input-image pixel coords
          scores: (M,)
          landmarks: (M,5,2) in input-image pixel coords
        """
        if not self.enabled:
            return (np.zeros((0, 4), dtype=np.float32), np.zeros((0,), dtype=np.float32), np.zeros((0, 5, 2), dtype=np.float32))

        img_h, img_w = bgr_img.shape[:2]
        inp = cv2.resize(bgr_img, (self.input_w, self.input_h), interpolation=cv2.INTER_LINEAR)
        x = inp.astype(np.float32)
        # common RetinaFace preprocessing (BGR mean subtraction)
        x -= np.array([104.0, 117.0, 123.0], dtype=np.float32)
        if self.input_layout == 'nchw':
            x = np.transpose(x, (2, 0, 1))[None, ...]  # 1x3xHxW
        else:
            x = x[None, ...]  # 1xHxWx3

        t0 = time.perf_counter()
        outs = self.session.run(None, {self.input_name: x})
        if self.logger is not None and hasattr(self.logger, 'info'):
            owner = getattr(self, '_timing_callback', None)
            if owner is not None:
                owner('retinaface', time.perf_counter() - t0)
        loc, conf, landm = self._select_outputs(outs)

        loc = np.asarray(loc).reshape(-1, 4).astype(np.float32)
        conf = np.asarray(conf).reshape(-1, conf.shape[-1]).astype(np.float32)
        landm = np.asarray(landm).reshape(-1, 10).astype(np.float32)

        if conf.shape[1] == 2:
            scores = conf[:, 1]
        else:
            scores = conf[:, 0]

        priors = self._prior_boxes(self.input_w, self.input_h)
        if priors.shape[0] != loc.shape[0]:
            raise RuntimeError(f'Prior count mismatch: priors={priors.shape[0]} loc={loc.shape[0]}')

        boxes = self._decode_boxes(loc, priors)
        lm = self._decode_landmarks(landm, priors)

        # to pixels
        boxes[:, 0] *= self.input_w
        boxes[:, 2] *= self.input_w
        boxes[:, 1] *= self.input_h
        boxes[:, 3] *= self.input_h

        lm = lm.reshape(-1, 5, 2)
        lm[:, :, 0] *= self.input_w
        lm[:, :, 1] *= self.input_h

        # filter + nms
        m = scores >= float(self.conf_thresh)
        boxes = boxes[m]
        scores_f = scores[m]
        lm = lm[m]

        if boxes.size == 0:
            return (np.zeros((0, 4), dtype=np.float32), np.zeros((0,), dtype=np.float32), np.zeros((0, 5, 2), dtype=np.float32))

        keep = _nms_xyxy(boxes, scores_f, self.nms_iou)
        boxes = boxes[keep]
        scores_f = scores_f[keep]
        lm = lm[keep]

        # Map back from input_w/h to original img_w/h
        sx = float(img_w) / float(self.input_w)
        sy = float(img_h) / float(self.input_h)
        boxes[:, [0, 2]] *= sx
        boxes[:, [1, 3]] *= sy
        lm[:, :, 0] *= sx
        lm[:, :, 1] *= sy

        return boxes.astype(np.float32), scores_f.astype(np.float32), lm.astype(np.float32)

class VisionNode(Node):
    """
    VisionNode (Rewritten)

    Subscribes (synced):
      - RGB   : /camera/camera/color/image_raw
      - Depth : /camera/camera/aligned_depth_to_color/image_raw
    Subscribes:
      - CameraInfo: /camera/camera/color/camera_info

    Publishes:
      - /cam_detections          (vision/CamDetections)    -> your custom message
      - /cam_detections_markers  (visualization/MarkerArray) -> RViz visualization

    Notes:
      - Use camera optical frame for pinhole projection:
        camera_color_optical_frame (based on your TF tree)
    """

    def __init__(self):
        super().__init__('vision_node')

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

        # ---------------- Parameters ----------------
        self.declare_parameter('rgb_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')

        # IMPORTANT: optical frame is correct for X=(u-cx)Z/fx
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')

        # For fusion with LiDAR, base_link/base_footprint is best
        self.declare_parameter('global_frame', 'base_footprint')

        self.declare_parameter('yolo_model_path', 'yolo11n.pt')
        self.declare_parameter('yolo_conf_thresh', 0.6)
        self.declare_parameter('yolo_device', 'auto')

        self.declare_parameter('debug_mode', False)     # publish fake detections
        self.declare_parameter('enable_gui', True)      # OpenCV window
        self.declare_parameter('publish_markers', True) # RViz markers
        self.declare_parameter('locked_target_topic', '/person_target_locked')
        self.declare_parameter('gui_lock_match_gate_m', 0.35)
        self.declare_parameter('gui_lock_timeout_sec', 0.5)
        self.declare_parameter('reid_debug_enabled', True)
        self.declare_parameter('reid_debug_log_period_sec', 0.5)
        self.declare_parameter('reid_debug_preview_n', 4)

        # Sync settings
        self.declare_parameter('sync_queue', 10)
        self.declare_parameter('sync_slop_sec', 0.06)   # 60 ms

        # Marker settings
        self.declare_parameter('marker_scale', 0.18)     # meters
        self.declare_parameter('marker_lifetime_sec', 0.5)

        # TF timeout
        self.declare_parameter('tf_timeout_sec', 0.10)
        self.declare_parameter('robot_radius_m', 0.2655)  # example, tune to your robot
        self.robot_radius_m = float(self.get_parameter('robot_radius_m').value)
        self.declare_parameter('depth_roi_scale_w', 0.4)
        self.declare_parameter('depth_roi_scale_h', 0.5)
        self.declare_parameter('min_depth_valid_ratio', 0.15)

        # RetinaFace + Head Pose (optional)
        self.declare_parameter('retinaface_enabled', True)
        self.declare_parameter('retinaface_onnx_path', 'models/RetinaFace_int.onnx')
        self.declare_parameter('retinaface_input_w', 640)
        self.declare_parameter('retinaface_input_h', 640)
        self.declare_parameter('retinaface_conf_thresh', 0.7)
        self.declare_parameter('retinaface_nms_iou', 0.4)
        self.declare_parameter('retinaface_out_scores', '')
        self.declare_parameter('retinaface_out_boxes', '')
        self.declare_parameter('retinaface_out_landmarks', '')
        self.declare_parameter('retinaface_providers', 'CUDAExecutionProvider,CPUExecutionProvider')
        self.declare_parameter('retinaface_every_n_frames', 5)
        self.declare_parameter('runtime_timing_debug', False)

        self.declare_parameter('facing_yaw_thresh_deg', 25.0)
        self.declare_parameter('facing_pitch_thresh_deg', 20.0)
        self.declare_parameter('torso_region_height_ratio', 0.22)
        self.declare_parameter('upper_body_region_height_ratio', 0.18)
        self.declare_parameter('lower_body_region_height_ratio', 0.18)
        self.declare_parameter('hist_visible_ratio_min', 0.60)
        self.declare_parameter('face_embed_min_area_px', 1024.0)
        self.declare_parameter('face_embed_max_yaw_deg', 35.0)
        self.declare_parameter('face_embed_max_pitch_deg', 25.0)
        self.declare_parameter('identity_quality_min_for_update', 0.60)
        self.declare_parameter('crop_blur_max_laplacian', 45.0)
        self.declare_parameter('crop_min_bbox_area_px', 6000.0)
        self.declare_parameter('crop_min_visible_body_ratio', 0.55)
        self.declare_parameter('crop_aspect_ratio_min', 0.25)
        self.declare_parameter('crop_aspect_ratio_max', 0.80)

        # Read params
        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.info_topic = self.get_parameter('camera_info_topic').value

        self.camera_frame = self.get_parameter('camera_frame').value
        self.global_frame = self.get_parameter('global_frame').value

        self.conf_thresh = float(self.get_parameter('yolo_conf_thresh').value)
        self.yolo_device_request = str(self.get_parameter('yolo_device').value)
        self.debug_mode = _param_bool(self.get_parameter('debug_mode').value)
        self.enable_gui = _param_bool(self.get_parameter('enable_gui').value)
        self.publish_markers = _param_bool(self.get_parameter('publish_markers').value)
        self.locked_target_topic = str(self.get_parameter('locked_target_topic').value)
        self.gui_lock_match_gate_m = max(0.05, float(self.get_parameter('gui_lock_match_gate_m').value))
        self.gui_lock_timeout_sec = max(0.05, float(self.get_parameter('gui_lock_timeout_sec').value))
        self.reid_debug_enabled = _param_bool(self.get_parameter('reid_debug_enabled').value)
        self.reid_debug_log_period_sec = float(self.get_parameter('reid_debug_log_period_sec').value)
        self.reid_debug_preview_n = max(1, int(self.get_parameter('reid_debug_preview_n').value))

        self.sync_queue = int(self.get_parameter('sync_queue').value)
        self.sync_slop = float(self.get_parameter('sync_slop_sec').value)

        self.marker_scale = float(self.get_parameter('marker_scale').value)
        self.marker_lifetime_sec = float(self.get_parameter('marker_lifetime_sec').value)
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        self.depth_roi_scale_w = float(self.get_parameter('depth_roi_scale_w').value)
        self.depth_roi_scale_h = float(self.get_parameter('depth_roi_scale_h').value)
        self.min_depth_valid_ratio = float(self.get_parameter('min_depth_valid_ratio').value)

        self.retinaface_enabled = _param_bool(self.get_parameter('retinaface_enabled').value)
        self.retinaface_onnx_path = _resolve_model_path(self.get_parameter('retinaface_onnx_path').value)
        self.retinaface_input_w = int(self.get_parameter('retinaface_input_w').value)
        self.retinaface_input_h = int(self.get_parameter('retinaface_input_h').value)
        self.retinaface_conf_thresh = float(self.get_parameter('retinaface_conf_thresh').value)
        self.retinaface_nms_iou = float(self.get_parameter('retinaface_nms_iou').value)
        self.retinaface_out_scores = str(self.get_parameter('retinaface_out_scores').value)
        self.retinaface_out_boxes = str(self.get_parameter('retinaface_out_boxes').value)
        self.retinaface_out_landmarks = str(self.get_parameter('retinaface_out_landmarks').value)
        self.retinaface_providers_request = self.get_parameter('retinaface_providers').value
        self.retinaface_every_n_frames = max(1, int(self.get_parameter('retinaface_every_n_frames').value))
        self.runtime_timing_debug = _param_bool(self.get_parameter('runtime_timing_debug').value)

        self.facing_yaw_thresh_deg = float(self.get_parameter('facing_yaw_thresh_deg').value)
        self.facing_pitch_thresh_deg = float(self.get_parameter('facing_pitch_thresh_deg').value)
        self.torso_region_height_ratio = float(self.get_parameter('torso_region_height_ratio').value)
        self.upper_body_region_height_ratio = float(self.get_parameter('upper_body_region_height_ratio').value)
        self.lower_body_region_height_ratio = float(self.get_parameter('lower_body_region_height_ratio').value)
        self.hist_visible_ratio_min = float(self.get_parameter('hist_visible_ratio_min').value)
        self.face_embed_min_area_px = float(self.get_parameter('face_embed_min_area_px').value)
        self.face_embed_max_yaw_deg = float(self.get_parameter('face_embed_max_yaw_deg').value)
        self.face_embed_max_pitch_deg = float(self.get_parameter('face_embed_max_pitch_deg').value)
        self.identity_quality_min_for_update = float(self.get_parameter('identity_quality_min_for_update').value)
        self.crop_blur_max_laplacian = float(self.get_parameter('crop_blur_max_laplacian').value)
        self.crop_min_bbox_area_px = float(self.get_parameter('crop_min_bbox_area_px').value)
        self.crop_min_visible_body_ratio = float(self.get_parameter('crop_min_visible_body_ratio').value)
        self.crop_aspect_ratio_min = float(self.get_parameter('crop_aspect_ratio_min').value)
        self.crop_aspect_ratio_max = float(self.get_parameter('crop_aspect_ratio_max').value)

        self._timing_last_log_sec = {'yolo': 0.0, 'retinaface': 0.0}

        self.yolo_import = resolve_yolo_import()
        self.yolo_resolution = resolve_yolo_device(self.yolo_device_request, self.yolo_import.torch_module)
        self.resolved_yolo_device = self.yolo_resolution.resolved
        self.retinaface_resolution = resolve_onnx_providers(self.retinaface_providers_request, ort)
        self.resolved_retinaface_providers = self.retinaface_resolution.resolved
        self.retinaface_runtime_enabled = bool(self.retinaface_enabled and self.retinaface_resolution.enabled)

        # ---------------- YOLO model ----------------
        self.model = None
        if not self.debug_mode and self.yolo_import.available:
            model_path = _resolve_model_path(self.get_parameter('yolo_model_path').value)
            self._log_yolo_runtime(model_path)
            try:
                self.model = self.yolo_import.yolo_class(model_path)
            except Exception as e:
                raise RuntimeError(f'[VisionNode] Failed to load required YOLO model ({model_path}): {e}') from e
        elif not self.debug_mode and not self.yolo_import.available:
            raise RuntimeError(f'[VisionNode] YOLO runtime unavailable: {self.yolo_import.reason}')

        # ---------------- ROS utils ----------------
        self.bridge = CvBridge()
        self.cam_info = None
        self._prev_center_time = 0.0
        self._prev_centers = []
        self.latest_locked_target: Optional[TrackedPose] = None
        self.latest_locked_target_rx_sec = 0.0
        self.latched_locked_target: Optional[TrackedPose] = None
        self.synced_frame_index = 0
        self._cached_face_results = []
        self._reid_debug_last_emit_sec = {}
        self._reid_debug_last_status = {}

        # ---------------- RetinaFace model ----------------
        self.retina = None
        if self.retinaface_enabled:
            self._log_onnx_runtime('RetinaFace', self.retinaface_resolution, optional=True)
        if self.retinaface_runtime_enabled:
            self.retina = RetinaFaceOnnx(
                onnx_path=self.retinaface_onnx_path,
                providers=self.resolved_retinaface_providers,
                input_w=self.retinaface_input_w,
                input_h=self.retinaface_input_h,
                conf_thresh=self.retinaface_conf_thresh,
                nms_iou=self.retinaface_nms_iou,
                out_scores=self.retinaface_out_scores,
                out_boxes=self.retinaface_out_boxes,
                out_landmarks=self.retinaface_out_landmarks,
                logger=self.get_logger(),
            )
            if self.retina is not None and not self.retina.enabled:
                self.retina = None
                self.retinaface_runtime_enabled = False
                self.get_logger().warn('[VisionNode] RetinaFace disabled after load failure')
            elif self.retina is not None:
                self.retina._timing_callback = self._maybe_log_timing
        elif self.retinaface_enabled:
            self.get_logger().warn('[VisionNode] RetinaFace disabled before load')

        # Sensor QoS (RealSense usually BEST_EFFORT)
        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = ReliabilityPolicy.BEST_EFFORT

        # CameraInfo (store intrinsics)
        self.sub_info = self.create_subscription(CameraInfo, self.info_topic, self.info_cb, qos_sensor)
        self.sub_locked_target = self.create_subscription(TrackedPose, self.locked_target_topic, self.locked_target_cb, 10)

        # Sync RGB + Depth
        self.rgb_sub = Subscriber(self, Image, self.rgb_topic, qos_profile=qos_sensor)
        self.depth_sub = Subscriber(self, Image, self.depth_topic, qos_profile=qos_sensor)

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=self.sync_queue,
            slop=self.sync_slop
        )
        self.sync.registerCallback(self.synced_cb)

        # Publishers
        self.pub_dets = self.create_publisher(CamDetections, '/cam_detections', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/cam_detections_markers', 10)

        # TF buffer/listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Heartbeat (once per second)
        self.timer = self.create_timer(1.0, self.heartbeat_cb)

        mode_str = "DEBUG" if self.debug_mode else "FULL"
        self.get_logger().info(f'[VisionNode] Initialized ({mode_str})')
        self.get_logger().info(f'  RGB   : {self.rgb_topic}')
        self.get_logger().info(f'  Depth : {self.depth_topic}')
        self.get_logger().info(f'  Info  : {self.info_topic}')
        self.get_logger().info(f'  camera_frame: {self.camera_frame}')
        self.get_logger().info(f'  global_frame: {self.global_frame}')
        self.get_logger().info(f'  GUI: {self.enable_gui}, Markers: {self.publish_markers}')
        self.get_logger().info(f'  locked_target_topic: {self.locked_target_topic}')
        self.get_logger().info(
            f'  reid_debug_enabled={self.reid_debug_enabled}, '
            f'reid_debug_log_period_sec={self.reid_debug_log_period_sec:.2f}, '
            f'reid_debug_preview_n={self.reid_debug_preview_n}'
        )
        self.get_logger().info(
            '[VisionNode] Runtime summary: '
            f'YOLO={"debug" if self.debug_mode else self.resolved_yolo_device}, '
            f'RetinaFace={"enabled" if self.retinaface_runtime_enabled and self.retina is not None else "disabled"} '
            f'on {self.resolved_retinaface_providers if self.resolved_retinaface_providers else "n/a"}'
        )
        self.get_logger().info(f'  retinaface_every_n_frames: {self.retinaface_every_n_frames}')

    # ==========================================================
    # Heartbeat
    # ==========================================================
    def heartbeat_cb(self):
        self.get_logger().info(
            f'heartbeat: cam_info={"yes" if self.cam_info else "no"}, '
            f'yolo={"yes" if (self.model is not None or self.debug_mode) else "no"}, '
            f'retinaface={"yes" if self.retina is not None else "no"}'
        )

    def _log_yolo_runtime(self, model_path: str):
        self.get_logger().info(
            f'[VisionNode] YOLO requested={self.yolo_resolution.requested} '
            f'resolved={self.resolved_yolo_device} '
            f'cuda_available={self.yolo_resolution.cuda_available} '
            f'cuda_device_count={self.yolo_resolution.cuda_device_count} '
            f'reason="{self.yolo_resolution.reason}" model="{model_path}"'
        )

    def _log_onnx_runtime(self, label: str, resolution, optional: bool):
        self.get_logger().info(
            f'[VisionNode] {label} requested={resolution.requested} '
            f'available={resolution.available} '
            f'resolved={resolution.resolved} '
            f'optional={optional} '
            f'reason="{resolution.reason}"'
        )

    def _maybe_log_timing(self, key: str, elapsed_sec: float):
        if not self.runtime_timing_debug:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        last = self._timing_last_log_sec.get(key, 0.0)
        if (now - last) < 2.0:
            return
        self._timing_last_log_sec[key] = now
        self.get_logger().info(f'[VisionNode][timing] {key}={elapsed_sec * 1000.0:.1f}ms')

    # ==========================================================
    # Callbacks
    # ==========================================================
    def info_cb(self, msg: CameraInfo):
        self.cam_info = msg

    def locked_target_cb(self, msg: TrackedPose):
        self.latest_locked_target = msg
        self.latest_locked_target_rx_sec = self.get_clock().now().nanoseconds * 1e-9
        if int(msg.track_id) <= 0 or (not bool(msg.selected)) or (not bool(msg.valid)):
            self.latched_locked_target = None
            return
        self.latched_locked_target = msg

    def synced_cb(self, rgb_msg: Image, depth_msg: Image):
        # Prepare output message
        dets_msg = CamDetections()
        dets_msg.stamp = rgb_msg.header.stamp

        # DEBUG MODE: publish 1 fake detection at origin
        if self.debug_mode:
            det = CamDetection()
            det.x = 0.0
            det.y = 0.0
            det.z = 0.0
            det.confidence = 1.0
            det.embedding = []
            dets_msg.detections.append(det)

            self.pub_dets.publish(dets_msg)

            if self.publish_markers:
                self.pub_markers.publish(self.make_markers(dets_msg, rgb_msg.header.stamp))

            if self.enable_gui:
                try:
                    rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
                    cv2.imshow("VisionNode - DEBUG", rgb_img)
                    cv2.waitKey(1)
                except Exception:
                    pass
            return

        # Must have intrinsics + YOLO
        if self.cam_info is None or self.model is None:
            return

        # Convert images
        try:
            rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'[VisionNode] RGB cv_bridge failed: {e}')
            return

        try:
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'[VisionNode] Depth cv_bridge failed: {e}')
            return

        fx = float(self.cam_info.k[0])
        fy = float(self.cam_info.k[4])
        cx = float(self.cam_info.k[2])
        cy = float(self.cam_info.k[5])
        self.synced_frame_index += 1

        detections = self.run_yolo(rgb_img)

        # Show empty GUI frame if no detections
        if not detections:
            if self.enable_gui:
                cv2.imshow("VisionNode", rgb_img)
                cv2.waitKey(1)
            self.pub_dets.publish(dets_msg)
            if self.publish_markers:
                self.pub_markers.publish(self.make_markers(dets_msg, rgb_msg.header.stamp))
            return

        h, w = depth_img.shape[:2]

        # Use the RGB timestamp for TF lookup
        stamp = Time.from_msg(rgb_msg.header.stamp)

        # For head pose, build camera matrix once
        camK = None
        if self.retina is not None:
            try:
                fx = float(self.cam_info.k[0])
                fy = float(self.cam_info.k[4])
                cx = float(self.cam_info.k[2])
                cy = float(self.cam_info.k[5])
                camK = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
            except Exception:
                camK = None
        run_retina_this_frame = (
            self.retina is not None
            and camK is not None
            and (
                self.synced_frame_index == 1
                or ((self.synced_frame_index - 1) % self.retinaface_every_n_frames) == 0
            )
        )
        next_face_cache = []

        for (xmin, ymin, xmax, ymax, conf) in detections:
            u = int((xmin + xmax) * 0.5)
            v = int((ymin + ymax) * 0.5)

            if not (0 <= u < w and 0 <= v < h):
                continue

            Z, depth_valid_ratio = self.depth_median_in_bbox(
                depth_img=depth_img,
                depth_encoding=depth_msg.encoding,
                bbox_xyxy=(int(xmin), int(ymin), int(xmax), int(ymax)),
            )
            if Z is None or Z <= 0.0:
                continue
            weak_depth = depth_valid_ratio < self.min_depth_valid_ratio
            
            self.get_logger().info(f"camera depth Z={Z:.2f} m at pixel ({u}, {v})")

            # Back-project (camera optical coordinates)
            Xc = (u - cx) * Z / fx
            Yc = (v - cy) * Z / fy
            Zc = Z

            # Transform camera -> global_frame
            pg = self.transform_point_camera_to_global(Xc, Yc, Zc, stamp)
            if pg is None:
                continue

            det = CamDetection()
            det.x = float(pg[0])
            det.y = float(pg[1])
            det.z = float(pg[2])
            det.confidence = float(conf)
            det.x1 = int(xmin)
            det.y1 = int(ymin)
            det.x2 = int(xmax)
            det.y2 = int(ymax)
            det.depth_median = float(Z)
            det.depth_valid_ratio = float(depth_valid_ratio)
            det.center_u = float(u)
            det.center_v = float(v)
            det.body_reid_embedding = []
            det.body_reid_valid = False
            det.body_reid_confidence = 0.0
            det.face_embedding = []
            det.face_embedding_valid = False
            det.face_embedding_confidence = 0.0
            det.detection_quality = 0.0
            det.identity_quality = 0.0
            det.continuity_only = False
            det.image_center_vel_u = 0.0
            det.image_center_vel_v = 0.0
            det.image_center_confidence = 0.0

            # distance in robot frame
            planar_dist = math.hypot(det.x, det.y)                 # center (base frame origin) to target
            clearance_dist = max(0.0, planar_dist - self.robot_radius_m)  # robot body edge to target
            dist_3d = math.sqrt(det.x**2 + det.y**2 + det.z**2)

            self.get_logger().info(
                f"center={planar_dist:.2f}m, clearance={clearance_dist:.2f}m, "
                f"x={det.x:.2f}, y={det.y:.2f}, z={det.z:.2f}"
            )



            # Optional debug payload in embedding: [u, v, Z]
            det.embedding = [float(u), float(v), float(Z)]

            # RetinaFace + head pose (optional): append [yaw, pitch, roll, facing01]
            yaw_deg = pitch_deg = roll_deg = facing01 = 0.0
            face_result = None
            if self.retina is not None and camK is not None:
                face_result = self._estimate_facing_from_person_bbox(
                    rgb_img=rgb_img,
                    person_bbox_xyxy=(float(xmin), float(ymin), float(xmax), float(ymax)),
                    camK=camK,
                    run_inference=run_retina_this_frame,
                )
                yaw_deg = float(face_result['yaw_deg'])
                pitch_deg = float(face_result['pitch_deg'])
                roll_deg = float(face_result['roll_deg'])
                facing01 = float(face_result['facing01'])
                det.embedding.extend([yaw_deg, pitch_deg, roll_deg, facing01])
                if face_result['cacheable']:
                    next_face_cache.append(face_result['cache_entry'])
            det.head_yaw_deg = float(yaw_deg)
            det.head_pitch_deg = float(pitch_deg)
            det.head_roll_deg = float(roll_deg)
            det.facing01 = float(facing01)
            det.continuity_only = bool(weak_depth)

            torso_hist, torso_valid, torso_conf = self._extract_region_hist(
                rgb_img,
                (int(xmin), int(ymin), int(xmax), int(ymax)),
                start_ratio=0.20,
                height_ratio=self.torso_region_height_ratio,
            )
            upper_hist, upper_valid, upper_conf = self._extract_region_hist(
                rgb_img,
                (int(xmin), int(ymin), int(xmax), int(ymax)),
                start_ratio=0.12,
                height_ratio=self.upper_body_region_height_ratio,
            )
            lower_hist, lower_valid, lower_conf = self._extract_region_hist(
                rgb_img,
                (int(xmin), int(ymin), int(xmax), int(ymax)),
                start_ratio=0.55,
                height_ratio=self.lower_body_region_height_ratio,
            )
            det.torso_hist = torso_hist
            det.torso_hist_valid = torso_valid
            det.torso_hist_confidence = float(torso_conf)
            det.upper_body_hist = upper_hist
            det.upper_body_hist_valid = upper_valid
            det.upper_body_hist_confidence = float(upper_conf)
            det.lower_body_hist = lower_hist
            det.lower_body_hist_valid = lower_valid
            det.lower_body_hist_confidence = float(lower_conf)

            detection_quality = self._clamp01(0.55 * conf + 0.45 * depth_valid_ratio)
            identity_quality, body_visible_ratio, blur_score = self._identity_quality(
                rgb_img,
                (int(xmin), int(ymin), int(xmax), int(ymax)),
                float(depth_valid_ratio),
            )
            det.detection_quality = float(detection_quality)
            det.identity_quality = float(identity_quality)
            if identity_quality >= self.identity_quality_min_for_update and not weak_depth:
                det.body_reid_valid = True
                det.body_reid_confidence = float(identity_quality)
                if torso_valid:
                    det.torso_hist_valid = True
                if upper_valid:
                    det.upper_body_hist_valid = True
                if lower_valid:
                    det.lower_body_hist_valid = True
            else:
                det.body_reid_confidence = 0.0
                det.torso_hist_confidence = min(det.torso_hist_confidence, float(identity_quality))
                det.upper_body_hist_confidence = min(det.upper_body_hist_confidence, float(identity_quality))
                det.lower_body_hist_confidence = min(det.lower_body_hist_confidence, float(identity_quality))

            vel_u, vel_v, vel_conf = self._estimate_center_velocity(float(u), float(v), rgb_msg.header.stamp)
            det.image_center_vel_u = float(vel_u)
            det.image_center_vel_v = float(vel_v)
            det.image_center_confidence = float(vel_conf)

            dets_msg.detections.append(det)

            # GUI overlay
            if self.enable_gui:
                is_locked = self._is_locked_detection(det)
                box_color = (0, 0, 255) if is_locked else (0, 255, 0)
                label_prefix = 'LOCKED ' if is_locked else ''
                cv2.rectangle(rgb_img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), box_color, 2)
                cv2.circle(rgb_img, (u, v), 4, (0, 0, 255), -1)
                cv2.putText(
                    rgb_img,
                    f"{label_prefix}person {conf:.2f} Z={Z:.2f}m",
                    (int(xmin), max(int(ymin) - 10, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    box_color,
                    2
                )
                if face_result is not None and face_result['face_bbox'] is not None:
                    self._draw_face_overlay(
                        rgb_img,
                        face_result['face_bbox'],
                        face_result['landmarks'],
                        is_cached=not bool(face_result['fresh']),
                    )
                if is_locked:
                    ok, reason, quality = self._vision_reid_status(det)
                    target = self._current_locked_target()
                    if target is not None:
                        self._emit_vision_reid_debug(det, int(target.track_id))
                    self._draw_reid_overlay(rgb_img, det, ok, reason, quality)
            elif self.reid_debug_enabled and self._is_locked_detection(det):
                target = self._current_locked_target()
                if target is not None:
                    self._emit_vision_reid_debug(det, int(target.track_id))

        if run_retina_this_frame:
            self._cached_face_results = next_face_cache
        if self.enable_gui:
            cv2.imshow("VisionNode", rgb_img)
            cv2.waitKey(1)

        # Publish outputs
        self._store_centers_for_velocity(dets_msg, rgb_msg.header.stamp)
        self.pub_dets.publish(dets_msg)
        if self.publish_markers:
            self.pub_markers.publish(self.make_markers(dets_msg, rgb_msg.header.stamp))

    def _estimate_facing_from_person_bbox(self, rgb_img, person_bbox_xyxy, camK: np.ndarray, run_inference: bool):
        if self.retina is None:
            return self._empty_face_result(person_bbox_xyxy)
        if not run_inference:
            return self._match_cached_face_result(person_bbox_xyxy)
        try:
            x1, y1, x2, y2 = person_bbox_xyxy
            H, W = rgb_img.shape[:2]
            x1 = int(max(0, min(W - 1, round(x1))))
            x2 = int(max(0, min(W - 1, round(x2))))
            y1 = int(max(0, min(H - 1, round(y1))))
            y2 = int(max(0, min(H - 1, round(y2))))
            if x2 <= x1 + 2 or y2 <= y1 + 2:
                return self._empty_face_result(person_bbox_xyxy)
            pw = x2 - x1
            ph = y2 - y1

            # Try a few head-centric ROIs before giving up. Different detectors and
            # body-box aspect ratios behave differently when the crop includes too
            # much torso or background.
            roi_specs = [
                (x1, y1, x2, y1 + int(round(ph * 0.55))),  # top half
                (
                    max(0, x1 - int(round(0.10 * pw))),
                    y1,
                    min(W - 1, x2 + int(round(0.10 * pw))),
                    y1 + int(round(ph * 0.45)),
                ),  # slightly wider head crop
                (x1, y1, x2, y1 + int(round(ph * 0.70))),  # previous behavior, broader
                (x1, y1, x2, y2),  # full person box fallback
            ]

            best_score = -1.0
            best_lm_full = None
            best_face_box_full = None
            for rx1, ry1, rx2, ry2 in roi_specs:
                rx1 = int(max(0, min(W - 1, rx1)))
                rx2 = int(max(0, min(W - 1, rx2)))
                ry1 = int(max(0, min(H - 1, ry1)))
                ry2 = int(max(0, min(H - 1, ry2)))
                if rx2 <= rx1 + 8 or ry2 <= ry1 + 8:
                    continue

                roi = rgb_img[ry1:ry2, rx1:rx2]
                if roi.size == 0:
                    continue

                boxes, scores, lms = self.retina.detect(roi)
                if boxes.shape[0] == 0:
                    continue

                det_idx = int(np.argmax(scores))
                det_score = float(scores[det_idx])
                if det_score <= best_score:
                    continue

                face_box = boxes[det_idx].copy().astype(np.float64)
                face_box[0] += float(rx1)
                face_box[1] += float(ry1)
                face_box[2] += float(rx1)
                face_box[3] += float(ry1)
                lm = lms[det_idx].copy().astype(np.float64)
                lm[:, 0] += float(rx1)
                lm[:, 1] += float(ry1)
                best_lm_full = lm
                best_face_box_full = face_box
                best_score = det_score

            if best_lm_full is None:
                # Conservative image-center fallback for close, large, centered person boxes.
                bbox_area = float(pw * ph)
                bbox_center_u = 0.5 * float(x1 + x2)
                center_offset_ratio = abs(bbox_center_u - 0.5 * float(W)) / max(0.5 * float(W), 1.0)
                bbox_aspect = float(pw) / max(float(ph), 1.0)
                if bbox_area >= 60000.0 and center_offset_ratio <= 0.18 and 0.25 <= bbox_aspect <= 0.95:
                    return self._build_face_result(
                        person_bbox_xyxy,
                        0.0,
                        0.0,
                        0.0,
                        1.0,
                        None,
                        None,
                        0.0,
                        True,
                    )
                return self._empty_face_result(person_bbox_xyxy)

            yaw, pitch, roll = self._head_pose_from_5pt(best_lm_full, camK)
            facing = 1.0 if (abs(yaw) <= self.facing_yaw_thresh_deg and abs(pitch) <= self.facing_pitch_thresh_deg) else 0.0
            return self._build_face_result(
                person_bbox_xyxy,
                float(yaw),
                float(pitch),
                float(roll),
                float(facing),
                best_face_box_full.tolist() if best_face_box_full is not None else None,
                best_lm_full.tolist(),
                float(best_score),
                True,
            )
        except Exception as exc:
            self.get_logger().warn(f'[VisionNode] Facing estimation failed: {exc}')
            return self._empty_face_result(person_bbox_xyxy)

    def _build_face_result(
        self,
        person_bbox_xyxy,
        yaw_deg: float,
        pitch_deg: float,
        roll_deg: float,
        facing01: float,
        face_bbox,
        landmarks,
        score: float,
        fresh: bool,
    ):
        cache_entry = {
            'person_bbox': [float(v) for v in person_bbox_xyxy],
            'yaw_deg': float(yaw_deg),
            'pitch_deg': float(pitch_deg),
            'roll_deg': float(roll_deg),
            'facing01': float(facing01),
            'face_bbox': [float(v) for v in face_bbox] if face_bbox is not None else None,
            'landmarks': [[float(v) for v in pt] for pt in landmarks] if landmarks is not None else None,
            'score': float(score),
        }
        return {
            'yaw_deg': float(yaw_deg),
            'pitch_deg': float(pitch_deg),
            'roll_deg': float(roll_deg),
            'facing01': float(facing01),
            'face_bbox': cache_entry['face_bbox'],
            'landmarks': cache_entry['landmarks'],
            'score': float(score),
            'fresh': bool(fresh),
            'cacheable': True,
            'cache_entry': cache_entry,
        }

    def _empty_face_result(self, person_bbox_xyxy):
        return {
            'yaw_deg': 0.0,
            'pitch_deg': 0.0,
            'roll_deg': 0.0,
            'facing01': 0.0,
            'face_bbox': None,
            'landmarks': None,
            'score': 0.0,
            'fresh': False,
            'cacheable': False,
            'cache_entry': {
                'person_bbox': [float(v) for v in person_bbox_xyxy],
            },
        }

    def _match_cached_face_result(self, person_bbox_xyxy):
        best_entry = None
        best_score = 0.0
        for entry in self._cached_face_results:
            cached_bbox = entry.get('person_bbox')
            if cached_bbox is None:
                continue
            iou = self._bbox_iou_xyxy(person_bbox_xyxy, cached_bbox)
            center_dist = self._bbox_center_distance(person_bbox_xyxy, cached_bbox)
            diag = self._bbox_diag(person_bbox_xyxy)
            center_score = 0.0 if diag <= 1e-6 else max(0.0, 1.0 - (center_dist / max(diag * 0.35, 1.0)))
            score = max(iou, center_score)
            if score > best_score:
                best_score = score
                best_entry = entry

        if best_entry is None or best_score < 0.2:
            return self._empty_face_result(person_bbox_xyxy)
        return {
            'yaw_deg': float(best_entry.get('yaw_deg', 0.0)),
            'pitch_deg': float(best_entry.get('pitch_deg', 0.0)),
            'roll_deg': float(best_entry.get('roll_deg', 0.0)),
            'facing01': float(best_entry.get('facing01', 0.0)),
            'face_bbox': best_entry.get('face_bbox'),
            'landmarks': best_entry.get('landmarks'),
            'score': float(best_entry.get('score', 0.0)),
            'fresh': False,
            'cacheable': False,
            'cache_entry': best_entry,
        }

    @staticmethod
    def _bbox_iou_xyxy(a, b) -> float:
        ax1, ay1, ax2, ay2 = [float(v) for v in a]
        bx1, by1, bx2, by2 = [float(v) for v in b]
        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)
        inter_w = max(0.0, inter_x2 - inter_x1)
        inter_h = max(0.0, inter_y2 - inter_y1)
        inter = inter_w * inter_h
        if inter <= 0.0:
            return 0.0
        area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
        denom = area_a + area_b - inter
        if denom <= 1e-6:
            return 0.0
        return inter / denom

    @staticmethod
    def _bbox_center_distance(a, b) -> float:
        ax1, ay1, ax2, ay2 = [float(v) for v in a]
        bx1, by1, bx2, by2 = [float(v) for v in b]
        acx = 0.5 * (ax1 + ax2)
        acy = 0.5 * (ay1 + ay2)
        bcx = 0.5 * (bx1 + bx2)
        bcy = 0.5 * (by1 + by2)
        return math.hypot(acx - bcx, acy - bcy)

    @staticmethod
    def _bbox_diag(bbox_xyxy) -> float:
        x1, y1, x2, y2 = [float(v) for v in bbox_xyxy]
        return math.hypot(max(0.0, x2 - x1), max(0.0, y2 - y1))

    def _draw_face_overlay(self, rgb_img, face_bbox, landmarks, is_cached: bool):
        x1, y1, x2, y2 = [int(round(float(v))) for v in face_bbox]
        box_color = (0, 200, 255) if is_cached else (255, 255, 0)
        cv2.rectangle(rgb_img, (x1, y1), (x2, y2), box_color, 2)
        if landmarks:
            for pt in landmarks:
                if len(pt) < 2:
                    continue
                cv2.circle(
                    rgb_img,
                    (int(round(float(pt[0]))), int(round(float(pt[1])))),
                    2,
                    box_color,
                    -1,
                )
        cv2.putText(
            rgb_img,
            'face cached' if is_cached else 'face',
            (x1, max(y1 - 8, 0)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            box_color,
            1,
        )

    @staticmethod
    def _head_pose_from_5pt(lm5_xy: np.ndarray, camK: np.ndarray):
        """
        lm5_xy: (5,2) in image pixels: [l_eye, r_eye, nose, l_mouth, r_mouth]
        Returns yaw_deg, pitch_deg, roll_deg
        """
        # Canonical 3D points (meters), rough face model
        model_pts = np.array([
            [-0.03,  0.03, 0.03],  # left eye
            [ 0.03,  0.03, 0.03],  # right eye
            [ 0.00,  0.00, 0.00],  # nose
            [-0.02, -0.03, 0.03],  # left mouth
            [ 0.02, -0.03, 0.03],  # right mouth
        ], dtype=np.float64)

        img_pts = lm5_xy.astype(np.float64).reshape(-1, 2)
        dist = np.zeros((4, 1), dtype=np.float64)

        ok, rvec, tvec = cv2.solvePnP(
            model_pts,
            img_pts,
            camK,
            dist,
            flags=cv2.SOLVEPNP_EPNP,
        )
        if not ok:
            return 0.0, 0.0, 0.0

        R, _ = cv2.Rodrigues(rvec)
        # OpenCV helper: returns (rx, ry, rz) in degrees (x=pitch-ish, y=yaw-ish, z=roll-ish)
        angles, _, _, _, _, _ = cv2.RQDecomp3x3(R)
        pitch_deg = VisionNode._normalize_pose_angle_deg(float(angles[0]))
        yaw_deg = VisionNode._normalize_pose_angle_deg(float(angles[1]))
        roll_deg = VisionNode._normalize_pose_angle_deg(float(angles[2]))
        return yaw_deg, pitch_deg, roll_deg

    @staticmethod
    def _normalize_pose_angle_deg(angle_deg: float) -> float:
        angle = ((float(angle_deg) + 180.0) % 360.0) - 180.0
        if angle > 90.0:
            angle -= 180.0
        elif angle < -90.0:
            angle += 180.0
        return float(angle)

    def _extract_region_hist(self, rgb_img, bbox_xyxy, start_ratio: float, height_ratio: float):
        x1, y1, x2, y2 = bbox_xyxy
        h_img, w_img = rgb_img.shape[:2]
        x1 = int(max(0, min(w_img - 1, x1)))
        x2 = int(max(0, min(w_img, x2)))
        y1 = int(max(0, min(h_img - 1, y1)))
        y2 = int(max(0, min(h_img, y2)))
        if x2 <= x1 + 2 or y2 <= y1 + 2:
            return [], False, 0.0

        box_h = y2 - y1
        roi_y1 = y1 + int(round(box_h * start_ratio))
        roi_y2 = roi_y1 + int(round(box_h * height_ratio))
        roi_y1 = max(y1, min(y2 - 1, roi_y1))
        roi_y2 = max(roi_y1 + 1, min(y2, roi_y2))

        roi = rgb_img[roi_y1:roi_y2, x1:x2]
        if roi.size == 0:
            return [], False, 0.0

        visible_ratio = float(roi.shape[0] * roi.shape[1]) / max(float(box_h * max(1, x2 - x1)), 1.0)
        if visible_ratio < self.hist_visible_ratio_min:
            return [], False, visible_ratio

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [12, 8], [0, 180, 0, 256]).astype(np.float32).reshape(-1)
        s = float(np.sum(hist))
        if s <= 1e-6:
            return [], False, 0.0
        hist /= s
        return hist.tolist(), True, min(1.0, visible_ratio)

    def _identity_quality(self, rgb_img, bbox_xyxy, depth_valid_ratio: float):
        x1, y1, x2, y2 = bbox_xyxy
        area = max(0.0, float((x2 - x1) * (y2 - y1)))
        if area <= 0.0:
            return 0.0, 0.0, 0.0
        aspect = float(x2 - x1) / max(float(y2 - y1), 1.0)
        crop = rgb_img[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
        if crop.size == 0:
            return 0.0, 0.0, 0.0
        blur = float(cv2.Laplacian(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), cv2.CV_32F).var())
        blur_score = self._clamp01(blur / max(self.crop_blur_max_laplacian, 1.0))
        area_score = self._clamp01(area / max(self.crop_min_bbox_area_px, 1.0))
        visible_ratio = self._clamp01(depth_valid_ratio / max(self.crop_min_visible_body_ratio, 1e-3))
        aspect_ok = self.crop_aspect_ratio_min <= aspect <= self.crop_aspect_ratio_max
        aspect_score = 1.0 if aspect_ok else 0.25
        score = 0.30 * blur_score + 0.25 * area_score + 0.25 * visible_ratio + 0.20 * aspect_score
        return self._clamp01(score), visible_ratio, blur_score

    def _estimate_center_velocity(self, u: float, v: float, stamp_msg):
        now_sec = float(stamp_msg.sec) + float(stamp_msg.nanosec) * 1e-9
        if self._prev_center_time <= 0.0 or not self._prev_centers:
            return 0.0, 0.0, 0.0
        best = min(self._prev_centers, key=lambda p: (p[0] - u) ** 2 + (p[1] - v) ** 2)
        dt = max(1e-3, now_sec - self._prev_center_time)
        du = (u - best[0]) / dt
        dv = (v - best[1]) / dt
        dist = math.hypot(u - best[0], v - best[1])
        conf = self._clamp01(1.0 - dist / 120.0)
        return du, dv, conf

    def _store_centers_for_velocity(self, dets_msg: CamDetections, stamp_msg):
        self._prev_center_time = float(stamp_msg.sec) + float(stamp_msg.nanosec) * 1e-9
        self._prev_centers = [(float(det.center_u), float(det.center_v)) for det in dets_msg.detections]

    def _current_locked_target(self) -> Optional[TrackedPose]:
        target = self.latched_locked_target
        if target is None:
            return None
        if int(target.track_id) <= 0 or not bool(target.selected) or not bool(target.valid):
            return None
        if not math.isfinite(float(target.pose.position.x)) or not math.isfinite(float(target.pose.position.y)):
            return None
        return target

    def _is_locked_detection(self, det: CamDetection) -> bool:
        target = self._current_locked_target()
        if target is None:
            return False

        dx = float(det.x) - float(target.pose.position.x)
        dy = float(det.y) - float(target.pose.position.y)
        return math.hypot(dx, dy) <= self.gui_lock_match_gate_m

    def _vision_reid_status(self, det: CamDetection) -> tuple[bool, Optional[str], Optional[float]]:
        bbox_w = max(0.0, float(int(getattr(det, 'x2', 0)) - int(getattr(det, 'x1', 0))))
        bbox_h = max(0.0, float(int(getattr(det, 'y2', 0)) - int(getattr(det, 'y1', 0))))
        bbox_area = bbox_w * bbox_h
        quality = None
        if hasattr(det, 'body_reid_confidence'):
            quality = float(det.body_reid_confidence)
        elif hasattr(det, 'identity_quality'):
            quality = float(det.identity_quality)

        valid = bool(getattr(det, 'body_reid_valid', False))
        if valid or (quality is not None and quality > 0.0):
            return True, None, quality
        if bbox_w <= 1.0 or bbox_h <= 1.0:
            return False, 'invalid_bbox', quality
        if bbox_area < self.crop_min_bbox_area_px:
            return False, 'crop_too_small', quality
        return False, 'extractor_returned_none', quality

    def _emit_vision_reid_debug(self, det: CamDetection, target_id: int):
        if not self.reid_debug_enabled:
            return
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        ok, reason, quality = self._vision_reid_status(det)
        event = 'success' if ok else 'failure'
        key = f'vision:{int(target_id)}:{event}'
        status = 'ok' if ok else canonical_reid_reason(reason)
        if not should_emit_reid_log(
            self._reid_debug_last_emit_sec,
            self._reid_debug_last_status,
            key,
            status,
            now_sec,
            self.reid_debug_log_period_sec,
        ):
            return

        if ok:
            fields = []
            if quality is not None and math.isfinite(quality):
                fields.append(('q', f'{quality:.2f}'))
            self.get_logger().info(format_reid_log_ok('vision', target_id, fields))
        else:
            self.get_logger().warn(format_reid_log_fail('vision', target_id, reason))

    def _draw_reid_overlay(self, rgb_img, det: CamDetection, ok: bool, reason: Optional[str], quality: Optional[float]):
        if not self.enable_gui or not self.reid_debug_enabled:
            return
        x = int(getattr(det, 'x1', 0))
        y = int(getattr(det, 'y2', 0)) + 18
        color = (0, 255, 0) if ok else (0, 0, 255)
        lines = ['reid: ok' if ok else 'reid: fail']
        if ok:
            if quality is not None and math.isfinite(quality):
                lines.append(f'q: {quality:.2f}')
        else:
            lines.append(f'why: {overlay_reid_reason(reason)}')

        yy = min(y, rgb_img.shape[0] - 6)
        for line in lines:
            cv2.putText(rgb_img, line, (x, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(rgb_img, line, (x, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
            yy = min(yy + 16, rgb_img.shape[0] - 6)

    @staticmethod
    def _clamp01(value: float) -> float:
        return max(0.0, min(1.0, value))

    # ==========================================================
    # YOLO helper
    # ==========================================================
    def run_yolo(self, bgr_img):
        t0 = time.perf_counter()
        results = self.model.predict(
            source=bgr_img,
            imgsz=640,
            conf=self.conf_thresh,
            verbose=False,
            device=self.resolved_yolo_device,
        )
        self._maybe_log_timing('yolo', time.perf_counter() - t0)
        if not results:
            return []

        res = results[0]
        if res.boxes is None:
            return []

        detections = []
        for box in res.boxes:
            cls_id = int(box.cls[0].item())
            if cls_id != 0:
                continue  # keep only 'person'
            conf = float(box.conf[0].item())
            xmin, ymin, xmax, ymax = box.xyxy[0].cpu().numpy().tolist()
            detections.append((xmin, ymin, xmax, ymax, conf))
        return detections

    # ==========================================================
    # Depth helper
    # ==========================================================
    def depth_median_in_bbox(self, depth_img, depth_encoding: str, bbox_xyxy):
        x1, y1, x2, y2 = bbox_xyxy
        h, w = depth_img.shape[:2]
        x1 = max(0, min(w - 1, int(x1)))
        x2 = max(0, min(w - 1, int(x2)))
        y1 = max(0, min(h - 1, int(y1)))
        y2 = max(0, min(h - 1, int(y2)))
        if x2 <= x1 + 2 or y2 <= y1 + 2:
            return None, 0.0

        bw = x2 - x1
        bh = y2 - y1
        rw = max(3, int(round(bw * self.depth_roi_scale_w)))
        rh = max(3, int(round(bh * self.depth_roi_scale_h)))
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        rx1 = max(0, cx - rw // 2)
        rx2 = min(w, cx + rw // 2)
        ry1 = max(0, cy - rh // 2)
        ry2 = min(h, cy + rh // 2)
        if rx2 <= rx1 or ry2 <= ry1:
            return None, 0.0

        roi = depth_img[ry1:ry2, rx1:rx2]
        if roi.size == 0:
            return None, 0.0
        flat = roi.reshape(-1)
        vals = []
        for dv in flat:
            z = self.depth_to_meters(dv, depth_encoding)
            if z is not None and z > 0.0 and math.isfinite(z):
                vals.append(z)
        if not vals:
            return None, 0.0

        valid_ratio = float(len(vals)) / float(flat.size)
        z_med = float(np.median(np.asarray(vals, dtype=np.float64)))
        return z_med, valid_ratio

    @staticmethod
    def depth_to_meters(depth_value, encoding: str):
        try:
            z = float(depth_value)
        except Exception:
            return None

        if math.isnan(z) or math.isinf(z):
            return None

        # RealSense aligned depth is commonly 16UC1 (mm)
        if encoding in ('16UC1', 'mono16'):
            if z <= 0.0:
                return None
            return z / 1000.0

        # Sometimes depth is float meters
        if encoding == '32FC1':
            if z <= 0.0:
                return None
            return z

        # Unknown encoding
        return None

    # ==========================================================
    # TF helpers
    # ==========================================================
    def transform_point_camera_to_global(self, Xc, Yc, Zc, stamp: Time):
        # Lookup transform at the same timestamp as the image
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.camera_frame,
                stamp,
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_sec)
            )
        except Exception as e:
            self.get_logger().warn(f'[VisionNode] TF lookup failed: {e}')
            return None

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        tz = tf.transform.translation.z

        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w

        pc = np.array([Xc, Yc, Zc], dtype=np.float64)
        R = self.quaternion_to_rotation_matrix(qx, qy, qz, qw)
        pg = R @ pc + np.array([tx, ty, tz], dtype=np.float64)

        return pg[0], pg[1], pg[2]

    @staticmethod
    def quaternion_to_rotation_matrix(qx, qy, qz, qw):
        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        return np.array([
            [1.0 - 2.0 * (yy + zz),     2.0 * (xy - wz),         2.0 * (xz + wy)],
            [2.0 * (xy + wz),           1.0 - 2.0 * (xx + zz),   2.0 * (yz - wx)],
            [2.0 * (xz - wy),           2.0 * (yz + wx),         1.0 - 2.0 * (xx + yy)]
        ], dtype=np.float64)

    # ==========================================================
    # RViz markers helper
    # ==========================================================
    def make_markers(self, dets_msg: CamDetections, stamp_msg):
        ma = MarkerArray()

        # If there are fewer detections than last time, RViz can keep old markers.
        # Easiest: publish markers with short lifetime (we already do).
        for i, det in enumerate(dets_msg.detections):
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp = stamp_msg

            m.ns = "cam_detections"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            m.pose.position.x = det.x
            m.pose.position.y = det.y
            m.pose.position.z = det.z
            m.pose.orientation.w = 1.0

            s = self.marker_scale
            m.scale.x = s
            m.scale.y = s
            m.scale.z = s

            # Green
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 1.0

            m.lifetime = Duration(sec=0, nanosec=int(self.marker_lifetime_sec * 1e9))
            ma.markers.append(m)

        return ma


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()



# change the global_frame to output_frame or detection_frame to avoid confussion
