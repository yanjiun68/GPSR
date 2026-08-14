#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from typing import Optional, Tuple

import numpy as np

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

try:
    import cv2
except Exception:
    cv2 = None

try:
    import onnxruntime as ort
except Exception:
    ort = None

from person_following.nodes.person_track_manager import ReIDExtractor
from person_following.utils.runtime_config import resolve_onnx_providers


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


def _param_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 't', 'yes', 'y', 'on')
    return bool(value)


class ReIDTestNode(Node):
    def __init__(self):
        super().__init__('reid_test_node')

        self.declare_parameter('rgb_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('debug_image_topic', '/person_following/reid_test/debug_image')
        self.declare_parameter('reid_model_path', 'models/osnet_x0_5.onnx')
        self.declare_parameter('reid_providers', 'CUDAExecutionProvider,CPUExecutionProvider')
        self.declare_parameter('reid_input_w', 128)
        self.declare_parameter('reid_input_h', 256)
        self.declare_parameter('bbox', '')
        self.declare_parameter('use_center_crop', True)
        self.declare_parameter('anchor_image_path', '')
        self.declare_parameter('anchor_bbox', '')
        self.declare_parameter('log_period_sec', 0.0)
        self.declare_parameter('log_first_n', 8)
        self.declare_parameter('process_once', False)
        self.declare_parameter('enable_gui', False)

        if cv2 is None:
            raise RuntimeError('[ReIDTestNode] OpenCV not available')

        self.bridge = CvBridge()
        self.rgb_topic = str(self.get_parameter('rgb_topic').value)
        self.debug_image_topic = str(self.get_parameter('debug_image_topic').value)
        self.reid_model_path = _resolve_model_path(self.get_parameter('reid_model_path').value)
        self.reid_providers_request = self.get_parameter('reid_providers').value
        self.reid_input_w = int(self.get_parameter('reid_input_w').value)
        self.reid_input_h = int(self.get_parameter('reid_input_h').value)
        self.frame_bbox = self._parse_bbox(self.get_parameter('bbox').value)
        self.use_center_crop = _param_bool(self.get_parameter('use_center_crop').value)
        self.anchor_image_path = str(self.get_parameter('anchor_image_path').value).strip()
        self.anchor_bbox = self._parse_bbox(self.get_parameter('anchor_bbox').value)
        self.log_period_sec = float(self.get_parameter('log_period_sec').value)
        self.log_first_n = max(1, int(self.get_parameter('log_first_n').value))
        self.process_once = _param_bool(self.get_parameter('process_once').value)
        self.enable_gui = _param_bool(self.get_parameter('enable_gui').value)

        self.reid_resolution = resolve_onnx_providers(self.reid_providers_request, ort)
        if not self.reid_resolution.enabled:
            raise RuntimeError(
                '[ReIDTestNode] ReID runtime unavailable: '
                f'{self.reid_resolution.reason}; available={self.reid_resolution.available}'
            )

        self.reid = ReIDExtractor(
            self.reid_model_path,
            self.reid_resolution.resolved,
            self.reid_input_w,
            self.reid_input_h,
            self.get_logger(),
        )
        if not self.reid.enabled:
            raise RuntimeError(f'[ReIDTestNode] Failed to load ReID model: {self.reid_model_path}')

        self.anchor_requested = bool(self.anchor_image_path)
        self.anchor_embedding = self._load_anchor_embedding()
        self.last_log_time_sec = 0.0
        self.frame_count = 0
        self.window_name = 'ReID Test'

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_rgb = self.create_subscription(Image, self.rgb_topic, self.rgb_cb, qos)
        self.pub_debug = self.create_publisher(Image, self.debug_image_topic, 10)

        self.get_logger().info(
            '[ReIDTestNode] Started '
            f'rgb_topic={self.rgb_topic} '
            f'model={self.reid_model_path} '
            f'providers={self.reid_resolution.resolved} '
            f'frame_bbox={self.frame_bbox} '
            f'use_center_crop={self.use_center_crop} '
            f'anchor_requested={self.anchor_requested} '
            f'anchor_loaded={self.anchor_embedding is not None}'
        )

    def _parse_bbox(self, value) -> Optional[Tuple[int, int, int, int]]:
        text = str(value or '').strip()
        if not text:
            return None
        parts = [part.strip() for part in text.split(',')]
        if len(parts) != 4:
            raise ValueError(f'invalid bbox "{text}", expected "x1,y1,x2,y2"')
        try:
            x1, y1, x2, y2 = [int(round(float(part))) for part in parts]
        except Exception as exc:
            raise ValueError(f'invalid bbox "{text}"') from exc
        if x2 <= x1 or y2 <= y1:
            raise ValueError(f'invalid bbox "{text}", x2/y2 must be larger than x1/y1')
        return (x1, y1, x2, y2)

    def _load_anchor_embedding(self) -> Optional[np.ndarray]:
        if not self.anchor_image_path:
            return None

        image_path = _resolve_model_path(self.anchor_image_path)
        image = cv2.imread(image_path)
        if image is None or image.size == 0:
            self.get_logger().warn(f'[ReID] anchor_missing image={image_path}')
            return None

        emb, reason = self._extract_embedding(image, self.anchor_bbox)
        if emb is None:
            self.get_logger().warn(
                '[ReID] '
                f'anchor_extract_failed image={image_path} bbox={self.anchor_bbox} reason={reason}'
            )
            return None

        self.get_logger().info(
            '[ReIDTestNode] Anchor loaded '
            f'image={image_path} bbox={self.anchor_bbox} dim={int(emb.shape[0])}'
        )
        return emb

    def _classify_bbox(
        self,
        bgr_img: np.ndarray,
        bbox: Optional[Tuple[int, int, int, int]],
    ) -> Tuple[Optional[Tuple[int, int, int, int]], Optional[str]]:
        h, w = bgr_img.shape[:2]
        if bbox is None:
            return None, None

        x1, y1, x2, y2 = bbox
        x1 = max(0, min(w - 1, int(x1)))
        y1 = max(0, min(h - 1, int(y1)))
        x2 = max(0, min(w, int(x2)))
        y2 = max(0, min(h, int(y2)))
        if x2 <= x1 or y2 <= y1:
            return None, 'invalid_bbox'
        if (x2 - x1) < 8 or (y2 - y1) < 8:
            return None, 'crop_too_small'
        return (x1, y1, x2, y2), None

    def _center_crop_bbox(self, bgr_img: np.ndarray) -> Tuple[int, int, int, int]:
        h, w = bgr_img.shape[:2]
        crop_w = max(24, int(w * 0.12))
        crop_h = max(48, int(h * 0.32))
        cx = int(round(0.5 * w))
        cy = int(round(0.5 * h))
        x0 = max(0, cx - crop_w // 2)
        x1 = min(w, cx + crop_w // 2)
        y0 = max(0, cy - int(crop_h * 0.55))
        y1 = min(h, cy + int(crop_h * 0.45))
        return (x0, y0, x1, y1)

    def _extract_embedding(
        self,
        bgr_img: np.ndarray,
        bbox: Optional[Tuple[int, int, int, int]],
    ) -> Tuple[Optional[np.ndarray], Optional[str]]:
        if bbox is not None:
            checked_bbox, reason = self._classify_bbox(bgr_img, bbox)
            if checked_bbox is None:
                return None, reason
            emb = self.reid.extract_from_bbox(bgr_img, *checked_bbox)
            return emb, None if emb is not None else 'extractor_returned_none'

        if not self.use_center_crop:
            return None, 'invalid_bbox'

        checked_bbox, reason = self._classify_bbox(bgr_img, self._center_crop_bbox(bgr_img))
        if checked_bbox is None:
            return None, reason
        emb = self.reid.extract_from_bbox(bgr_img, *checked_bbox)
        return emb, None if emb is not None else 'extractor_returned_none'

    def _cosine_similarity(self, emb_a: np.ndarray, emb_b: np.ndarray) -> float:
        return float(np.dot(emb_a.reshape(-1), emb_b.reshape(-1)))

    def _format_preview(self, emb: np.ndarray) -> str:
        count = max(1, min(self.log_first_n, int(emb.shape[0])))
        return np.array2string(
            emb[:count],
            precision=3,
            separator=', ',
            suppress_small=False,
        )

    def _draw_text_block(self, image: np.ndarray, lines, color):
        if not lines:
            return
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.55
        thickness = 1
        line_gap = 8
        x0 = 12
        y0 = 18
        sizes = [cv2.getTextSize(line, font, font_scale, thickness)[0] for line in lines]
        max_w = max(size[0] for size in sizes)
        total_h = sum(size[1] for size in sizes) + line_gap * (len(lines) - 1)
        cv2.rectangle(image, (x0 - 8, y0 - 12), (x0 + max_w + 8, y0 + total_h + 10), (0, 0, 0), -1)

        y = y0
        for line, size in zip(lines, sizes):
            y += size[1]
            cv2.putText(image, line, (x0, y), font, font_scale, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(image, line, (x0, y), font, font_scale, color, thickness, cv2.LINE_AA)
            y += line_gap

    def _log_status(
        self,
        now_sec: float,
        emb: Optional[np.ndarray],
        reason: Optional[str],
        similarity: Optional[float],
        norm: Optional[float],
    ):
        should_log = (
            self.log_period_sec <= 0.0
            or (now_sec - self.last_log_time_sec) >= self.log_period_sec
            or self.frame_count == 1
        )
        if not should_log:
            return

        if emb is None:
            self.get_logger().warn(f'[ReID] fail reason={reason or "extractor_returned_none"}')
        else:
            msg = (
                f'[ReID] ok dim={int(emb.shape[0])} '
                f'norm={(norm if norm is not None else 0.0):.3f} '
                f'emb[:{min(self.log_first_n, int(emb.shape[0]))}]={self._format_preview(emb)}'
            )
            if similarity is not None:
                msg += f' sim={similarity:.3f}'
            self.get_logger().info(msg)
        self.last_log_time_sec = now_sec

    def rgb_cb(self, msg: Image):
        try:
            bgr_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'[ReIDTestNode] cv_bridge failed: {exc}')
            return

        self.frame_count += 1
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        emb, reason = self._extract_embedding(bgr_img, self.frame_bbox)
        similarity = None
        norm = None

        if emb is not None:
            try:
                norm = float(np.linalg.norm(emb))
            except Exception:
                emb = None
                reason = 'extractor_returned_none'

        if emb is not None and self.anchor_embedding is not None:
            similarity = self._cosine_similarity(self.anchor_embedding, emb)

        self._log_status(now_sec, emb, reason, similarity, norm)
        self._publish_debug(bgr_img, msg, emb, reason, similarity, norm)

        if self.process_once and emb is not None:
            self.get_logger().info('[ReIDTestNode] process_once=true, shutting down after successful extraction')
            rclpy.shutdown()

    def _publish_debug(
        self,
        bgr_img: np.ndarray,
        src_msg: Image,
        emb: Optional[np.ndarray],
        reason: Optional[str],
        similarity: Optional[float],
        norm: Optional[float],
    ):
        annotated = bgr_img.copy()
        lines = []
        color = (0, 0, 255)

        if self.frame_bbox is not None:
            x1, y1, x2, y2 = self.frame_bbox
            h, w = annotated.shape[:2]
            x1 = max(0, min(w - 1, x1))
            y1 = max(0, min(h - 1, y1))
            x2 = max(0, min(w, x2))
            y2 = max(0, min(h, y2))
            if x2 > x1 and y2 > y1:
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

        if emb is None:
            lines.append('reid: fail')
            lines.append(f'reason: {reason or "extractor_returned_none"}')
        else:
            color = (0, 255, 0)
            lines.append('reid: ok')
            lines.append(f'dim: {int(emb.shape[0])}')
            lines.append(f'norm: {(norm if norm is not None else 0.0):.3f}')
            lines.append(f'emb[:{min(self.log_first_n, int(emb.shape[0]))}]: {self._format_preview(emb)}')
            if similarity is not None:
                lines.append(f'sim: {similarity:.3f}')

        self._draw_text_block(annotated, lines, color)

        debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        debug_msg.header = src_msg.header
        self.pub_debug.publish(debug_msg)

        if self.enable_gui:
            cv2.imshow(self.window_name, annotated)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ReIDTestNode()
    try:
        rclpy.spin(node)
    finally:
        if cv2 is not None:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
