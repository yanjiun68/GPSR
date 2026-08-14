#!/usr/bin/env python3
"""
Automatic bag-handle selector node.

Selects a stable bag-handle detection without MediaPipe. Once a candidate stays
spatially stable for a configured amount of time, it is hard-locked until the
unlock service is called.
"""

import math
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2D, Detection2DArray


class HandBagSelector(Node):
    def __init__(self) -> None:
        super().__init__('hand_bag_selector')

        self.declare_parameter('image_topic', '/camera/camera/color/image_raw/compressed')
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('selected_topic', '/handbag/selected_detection')
        self.declare_parameter('selected_array_topic', '/handbag/selected_detections')
        self.declare_parameter('selected_flag_topic', '/handbag/selected')
        self.declare_parameter('viz_topic', '/handbag/pointing_annotated/compressed')
        self.declare_parameter('check_service', '/handbag/check_detection')
        self.declare_parameter('unlock_service', '/handbag/unlock_selection')
        self.declare_parameter('show_window', True)
        self.declare_parameter('window_name', 'Step4: Auto Lock Candidate')
        self.declare_parameter('selection_hold_sec', 2.5)
        self.declare_parameter('reuse_last_selected_when_missing', True)
        self.declare_parameter('lock.enable_hard_lock', True)
        self.declare_parameter('lock.follow_max_center_distance_px', 140.0)
        self.declare_parameter('lock.follow_max_size_ratio_delta', 0.75)
        self.declare_parameter('lock.keep_frames_on_jitter', 20)
        self.declare_parameter('auto_lock.stable_time_sec', 2.0)
        self.declare_parameter('stability.center_tolerance_px', 45.0)
        self.declare_parameter('stability.size_ratio_tolerance', 0.30)
        self.declare_parameter('max_detection_age_sec', 0.8)
        self.declare_parameter('min_detection_score', 0.2)
        self.declare_parameter('strict_min_detection_score', 0.45)
        self.declare_parameter('strict_bag_only', True)
        self.declare_parameter('strict_target_keywords', [
            'bag handle',
            'handbag handle',
            'handle',
            'strap',
        ])
        self.declare_parameter('target_keywords', [
            'bag',
            'handbag',
            'purse',
            'backpack',
            'tote',
            'strap',
            'handle',
        ])

        image_topic = self.get_parameter('image_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        self.selected_topic = self.get_parameter('selected_topic').value
        self.selected_array_topic = self.get_parameter('selected_array_topic').value
        self.selected_flag_topic = self.get_parameter('selected_flag_topic').value
        self.viz_topic = self.get_parameter('viz_topic').value
        self.check_service = self.get_parameter('check_service').value
        self.unlock_service = self.get_parameter('unlock_service').value
        self.show_window = bool(self.get_parameter('show_window').value)
        self.window_name = self.get_parameter('window_name').value
        self.selection_hold_sec = float(self.get_parameter('selection_hold_sec').value)
        self.reuse_last_selected_when_missing = bool(
            self.get_parameter('reuse_last_selected_when_missing').value
        )
        self.enable_hard_lock = bool(self.get_parameter('lock.enable_hard_lock').value)
        self.lock_follow_max_center_distance_px = float(
            self.get_parameter('lock.follow_max_center_distance_px').value
        )
        self.lock_follow_max_size_ratio_delta = float(
            self.get_parameter('lock.follow_max_size_ratio_delta').value
        )
        self.lock_keep_frames_on_jitter = int(self.get_parameter('lock.keep_frames_on_jitter').value)
        self.auto_lock_stable_time_sec = float(
            self.get_parameter('auto_lock.stable_time_sec').value
        )
        self.stable_center_tolerance_px = float(
            self.get_parameter('stability.center_tolerance_px').value
        )
        self.stable_size_ratio_tolerance = float(
            self.get_parameter('stability.size_ratio_tolerance').value
        )
        self.max_detection_age_sec = float(self.get_parameter('max_detection_age_sec').value)
        self.min_detection_score = float(self.get_parameter('min_detection_score').value)
        self.strict_min_detection_score = float(self.get_parameter('strict_min_detection_score').value)
        self.strict_bag_only = bool(self.get_parameter('strict_bag_only').value)
        self.strict_target_keywords = [
            str(x).strip().lower() for x in self.get_parameter('strict_target_keywords').value
        ]
        self.target_keywords = [str(x).lower() for x in self.get_parameter('target_keywords').value]

        self._latest_detections: Optional[List[Detection2D]] = None
        self._latest_det_header = None
        self._latest_det_time = 0.0
        self._last_selected_det: Optional[Detection2D] = None
        self._last_selected_time = 0.0
        self._display_det: Optional[Detection2D] = None
        self._display_hold_frames = 0
        self._locked_det: Optional[Detection2D] = None
        self._lock_time = 0.0
        self._candidate_det: Optional[Detection2D] = None
        self._candidate_since = 0.0

        self.image_sub = self.create_subscription(
            CompressedImage, image_topic, self._image_cb, 10
        )
        self.det_sub = self.create_subscription(
            Detection2DArray, detections_topic, self._detections_cb, 10
        )

        self.selected_pub = self.create_publisher(Detection2D, self.selected_topic, 10)
        self.selected_array_pub = self.create_publisher(Detection2DArray, self.selected_array_topic, 10)
        self.selected_flag_pub = self.create_publisher(Bool, self.selected_flag_topic, 10)
        self.viz_pub = self.create_publisher(CompressedImage, self.viz_topic, 10)
        self.check_srv = self.create_service(Trigger, self.check_service, self._check_detection_service)
        self.unlock_srv = self.create_service(Trigger, self.unlock_service, self._unlock_selection_service)

        self.get_logger().info('HandBagSelector started in auto-lock mode')
        self.get_logger().info(f'  image_topic: {image_topic}')
        self.get_logger().info(f'  detections_topic: {detections_topic}')
        self.get_logger().info(f'  selected_topic: {self.selected_topic}')
        self.get_logger().info(f'  check_service: {self.check_service}')
        self.get_logger().info(f'  unlock_service: {self.unlock_service}')
        self.get_logger().info(f'  viz_topic: {self.viz_topic}')
        self.get_logger().info(f'  auto_lock.stable_time_sec: {self.auto_lock_stable_time_sec:.1f}')
        self.get_logger().info(f'  hard_lock_enabled: {self.enable_hard_lock}')

    def _detections_cb(self, msg: Detection2DArray) -> None:
        self._latest_detections = list(msg.detections)
        self._latest_det_header = msg.header
        self._latest_det_time = time.time()

    def _is_bag_class(self, det: Detection2D) -> bool:
        if not det.results:
            return False
        class_id = str(det.results[0].hypothesis.class_id).strip().lower().replace('_', ' ')
        if self.strict_bag_only:
            return any(k in class_id for k in self.strict_target_keywords)
        return any(k in class_id for k in self.target_keywords)

    def _score(self, det: Detection2D) -> float:
        if not det.results:
            return 0.0
        return float(det.results[0].hypothesis.score)

    def _valid_detections(self) -> List[Detection2D]:
        if self._latest_detections is None:
            return []
        if (time.time() - self._latest_det_time) > self.max_detection_age_sec:
            return []

        filtered = []
        for det in self._latest_detections:
            if not det.results:
                continue
            score_thresh = self.strict_min_detection_score if self.strict_bag_only else self.min_detection_score
            if self._score(det) < score_thresh:
                continue
            filtered.append(det)

        bag_only = [d for d in filtered if self._is_bag_class(d)]
        candidates = bag_only if (self.strict_bag_only or bag_only) else filtered
        candidates.sort(key=self._score, reverse=True)
        return candidates

    def _bbox_center(self, det: Detection2D) -> Tuple[float, float]:
        return (
            float(det.bbox.center.position.x),
            float(det.bbox.center.position.y),
        )

    def _bbox_size(self, det: Detection2D) -> Tuple[float, float]:
        return (
            max(float(det.bbox.size_x), 1.0),
            max(float(det.bbox.size_y), 1.0),
        )

    def _same_track(self, reference: Detection2D, candidate: Detection2D) -> bool:
        ref_cls = ''
        cand_cls = ''
        if reference.results:
            ref_cls = str(reference.results[0].hypothesis.class_id).strip().lower()
        if candidate.results:
            cand_cls = str(candidate.results[0].hypothesis.class_id).strip().lower()
        if ref_cls and cand_cls and ref_cls != cand_cls:
            return False

        ref_cx, ref_cy = self._bbox_center(reference)
        cand_cx, cand_cy = self._bbox_center(candidate)
        center_dist = math.hypot(cand_cx - ref_cx, cand_cy - ref_cy)
        if center_dist > self.stable_center_tolerance_px:
            return False

        ref_w, ref_h = self._bbox_size(reference)
        cand_w, cand_h = self._bbox_size(candidate)
        width_ratio_delta = abs((cand_w / ref_w) - 1.0)
        height_ratio_delta = abs((cand_h / ref_h) - 1.0)
        return (
            width_ratio_delta <= self.stable_size_ratio_tolerance
            and height_ratio_delta <= self.stable_size_ratio_tolerance
        )

    def _match_detection(self, reference: Detection2D, detections: List[Detection2D]) -> Optional[Detection2D]:
        best_det = None
        best_score = float('inf')

        ref_cx, ref_cy = self._bbox_center(reference)
        ref_w, ref_h = self._bbox_size(reference)
        ref_cls = ''
        if reference.results:
            ref_cls = str(reference.results[0].hypothesis.class_id).strip().lower()

        for det in detections:
            det_cls = ''
            if det.results:
                det_cls = str(det.results[0].hypothesis.class_id).strip().lower()
            if ref_cls and det_cls and ref_cls != det_cls:
                continue

            cx, cy = self._bbox_center(det)
            center_dist = math.hypot(cx - ref_cx, cy - ref_cy)
            if center_dist > self.lock_follow_max_center_distance_px:
                continue

            w, h = self._bbox_size(det)
            width_ratio_delta = abs((w / ref_w) - 1.0)
            height_ratio_delta = abs((h / ref_h) - 1.0)
            if (
                width_ratio_delta > self.lock_follow_max_size_ratio_delta
                or height_ratio_delta > self.lock_follow_max_size_ratio_delta
            ):
                continue

            size_penalty = 20.0 * (width_ratio_delta + height_ratio_delta)
            score = center_dist + size_penalty - (10.0 * self._score(det))
            if score < best_score:
                best_score = score
                best_det = det

        return best_det

    def _reset_candidate(self) -> None:
        self._candidate_det = None
        self._candidate_since = 0.0
        self._display_det = None
        self._display_hold_frames = 0

    def _lock_selection(self, det: Detection2D) -> None:
        self._locked_det = det
        self._lock_time = time.time()
        self._last_selected_det = det
        self._last_selected_time = self._lock_time
        self._display_det = det
        self._display_hold_frames = self.lock_keep_frames_on_jitter
        self.get_logger().info('Locked handbag candidate automatically; manual unlock required to change target')

    def _unlock_selection(self) -> None:
        self._locked_det = None
        self._lock_time = 0.0
        self._last_selected_det = None
        self._last_selected_time = 0.0
        self._reset_candidate()

    def _unlock_selection_service(self, request, response):
        del request
        if self._locked_det is None:
            response.success = True
            response.message = 'No locked handbag candidate to clear'
            return response

        self._unlock_selection()
        response.success = True
        response.message = 'Unlocked handbag candidate'
        self.get_logger().info(response.message)
        return response

    def _draw_detection(self, img: np.ndarray, det: Detection2D, color, text: str) -> None:
        cx = int(det.bbox.center.position.x)
        cy = int(det.bbox.center.position.y)
        w = int(det.bbox.size_x)
        h = int(det.bbox.size_y)
        x1 = max(0, cx - w // 2)
        y1 = max(0, cy - h // 2)
        x2 = min(img.shape[1] - 1, cx + w // 2)
        y2 = min(img.shape[0] - 1, cy + h // 2)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        cv2.circle(img, (cx, cy), 5, color, -1)
        cv2.putText(img, text, (x1, max(15, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    def _publish_selected(self, header, selected: Optional[Detection2D]) -> None:
        out_array = Detection2DArray()
        out_array.header = header

        if selected is not None:
            self._last_selected_det = selected
            self._last_selected_time = time.time()
            out_array.detections.append(selected)
            self.selected_pub.publish(selected)
            self.selected_flag_pub.publish(Bool(data=True))
        else:
            age = time.time() - self._last_selected_time
            if (
                self.reuse_last_selected_when_missing
                and self._last_selected_det is not None
                and age <= self.selection_hold_sec
            ):
                out_array.detections.append(self._last_selected_det)
                self.selected_pub.publish(self._last_selected_det)
                self.selected_flag_pub.publish(Bool(data=True))
            else:
                self.selected_flag_pub.publish(Bool(data=False))

        self.selected_array_pub.publish(out_array)

    def _check_detection_service(self, request, response):
        del request
        current_det = self._locked_det if self._locked_det is not None else self._last_selected_det
        age = time.time() - self._last_selected_time
        if current_det is not None and (
            self._locked_det is not None or age <= self.max_detection_age_sec
        ):
            cls_name = 'unknown'
            score = 0.0
            if current_det.results:
                cls_name = str(current_det.results[0].hypothesis.class_id)
                score = float(current_det.results[0].hypothesis.score)
            response.success = True
            lock_suffix = ' [LOCKED]' if self._locked_det is not None else ''
            response.message = f'detect successful: {cls_name} ({score:.2f}){lock_suffix}'
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = 'detect failed: no recent selected bag'
            self.get_logger().warn(response.message)
        return response

    def _update_auto_selection(self, detections: List[Detection2D]) -> Tuple[Optional[Detection2D], str]:
        now = time.time()

        if self._locked_det is not None:
            matched_locked = self._match_detection(self._locked_det, detections)
            if matched_locked is not None:
                self._locked_det = matched_locked
                self._last_selected_det = matched_locked
                self._last_selected_time = now
            self._display_det = self._locked_det
            return self._locked_det, 'LOCKED'

        if not detections:
            self._reset_candidate()
            return None, 'No valid bag detections'

        if self._candidate_det is None:
            self._candidate_det = detections[0]
            self._candidate_since = now
        else:
            matched = self._match_detection(self._candidate_det, detections)
            if matched is not None and self._same_track(self._candidate_det, matched):
                self._candidate_det = matched
            else:
                self._candidate_det = detections[0]
                self._candidate_since = now

        elapsed = now - self._candidate_since
        self._display_det = self._candidate_det
        self._display_hold_frames = self.lock_keep_frames_on_jitter

        if elapsed >= self.auto_lock_stable_time_sec:
            if self.enable_hard_lock and self._candidate_det is not None:
                self._lock_selection(self._candidate_det)
                return self._locked_det, 'LOCKED'
            return self._candidate_det, 'READY'

        return self._candidate_det, f'STABLE {elapsed:.1f}/{self.auto_lock_stable_time_sec:.1f}s'

    def _image_cb(self, msg: CompressedImage) -> None:
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is None:
                return

            detections = self._valid_detections()
            selected_det, status_text = self._update_auto_selection(detections)

            for det in detections:
                label = 'bag'
                if det.results:
                    cls_name = str(det.results[0].hypothesis.class_id)
                    score = float(det.results[0].hypothesis.score)
                    label = f'{cls_name} {score:.2f}'
                self._draw_detection(img, det, (0, 220, 0), label)

            if self._display_det is not None:
                label = 'candidate'
                color = (0, 200, 200)
                if self._locked_det is not None:
                    label = 'LOCKED candidate'
                    color = (0, 165, 255)
                elif self._display_det.results:
                    cls_name = str(self._display_det.results[0].hypothesis.class_id)
                    score = float(self._display_det.results[0].hypothesis.score)
                    label = f'candidate {cls_name} {score:.2f}'
                self._draw_detection(img, self._display_det, color, label)
            else:
                cv2.putText(
                    img,
                    'No candidate selected',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                )

            if self._locked_det is not None:
                cv2.putText(
                    img,
                    f'LOCKED - call {self.unlock_service} to unlock',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 165, 255),
                    2,
                )
            else:
                cv2.putText(
                    img,
                    f'Auto lock: {status_text}',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 180, 255),
                    2,
                )

            self._publish_selected(msg.header, self._display_det)

            out_msg = CompressedImage()
            out_msg.header = msg.header
            out_msg.format = 'jpeg'
            ok, encoded = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                out_msg.data = encoded.tobytes()
                self.viz_pub.publish(out_msg)

            if self.show_window:
                cv2.imshow(self.window_name, img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.get_logger().info('Window closed by user.')
                    rclpy.shutdown()

        except Exception as exc:
            self.get_logger().error(f'Image callback error: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandBagSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
