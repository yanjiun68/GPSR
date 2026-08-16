#!/usr/bin/env python3
import json

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from receptionist_interfaces.msg import GuestAppearance
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from ultralytics import YOLO

from face_detection.appearance_estimators import (
    HsvShirtColorEstimator,
    TemporalCategoricalVoter,
    YoloAttributeEstimator,
    best_attribute_confidence,
    crop_person_roi,
    get_person_roi,
    get_upper_body_roi,
)
from face_detection.paths import default_detection_model_paths, default_glasses_model_path


def as_bool(value):
    """Special function: Parse ROS boolean parameters supplied as bools or strings."""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def binary_attribute_from_confidence(confidence):
    if confidence <= 0.0:
        return GuestAppearance.UNKNOWN, 0.0
    return GuestAppearance.YES, confidence


class AppearanceNode(Node):
    """Special function: Associate face and clothing attributes with one selected person ROI."""

    def __init__(self):
        """Special function: Configure CPU-first, debug-off human appearance inference."""
        super().__init__('appearance_node')

        # Fallback YOLO model used when yolo_model_paths is empty.
        self.declare_parameter('yolo_model_path', default_glasses_model_path())
        # Ordered list of YOLO models used to detect visual attributes.
        self.declare_parameter('yolo_model_paths', default_detection_model_paths())
        # Minimum YOLO confidence required before an attribute is accepted.
        self.declare_parameter('yolo_conf_threshold', 0.4)
        # Camera color image topic consumed for appearance detection.
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        # Face JSON topic used for face bbox and face-depth context.
        self.declare_parameter('face_topic', '/perception/largest_face')
        # GuestAppearance topic published for downstream capture/recognition nodes.
        self.declare_parameter('appearance_topic', '/appearance/current_estimate')
        # Optional JSON debug topic with raw attribute and shirt ROI details.
        self.declare_parameter('debug_topic', '/perception/appearance')
        # JSON stream metadata published by detection_receptionist/camera_node.
        self.declare_parameter('stream_info_topic', '/camera/stream_info')
        # Dedicated stable-recognition JSON topic displayed in the appearance GUI.
        self.declare_parameter(
            'recognized_identity_topic',
            '/appearance/recognized_human_identity',
        )
        # Maximum silence before the GUI marks a cached identity state as stale.
        self.declare_parameter('recognized_identity_max_age_sec', 2.0)
        # Draw camera refresh rate and resolution from stream_info_topic in the GUI.
        self.declare_parameter('show_stream_info_overlay', False)
        # Explicit inference device prevents appearance models from taking local-LLM GPU memory.
        # Section: Ultralytics uses CUDA GPU 0 by default for appearance inference.
        self.declare_parameter('appearance_device', '0')
        # Maximum age keeps a stale face box from being applied to a new camera frame.
        self.declare_parameter('max_face_age_sec', 0.5)
        # Boolean output topic that becomes true when glasses are confidently detected.
        self.declare_parameter('glasses_detected_topic', '/appearance/glasses_detected')
        # Boolean output topic that becomes true when shirt color is confidently known.
        self.declare_parameter('shirt_color_known_topic', '/appearance/shirt_color_known')
        # Confidence threshold used for the glasses_detected boolean topic.
        self.declare_parameter('glasses_detected_threshold', 0.5)
        # Confidence threshold used for the shirt_color_known boolean topic.
        self.declare_parameter('shirt_color_known_threshold', 0.3)
        # Number of recent shirt-color observations kept for temporal smoothing.
        self.declare_parameter('shirt_color_history_size', 7)
        # Minimum matching shirt-color observations needed before accepting a stable color.
        self.declare_parameter('shirt_color_min_observations', 3)
        # Minimum per-frame shirt-color confidence before it contributes to history.
        self.declare_parameter('shirt_color_min_confidence', 0.15)
        # Seconds to keep publishing the last stable shirt color after weak/no observations.
        self.declare_parameter('shirt_color_hold_sec', 2.0)
        # Enables the OpenCV visual debug panel when a desktop display is available.
        self.declare_parameter('show_gui', False)
        # OpenCV window title used when show_gui is enabled.
        self.declare_parameter('window_name', 'Receptionist Appearance')

        # Resolve model-path parameters before loading YOLO detectors.
        self.yolo_model_path = str(self.get_parameter('yolo_model_path').value)
        self.yolo_model_paths = [
            str(path) for path in self.get_parameter('yolo_model_paths').value if str(path).strip()
        ]
        if not self.yolo_model_paths:
            self.yolo_model_paths = [self.yolo_model_path]
        # Resolve threshold and topic parameters into typed instance fields.
        self.yolo_conf_threshold = float(self.get_parameter('yolo_conf_threshold').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.face_topic = str(self.get_parameter('face_topic').value)
        self.appearance_topic = str(self.get_parameter('appearance_topic').value)
        self.debug_topic = str(self.get_parameter('debug_topic').value)
        self.stream_info_topic = str(self.get_parameter('stream_info_topic').value)
        self.recognized_identity_topic = str(
            self.get_parameter('recognized_identity_topic').value
        )
        self.recognized_identity_max_age_sec = max(
            0.1,
            float(self.get_parameter('recognized_identity_max_age_sec').value),
        )
        self.show_stream_info_overlay = as_bool(
            self.get_parameter('show_stream_info_overlay').value
        )
        self.appearance_device = str(self.get_parameter('appearance_device').value).strip() or '0'
        self.max_face_age_sec = max(0.0, float(self.get_parameter('max_face_age_sec').value))
        self.glasses_detected_topic = str(self.get_parameter('glasses_detected_topic').value)
        self.shirt_color_known_topic = str(self.get_parameter('shirt_color_known_topic').value)
        self.glasses_detected_threshold = float(
            self.get_parameter('glasses_detected_threshold').value
        )
        self.shirt_color_known_threshold = float(
            self.get_parameter('shirt_color_known_threshold').value
        )
        self.shirt_color_history_size = max(
            1, int(self.get_parameter('shirt_color_history_size').value)
        )
        self.shirt_color_min_observations = max(
            1, int(self.get_parameter('shirt_color_min_observations').value)
        )
        self.shirt_color_min_confidence = float(
            self.get_parameter('shirt_color_min_confidence').value
        )
        self.shirt_color_hold_sec = max(
            0.0, float(self.get_parameter('shirt_color_hold_sec').value)
        )
        self.show_gui = as_bool(self.get_parameter('show_gui').value)
        self.gui_enabled = self.show_gui
        self.window_name = str(self.get_parameter('window_name').value)

        # Runtime state populated by image and face-topic callbacks.
        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_frame_stamp_ns = None
        self.last_processed_frame_stamp_ns = None
        self.latest_face = None
        self.latest_stream_info = None
        self.latest_recognized_identity = None
        self.latest_recognized_identity_received_ns = 0
        self.received_first_image = False
        self.received_first_face = False
        self.received_first_stream_info = False
        self.received_first_recognized_identity = False
        self.logged_missing_face_input = False
        self.completed_first_estimate = False
        self.last_waiting_log_ns = 0
        self.shirt_color_voter = TemporalCategoricalVoter(
            self.shirt_color_history_size,
            self.shirt_color_min_observations,
            self.shirt_color_min_confidence,
        )
        self.stable_shirt_color = ''
        self.stable_shirt_color_confidence = 0.0
        self.stable_shirt_color_updated_ns = 0

        # Main appearance outputs consumed by capture, recognition, and simple boolean checks.
        self.appearance_pub = self.create_publisher(GuestAppearance, self.appearance_topic, 10)
        self.glasses_detected_pub = self.create_publisher(Bool, self.glasses_detected_topic, 10)
        self.shirt_color_known_pub = self.create_publisher(Bool, self.shirt_color_known_topic, 10)
        self.debug_pub = None
        if self.debug_topic:
            self.debug_pub = self.create_publisher(String, self.debug_topic, 10)

        # Image frames drive the periodic appearance estimation.
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        # Face messages provide bbox/depth context for shirt ROI and GUI display.
        self.face_sub = self.create_subscription(
            String,
            self.face_topic,
            self.face_callback,
            10,
        )
        self.stream_info_sub = self.create_subscription(
            String,
            self.stream_info_topic,
            self.stream_info_callback,
            10,
        )
        # Section: Identity is consumed independently from the face/appearance estimate topics.
        self.recognized_identity_sub = self.create_subscription(
            String,
            self.recognized_identity_topic,
            self.recognized_identity_callback,
            10,
        )

        self.get_logger().info(f'Loading YOLO detectors from {", ".join(self.yolo_model_paths)}')
        self.yolo_models = self._load_yolo_models()
        self.attribute_estimator = YoloAttributeEstimator(
            self.yolo_models,
            self.yolo_conf_threshold,
            self.appearance_device,
        )
        self.shirt_color_estimator = HsvShirtColorEstimator()

        # Run appearance estimation at 5 Hz using the latest received image and face data.
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info(
            'Appearance node started. '
            f'image_topic={self.image_topic}, face_topic={self.face_topic}, '
            f'appearance_topic={self.appearance_topic}, '
            f'recognized_identity_topic={self.recognized_identity_topic}, '
            f'stream_info_topic={self.stream_info_topic}, '
            f'detector_count={len(self.yolo_models)}, show_gui={self.show_gui}'
        )
        self.get_logger().info(
            'YOLO detectors ready. Waiting for image messages to start appearance estimates. '
            'Face input is optional and only improves shirt-color estimation.'
        )

    def _load_yolo_models(self):
        """Special function: Load only configured human-attribute model files."""
        yolo_models = []
        total_models = len(self.yolo_model_paths)
        for index, model_path in enumerate(self.yolo_model_paths, start=1):
            self.get_logger().info(f'Loading YOLO detector {index}/{total_models}: {model_path}')
            yolo_models.append(YOLO(model_path))
        return yolo_models

    def _update_stable_shirt_color(self, color, confidence):
        """Special function: Update the reusable temporal voter and apply hold semantics."""
        now_ns = self.get_clock().now().nanoseconds
        stable_color, stable_confidence = self.shirt_color_voter.update(color, confidence)
        # Section: Only a new meaningful frame extends the stable estimate's hold time.
        meaningful = color and color != 'unknown' and confidence >= self.shirt_color_min_confidence
        if stable_color != 'unknown' and meaningful:
            self.stable_shirt_color = stable_color
            self.stable_shirt_color_confidence = stable_confidence
            self.stable_shirt_color_updated_ns = now_ns

        if not self.stable_shirt_color:
            return '', 0.0

        hold_ns = int(self.shirt_color_hold_sec * 1_000_000_000.0)
        if hold_ns > 0 and (now_ns - self.stable_shirt_color_updated_ns) <= hold_ns:
            return self.stable_shirt_color, self.stable_shirt_color_confidence

        if hold_ns == 0:
            return self.stable_shirt_color, self.stable_shirt_color_confidence

        return '', 0.0

    def image_callback(self, msg):
        """Special function: Cache the latest image and its ROS timestamp."""
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_frame_stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        )
        if not self.received_first_image:
            self.received_first_image = True
            self.get_logger().info(f'Received first image on {self.image_topic}')

    def face_callback(self, msg):
        try:
            self.latest_face = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Failed to parse face message on {self.face_topic}: {exc}')
            self.latest_face = None
            return

        if not self.received_first_face:
            self.received_first_face = True
            detected = bool(self.latest_face.get('detected', False))
            self.get_logger().info(
                f'Received first face message on {self.face_topic} (detected={detected})'
            )

    def stream_info_callback(self, msg):
        try:
            self.latest_stream_info = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(
                f'Failed to parse stream info message on {self.stream_info_topic}: {exc}'
            )
            self.latest_stream_info = None
            return

        if not self.received_first_stream_info:
            self.received_first_stream_info = True
            color_info = self.latest_stream_info.get('color', {})
            self.get_logger().info(
                f'Received first stream info on {self.stream_info_topic}: '
                f'{color_info.get("resolution", "unknown")} '
                f'@ {float(color_info.get("refresh_rate_hz", 0.0) or 0.0):.2f} Hz'
            )

    def recognized_identity_callback(self, msg):
        """Special function: Cache the latest stable-recognition state for GUI debugging."""
        # Section: Reject malformed debug input without disturbing appearance processing.
        try:
            identity = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(
                f'Failed to parse identity message on {self.recognized_identity_topic}: {exc}'
            )
            return
        if not isinstance(identity, dict):
            self.get_logger().warning(
                f'Ignored non-object identity message on {self.recognized_identity_topic}.'
            )
            return
        self.latest_recognized_identity = identity
        self.latest_recognized_identity_received_ns = self.get_clock().now().nanoseconds
        if not self.received_first_recognized_identity:
            self.received_first_recognized_identity = True
            self.get_logger().info(
                f'Received first identity state on {self.recognized_identity_topic}: '
                f'{identity.get("state", "unknown")}'
            )

    def timer_callback(self):
        """Special function: Estimate attributes only within the selected current-person ROI."""
        if self.latest_frame is None:
            self._log_waiting_for_inputs()
            return
        if self.latest_frame_stamp_ns == self.last_processed_frame_stamp_ns:
            return
        self.last_processed_frame_stamp_ns = self.latest_frame_stamp_ns

        debug_frame = self.latest_frame.copy() if self.gui_enabled else None
        debug_out = {
            'valid_face': False,
            'shirt_color': 'unknown',
            'attributes': {},
            'roi_box': [],
        }
        appearance_msg = GuestAppearance()
        appearance_msg.glasses = GuestAppearance.UNKNOWN
        appearance_msg.hat = GuestAppearance.UNKNOWN
        appearance_msg.face_mask = GuestAppearance.UNKNOWN
        appearance_msg.shirt_color = ''
        appearance_msg.glasses_confidence = 0.0
        appearance_msg.hat_confidence = 0.0
        appearance_msg.face_mask_confidence = 0.0
        appearance_msg.shirt_color_confidence = 0.0

        attribute_scores = {}
        face_data = self.latest_face if isinstance(self.latest_face, dict) else None
        face_stamp_ns = None
        if face_data is not None:
            face_stamp_ns = (
                int(face_data.get('stamp_sec', 0)) * 1_000_000_000
                + int(face_data.get('stamp_nanosec', 0))
            )
        face_age_sec = float('inf')
        if face_stamp_ns and self.latest_frame_stamp_ns is not None:
            face_age_sec = abs(self.latest_frame_stamp_ns - face_stamp_ns) / 1_000_000_000.0
        if (
            face_data is not None
            and face_data.get('detected', False)
            and face_age_sec <= self.max_face_age_sec
        ):
            face_bbox = face_data['bbox']
            selected_person_bbox = face_data.get('selected_roi', [])
            if len(selected_person_bbox) == 4:
                person_roi, person_roi_box = crop_person_roi(
                    self.latest_frame,
                    selected_person_bbox,
                )
            else:
                person_roi, person_roi_box = get_person_roi(self.latest_frame, face_bbox)
            if person_roi is not None:
                attribute_scores = self.attribute_estimator.estimate(person_roi)
            # Section: Attribute decisions are derived only from the associated person crop.
            glasses_confidence = best_attribute_confidence(
                attribute_scores,
                ('glasses', 'eyeglasses', 'spectacles', 'sunglasses'),
            )
            hat_confidence = best_attribute_confidence(
                attribute_scores,
                ('hat', 'cap', 'beanie', 'helmet'),
            )
            face_mask_confidence = best_attribute_confidence(
                attribute_scores,
                ('face_mask', 'facemask', 'mask'),
            )
            (
                appearance_msg.glasses,
                appearance_msg.glasses_confidence,
            ) = binary_attribute_from_confidence(glasses_confidence)
            appearance_msg.hat, appearance_msg.hat_confidence = (
                binary_attribute_from_confidence(hat_confidence)
            )
            (
                appearance_msg.face_mask,
                appearance_msg.face_mask_confidence,
            ) = binary_attribute_from_confidence(face_mask_confidence)
            roi, roi_box = get_upper_body_roi(
                self.latest_frame,
                face_bbox,
                person_bbox=person_roi_box,
            )
            shirt_color, shirt_color_confidence = self.shirt_color_estimator.estimate(roi)
            stable_shirt_color, stable_shirt_color_confidence = self._update_stable_shirt_color(
                shirt_color,
                shirt_color_confidence,
            )
            appearance_msg.shirt_color = stable_shirt_color
            appearance_msg.shirt_color_confidence = stable_shirt_color_confidence

            debug_out['valid_face'] = True
            debug_out['shirt_color'] = stable_shirt_color or shirt_color
            debug_out['shirt_color_raw'] = shirt_color
            debug_out['shirt_color_raw_confidence'] = shirt_color_confidence
            debug_out['shirt_color_stable_confidence'] = stable_shirt_color_confidence
            debug_out['attributes'] = attribute_scores
            debug_out['roi_box'] = list(roi_box) if roi_box is not None else []
            debug_out['person_roi_box'] = list(person_roi_box) if person_roi_box is not None else []
        else:
            debug_out['attributes'] = attribute_scores
            stable_shirt_color, stable_shirt_color_confidence = self._update_stable_shirt_color(
                '',
                0.0,
            )
            appearance_msg.shirt_color = stable_shirt_color
            appearance_msg.shirt_color_confidence = stable_shirt_color_confidence
            debug_out['shirt_color'] = stable_shirt_color or 'unknown'
            debug_out['shirt_color_stable_confidence'] = stable_shirt_color_confidence
            if not self.received_first_face and not self.logged_missing_face_input:
                self.get_logger().info(
                    f'No face messages received yet on {self.face_topic}; '
                    'continuing with image-only appearance estimation.'
                )
                self.logged_missing_face_input = True

        # Section: Skip all visualization work unless GUI output was explicitly requested.
        if debug_frame is not None:
            self._draw_attribute_labels(debug_frame, attribute_scores)
            if debug_out['valid_face']:
                self._draw_face_and_roi(
                    debug_frame,
                    face_data['bbox'],
                    tuple(debug_out['roi_box']) if debug_out['roi_box'] else None,
                    debug_out['shirt_color'],
                    float(debug_out.get('shirt_color_stable_confidence', 0.0)),
                )
            self._draw_status_panel(debug_frame, appearance_msg, debug_out)
            self._draw_stream_info_overlay(debug_frame)
        appearance_msg.stamp = self.get_clock().now().to_msg()
        self.appearance_pub.publish(appearance_msg)
        self._publish_boolean_outputs(appearance_msg)

        if self.debug_pub is not None:
            debug_msg = String()
            debug_msg.data = json.dumps(debug_out)
            self.debug_pub.publish(debug_msg)

        if not self.completed_first_estimate:
            self.completed_first_estimate = True
            self.get_logger().info(
                f'Published first appearance estimate on {self.appearance_topic} '
                f'(face_detected={debug_out["valid_face"]})'
            )

        if debug_frame is not None:
            self._show_gui_frame(debug_frame)

    def _draw_attribute_labels(self, frame, attribute_scores):
        if not attribute_scores:
            return

        y = 28
        for label, confidence in sorted(attribute_scores.items()):
            cv2.putText(
                frame,
                f'{label}: {confidence:.2f}',
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 0, 255),
                2,
            )
            y += 24

    def _draw_face_and_roi(self, frame, face_bbox, roi_box, shirt_color, shirt_confidence):
        x1, y1, x2, y2 = [int(value) for value in face_bbox]
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            frame,
            'face',
            (x1, max(y1 - 10, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

        if roi_box is None:
            return

        rx1, ry1, rx2, ry2 = [int(value) for value in roi_box]
        cv2.rectangle(frame, (rx1, ry1), (rx2, ry2), (255, 0, 0), 2)
        cv2.putText(
            frame,
            f'shirt: {shirt_color or "unknown"} {shirt_confidence:.2f}',
            (rx1, max(ry1 - 10, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 0, 0),
            2,
        )

    def _draw_stream_info_overlay(self, frame):
        if not self.show_stream_info_overlay:
            return

        if not isinstance(self.latest_stream_info, dict):
            return

        color_info = self.latest_stream_info.get('color', {})
        resolution = str(color_info.get('resolution') or 'unknown')
        refresh_rate_hz = float(color_info.get('refresh_rate_hz', 0.0) or 0.0)
        average_fps = float(color_info.get('average_fps', 0.0) or 0.0)
        topic = str(color_info.get('topic') or self.image_topic)
        source = str(self.latest_stream_info.get('camera_source') or 'unknown')

        lines = [
            f'Input: {resolution} @ {refresh_rate_hz:.1f} Hz',
            f'Avg: {average_fps:.1f} FPS',
            f'Source: {source}',
            f'Topic: {topic}',
        ]

        height, width = frame.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        line_height = 21
        padding = 8
        text_width = max(
            cv2.getTextSize(line, font, font_scale, thickness)[0][0] for line in lines
        )
        panel_width = min(width - 16, text_width + padding * 2)
        panel_height = len(lines) * line_height + padding
        x1 = max(8, width - panel_width - 8)
        y1 = 8
        x2 = min(width - 8, x1 + panel_width)
        y2 = min(height - 8, y1 + panel_height)

        overlay = frame.copy()
        cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0.0, frame)

        y = y1 + padding + 13
        for line in lines:
            cv2.putText(
                frame,
                line,
                (x1 + padding, y),
                font,
                font_scale,
                (255, 255, 255),
                thickness,
            )
            y += line_height

    def _draw_status_panel(self, frame, appearance_msg, debug_out):
        height, width = frame.shape[:2]
        # Section: Reserve one additional row for the dedicated recognition identity state.
        panel_height = 116
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, height - panel_height), (width, height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.45, frame, 0.55, 0.0, frame)

        glasses_text = 'yes' if appearance_msg.glasses == GuestAppearance.YES else 'unknown'
        if appearance_msg.glasses == GuestAppearance.NO:
            glasses_text = 'no'
        hat_text = 'yes' if appearance_msg.hat == GuestAppearance.YES else 'unknown'
        if appearance_msg.hat == GuestAppearance.NO:
            hat_text = 'no'
        mask_text = 'yes' if appearance_msg.face_mask == GuestAppearance.YES else 'unknown'
        if appearance_msg.face_mask == GuestAppearance.NO:
            mask_text = 'no'
        face_data = self.latest_face if isinstance(self.latest_face, dict) else {}
        depth_text = 'unknown'
        if face_data.get('depth_valid', False):
            depth_text = f'{float(face_data.get("depth_m", 0.0)):.2f}m'

        # Section: Never leave a recognized name frozen in the GUI after its publisher stops.
        identity_age_ns = (
            self.get_clock().now().nanoseconds - self.latest_recognized_identity_received_ns
        )
        identity_is_fresh = (
            self.latest_recognized_identity_received_ns > 0
            and identity_age_ns
            <= int(self.recognized_identity_max_age_sec * 1_000_000_000)
        )
        identity = (
            self.latest_recognized_identity
            if identity_is_fresh and isinstance(self.latest_recognized_identity, dict)
            else {}
        )
        identity_state = str(identity.get('state', 'unavailable')).strip() or 'unavailable'
        identity_label = str(identity.get('name', '')).strip()
        identity_profile_id = str(identity.get('profile_id', '')).strip()
        if identity_state == 'known':
            identity_label = identity_label or identity_profile_id or 'unknown'
            if identity_profile_id and identity_profile_id != identity_label:
                identity_label = f'{identity_label} [{identity_profile_id}]'
            identity_text = (
                f'identity={identity_label} score={float(identity.get("match_score", -1.0)):.2f}'
            )
        else:
            identity_text = f'identity={identity_state}'

        lines = [
            identity_text,
            f'face_detected={debug_out["valid_face"]} depth={depth_text}',
            f'shirt={appearance_msg.shirt_color or "unknown"} '
            f'({appearance_msg.shirt_color_confidence:.2f})',
            f'glasses={glasses_text} ({appearance_msg.glasses_confidence:.2f}) '
            f'hat={hat_text} ({appearance_msg.hat_confidence:.2f}) '
            f'mask={mask_text} ({appearance_msg.face_mask_confidence:.2f})',
        ]

        y = height - panel_height + 24
        for line in lines:
            cv2.putText(
                frame,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
            )
            y += 24

    def _show_gui_frame(self, frame):
        if not self.gui_enabled:
            return

        try:
            cv2.imshow(self.window_name, frame)
            key = cv2.waitKey(1) & 0xFF
        except cv2.error as exc:
            self.get_logger().warning(
                f'OpenCV GUI is unavailable ({exc}); disabling appearance GUI.'
            )
            self.gui_enabled = False
            return

        if key in (ord('q'), 27):
            self.get_logger().info('Appearance GUI disabled by keyboard request.')
            self.gui_enabled = False
            try:
                cv2.destroyWindow(self.window_name)
            except cv2.error:
                pass

    def _publish_boolean_outputs(self, appearance_msg):
        glasses_detected = Bool()
        glasses_detected.data = (
            appearance_msg.glasses == GuestAppearance.YES
            and appearance_msg.glasses_confidence >= self.glasses_detected_threshold
        )
        self.glasses_detected_pub.publish(glasses_detected)

        shirt_color_known = Bool()
        shirt_color_known.data = (
            bool(appearance_msg.shirt_color)
            and appearance_msg.shirt_color.lower() != 'unknown'
            and appearance_msg.shirt_color_confidence >= self.shirt_color_known_threshold
        )
        self.shirt_color_known_pub.publish(shirt_color_known)

    def _log_waiting_for_inputs(self):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_waiting_log_ns < 5_000_000_000:
            return

        missing_topics = []
        if self.latest_frame is None:
            missing_topics.append(self.image_topic)

        self.get_logger().warn(
            'Waiting for required image topic before running appearance estimation: '
            + ', '.join(missing_topics)
        )
        self.last_waiting_log_ns = now_ns

    def destroy_node(self):
        if self.gui_enabled:
            try:
                cv2.destroyWindow(self.window_name)
            except cv2.error:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AppearanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
