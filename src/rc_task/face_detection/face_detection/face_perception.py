#!/usr/bin/env python3
"""CPU-first throttled YuNet/SFace perception with compatible JSON output."""

import json
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from face_detection.face_backend import (
    DEFAULT_SFACE_MODEL,
    DEFAULT_YUNET_MODEL,
    EMBEDDING_BACKEND,
    FaceBackend,
    resolve_model_path,
)
from face_detection.paths import package_share_dir
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Section: Tracker integration is optional unless tracked_person mode is selected.
try:
    from vision.msg import RecognizedPerson
except ImportError:
    RecognizedPerson = None


# Section: Stateless helpers remain importable for camera-free unit tests.
def as_bool(value):
    """Special function: Parse ROS boolean parameters supplied as bools or strings."""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def observation_stamp_ns(message):
    """Special function: Convert a ROS Header stamp into nanoseconds or None."""
    seconds = int(message.header.stamp.sec)
    nanoseconds = int(message.header.stamp.nanosec)
    if seconds == 0 and nanoseconds == 0:
        return None
    return seconds * 1_000_000_000 + nanoseconds


def bbox_area_xywh(bbox):
    """Special function: Return non-negative area for an x/y/width/height face box."""
    return max(0.0, float(bbox[2])) * max(0.0, float(bbox[3]))


def select_observation(observations):
    """Special function: Select the largest face only after any configured person ROI is applied."""
    candidates = list(observations or [])
    return max(candidates, key=lambda item: bbox_area_xywh(item.bbox)) if candidates else None


def xywh_to_xyxy(bbox):
    """Special function: Preserve the legacy JSON xyxy face bounding-box convention."""
    x, y, width, height = [float(value) for value in bbox]
    return [x, y, x + width, y + height]


def bbox_iou_xyxy(first, second):
    """Special function: Measure overlap between two full-frame xyxy boxes."""
    ax1, ay1, ax2, ay2 = [float(value) for value in first]
    bx1, by1, bx2, by2 = [float(value) for value in second]
    intersection = max(0.0, min(ax2, bx2) - max(ax1, bx1)) * max(
        0.0, min(ay2, by2) - max(ay1, by1)
    )
    first_area = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    second_area = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


def associate_faces_to_tracks(
    face_boxes,
    person_boxes,
    minimum_iou=0.01,
    ambiguity_margin=0.05,
):
    """Special function: Associate faces to people one-to-one with ambiguity rejection."""
    # Section: Face-center containment outranks fallback IoU as the primary geometric cue.
    eligible_by_face = {}
    for face_index, face_box in enumerate(face_boxes):
        x1, y1, x2, y2 = [float(value) for value in face_box]
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        pairs = []
        for person_index, person_box in enumerate(person_boxes):
            px1, py1, px2, py2 = [float(value) for value in person_box]
            contained = px1 <= center_x <= px2 and py1 <= center_y <= py2
            iou = bbox_iou_xyxy(face_box, person_box)
            if contained or iou >= float(minimum_iou):
                pairs.append((1 if contained else 0, iou, face_index, person_index))
        pairs.sort(key=lambda item: (-item[0], -item[1], item[3]))
        if len(pairs) > 1:
            best = pairs[0]
            second = pairs[1]
            if best[0] == second[0] and best[1] - second[1] < float(ambiguity_margin):
                continue
        if pairs:
            eligible_by_face[face_index] = pairs[0]

    # Section: Descending score assignment prevents one track receiving multiple faces.
    ranked = sorted(
        eligible_by_face.values(),
        key=lambda item: (-item[0], -item[1], item[2], item[3]),
    )
    assignments = {}
    used_tracks = set()
    for _contained, _iou, face_index, person_index in ranked:
        if person_index in used_tracks:
            continue
        assignments[face_index] = person_index
        used_tracks.add(person_index)
    return assignments


def median_depth_m(depth_image, bbox_xyxy, radius_px):
    """Special function: Compute robust non-zero median depth around the selected face center."""
    height, width = depth_image.shape[:2]
    x1, y1, x2, y2 = [float(value) for value in bbox_xyxy]
    center_x = int(np.clip((x1 + x2) / 2.0, 0, width - 1))
    center_y = int(np.clip((y1 + y2) / 2.0, 0, height - 1))
    radius = max(0, int(radius_px))
    patch = depth_image[
        max(0, center_y - radius):min(height, center_y + radius + 1),
        max(0, center_x - radius):min(width, center_x + radius + 1),
    ]
    values = patch.astype(np.float32).reshape(-1)
    values = values[np.isfinite(values)]
    values = values[values > 0.0]
    if values.size == 0:
        return 0.0, center_x, center_y, 0
    median = float(np.median(values))
    if depth_image.dtype == np.uint16:
        median *= 0.001
    return median, center_x, center_y, int(values.size)


class FacePerceptionNode(Node):
    """Special node: Cache camera frames and run bounded-rate YuNet/SFace inference."""

    def __init__(self):
        """Special function: Configure model paths, CPU policy, topics, ROI selection, and timer."""
        super().__init__('face_perception_node')

        # Section: YuNet/SFace parameters mirror OpenCV's APIs and competition-safe defaults.
        self.declare_parameter('yunet_model_path', '')
        self.declare_parameter('sface_model_path', '')
        self.declare_parameter('face_model_dir', '')
        self.declare_parameter('yunet_score_threshold', 0.8)
        self.declare_parameter('yunet_nms_threshold', 0.3)
        self.declare_parameter('yunet_top_k', 5000)
        self.declare_parameter('opencv_dnn_backend', 'default')
        self.declare_parameter('opencv_dnn_target', 'cpu')
        self.declare_parameter('face_inference_rate_hz', 5.0)
        self.declare_parameter('recognition_rate_hz', 2.0)
        self.declare_parameter('max_frame_age_sec', 0.5)
        self.declare_parameter('face_selection_mode', 'largest')
        self.declare_parameter('target_roi', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('tracked_person_topic', '/human_identity/recognized_people')
        self.declare_parameter('target_track_id', 0)
        self.declare_parameter('tracked_roi_max_age_sec', 0.5)
        self.declare_parameter('face_track_association_iou_threshold', 0.01)
        self.declare_parameter('face_track_ambiguity_margin', 0.05)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('enable_depth', False)
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('depth_sync_slop_sec', 0.08)
        self.declare_parameter('depth_roi_radius_px', 4)
        self.declare_parameter('face_topic', '/perception/largest_face')
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('enable_inference_timing', False)
        self.declare_parameter('timing_log_period_sec', 5.0)

        self.face_inference_rate_hz = max(
            0.1, float(self.get_parameter('face_inference_rate_hz').value)
        )
        self.recognition_rate_hz = max(
            0.1, float(self.get_parameter('recognition_rate_hz').value)
        )
        self.max_frame_age_sec = max(0.0, float(self.get_parameter('max_frame_age_sec').value))
        self.face_selection_mode = str(
            self.get_parameter('face_selection_mode').value
        ).strip().lower()
        self.target_roi = list(self.get_parameter('target_roi').value)
        self.tracked_person_topic = str(self.get_parameter('tracked_person_topic').value)
        self.target_track_id = max(0, int(self.get_parameter('target_track_id').value))
        self.tracked_roi_max_age_sec = max(
            0.0, float(self.get_parameter('tracked_roi_max_age_sec').value)
        )
        self.face_track_association_iou_threshold = max(
            0.0,
            float(self.get_parameter('face_track_association_iou_threshold').value),
        )
        self.face_track_ambiguity_margin = max(
            0.0,
            float(self.get_parameter('face_track_ambiguity_margin').value),
        )
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.enable_depth = as_bool(self.get_parameter('enable_depth').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.depth_sync_slop_sec = float(self.get_parameter('depth_sync_slop_sec').value)
        self.depth_roi_radius_px = int(self.get_parameter('depth_roi_radius_px').value)
        self.face_topic = str(self.get_parameter('face_topic').value)
        self.debug_mode = as_bool(self.get_parameter('debug_mode').value)
        self.enable_inference_timing = as_bool(
            self.get_parameter('enable_inference_timing').value
        )
        self.timing_log_period_sec = max(
            0.5, float(self.get_parameter('timing_log_period_sec').value)
        )
        if self.face_selection_mode not in ('largest', 'target_roi', 'tracked_person', 'tracked_people'):
            raise ValueError(
                "face_selection_mode must be largest, target_roi, tracked_person, or tracked_people."
            )

        # Section: Resolve and load both ONNX models during node configuration, never at import time.
        package_models_dir = str(package_share_dir() / 'models')
        external_model_dir = str(self.get_parameter('face_model_dir').value)
        yunet_model_path = resolve_model_path(
            str(self.get_parameter('yunet_model_path').value),
            DEFAULT_YUNET_MODEL,
            package_models_dir,
            external_model_dir,
        )
        sface_model_path = resolve_model_path(
            str(self.get_parameter('sface_model_path').value),
            DEFAULT_SFACE_MODEL,
            package_models_dir,
            external_model_dir,
        )
        self.backend = FaceBackend(
            yunet_model_path=yunet_model_path,
            sface_model_path=sface_model_path,
            score_threshold=float(self.get_parameter('yunet_score_threshold').value),
            nms_threshold=float(self.get_parameter('yunet_nms_threshold').value),
            top_k=int(self.get_parameter('yunet_top_k').value),
            dnn_backend=str(self.get_parameter('opencv_dnn_backend').value),
            dnn_target=str(self.get_parameter('opencv_dnn_target').value),
        )

        # Section: Subscribers only cache the latest synchronized observation.
        self.bridge = CvBridge()
        self.latest_image_msg = None
        self.latest_depth_msg = None
        self.last_processed_stamp_ns = None
        self.latest_tracked_person = None
        self.latest_tracked_person_stamp_ns = None
        self.latest_tracked_people = {}
        self.last_timing_log_monotonic = 0.0
        self.face_pub = self.create_publisher(String, self.face_topic, 10)
        self.subscription = None
        self.image_sub = None
        self.depth_sub = None
        self.sync = None
        if self.enable_depth:
            self.image_sub = Subscriber(
                self, Image, self.image_topic, qos_profile=qos_profile_sensor_data
            )
            self.depth_sub = Subscriber(
                self, Image, self.depth_topic, qos_profile=qos_profile_sensor_data
            )
            self.sync = ApproximateTimeSynchronizer(
                [self.image_sub, self.depth_sub],
                queue_size=10,
                slop=self.depth_sync_slop_sec,
            )
            self.sync.registerCallback(self.synced_image_depth_callback)
        else:
            self.subscription = self.create_subscription(
                Image, self.image_topic, self.image_callback, qos_profile_sensor_data
            )
        self.tracked_person_sub = None
        if self.face_selection_mode in ('tracked_person', 'tracked_people'):
            if RecognizedPerson is None:
                raise RuntimeError(
                    'tracked_person mode requires the built vision/msg/RecognizedPerson interface.'
                )
            self.tracked_person_sub = self.create_subscription(
                RecognizedPerson,
                self.tracked_person_topic,
                self.tracked_person_callback,
                10,
            )
        self.inference_timer = self.create_timer(
            1.0 / self.face_inference_rate_hz,
            self.inference_timer_callback,
        )
        self.get_logger().info(
            'YuNet/SFace face perception started. '
            f'yunet={yunet_model_path}, sface={sface_model_path}, device=CPU, '
            f'inference_rate_hz={self.face_inference_rate_hz:.2f}, '
            f'selection_mode={self.face_selection_mode}'
        )

    def image_callback(self, msg):
        """Special function: Cache the newest RGB frame without running inference."""
        self.latest_image_msg = msg
        self.latest_depth_msg = None

    def synced_image_depth_callback(self, image_msg, depth_msg):
        """Special function: Cache the newest synchronized RGB/depth pair."""
        self.latest_image_msg = image_msg
        self.latest_depth_msg = depth_msg

    def tracked_person_callback(self, msg):
        """Special function: Cache every canonical track and the compatible selected track."""
        session_id = str(getattr(msg, 'tracking_session_id', '')).strip()
        key = f"{session_id}:{int(msg.track_id)}" if session_id else f"legacy:{int(msg.track_id)}"
        stamp_ns = observation_stamp_ns(msg)
        self.latest_tracked_people[key] = (msg, stamp_ns)
        if self.target_track_id > 0:
            selected = int(msg.track_id) == self.target_track_id
        else:
            selected = bool(msg.is_nearest)
        if selected:
            self.latest_tracked_person = msg
            self.latest_tracked_person_stamp_ns = observation_stamp_ns(msg)

    def _tracked_people_for_frame(self, image_stamp_ns):
        """Special function: Return only session-aware person boxes fresh for this image."""
        maximum_delta = int(self.tracked_roi_max_age_sec * 1_000_000_000)
        fresh = []
        for key, (person, stamp_ns) in list(self.latest_tracked_people.items()):
            if stamp_ns is None or abs(int(image_stamp_ns) - int(stamp_ns)) > maximum_delta:
                self.latest_tracked_people.pop(key, None)
                continue
            fresh.append((key, person))
        return sorted(fresh, key=lambda item: item[0])

    def _frame_is_fresh(self, image_msg):
        """Special function: Reject unstamped or stale cached camera observations."""
        stamp_ns = observation_stamp_ns(image_msg)
        if stamp_ns is None:
            return False, None
        age_ns = self.get_clock().now().nanoseconds - stamp_ns
        return 0 <= age_ns <= int(self.max_frame_age_sec * 1_000_000_000), stamp_ns

    def _selection_roi(self, image_stamp_ns):
        """Special function: Resolve static or tracked-person ROI and associated track ID."""
        if self.face_selection_mode == 'largest':
            return None, 0, False
        if self.face_selection_mode == 'target_roi':
            if len(self.target_roi) != 4 or float(self.target_roi[2]) <= float(self.target_roi[0]):
                return None, 0, False
            return self.target_roi, 0, False
        if self.face_selection_mode == 'tracked_people':
            return None, 0, False
        if self.latest_tracked_person is None or self.latest_tracked_person_stamp_ns is None:
            return None, 0, False
        roi_age_ns = abs(int(image_stamp_ns) - int(self.latest_tracked_person_stamp_ns))
        if roi_age_ns > int(self.tracked_roi_max_age_sec * 1_000_000_000):
            return None, 0, False
        return (
            list(self.latest_tracked_person.bbox_xyxy),
            int(self.latest_tracked_person.track_id),
            True,
        )

    def inference_timer_callback(self):
        """Special function: Process each newest fresh frame at most once at the timer rate."""
        image_msg = self.latest_image_msg
        if image_msg is None:
            return
        is_fresh, stamp_ns = self._frame_is_fresh(image_msg)
        if not is_fresh or stamp_ns == self.last_processed_stamp_ns:
            return
        self.last_processed_stamp_ns = stamp_ns
        self._process_frame(image_msg, self.latest_depth_msg, stamp_ns)

    def _face_payload(
        self,
        observation,
        track_id=0,
        has_track_id=False,
        tracking_session_id='',
        person_bbox=None,
    ):
        """Special function: Isolate the legacy JSON conversion from backend inference."""
        return {
            'bbox': xywh_to_xyxy(observation.bbox),
            'embedding': [float(value) for value in observation.embedding.tolist()],
            'det_score': float(observation.score),
            'kps': [[float(x), float(y)] for x, y in observation.landmarks.tolist()],
            'track_id': int(track_id),
            'has_track_id': bool(has_track_id),
            'tracking_session_id': str(tracking_session_id),
            'person_bbox': list(person_bbox or []),
            'embedding_backend': EMBEDDING_BACKEND,
            'embedding_model': self.backend.embedding_model,
            'embedding_dimension': int(observation.embedding.size),
        }

    def _empty_output(self, image_msg, track_id=0, has_track_id=False, selected_roi=None):
        """Special function: Build a complete no-face observation for downstream freshness logic."""
        return {
            'detected': False,
            'bbox': [],
            'embedding': [],
            'det_score': 0.0,
            'kps': [],
            'depth_m': 0.0,
            'depth_valid': False,
            'depth_center_px': [],
            'depth_sample_count': 0,
            'stamp_sec': int(image_msg.header.stamp.sec),
            'stamp_nanosec': int(image_msg.header.stamp.nanosec),
            'track_id': int(track_id),
            'has_track_id': bool(has_track_id),
            'tracking_session_id': '',
            'face_count': 0,
            'faces': [],
            'selection_mode': self.face_selection_mode,
            'selected_roi': list(selected_roi or []),
            'embedding_backend': EMBEDDING_BACKEND,
            'embedding_model': self.backend.embedding_model,
            'embedding_dimension': int(self.backend.embedding_dimension or 0),
        }

    def _publish_output(self, output):
        """Special function: Publish exactly one serialized observation for each processed frame."""
        message = String()
        message.data = json.dumps(output)
        self.face_pub.publish(message)

    def _process_frame(self, image_msg, depth_msg, stamp_ns):
        """Special function: Run YuNet/SFace once and publish selected plus all ROI face results."""
        roi_xyxy, track_id, has_track_id = self._selection_roi(stamp_ns)
        output = self._empty_output(image_msg, track_id, has_track_id, roi_xyxy)
        if self.face_selection_mode in ('target_roi', 'tracked_person') and roi_xyxy is None:
            self._publish_output(output)
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            observations = self.backend.detect_and_embed(frame, roi_xyxy=roi_xyxy)
        except (cv2.error, ValueError) as exc:
            self.get_logger().warning(f'Face inference rejected a frame: {exc}')
            self._publish_output(output)
            return
        selected = select_observation(observations)
        output['face_count'] = len(observations)
        face_payloads = [self._face_payload(observation) for observation in observations]
        if self.face_selection_mode == 'tracked_people':
            tracked_people = self._tracked_people_for_frame(stamp_ns)
            assignments = associate_faces_to_tracks(
                [xywh_to_xyxy(observation.bbox) for observation in observations],
                [list(person.bbox_xyxy) for _key, person in tracked_people],
                self.face_track_association_iou_threshold,
                self.face_track_ambiguity_margin,
            )
            for face_index, person_index in assignments.items():
                _key, person = tracked_people[person_index]
                face_payloads[face_index].update({
                    'track_id': int(person.track_id),
                    'has_track_id': True,
                    'tracking_session_id': str(
                        getattr(person, 'tracking_session_id', '')
                    ),
                    'person_bbox': list(person.bbox_xyxy),
                })
            associated_indices = list(assignments)
            selected_index = (
                max(
                    associated_indices,
                    key=lambda index: bbox_area_xywh(observations[index].bbox),
                )
                if associated_indices else None
            )
            selected = observations[selected_index] if selected_index is not None else None
            if selected_index is not None:
                selected_payload = face_payloads[selected_index]
                track_id = int(selected_payload['track_id'])
                has_track_id = True
                output['tracking_session_id'] = selected_payload['tracking_session_id']
                output['selected_roi'] = list(selected_payload['person_bbox'])
        else:
            face_payloads = [
                self._face_payload(observation, track_id, has_track_id)
                for observation in observations
            ]
        output['faces'] = face_payloads
        output['embedding_dimension'] = int(self.backend.embedding_dimension or 0)
        if selected is not None:
            output.update(self._face_payload(selected, track_id, has_track_id))
            if self.face_selection_mode == 'tracked_people':
                output.update(face_payloads[selected_index])
            output['detected'] = True
            if depth_msg is not None:
                try:
                    depth_image = self.bridge.imgmsg_to_cv2(
                        depth_msg, desired_encoding='passthrough'
                    )
                    depth_m, center_x, center_y, sample_count = median_depth_m(
                        depth_image,
                        output['bbox'],
                        self.depth_roi_radius_px,
                    )
                    output['depth_m'] = float(depth_m)
                    output['depth_valid'] = bool(depth_m > 0.0)
                    output['depth_center_px'] = [int(center_x), int(center_y)]
                    output['depth_sample_count'] = int(sample_count)
                except (cv2.error, ValueError) as exc:
                    self.get_logger().warning(f'Failed to sample aligned depth: {exc}')
        self._publish_output(output)
        self._log_timing_if_enabled()

    def _log_timing_if_enabled(self):
        """Special function: Periodically log inference metrics without per-frame spam."""
        if not self.enable_inference_timing:
            return
        now = time.monotonic()
        if now - self.last_timing_log_monotonic < self.timing_log_period_sec:
            return
        self.last_timing_log_monotonic = now
        timings = self.backend.last_timings_ms
        self.get_logger().info(
            'Face timings: '
            f"yunet_ms={timings['yunet_ms']:.1f}, "
            f"sface_ms={timings['sface_ms']:.1f}, "
            f"total_face_pipeline_ms={timings['total_face_pipeline_ms']:.1f}"
        )


def main(args=None):
    """Special function: Run the face perception node until shutdown."""
    rclpy.init(args=args)
    node = FacePerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
