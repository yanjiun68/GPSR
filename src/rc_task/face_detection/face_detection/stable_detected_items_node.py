#!/usr/bin/env python3
"""Stable object-name memory for face_detection detected item JSON output."""

from collections import defaultdict, deque
import json
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from putrabot_interfaces.msg import VisibleItems
from putrabot_interfaces.srv import GetVisibleItems


class StableDetectedItemsNode(Node):
    """Confirm item names across several recent frames before publishing them."""

    def __init__(self):
        super().__init__('stable_detected_items_node')

        # Topic parameters: input keeps the original detector topic unchanged.
        self.declare_parameter('input_detections_topic', '/perception/detected_items')
        self.declare_parameter('stable_items_topic', '/perception/recognized_items')
        self.declare_parameter('get_visible_items_service', '/perception/get_visible_items')

        # Sampling parameters: these tune how strict the multi-frame voting is.
        self.declare_parameter('min_confidence', 0.50)
        self.declare_parameter('window_size', 5)
        self.declare_parameter('min_votes', 3)
        self.declare_parameter('stale_timeout_sec', 1.0)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('publish_empty_lists', True)

        # Parameter cache: clamp values so bad launch input cannot crash the node.
        self.input_detections_topic = str(self.get_parameter('input_detections_topic').value)
        self.stable_items_topic = str(self.get_parameter('stable_items_topic').value)
        self.get_visible_items_service = str(self.get_parameter('get_visible_items_service').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.window_size = max(1, int(self.get_parameter('window_size').value))
        self.min_votes = max(1, int(self.get_parameter('min_votes').value))
        self.stale_timeout_sec = max(0.0, float(self.get_parameter('stale_timeout_sec').value))
        self.publish_empty_lists = bool(self.get_parameter('publish_empty_lists').value)

        # Voting state: one history entry is one camera frame's accepted labels.
        if self.min_votes > self.window_size:
            self.get_logger().warn('min_votes is larger than window_size; clamping it to window_size')
            self.min_votes = self.window_size
        self.history: deque[Dict[str, float]] = deque(maxlen=self.window_size)
        self.last_seen_sec: Dict[str, float] = {}
        self.best_confidence: Dict[str, float] = {}
        self._last_parse_warning_sec = 0.0

        # ROS interfaces: subscribe to raw JSON, publish typed stable items, expose query service.
        self.create_subscription(String, self.input_detections_topic, self.detections_callback, 10)
        self.stable_items_pub = self.create_publisher(VisibleItems, self.stable_items_topic, 10)
        self.create_service(GetVisibleItems, self.get_visible_items_service, self.get_visible_items_callback)

        # Timer: publish the current stable set at a fixed rate for task nodes.
        publish_rate_hz = max(0.1, float(self.get_parameter('publish_rate_hz').value))
        self.publish_timer = self.create_timer(1.0 / publish_rate_hz, self.publish_stable_items)

        # Startup log: useful when debugging topic wiring from launch files.
        self.get_logger().info(
            'Stable detected-items memory started: '
            f'input={self.input_detections_topic}, output={self.stable_items_topic}, '
            f'service={self.get_visible_items_service}, window_size={self.window_size}, '
            f'min_votes={self.min_votes}, min_confidence={self.min_confidence:.2f}, '
            f'stale_timeout={self.stale_timeout_sec:.2f}s'
        )

    # ------------------------------------------------------------------
    # Time helpers
    # ------------------------------------------------------------------
    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------------
    # Input parsing helpers
    # ------------------------------------------------------------------
    def _warn_parse_problem(self, message: str) -> None:
        now = self.now_sec()
        if now - self._last_parse_warning_sec >= 5.0:
            self.get_logger().warn(message)
            self._last_parse_warning_sec = now

    def _parse_json_payload(self, raw_data: str) -> Optional[dict]:
        try:
            payload = json.loads(raw_data)
        except json.JSONDecodeError as exc:
            self._warn_parse_problem(f'Ignoring malformed detected-items JSON: {exc}')
            return None
        if not isinstance(payload, dict):
            self._warn_parse_problem('Ignoring detected-items JSON because the payload is not an object')
            return None
        return payload

    def _normalise_name(self, value) -> str:
        return str(value or '').strip().lower().replace(' ', '_').replace('-', '_')

    def _read_confidence(self, detection: dict) -> float:
        try:
            return float(detection.get('confidence', 0.0))
        except (TypeError, ValueError):
            return 0.0

    def _extract_frame_items(self, payload: dict) -> Dict[str, float]:
        frame_items: Dict[str, float] = {}
        detections = payload.get('detections', [])
        if not isinstance(detections, list):
            self._warn_parse_problem('Ignoring detected-items JSON because detections is not a list')
            return frame_items

        for detection in detections:
            if not isinstance(detection, dict):
                continue
            name = self._normalise_name(
                detection.get('label')
                or detection.get('name')
                or detection.get('class_name')
                or detection.get('object_name')
            )
            confidence = self._read_confidence(detection)
            if not name or confidence < self.min_confidence:
                continue
            frame_items[name] = max(confidence, frame_items.get(name, 0.0))
        return frame_items

    # ------------------------------------------------------------------
    # Detection callback and stale cleanup
    # ------------------------------------------------------------------
    def detections_callback(self, msg: String) -> None:
        payload = self._parse_json_payload(msg.data)
        if payload is None:
            return

        now = self.now_sec()
        frame_items = self._extract_frame_items(payload)
        for name, confidence in frame_items.items():
            self.last_seen_sec[name] = now
            self.best_confidence[name] = max(confidence, self.best_confidence.get(name, 0.0))

        self.history.append(frame_items)
        self._prune_stale(now)

    def _prune_stale(self, now: float) -> None:
        stale_names = [
            name for name, last_seen in self.last_seen_sec.items()
            if (now - last_seen) > self.stale_timeout_sec
        ]
        for name in stale_names:
            self.last_seen_sec.pop(name, None)
            self.best_confidence.pop(name, None)

    # ------------------------------------------------------------------
    # Stable item computation
    # ------------------------------------------------------------------
    def stable_items(self) -> Tuple[List[str], List[float], List[float]]:
        now = self.now_sec()
        self._prune_stale(now)

        votes = defaultdict(int)
        frame_confidences = defaultdict(float)
        for frame_items in self.history:
            for name, confidence in frame_items.items():
                votes[name] += 1
                frame_confidences[name] = max(frame_confidences[name], confidence)

        items = []
        for name, count in votes.items():
            if name not in self.last_seen_sec:
                continue
            age = now - self.last_seen_sec[name]
            if count >= self.min_votes and age <= self.stale_timeout_sec:
                confidence = max(self.best_confidence.get(name, 0.0), frame_confidences[name])
                items.append((name, confidence, age))

        items.sort(key=lambda item: (-item[1], item[0]))
        names = [name for name, _confidence, _age in items]
        confidences = [float(confidence) for _name, confidence, _age in items]
        ages = [float(age) for _name, _confidence, age in items]
        return names, confidences, ages

    # ------------------------------------------------------------------
    # Output topic and query service
    # ------------------------------------------------------------------
    def build_visible_items_msg(self) -> VisibleItems:
        names, confidences, ages = self.stable_items()
        msg = VisibleItems()
        msg.stamp = self.get_clock().now().to_msg()
        msg.names = names
        msg.confidences = confidences
        msg.ages_sec = ages
        return msg

    def publish_stable_items(self) -> None:
        msg = self.build_visible_items_msg()
        if not msg.names and not self.publish_empty_lists:
            return
        self.stable_items_pub.publish(msg)

    def get_visible_items_callback(self, _request, response):
        names, confidences, ages = self.stable_items()
        response.names = names
        response.confidences = confidences
        response.ages_sec = ages
        return response


def main(args=None):
    rclpy.init(args=args)
    node = StableDetectedItemsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
