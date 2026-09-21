#!/usr/bin/env -S bash -c 'exec "${VISION_PYTHON:-${HOME}/ai/qwen3v1/.venv/bin/python}" "$0" "$@"'

from collections import defaultdict, deque
from typing import Dict, List, Set

import rclpy
from rclpy.node import Node
from putrabot_interfaces.msg import ObjectDetections, VisibleItems
from putrabot_interfaces.srv import GetVisibleItems


class PerceptionMemoryNode(Node):
    def __init__(self):
        super().__init__("perception_memory_node")

        self.declare_parameter("input_detections_topic", "/perception/raw_object_detections")
        self.declare_parameter("stable_items_topic", "/perception/recognized_items")
        self.declare_parameter("get_visible_items_service", "/perception/get_visible_items")
        self.declare_parameter("min_confidence", 0.50)
        self.declare_parameter("window_size", 5)
        self.declare_parameter("min_votes", 3)
        self.declare_parameter("stale_timeout_sec", 1.0)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("publish_empty_lists", True)

        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.window_size = max(1, int(self.get_parameter("window_size").value))
        self.min_votes = max(1, int(self.get_parameter("min_votes").value))
        self.stale_timeout_sec = max(0.0, float(self.get_parameter("stale_timeout_sec").value))
        self.publish_empty_lists = bool(self.get_parameter("publish_empty_lists").value)

        if self.min_votes > self.window_size:
            self.get_logger().warn("min_votes is larger than window_size; clamping min_votes to window_size")
            self.min_votes = self.window_size

        self.history: deque[Set[str]] = deque(maxlen=self.window_size)
        self.last_seen_sec: Dict[str, float] = {}
        self.best_confidence: Dict[str, float] = {}

        self.create_subscription(
            ObjectDetections,
            str(self.get_parameter("input_detections_topic").value),
            self.detections_callback,
            10,
        )

        self.pub = self.create_publisher(
            VisibleItems,
            str(self.get_parameter("stable_items_topic").value),
            10,
        )

        self.create_service(
            GetVisibleItems,
            str(self.get_parameter("get_visible_items_service").value),
            self.get_visible_items_callback,
        )

        publish_rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_stable_items)

        self.get_logger().info(
            "Perception memory started: "
            f"input={self.get_parameter('input_detections_topic').value}, "
            f"output={self.get_parameter('stable_items_topic').value}, "
            f"window_size={self.window_size}, min_votes={self.min_votes}, "
            f"min_confidence={self.min_confidence:.2f}, stale_timeout={self.stale_timeout_sec:.2f}s"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def detections_callback(self, msg: ObjectDetections) -> None:
        now = self.now_sec()
        frame_names: Set[str] = set()

        for det in msg.detections:
            name = str(det.name).strip()
            if not name or float(det.confidence) < self.min_confidence:
                continue
            frame_names.add(name)
            self.last_seen_sec[name] = now
            self.best_confidence[name] = max(
                float(det.confidence),
                float(self.best_confidence.get(name, 0.0)),
            )

        self.history.append(frame_names)
        self._prune_stale(now)

    def _prune_stale(self, now: float) -> None:
        stale_names = [
            name for name, last_seen in self.last_seen_sec.items()
            if (now - last_seen) > self.stale_timeout_sec
        ]
        for name in stale_names:
            self.last_seen_sec.pop(name, None)
            self.best_confidence.pop(name, None)

    def stable_items(self) -> tuple[List[str], List[float], List[float]]:
        now = self.now_sec()
        self._prune_stale(now)

        votes = defaultdict(int)
        for frame_names in self.history:
            for name in frame_names:
                votes[name] += 1

        items = []
        for name, count in votes.items():
            if name not in self.last_seen_sec:
                continue
            age = now - self.last_seen_sec[name]
            if count >= self.min_votes and age <= self.stale_timeout_sec:
                items.append((name, self.best_confidence.get(name, 0.0), age))

        items.sort(key=lambda item: (-item[1], item[0]))
        names = [name for name, _confidence, _age in items]
        confidences = [float(confidence) for _name, confidence, _age in items]
        ages = [float(age) for _name, _confidence, age in items]
        return names, confidences, ages

    def publish_stable_items(self) -> None:
        names, confidences, ages = self.stable_items()
        if not names and not self.publish_empty_lists:
            return
        msg = VisibleItems()
        msg.stamp = self.get_clock().now().to_msg()
        msg.names = names
        msg.confidences = confidences
        msg.ages_sec = ages
        self.pub.publish(msg)

    def get_visible_items_callback(self, _request, response):
        names, confidences, ages = self.stable_items()
        response.names = names
        response.confidences = confidences
        response.ages_sec = ages
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionMemoryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
