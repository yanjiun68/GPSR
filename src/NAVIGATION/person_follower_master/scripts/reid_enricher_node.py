#!/usr/bin/env python3
"""Re-ID enricher node.

Consumes /camera/.../color/image_raw and /cam_detections (with bbox
fields populated by vision_node), crops each person bbox, extracts
an OSNet-x0_25 appearance embedding, and republishes the detections
on /cam_detections_reid with the embedding field filled.

Because vision/msg/CamDetections carries a builtin_interfaces/Time
instead of a std_msgs/Header, we cannot use message_filters'
ApproximateTimeSynchronizer directly. A small rolling buffer of
recent RGB frames keyed by (sec, nanosec) is used to look up the
RGB that matches each detection set within time_slop_sec.
"""

from __future__ import annotations

import os
import sys
import time
from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from vision.msg import CamDetection, CamDetections

from pfm_utils.reid_model import OsNetExtractor, EMBED_DIM


def _stamp_key(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class ReidEnricherNode(Node):
    def __init__(self) -> None:
        super().__init__("reid_enricher_node")

        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("detections_topic", "/cam_detections")
        self.declare_parameter("output_topic", "/cam_detections_reid")
        self.declare_parameter("min_bbox_area", 1500)
        self.declare_parameter("max_detections", 8)
        self.declare_parameter("time_slop_sec", 0.05)
        self.declare_parameter("rgb_buffer_size", 15)
        self.declare_parameter("reid_warmup", True)
        self.declare_parameter("model_name", "osnet_x0_25")
        self.declare_parameter("num_threads", 2)
        # Device: "cuda" (auto-fallback to cpu if unavailable), "cpu",
        # or an explicit index like "cuda:0".
        self.declare_parameter("device", "cuda")

        rgb_topic = str(self.get_parameter("rgb_topic").value)
        det_topic = str(self.get_parameter("detections_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        self.min_bbox_area = int(self.get_parameter("min_bbox_area").value)
        self.max_detections = int(self.get_parameter("max_detections").value)
        self.time_slop_ns = int(float(self.get_parameter("time_slop_sec").value) * 1e9)
        self.rgb_buffer_size = int(self.get_parameter("rgb_buffer_size").value)
        warmup = bool(self.get_parameter("reid_warmup").value)
        model_name = str(self.get_parameter("model_name").value)
        num_threads = int(self.get_parameter("num_threads").value)
        requested_device = str(self.get_parameter("device").value)
        device = self._resolve_device(requested_device)

        self.bridge = CvBridge()
        self._rgb_buf: Deque[Tuple[int, np.ndarray]] = deque(maxlen=self.rgb_buffer_size)

        self.get_logger().info(
            f"[reid_enricher] loading Re-ID model '{model_name}' on {device} "
            f"with {num_threads} threads…"
        )
        self.extractor = OsNetExtractor(
            model_name=model_name,
            device=device,
            num_threads=num_threads,
        )
        if warmup:
            self.extractor.warmup()
        self.get_logger().info("[reid_enricher] Re-ID model ready.")

        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_sensor.history = HistoryPolicy.KEEP_LAST
        qos_sensor.durability = DurabilityPolicy.VOLATILE

        qos_reliable = QoSProfile(depth=10)
        qos_reliable.reliability = ReliabilityPolicy.RELIABLE
        qos_reliable.history = HistoryPolicy.KEEP_LAST

        self.sub_rgb = self.create_subscription(
            Image, rgb_topic, self._on_rgb, qos_sensor)
        self.sub_det = self.create_subscription(
            CamDetections, det_topic, self._on_detections, qos_reliable)
        self.pub_out = self.create_publisher(
            CamDetections, out_topic, qos_reliable)

        self._n_processed = 0
        self._n_dropped_no_rgb = 0
        self._total_extract_ms = 0.0
        self._last_report = self.get_clock().now()
        self.create_timer(1.0, self._report)

    def _resolve_device(self, requested: str) -> str:
        device = (requested or "").strip() or "cpu"
        normalized = device.lower()
        wants_cuda = (
            normalized in ("cuda", "gpu")
            or normalized.startswith("cuda:")
            or normalized.isdigit()
        )
        if not wants_cuda:
            return device
        try:
            import torch
            if torch.cuda.is_available():
                return device if normalized != "gpu" else "cuda"
        except Exception as exc:
            self.get_logger().warn(
                f"[reid_enricher] torch unavailable ({exc}); falling back to CPU."
            )
            return "cpu"
        self.get_logger().warn(
            f"[reid_enricher] CUDA requested ({device}) but unavailable; "
            f"falling back to CPU."
        )
        return "cpu"

    def _on_rgb(self, msg: Image) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge failed: {exc}", throttle_duration_sec=2.0)
            return
        self._rgb_buf.append((_stamp_key(msg.header.stamp), bgr))

    def _find_rgb(self, target_key: int) -> Optional[np.ndarray]:
        if not self._rgb_buf:
            return None
        best = None
        best_delta = None
        for key, img in self._rgb_buf:
            delta = abs(key - target_key)
            if best_delta is None or delta < best_delta:
                best_delta = delta
                best = img
        if best_delta is None or best_delta > self.time_slop_ns:
            return None
        return best

    def _on_detections(self, msg: CamDetections) -> None:
        target_key = _stamp_key(msg.stamp)
        rgb = self._find_rgb(target_key)

        out = CamDetections()
        out.stamp = msg.stamp

        if not msg.detections:
            self.pub_out.publish(out)
            return

        if rgb is None:
            self._n_dropped_no_rgb += 1
            # Forward detections without embedding so tracker can still use
            # motion-only gating as a graceful degradation.
            for d in msg.detections[: self.max_detections]:
                new_d = CamDetection()
                new_d.x = d.x
                new_d.y = d.y
                new_d.z = d.z
                new_d.confidence = d.confidence
                new_d.embedding = []
                new_d.xmin = d.xmin
                new_d.ymin = d.ymin
                new_d.xmax = d.xmax
                new_d.ymax = d.ymax
                out.detections.append(new_d)
            self.pub_out.publish(out)
            return

        boxes = []
        dets_sorted = sorted(
            msg.detections,
            key=lambda d: float(d.confidence),
            reverse=True,
        )[: self.max_detections]

        for d in dets_sorted:
            area = (int(d.xmax) - int(d.xmin)) * (int(d.ymax) - int(d.ymin))
            if area <= 0 or area < self.min_bbox_area:
                boxes.append(None)
            else:
                boxes.append((int(d.xmin), int(d.ymin), int(d.xmax), int(d.ymax)))

        valid_indices = [i for i, b in enumerate(boxes) if b is not None]
        valid_boxes = [boxes[i] for i in valid_indices]

        embeddings = np.zeros((0, EMBED_DIM), dtype=np.float32)
        kept_local: list = []
        if valid_boxes:
            t0 = time.perf_counter()
            embeddings, kept_local = self.extractor.extract(rgb, valid_boxes)
            self._total_extract_ms += (time.perf_counter() - t0) * 1000.0

        # Map kept_local (index into valid_boxes) back to dets_sorted index.
        extracted = {}
        for row_idx, local_idx in enumerate(kept_local):
            original_idx = valid_indices[local_idx]
            extracted[original_idx] = embeddings[row_idx]

        for i, d in enumerate(dets_sorted):
            new_d = CamDetection()
            new_d.x = d.x
            new_d.y = d.y
            new_d.z = d.z
            new_d.confidence = d.confidence
            new_d.xmin = d.xmin
            new_d.ymin = d.ymin
            new_d.xmax = d.xmax
            new_d.ymax = d.ymax
            if i in extracted:
                new_d.embedding = [float(x) for x in extracted[i].tolist()]
            else:
                new_d.embedding = []
            out.detections.append(new_d)

        self._n_processed += 1
        self.pub_out.publish(out)

    def _report(self) -> None:
        if self._n_processed == 0 and self._n_dropped_no_rgb == 0:
            return
        avg = (
            self._total_extract_ms / self._n_processed
            if self._n_processed > 0 else 0.0
        )
        self.get_logger().info(
            f"reid: processed={self._n_processed} "
            f"drop_no_rgb={self._n_dropped_no_rgb} "
            f"avg_extract_ms={avg:.1f}"
        )
        self._n_processed = 0
        self._n_dropped_no_rgb = 0
        self._total_extract_ms = 0.0


def main() -> None:
    rclpy.init()
    node = ReidEnricherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
