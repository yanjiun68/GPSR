#!/usr/bin/env python3
import sys
from pathlib import Path

import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from putrabot_interfaces.msg import CamDetection, CamDetections, ObjectDetection, ObjectDetections

from bag_point_selection.path_utils import resolve_package_model_path

try:
    from ultralytics import YOLO
except ImportError:  # pragma: no cover - runtime dependency
    YOLO = None


class BagDetectorNode(Node):
    def __init__(self):
        super().__init__("bag_detector_node")
        self.declare_parameter("rgb_topic", "/bag_selection/image_enhanced")
        self.declare_parameter("yolo_model_path", "")
        self.declare_parameter("bag_class_ids", [0])
        self.declare_parameter("bag_conf_thresh", 0.4)
        self.declare_parameter("bag_iou_thresh", 0.5)
        self.declare_parameter("bag_bbox_min_area_px", 100)
        self.declare_parameter("bag_bbox_max_area_ratio", 0.70)
        self.declare_parameter("bag_bbox_min_aspect", 0.30)
        self.declare_parameter("bag_bbox_max_aspect", 2.50)
        self.declare_parameter("bag_bbox_edge_margin_px", 8)
        self.declare_parameter("required_bag_count", 2)
        self.declare_parameter("camera_watchdog_sec", 2.0)
        self.declare_parameter("raw_object_detections_topic", "/perception/raw_object_detections")
        self.declare_parameter("object_name_overrides", [])

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(CamDetections, "/detected_bags", 10)
        self.object_publisher = self.create_publisher(
            ObjectDetections,
            self.get_parameter("raw_object_detections_topic").value,
            10,
        )
        self.subscription = self.create_subscription(
            Image,
            self.get_parameter("rgb_topic").value,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.model = None
        self.bag_class_ids = {int(v) for v in self.get_parameter("bag_class_ids").value}
        self.required_bag_count = max(1, int(self.get_parameter("required_bag_count").value))
        self.object_name_overrides = self._parse_name_overrides(
            self.get_parameter("object_name_overrides").value
        )
        self.class_names = {}

        model_path = resolve_package_model_path("bag_point_selection", self.get_parameter("yolo_model_path").value)
        if YOLO is None:
            self.get_logger().fatal("ultralytics is not installed. Shutting down.")
            rclpy.shutdown()
            sys.exit(1)
        if not model_path:
            self.get_logger().fatal("YOLO model path is empty. Set yolo_model_path in params.yaml.")
            rclpy.shutdown()
            sys.exit(1)
        if not Path(model_path).exists():
            self.get_logger().fatal(f"YOLO model not found at: {model_path}")
            rclpy.shutdown()
            sys.exit(1)

        self.model = YOLO(model_path)
        self.get_logger().info(f"Loaded YOLO model from {model_path}")
        self.get_logger().info(f"Configured bag_class_ids: {sorted(self.bag_class_ids)}")
        self._load_class_names()
        self._validate_class_ids()
        self._warmup_model()

        self.last_image_time = self.get_clock().now()
        watchdog_sec = float(self.get_parameter("camera_watchdog_sec").value)
        if watchdog_sec > 0.0:
            self.watchdog_sec = watchdog_sec
            self.create_timer(watchdog_sec, self._camera_watchdog)

    def _parse_name_overrides(self, values) -> dict[int, str]:
        overrides = {}
        for value in values:
            item = str(value)
            if ":" not in item:
                self.get_logger().warn(f"Ignoring invalid object_name_overrides entry: {item}")
                continue
            class_id, name = item.split(":", 1)
            try:
                overrides[int(class_id.strip())] = name.strip()
            except ValueError:
                self.get_logger().warn(f"Ignoring invalid object_name_overrides class id: {item}")
        return overrides

    def _load_class_names(self) -> None:
        names = getattr(self.model, "names", None)
        if isinstance(names, dict):
            self.class_names = {int(idx): str(name) for idx, name in names.items()}
        elif isinstance(names, list):
            self.class_names = {int(idx): str(name) for idx, name in enumerate(names)}
        else:
            self.class_names = {}

    def _object_name_for_class(self, class_id: int) -> str:
        if class_id in self.object_name_overrides:
            return self.object_name_overrides[class_id]
        return self.class_names.get(class_id, f"class_{class_id}")

    def _validate_class_ids(self) -> None:
        names = getattr(self.model, "names", None)
        if isinstance(names, dict):
            self.get_logger().info(f"YOLO class mapping: {dict(sorted(names.items()))}")
            invalid = sorted(cid for cid in self.bag_class_ids if cid not in names)
        elif isinstance(names, list):
            mapping = {idx: name for idx, name in enumerate(names)}
            self.get_logger().info(f"YOLO class mapping: {mapping}")
            invalid = sorted(cid for cid in self.bag_class_ids if cid >= len(names))
        else:
            return
        if invalid:
            self.get_logger().fatal(
                f"Configured bag_class_ids {invalid} are not present in the YOLO model. "
                f"Shutting down to avoid silent mis-detection."
            )
            rclpy.shutdown()
            sys.exit(1)

    def _warmup_model(self) -> None:
        try:
            dummy = np.zeros((480, 640, 3), dtype=np.uint8)
            self.model.predict(
                source=dummy,
                verbose=False,
                conf=float(self.get_parameter("bag_conf_thresh").value),
                iou=float(self.get_parameter("bag_iou_thresh").value),
            )
            self.get_logger().info("YOLO warmup complete.")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"YOLO warmup failed (will retry on live frames): {exc}")

    def _valid_bbox(
        self,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
        frame_width: int,
        frame_height: int,
    ) -> bool:
        bbox_width = x2 - x1
        bbox_height = y2 - y1
        if bbox_width <= 0 or bbox_height <= 0:
            return False

        area = bbox_width * bbox_height
        img_area = frame_width * frame_height
        if img_area <= 0:
            return False

        min_area = int(self.get_parameter("bag_bbox_min_area_px").value)
        max_area_ratio = float(self.get_parameter("bag_bbox_max_area_ratio").value)
        min_aspect = float(self.get_parameter("bag_bbox_min_aspect").value)
        max_aspect = float(self.get_parameter("bag_bbox_max_aspect").value)
        edge_margin = int(self.get_parameter("bag_bbox_edge_margin_px").value)
        aspect = float(bbox_width) / float(bbox_height)

        if area < min_area:
            return False
        if area > max_area_ratio * img_area:
            self.get_logger().warn(
                f"Dropping oversized bag bbox: area_ratio={area / img_area:.2f}",
                throttle_duration_sec=2.0,
            )
            return False
        if aspect < min_aspect or aspect > max_aspect:
            return False
        if edge_margin > 0 and (
            x1 < edge_margin
            or y1 < edge_margin
            or x2 > frame_width - edge_margin
            or y2 > frame_height - edge_margin
        ):
            return False
        return True

    def _camera_watchdog(self) -> None:
        age_sec = (self.get_clock().now() - self.last_image_time).nanoseconds / 1e9
        if age_sec > self.watchdog_sec:
            self.get_logger().error(
                f"No frames on {self.get_parameter('rgb_topic').value} for {age_sec:.1f}s. "
                "Check upstream preprocessor / camera.",
                throttle_duration_sec=5.0,
            )

    def image_callback(self, msg: Image) -> None:
        self.last_image_time = self.get_clock().now()
        output = CamDetections()
        output.stamp = msg.header.stamp
        object_output = ObjectDetections()
        object_output.stamp = msg.header.stamp
        if self.model is None:
            self.publisher.publish(output)
            self.object_publisher.publish(object_output)
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"cv_bridge failed: {exc}", throttle_duration_sec=2.0
            )
            self.publisher.publish(output)
            self.object_publisher.publish(object_output)
            return

        frame_height, frame_width = frame.shape[:2]
        try:
            results = self.model.predict(
                source=frame,
                verbose=False,
                conf=float(self.get_parameter("bag_conf_thresh").value),
                iou=float(self.get_parameter("bag_iou_thresh").value),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"YOLO predict failed: {exc}", throttle_duration_sec=2.0
            )
            self.publisher.publish(output)
            self.object_publisher.publish(object_output)
            return

        detections = []
        if results:
            boxes = results[0].boxes
            if boxes is not None:
                for box in boxes:
                    cls_id = int(box.cls[0].item())
                    if cls_id not in self.bag_class_ids:
                        continue
                    xyxy = box.xyxy[0].tolist()
                    x1, y1, x2, y2 = [int(v) for v in xyxy]
                    x1 = max(0, min(frame_width, x1))
                    y1 = max(0, min(frame_height, y1))
                    x2 = max(0, min(frame_width, x2))
                    y2 = max(0, min(frame_height, y2))
                    if not self._valid_bbox(x1, y1, x2, y2, frame_width, frame_height):
                        continue
                    det = CamDetection()
                    det.x1 = x1
                    det.y1 = y1
                    det.x2 = x2
                    det.y2 = y2
                    det.center_u = float((x1 + x2) / 2.0)
                    det.center_v = float((y1 + y2) / 2.0)
                    det.confidence = float(box.conf[0].item())
                    detections.append(det)

                    object_det = ObjectDetection()
                    object_det.stamp = msg.header.stamp
                    object_det.name = self._object_name_for_class(cls_id)
                    object_det.class_id = cls_id
                    object_det.confidence = float(box.conf[0].item())
                    object_det.x1 = x1
                    object_det.y1 = y1
                    object_det.x2 = x2
                    object_det.y2 = y2
                    object_det.center_u = float((x1 + x2) / 2.0)
                    object_det.center_v = float((y1 + y2) / 2.0)
                    object_output.detections.append(object_det)

        detections.sort(key=lambda d: d.confidence, reverse=True)
        object_output.detections = sorted(
            object_output.detections,
            key=lambda d: d.confidence,
            reverse=True,
        )
        if len(detections) != self.required_bag_count:
            self.get_logger().warn(
                f"Detected {len(detections)} bag(s), required {self.required_bag_count}.",
                throttle_duration_sec=2.0,
            )
        output.detections = detections
        self.publisher.publish(output)
        self.object_publisher.publish(object_output)


def main(args=None):
    rclpy.init(args=args)
    node = BagDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
