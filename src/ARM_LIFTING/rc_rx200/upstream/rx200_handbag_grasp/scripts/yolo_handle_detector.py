#!/usr/bin/env python3
"""
Minimal YOLO detector node for Step 2 perception.

Subscribes to a compressed RGB image topic, runs Ultralytics YOLO inference,
publishes `vision_msgs/Detection2DArray`, and republishes an annotated image.
"""

import os
from typing import Dict, List

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from ultralytics import YOLO


class YoloHandleDetector(Node):
    """ROS2 wrapper around an Ultralytics YOLO model."""

    def __init__(self) -> None:
        super().__init__('yolo_handle_detector')

        self.declare_parameter('image_topic', '/camera/camera/color/image_raw/compressed')
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('annotated_image_topic', '/yolo/image_annotated/compressed')
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('target_class', 'chair')
        self.declare_parameter('device', '')

        image_topic = str(self.get_parameter('image_topic').value)
        self.detections_topic = str(self.get_parameter('detections_topic').value)
        self.annotated_image_topic = str(self.get_parameter('annotated_image_topic').value)
        self.model_path = str(self.get_parameter('model_path').value)
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.target_class = str(self.get_parameter('target_class').value).strip().lower()
        self.device = str(self.get_parameter('device').value).strip()

        if not self.model_path:
            raise RuntimeError('Parameter "model_path" must not be empty')
        if not os.path.exists(self.model_path):
            raise RuntimeError(f'YOLO model not found: {self.model_path}')

        self.model = YOLO(self.model_path)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.detections_pub = self.create_publisher(Detection2DArray, self.detections_topic, 10)
        self.annotated_pub = self.create_publisher(CompressedImage, self.annotated_image_topic, 10)
        self.image_sub = self.create_subscription(
            CompressedImage,
            image_topic,
            self.image_callback,
            qos,
        )

        self.get_logger().info('YOLO detector started')
        self.get_logger().info(f'  image_topic: {image_topic}')
        self.get_logger().info(f'  detections_topic: {self.detections_topic}')
        self.get_logger().info(f'  annotated_image_topic: {self.annotated_image_topic}')
        self.get_logger().info(f'  model_path: {self.model_path}')
        self.get_logger().info(f'  target_class: {self.target_class}')
        self.get_logger().info(f'  confidence_threshold: {self.confidence_threshold:.2f}')

    def image_callback(self, msg: CompressedImage) -> None:
        """Decode image, run YOLO, publish detections and visualization."""
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning('Failed to decode compressed image')
            return

        kwargs = {
            'conf': self.confidence_threshold,
            'verbose': False,
        }
        if self.device:
            kwargs['device'] = self.device

        results = self.model.predict(frame, **kwargs)
        result = results[0]
        names: Dict[int, str] = result.names

        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        annotated = frame.copy()
        boxes = result.boxes
        if boxes is not None:
            xyxy = boxes.xyxy.cpu().numpy() if boxes.xyxy is not None else np.empty((0, 4))
            confs = boxes.conf.cpu().numpy() if boxes.conf is not None else np.empty((0,))
            classes = boxes.cls.cpu().numpy().astype(int) if boxes.cls is not None else np.empty((0,), dtype=int)

            for index, (bbox, score, cls_idx) in enumerate(zip(xyxy, confs, classes)):
                class_name = str(names.get(int(cls_idx), str(cls_idx))).strip()
                normalized_name = class_name.lower().replace('_', ' ')
                if self.target_class and normalized_name != self.target_class:
                    continue

                x1, y1, x2, y2 = [float(v) for v in bbox.tolist()]
                width = max(0.0, x2 - x1)
                height = max(0.0, y2 - y1)
                center_x = x1 + width / 2.0
                center_y = y1 + height / 2.0

                detection = Detection2D()
                detection.header = msg.header
                detection.id = f'{normalized_name}-{index}'
                detection.bbox.center.position.x = center_x
                detection.bbox.center.position.y = center_y
                detection.bbox.center.theta = 0.0
                detection.bbox.size_x = width
                detection.bbox.size_y = height

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = normalized_name
                hypothesis.hypothesis.score = float(score)
                detection.results.append(hypothesis)
                detections_msg.detections.append(detection)

                label = f'{normalized_name} {float(score):.2f}'
                cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (40, 180, 40), 2)
                cv2.putText(
                    annotated,
                    label,
                    (int(x1), max(20, int(y1) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (40, 180, 40),
                    2,
                    cv2.LINE_AA,
                )

        self.detections_pub.publish(detections_msg)

        ok, encoded = cv2.imencode('.jpg', annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not ok:
            self.get_logger().warning('Failed to encode annotated image')
            return

        annotated_msg = CompressedImage()
        annotated_msg.header = msg.header
        annotated_msg.format = 'jpeg'
        annotated_msg.data = encoded.tobytes()
        self.annotated_pub.publish(annotated_msg)


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = YoloHandleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
