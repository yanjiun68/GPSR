#!/usr/bin/env python3
import json

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO

from face_detection.paths import default_detection_model_paths, default_glasses_model_path


class DetectedItemsNode(Node):
    def __init__(self):
        super().__init__('detected_items_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detections_topic', '/perception/detected_items')
        self.declare_parameter('debug_image_topic', '/perception/detected_items_image')
        self.declare_parameter('yolo_model_path', default_glasses_model_path())
        self.declare_parameter('yolo_model_paths', default_detection_model_paths())
        self.declare_parameter('yolo_conf_threshold', 0.4)
        self.declare_parameter('publish_debug_image', False)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.detections_topic = str(self.get_parameter('detections_topic').value)
        self.debug_image_topic = str(self.get_parameter('debug_image_topic').value)
        self.yolo_model_path = str(self.get_parameter('yolo_model_path').value)
        self.yolo_model_paths = [
            str(path) for path in self.get_parameter('yolo_model_paths').value if str(path).strip()
        ]
        if not self.yolo_model_paths:
            self.yolo_model_paths = [self.yolo_model_path]

        self.yolo_conf_threshold = float(self.get_parameter('yolo_conf_threshold').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)

        self.bridge = CvBridge()

        self.detections_pub = self.create_publisher(String, self.detections_topic, 10)
        self.debug_image_pub = None
        if self.publish_debug_image:
            self.debug_image_pub = self.create_publisher(Image, self.debug_image_topic, 10)

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )

        self.get_logger().info(f'Loading YOLO detectors from {", ".join(self.yolo_model_paths)}')
        self.yolo_models = [YOLO(model_path) for model_path in self.yolo_model_paths]

        self.get_logger().info(
            f'DetectedItemsNode started. '
            f'image_topic={self.image_topic}, detections_topic={self.detections_topic}, '
            f'conf_threshold={self.yolo_conf_threshold}, model_count={len(self.yolo_models)}'
        )

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        debug_frame = frame.copy()

        detections = []

        for model_index, yolo_model in enumerate(self.yolo_models):
            results = yolo_model(frame, verbose=False)

            for result in results:
                boxes = result.boxes
                names = result.names
                if boxes is None:
                    continue

                for box in boxes:
                    conf = float(box.conf[0])
                    if conf < self.yolo_conf_threshold:
                        continue

                    cls_id = int(box.cls[0])
                    label = str(names[cls_id]).strip().lower().replace(' ', '_').replace('-', '_')
                    xyxy = box.xyxy[0].tolist()
                    x1, y1, x2, y2 = [float(v) for v in xyxy]

                    detections.append({
                        'label': label,
                        'confidence': conf,
                        'bbox': [x1, y1, x2, y2],
                        'model_index': model_index,
                    })

                    if self.debug_image_pub is not None:
                        import cv2
                        cv2.rectangle(
                            debug_frame,
                            (int(x1), int(y1)),
                            (int(x2), int(y2)),
                            (0, 255, 0),
                            2,
                        )
                        cv2.putText(
                            debug_frame,
                            f'{label} {conf:.2f}',
                            (int(x1), max(int(y1) - 10, 20)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 255, 0),
                            2,
                        )

        out = {
            'frame_id': msg.header.frame_id,
            'stamp_sec': msg.header.stamp.sec,
            'stamp_nanosec': msg.header.stamp.nanosec,
            'count': len(detections),
            'detections': detections,
        }

        detections_msg = String()
        detections_msg.data = json.dumps(out)
        self.detections_pub.publish(detections_msg)

        if self.debug_image_pub is not None:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectedItemsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
