#!/usr/bin/env python3
"""MediaPipe hand viewer with bag-detection overlay."""

import math
import time
from typing import List, Optional, Tuple

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2D, Detection2DArray


class MediaPipeBagViewer(Node):
    """Show camera frames annotated with MediaPipe hands and up to two bag boxes."""

    def __init__(self):
        super().__init__('mediapipe_bag_viewer')

        self.declare_parameter('image_topic', '/camera/camera/color/image_raw/compressed')
        self.declare_parameter('detections_topic', '/bag_detector/detections')
        self.declare_parameter('output_topic', '/bag_detector/mediapipe_annotated/compressed')
        self.declare_parameter('window_name', 'MediaPipe + 2 Bags')
        self.declare_parameter('show_window', True)
        self.declare_parameter('max_detection_age_sec', 1.0)
        self.declare_parameter('max_draw_detections', 2)
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)
        self.declare_parameter('pointing.arrow_length_ratio', 0.15)
        self.declare_parameter('pointing.max_perp_distance_px', 180.0)

        image_topic = self.get_parameter('image_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.window_name = self.get_parameter('window_name').value
        self.show_window = bool(self.get_parameter('show_window').value)
        self.max_detection_age_sec = float(self.get_parameter('max_detection_age_sec').value)
        self.max_draw_detections = int(self.get_parameter('max_draw_detections').value)
        min_detection_confidence = float(self.get_parameter('min_detection_confidence').value)
        min_tracking_confidence = float(self.get_parameter('min_tracking_confidence').value)
        self.arrow_length_ratio = float(self.get_parameter('pointing.arrow_length_ratio').value)
        self.max_perp_distance_px = float(self.get_parameter('pointing.max_perp_distance_px').value)

        self._latest_detections: List[Detection2D] = []
        self._latest_detection_time = 0.0

        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_draw_styles = mp.solutions.drawing_styles
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            model_complexity=1,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )

        self.image_sub = self.create_subscription(
            CompressedImage, image_topic, self._image_callback, 10
        )
        self.detections_sub = self.create_subscription(
            Detection2DArray, detections_topic, self._detections_callback, 10
        )
        self.output_pub = self.create_publisher(CompressedImage, self.output_topic, 10)

        self.get_logger().info('MediaPipe bag viewer started')
        self.get_logger().info(f'  image_topic: {image_topic}')
        self.get_logger().info(f'  detections_topic: {detections_topic}')
        self.get_logger().info(f'  output_topic: {self.output_topic}')
        self.get_logger().info(f'  window_name: {self.window_name}')

    def _detections_callback(self, msg: Detection2DArray) -> None:
        detections = list(msg.detections)
        detections.sort(key=self._score_detection, reverse=True)
        self._latest_detections = detections[: self.max_draw_detections]
        self._latest_detection_time = time.time()

    def _image_callback(self, msg: CompressedImage) -> None:
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if image is None:
                return

            annotated = image.copy()
            pointing_ray = self._draw_mediapipe_hands(annotated)
            selected_index = self._select_pointed_detection(pointing_ray)
            self._draw_detections(annotated, selected_index)

            out_msg = CompressedImage()
            out_msg.header = msg.header
            out_msg.format = 'jpeg'
            ok, encoded = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                out_msg.data = encoded.tobytes()
                self.output_pub.publish(out_msg)

            if self.show_window:
                cv2.imshow(self.window_name, annotated)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.get_logger().info('Viewer closed by user')
                    rclpy.shutdown()
        except Exception as exc:
            self.get_logger().error(f'MediaPipe viewer error: {exc}')

    def _draw_mediapipe_hands(self, image) -> Optional[Tuple[Tuple[int, int], Tuple[int, int]]]:
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)
        if not results.multi_hand_landmarks:
            cv2.putText(
                image,
                'MediaPipe: no hand',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )
            return None

        pointing_ray = None
        for hand_landmarks in results.multi_hand_landmarks:
            self.mp_draw.draw_landmarks(
                image,
                hand_landmarks,
                self.mp_hands.HAND_CONNECTIONS,
                self.mp_draw_styles.get_default_hand_landmarks_style(),
                self.mp_draw_styles.get_default_hand_connections_style(),
            )
            if pointing_ray is None:
                pointing_ray = self._draw_index_arrow(image, hand_landmarks)
        cv2.putText(
            image,
            f'MediaPipe: {len(results.multi_hand_landmarks)} hand(s)',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 180, 255),
            2,
        )
        return pointing_ray

    def _draw_index_arrow(self, image, hand_landmarks) -> Optional[Tuple[Tuple[int, int], Tuple[int, int]]]:
        height, width = image.shape[:2]
        index_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]
        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]

        start = (int(index_mcp.x * width), int(index_mcp.y * height))
        tip = (int(index_tip.x * width), int(index_tip.y * height))
        direction = np.array([tip[0] - start[0], tip[1] - start[1]], dtype=np.float32)
        norm = float(np.linalg.norm(direction))
        if norm < 5.0:
            return None

        direction /= norm
        arrow_len = int(max(width, height) * self.arrow_length_ratio)
        end = (
            int(tip[0] + direction[0] * arrow_len),
            int(tip[1] + direction[1] * arrow_len),
        )

        cv2.arrowedLine(image, tip, end, (255, 80, 0), 5, tipLength=0.08)
        cv2.circle(image, tip, 8, (255, 80, 0), -1)
        cv2.putText(
            image,
            'index pointing ray',
            (max(0, tip[0] + 10), max(25, tip[1] - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 80, 0),
            2,
        )
        return tip, end

    def _draw_detections(self, image, selected_index: Optional[int]) -> None:
        if (time.time() - self._latest_detection_time) > self.max_detection_age_sec:
            cv2.putText(
                image,
                'Detector: no recent bag detections',
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )
            return

        colors = [(0, 255, 0), (255, 180, 0)]
        for index, det in enumerate(self._latest_detections):
            cx = float(det.bbox.center.position.x)
            cy = float(det.bbox.center.position.y)
            w = float(det.bbox.size_x)
            h = float(det.bbox.size_y)
            x1 = int(cx - w / 2.0)
            y1 = int(cy - h / 2.0)
            x2 = int(cx + w / 2.0)
            y2 = int(cy + h / 2.0)
            is_selected = selected_index == index
            color = (255, 80, 0) if is_selected else colors[index % len(colors)]
            label = self._label_detection(det, index)
            if is_selected:
                label = f'POINTED -> {label}'

            cv2.rectangle(image, (x1, y1), (x2, y2), color, 4 if is_selected else 2)
            cv2.putText(
                image,
                label,
                (max(0, x1), max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )

    def _select_pointed_detection(
        self,
        pointing_ray: Optional[Tuple[Tuple[int, int], Tuple[int, int]]],
    ) -> Optional[int]:
        if pointing_ray is None:
            return None
        if (time.time() - self._latest_detection_time) > self.max_detection_age_sec:
            return None
        if not self._latest_detections:
            return None

        ray_start, ray_end = pointing_ray
        ray_vec = np.array([ray_end[0] - ray_start[0], ray_end[1] - ray_start[1]], dtype=np.float32)
        ray_norm = float(np.linalg.norm(ray_vec))
        if ray_norm < 1.0:
            return None
        ray_unit = ray_vec / ray_norm

        best_index = None
        best_score = math.inf
        for index, det in enumerate(self._latest_detections):
            center = np.array(
                [float(det.bbox.center.position.x), float(det.bbox.center.position.y)],
                dtype=np.float32,
            )
            to_center = center - np.array(ray_start, dtype=np.float32)
            forward_distance = float(np.dot(to_center, ray_unit))
            if forward_distance < 0.0:
                continue
            closest = np.array(ray_start, dtype=np.float32) + ray_unit * forward_distance
            perpendicular_distance = float(np.linalg.norm(center - closest))
            if perpendicular_distance > self.max_perp_distance_px:
                continue
            score = perpendicular_distance + forward_distance * 0.02
            if score < best_score:
                best_score = score
                best_index = index
        return best_index

    @staticmethod
    def _score_detection(det: Detection2D) -> float:
        if not det.results:
            return 0.0
        return float(det.results[0].hypothesis.score)

    def _label_detection(self, det: Detection2D, index: int) -> str:
        if not det.results:
            return f'bag {index + 1}'
        class_id = str(det.results[0].hypothesis.class_id)
        score = float(det.results[0].hypothesis.score)
        return f'bag {index + 1}: {class_id} {score:.2f}'

    def destroy_node(self):  # type: ignore[override]
        self.hands.close()
        try:
            cv2.destroyWindow(self.window_name)
        except cv2.error:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MediaPipeBagViewer()
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
