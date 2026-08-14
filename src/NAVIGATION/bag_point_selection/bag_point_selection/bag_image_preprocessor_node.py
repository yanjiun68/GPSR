#!/usr/bin/env python3
from typing import Sequence

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class BagImagePreprocessorNode(Node):
    def __init__(self):
        super().__init__("bag_image_preprocessor_node")
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("enhanced_rgb_topic", "/bag_selection/image_enhanced")
        self.declare_parameter("gamma_enable", True)
        self.declare_parameter("gamma_value", 1.25)
        self.declare_parameter("clahe_enable", True)
        self.declare_parameter("clahe_clip_limit", 2.0)
        self.declare_parameter("clahe_tile_grid_size", [8, 8])
        self.declare_parameter("camera_watchdog_sec", 2.0)

        self.bridge = CvBridge()
        self.gamma_enable = bool(self.get_parameter("gamma_enable").value)
        self.gamma_value = max(1e-3, float(self.get_parameter("gamma_value").value))
        self.clahe_enable = bool(self.get_parameter("clahe_enable").value)
        self.clahe = None
        if self.clahe_enable:
            grid_size = self._parse_grid_size(self.get_parameter("clahe_tile_grid_size").value)
            clip_limit = float(self.get_parameter("clahe_clip_limit").value)
            self.clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=grid_size)
        self.gamma_lut = self._build_gamma_lut(self.gamma_value) if self.gamma_enable else None

        self.publisher = self.create_publisher(
            Image,
            str(self.get_parameter("enhanced_rgb_topic").value),
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            Image,
            str(self.get_parameter("rgb_topic").value),
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.last_image_time = self.get_clock().now()
        watchdog_sec = float(self.get_parameter("camera_watchdog_sec").value)
        if watchdog_sec > 0.0:
            self.watchdog_sec = watchdog_sec
            self.create_timer(watchdog_sec, self._camera_watchdog)

        self.get_logger().info(
            "Processing order: manual exposure at driver -> gamma correction -> CLAHE"
        )

    def _parse_grid_size(self, value: Sequence[int]) -> tuple[int, int]:
        try:
            width = max(1, int(value[0]))
            height = max(1, int(value[1]))
        except (TypeError, ValueError, IndexError):
            width, height = 8, 8
        return width, height

    def _build_gamma_lut(self, gamma_value: float) -> np.ndarray:
        inv_gamma = 1.0 / gamma_value
        values = np.arange(256, dtype=np.float32) / 255.0
        corrected = np.power(values, inv_gamma) * 255.0
        return np.clip(corrected, 0.0, 255.0).astype(np.uint8)

    def _apply_gamma(self, frame: np.ndarray) -> np.ndarray:
        if self.gamma_lut is None:
            return frame
        return cv2.LUT(frame, self.gamma_lut)

    def _apply_clahe(self, frame: np.ndarray) -> np.ndarray:
        if self.clahe is None:
            return frame
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l_channel, a_channel, b_channel = cv2.split(lab)
        l_channel = self.clahe.apply(l_channel)
        enhanced = cv2.merge((l_channel, a_channel, b_channel))
        return cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)

    def _camera_watchdog(self) -> None:
        age_sec = (self.get_clock().now() - self.last_image_time).nanoseconds / 1e9
        if age_sec > self.watchdog_sec:
            self.get_logger().error(
                f"No camera frames received for {age_sec:.1f}s (topic: "
                f"{self.get_parameter('rgb_topic').value}). Check camera driver.",
                throttle_duration_sec=5.0,
            )

    def image_callback(self, msg: Image) -> None:
        self.last_image_time = self.get_clock().now()
        try:
            if not self.gamma_enable and not self.clahe_enable:
                self.publisher.publish(msg)
                return

            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            frame = self._apply_gamma(frame)
            frame = self._apply_clahe(frame)

            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            out_msg.header = msg.header
            self.publisher.publish(out_msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"Preprocessing failed: {exc}", throttle_duration_sec=2.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = BagImagePreprocessorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
