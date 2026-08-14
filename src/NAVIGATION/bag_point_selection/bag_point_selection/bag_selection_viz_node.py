#!/usr/bin/env python3
from typing import List, Optional

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String

from putrabot_interfaces.msg import CamDetection, CamDetections


class BagSelectionVizNode(Node):
    def __init__(self):
        super().__init__("bag_selection_viz_node")
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("bag_viz_enabled", True)
        self.declare_parameter("bag_viz_topic", "/bag_selection/annotated_image")
        self.declare_parameter("bag_viz_show_window", True)
        self.declare_parameter("bag_viz_window_name", "Bag Selection Debug")
        self.declare_parameter("bag_viz_use_pose_debug", False)
        self.declare_parameter("pose_debug_topic", "/bag_selection/pose_debug_image")
        self.declare_parameter("pose_debug_max_age_sec", 0.5)
        self.declare_parameter("max_pointed_bag_age_sec", 0.75)

        self.bridge = CvBridge()
        self.latest_detections: List[CamDetection] = []
        self.selected_bag: Optional[CamDetection] = None
        self.pointed_bag: Optional[CamDetection] = None
        self.pointed_bag_received_at = None
        self.latest_pose_image: Optional[Image] = None
        self.latest_pointing_label = "none"
        self.latest_tracking_mode = "target_lost"
        self.latest_motion_status = "idle"
        self.enabled = bool(self.get_parameter("bag_viz_enabled").value)
        self.show_window = bool(self.get_parameter("bag_viz_show_window").value)
        self.window_name = str(self.get_parameter("bag_viz_window_name").value)
        self.use_pose_debug = bool(self.get_parameter("bag_viz_use_pose_debug").value)

        if not self.enabled:
            self.get_logger().info("Bag selection visualization disabled by parameter.")
            self.publisher = None
            return

        self.publisher = self.create_publisher(
            Image, str(self.get_parameter("bag_viz_topic").value), 10
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("rgb_topic").value),
            self.image_callback,
            10,
        )
        self.create_subscription(CamDetections, "/detected_bags", self.detections_callback, 10)
        self.create_subscription(CamDetection, "/selected_bag_bbox", self.selected_callback, 10)
        self.create_subscription(CamDetection, "/pointed_bag_bbox", self.pointed_callback, 10)
        self.create_subscription(String, "/pointing_direction_raw", self.pointing_callback, 10)
        self.create_subscription(String, "/target_tracking_mode", self.tracking_mode_callback, 10)
        self.create_subscription(String, "/bag_motion_status", self.motion_status_callback, 10)
        if self.use_pose_debug:
            self.create_subscription(
                Image,
                str(self.get_parameter("pose_debug_topic").value),
                self.pose_image_callback,
                10,
            )
        self.get_logger().info(
            f"Publishing annotated bag selection images on {self.get_parameter('bag_viz_topic').value}"
        )
        if self.show_window:
            self.get_logger().info(f"OpenCV window enabled: {self.window_name}")
        if self.use_pose_debug:
            self.get_logger().info(
                f"Using pose debug image as visualization base from {self.get_parameter('pose_debug_topic').value}"
            )

    def detections_callback(self, msg: CamDetections) -> None:
        self.latest_detections = list(msg.detections)

    def selected_callback(self, msg: CamDetection) -> None:
        self.selected_bag = msg

    def pointed_callback(self, msg: CamDetection) -> None:
        self.pointed_bag = msg
        self.pointed_bag_received_at = self.get_clock().now()

    def pointing_callback(self, msg: String) -> None:
        self.latest_pointing_label = msg.data.strip() or "none"

    def tracking_mode_callback(self, msg: String) -> None:
        self.latest_tracking_mode = msg.data.strip() or "target_lost"

    def motion_status_callback(self, msg: String) -> None:
        self.latest_motion_status = msg.data.strip() or "idle"

    def pose_image_callback(self, msg: Image) -> None:
        self.latest_pose_image = msg

    def _draw_box(self, frame, det: CamDetection, color, label: str) -> None:
        x1 = int(det.x1)
        y1 = int(det.y1)
        x2 = int(det.x2)
        y2 = int(det.y2)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            frame,
            label,
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
            cv2.LINE_AA,
        )

    def _stamp_age_sec(self, stamp) -> Optional[float]:
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None
        msg_time = Time.from_msg(stamp)
        return max(0.0, (self.get_clock().now() - msg_time).nanoseconds / 1e9)

    def _get_base_frame(self, rgb_msg: Image):
        if self.use_pose_debug and self.latest_pose_image is not None:
            age_sec = self._stamp_age_sec(self.latest_pose_image.header.stamp)
            if age_sec is None or age_sec <= float(self.get_parameter("pose_debug_max_age_sec").value):
                return self.bridge.imgmsg_to_cv2(self.latest_pose_image, desired_encoding="bgr8")
        return self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")

    def _fresh_pointed_bag(self) -> Optional[CamDetection]:
        if self.pointed_bag is None or self.pointed_bag_received_at is None:
            return None
        age_sec = (self.get_clock().now() - self.pointed_bag_received_at).nanoseconds / 1e9
        if age_sec > float(self.get_parameter("max_pointed_bag_age_sec").value):
            return None
        return self.pointed_bag

    def image_callback(self, msg: Image) -> None:
        if not self.enabled or self.publisher is None:
            return

        frame = self._get_base_frame(msg)

        for idx, det in enumerate(self.latest_detections):
            label = f"bag {idx + 1} {det.confidence:.2f}"
            self._draw_box(frame, det, (0, 255, 0), label)

        pointed_bag = self._fresh_pointed_bag()
        if pointed_bag is not None:
            self._draw_box(frame, pointed_bag, (255, 128, 0), "pointed bag")

        if self.selected_bag is not None:
            self._draw_box(frame, self.selected_bag, (0, 0, 255), "selected bag")

        cv2.putText(
            frame,
            f"Pointing: {self.latest_pointing_label}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            f"Tracking: {self.latest_tracking_mode}",
            (20, 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            f"Motion: {self.latest_motion_status}",
            (20, 92),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        annotated = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        annotated.header = msg.header
        self.publisher.publish(annotated)

        if self.show_window:
            cv2.imshow(self.window_name, frame)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.enabled and self.show_window:
            cv2.destroyWindow(self.window_name)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BagSelectionVizNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
