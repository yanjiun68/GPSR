#!/usr/bin/env python3
import math
from typing import Optional

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs  # noqa: F401

from putrabot_interfaces.msg import CamDetection


class BagDepthLocalizerNode(Node):
    def __init__(self):
        super().__init__("bag_depth_localizer_node")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("depth_patch_width_px", 20)
        self.declare_parameter("depth_patch_height_px", 20)
        self.declare_parameter("depth_min_valid_ratio", 0.5)
        self.declare_parameter("max_bbox_age_sec", 0.75)
        self.declare_parameter("max_depth_age_sec", 0.75)
        self.declare_parameter("max_cross_sensor_skew_sec", 0.35)

        self.bridge = CvBridge()
        self.camera_info: Optional[CameraInfo] = None
        self.depth_msg: Optional[Image] = None
        self.selected_bbox: Optional[CamDetection] = None
        self.selected_bbox_received_at: Optional[Time] = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, self.get_parameter("depth_topic").value, self.depth_callback, 10)
        self.create_subscription(CameraInfo, self.get_parameter("camera_info_topic").value, self.camera_info_callback, 10)
        self.create_subscription(CamDetection, "/selected_bag_bbox", self.selected_bbox_callback, 10)
        self.camera_pub = self.create_publisher(PointStamped, "/selected_bag_point_camera", 10)
        self.base_pub = self.create_publisher(PointStamped, "/selected_bag_point_base", 10)

    def depth_callback(self, msg: Image) -> None:
        self.depth_msg = msg
        self.try_publish()

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def selected_bbox_callback(self, msg: CamDetection) -> None:
        self.selected_bbox = msg
        self.selected_bbox_received_at = self.get_clock().now()
        self.try_publish()

    def _depth_to_meters(self, depth_patch: np.ndarray, encoding: str) -> np.ndarray:
        if encoding in ("16UC1", "mono16"):
            return depth_patch.astype(np.float32) / 1000.0
        return depth_patch.astype(np.float32)

    def _stamp_age_sec(self, stamp) -> Optional[float]:
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None
        msg_time = Time.from_msg(stamp)
        return max(0.0, (self.get_clock().now() - msg_time).nanoseconds / 1e9)

    def _skew_sec(self, a_stamp, b_stamp) -> Optional[float]:
        if (a_stamp.sec == 0 and a_stamp.nanosec == 0) or (b_stamp.sec == 0 and b_stamp.nanosec == 0):
            return None
        a_time = Time.from_msg(a_stamp)
        b_time = Time.from_msg(b_stamp)
        return abs((a_time - b_time).nanoseconds) / 1e9

    def try_publish(self) -> None:
        if self.selected_bbox is None or self.depth_msg is None or self.camera_info is None:
            return
        bbox_age = None
        if self.selected_bbox_received_at is not None:
            bbox_age = max(0.0, (self.get_clock().now() - self.selected_bbox_received_at).nanoseconds / 1e9)
        if bbox_age is not None and bbox_age > float(self.get_parameter("max_bbox_age_sec").value):
            self.get_logger().warn("Skipping depth localization due to stale selected bbox.", throttle_duration_sec=2.0)
            return
        depth_age = self._stamp_age_sec(self.depth_msg.header.stamp)
        if depth_age is not None and depth_age > float(self.get_parameter("max_depth_age_sec").value):
            self.get_logger().warn("Skipping depth localization due to stale depth frame.", throttle_duration_sec=2.0)
            return
        skew = None
        if self.selected_bbox_received_at is not None and not (self.depth_msg.header.stamp.sec == 0 and self.depth_msg.header.stamp.nanosec == 0):
            skew = abs((self.selected_bbox_received_at - Time.from_msg(self.depth_msg.header.stamp)).nanoseconds) / 1e9
        if skew is not None and skew > float(self.get_parameter("max_cross_sensor_skew_sec").value):
            self.get_logger().warn("Skipping depth localization due to bbox/depth timestamp skew.", throttle_duration_sec=2.0)
            return

        x1 = max(0, int(self.selected_bbox.x1))
        x2 = max(x1 + 1, int(self.selected_bbox.x2))
        y1 = max(0, int(self.selected_bbox.y1))
        y2 = max(y1 + 1, int(self.selected_bbox.y2))
        width = max(1, int(self.get_parameter("depth_patch_width_px").value))
        height = max(1, int(self.get_parameter("depth_patch_height_px").value))
        cx = int((x1 + x2) / 2)
        cy = int(y1 + 0.7 * (y2 - y1))
        half_w = width // 2
        half_h = height // 2

        depth_img = self.bridge.imgmsg_to_cv2(self.depth_msg, desired_encoding="passthrough")
        h, w = depth_img.shape[:2]
        left = max(0, min(w - 1, cx - half_w))
        right = max(left + 1, min(w, cx + half_w))
        top = max(0, min(h - 1, cy - half_h))
        bottom = max(top + 1, min(h, cy + half_h))
        patch = self._depth_to_meters(depth_img[top:bottom, left:right], self.depth_msg.encoding)
        valid = np.isfinite(patch) & (patch > 0.05)
        valid_ratio = float(valid.sum()) / float(patch.size) if patch.size else 0.0
        if valid_ratio < float(self.get_parameter("depth_min_valid_ratio").value):
            self.get_logger().warn("Depth patch has insufficient valid pixels.", throttle_duration_sec=2.0)
            return

        depth_m = float(np.median(patch[valid]))
        k = self.camera_info.k
        fx, fy, cx0, cy0 = k[0], k[4], k[2], k[5]
        x = (cx - cx0) * depth_m / fx
        y = (cy - cy0) * depth_m / fy
        z = depth_m

        camera_point = PointStamped()
        camera_point.header = self.depth_msg.header
        if not camera_point.header.frame_id:
            # TODO: adjust camera_frame if the robot publishes a different optical frame.
            camera_point.header.frame_id = str(self.get_parameter("camera_frame").value)
        camera_point.point.x = float(x)
        camera_point.point.y = float(y)
        camera_point.point.z = float(z)
        self.camera_pub.publish(camera_point)

        target_frame = str(self.get_parameter("base_frame").value)
        try:
            transformed = self.tf_buffer.transform(camera_point, target_frame, timeout=Duration(seconds=0.2))
            self.base_pub.publish(transformed)
        except TransformException as exc:
            self.get_logger().warn(f"TF transform failed: {exc}", throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = BagDepthLocalizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
