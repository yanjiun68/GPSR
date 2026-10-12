import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

from .astra_adapter import AstraAdapter


class AstraRegistrationNode(Node):
    def __init__(self):
        super().__init__("astra_registration_node")

        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("color_topic", "/camera/image_raw")
        self.declare_parameter("depth_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("color_info_topic", "/camera/camera_info")
        self.declare_parameter(
            "aligned_color_topic", "/camera/aligned_color_to_depth/image_raw"
        )
        self.declare_parameter(
            "valid_mask_topic", "/camera/aligned_color_to_depth/valid_mask"
        )
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop", 0.05)
        # Section: Cap registered RGB-D output so downstream tracking processes no more than the configured rate.
        self.declare_parameter("max_output_rate_hz", 15.0)
        self.declare_parameter(
            "rotation",
            [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0],
        )
        self.declare_parameter("translation", [0.0, 0.0, 0.0])

        self.bridge = CvBridge()
        self.adapter = None
        self.depth_info = None
        self.color_info = None
        self._lock = threading.Lock()
        # Section: A monotonic deadline rate-limits complete synchronized pairs before costly OpenCV registration.
        self._last_published_monotonic = float("-inf")

        self.create_subscription(
            CameraInfo,
            self.get_parameter("depth_info_topic").value,
            self.depth_info_cb,
            10,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("color_info_topic").value,
            self.color_info_cb,
            10,
        )

        self.depth_sub = Subscriber(
            self,
            Image,
            self.get_parameter("depth_topic").value,
            qos_profile=qos_profile_sensor_data,
        )
        self.color_sub = Subscriber(
            self,
            Image,
            self.get_parameter("color_topic").value,
            qos_profile=qos_profile_sensor_data,
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.depth_sub, self.color_sub],
            queue_size=int(self.get_parameter("sync_queue_size").value),
            slop=float(self.get_parameter("sync_slop").value),
        )
        self.sync.registerCallback(self.synced_cb)

        self.aligned_pub = self.create_publisher(
            Image, self.get_parameter("aligned_color_topic").value, 10
        )
        self.mask_pub = self.create_publisher(
            Image, self.get_parameter("valid_mask_topic").value, 10
        )

        self.get_logger().info("Astra color-to-depth registration started.")

    def depth_info_cb(self, msg):
        with self._lock:
            self.depth_info = msg
            self.try_build_adapter()

    def color_info_cb(self, msg):
        with self._lock:
            self.color_info = msg
            self.try_build_adapter()

    @staticmethod
    def intrinsics(msg):
        return {"fx": msg.k[0], "fy": msg.k[4], "cx": msg.k[2], "cy": msg.k[5]}

    def try_build_adapter(self):
        if self.adapter is not None or self.depth_info is None or self.color_info is None:
            return

        rotation = np.asarray(
            self.get_parameter("rotation").value, dtype=np.float32
        ).reshape(3, 3)
        translation = np.asarray(
            self.get_parameter("translation").value, dtype=np.float32
        ).reshape(3)

        self.adapter = AstraAdapter(
            self.intrinsics(self.depth_info),
            self.intrinsics(self.color_info),
            rotation,
            translation,
            depth_width=self.depth_info.width,
            depth_height=self.depth_info.height,
            color_width=self.color_info.width,
            color_height=self.color_info.height,
            depth_scale=float(self.get_parameter("depth_scale").value),
        )

        self.get_logger().info(
            f"Registration ready: depth={self.depth_info.width}x{self.depth_info.height}, "
            f"color={self.color_info.width}x{self.color_info.height}"
        )

    def synced_cb(self, depth_msg, color_msg):
        """Special function: Register one timestamp-compatible RGB-D pair at the configured maximum rate."""
        # Section: Drop excess synchronized pairs before image conversion so the slowest valid stream sets the 3D rate.
        with self._lock:
            adapter = self.adapter
            if adapter is None:
                return
            max_output_rate_hz = max(0.1, float(self.get_parameter("max_output_rate_hz").value))
            now_monotonic = time.monotonic()
            if now_monotonic - self._last_published_monotonic < 1.0 / max_output_rate_hz:
                return
            self._last_published_monotonic = now_monotonic

        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            aligned, mask = adapter.align_color_to_depth(depth, color)

            aligned_msg = self.bridge.cv2_to_imgmsg(aligned, encoding="bgr8")
            aligned_msg.header = depth_msg.header
            self.aligned_pub.publish(aligned_msg)

            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            mask_msg.header = depth_msg.header
            self.mask_pub.publish(mask_msg)
        except Exception as exc:
            self.get_logger().error(f"Registration failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = AstraRegistrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Section: Launch may already have shut down the shared rclpy context on SIGINT.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
