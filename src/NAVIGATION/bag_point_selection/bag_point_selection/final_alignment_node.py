#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from putrabot_interfaces.msg import CamDetection


class FinalAlignmentNode(Node):
    def __init__(self):
        super().__init__("final_alignment_node")
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("align_center_tolerance_px", 20)
        self.declare_parameter("align_yaw_gain", 0.002)
        self.declare_parameter("final_stop_distance_m", 0.35)
        self.declare_parameter("final_forward_speed_mps", 0.05)
        self.declare_parameter("max_align_angular_speed", 0.35)
        self.declare_parameter("max_align_forward_speed", 0.08)
        self.declare_parameter("final_alignment_cmd_vel_topic", "/cmd_vel_bag")
        self.declare_parameter("max_target_point_age_sec", 0.75)
        self.declare_parameter("enable_lidar_final_stop", True)

        self.nav_status = "idle"
        self.latest_bbox: Optional[CamDetection] = None
        self.latest_base_point: Optional[PointStamped] = None
        self.latest_cluster_point: Optional[PointStamped] = None
        self.latest_image_width: Optional[int] = None
        self.ready = False

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("final_alignment_cmd_vel_topic").value), 10
        )
        self.ready_pub = self.create_publisher(Bool, "/bag_ready_for_pickup", 10)
        self.status_pub = self.create_publisher(String, "/bag_align_status", 10)
        self.create_subscription(String, "/bag_nav_status", self.nav_callback, 10)
        self.create_subscription(CamDetection, "/selected_bag_bbox", self.bbox_callback, 10)
        self.create_subscription(PointStamped, "/selected_bag_point_base", self.base_point_callback, 10)
        self.create_subscription(PointStamped, "/selected_bag_cluster", self.cluster_point_callback, 10)
        self.create_subscription(Image, self.get_parameter("rgb_topic").value, self.image_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.publish_ready(False)
        self.status_pub.publish(String(data="idle"))

    def nav_callback(self, msg: String) -> None:
        self.nav_status = msg.data
        if self.nav_status != "succeeded":
            self.publish_ready(False)
            self.stop_robot()
            self.status_pub.publish(String(data="idle"))

    def bbox_callback(self, msg: CamDetection) -> None:
        self.latest_bbox = msg

    def base_point_callback(self, msg: PointStamped) -> None:
        self.latest_base_point = msg

    def cluster_point_callback(self, msg: PointStamped) -> None:
        self.latest_cluster_point = msg

    def image_callback(self, msg: Image) -> None:
        self.latest_image_width = int(msg.width)

    def publish_ready(self, ready: bool) -> None:
        self.ready = ready
        self.ready_pub.publish(Bool(data=ready))

    def stop_robot(self) -> None:
        self.cmd_pub.publish(Twist())

    def _stamp_age_sec(self, stamp) -> Optional[float]:
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None
        msg_time = Time.from_msg(stamp)
        return max(0.0, (self.get_clock().now() - msg_time).nanoseconds / 1e9)

    def _choose_target_point(self) -> tuple[Optional[PointStamped], bool]:
        if bool(self.get_parameter("enable_lidar_final_stop").value) and self.latest_cluster_point is not None:
            return self.latest_cluster_point, True
        return self.latest_base_point, False

    def control_loop(self) -> None:
        if self.nav_status != "succeeded":
            return
        if self.latest_bbox is None or self.latest_image_width is None:
            self.status_pub.publish(String(data="tracking_lost"))
            self.stop_robot()
            return

        bag_center = 0.5 * (self.latest_bbox.x1 + self.latest_bbox.x2)
        image_center = 0.5 * self.latest_image_width
        pixel_error = bag_center - image_center
        tolerance = float(self.get_parameter("align_center_tolerance_px").value)

        cmd = Twist()
        angular = -pixel_error * float(self.get_parameter("align_yaw_gain").value)
        max_angular = float(self.get_parameter("max_align_angular_speed").value)
        angular = max(-max_angular, min(max_angular, angular))

        target_point, using_cluster = self._choose_target_point()
        target_fresh = False
        if target_point is not None:
            target_age = self._stamp_age_sec(target_point.header.stamp)
            target_fresh = target_age is None or target_age <= float(self.get_parameter("max_target_point_age_sec").value)
        if bool(self.get_parameter("enable_lidar_final_stop").value) and using_cluster and not target_fresh:
            target_point = self.latest_base_point
            using_cluster = False
            if target_point is not None:
                target_age = self._stamp_age_sec(target_point.header.stamp)
                target_fresh = target_age is None or target_age <= float(self.get_parameter("max_target_point_age_sec").value)

        distance = math.hypot(target_point.point.x, target_point.point.y) if target_point is not None and target_fresh else None
        aligned = abs(pixel_error) <= tolerance
        close_enough = distance is not None and distance <= float(self.get_parameter("final_stop_distance_m").value)

        if not aligned:
            cmd.angular.z = angular
            self.status_pub.publish(String(data="aligning_yaw"))
            self.publish_ready(False)
        elif bool(self.get_parameter("enable_lidar_final_stop").value) and target_point is None:
            self.status_pub.publish(String(data="waiting_for_target"))
            self.publish_ready(False)
        elif bool(self.get_parameter("enable_lidar_final_stop").value) and not target_fresh:
            self.status_pub.publish(String(data="target_stale"))
            self.publish_ready(False)
        elif bool(self.get_parameter("enable_lidar_final_stop").value) and distance is not None and not close_enough:
            cmd.linear.x = min(
                float(self.get_parameter("final_forward_speed_mps").value),
                float(self.get_parameter("max_align_forward_speed").value),
            )
            self.status_pub.publish(String(data="aligning_distance"))
            self.publish_ready(False)
        elif not bool(self.get_parameter("enable_lidar_final_stop").value) and aligned:
            self.status_pub.publish(String(data="ready"))
            self.publish_ready(True)
        else:
            self.status_pub.publish(String(data="ready"))
            self.publish_ready(True)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FinalAlignmentNode()
    try:
        rclpy.spin(node)
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
