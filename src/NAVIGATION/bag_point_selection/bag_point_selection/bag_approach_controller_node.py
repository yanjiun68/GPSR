#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Float32, String

from putrabot_interfaces.msg import CamDetection


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class BagApproachControllerNode(Node):
    def __init__(self):
        super().__init__("bag_approach_controller_node")
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("heading_tolerance_rad", 0.12)
        self.declare_parameter("approach_heading_deadband_rad", 0.35)
        self.declare_parameter("prealign_distance_m", 0.80)
        self.declare_parameter("final_stop_distance_m", 0.35)
        self.declare_parameter("max_forward_speed_mps", 0.16)
        self.declare_parameter("final_approach_speed_mps", 0.05)
        self.declare_parameter("max_angular_speed_radps", 0.45)
        self.declare_parameter("turn_kp", 1.2)
        self.declare_parameter("approach_linear_kp", 0.35)
        self.declare_parameter("approach_angular_kp", 1.1)
        self.declare_parameter("visual_center_kp", 0.0025)
        self.declare_parameter("image_centering_tolerance_px", 20)
        self.declare_parameter("target_stale_timeout_sec", 0.6)
        self.declare_parameter("max_bbox_age_sec", 0.6)
        self.declare_parameter("min_tracking_confidence_to_move", 0.35)
        self.declare_parameter("obstacle_sector_half_angle_deg", 18.0)
        self.declare_parameter("obstacle_stop_distance_m", 0.38)
        self.declare_parameter("obstacle_rotation_clearance_m", 0.20)

        self.selection_state = "IDLE"
        self.target_mode = "target_lost"
        self.fused_target: Optional[PointStamped] = None
        self.fused_confidence = 0.0
        self.latest_bbox: Optional[CamDetection] = None
        self.latest_bbox_received_at: Optional[Time] = None
        self.latest_image_width: Optional[int] = None
        self.latest_scan: Optional[LaserScan] = None
        self.ready = False

        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.ready_pub = self.create_publisher(Bool, "/bag_ready_for_pickup", 10)
        self.status_pub = self.create_publisher(String, "/bag_motion_status", 10)

        self.create_subscription(String, "/selection_state", self.selection_state_callback, 10)
        self.create_subscription(String, "/target_tracking_mode", self.target_mode_callback, 10)
        self.create_subscription(PointStamped, "/selected_bag_fused_center", self.fused_target_callback, 10)
        self.create_subscription(Float32, "/selected_bag_fused_confidence", self.fused_confidence_callback, 10)
        self.create_subscription(CamDetection, "/selected_bag_bbox", self.bbox_callback, 10)
        self.create_subscription(Image, self.get_parameter("rgb_topic").value, self.image_callback, 10)
        self.create_subscription(LaserScan, self.get_parameter("scan_topic").value, self.scan_callback, 10)
        self.create_subscription(Bool, "/bag_selection/reset", self.reset_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.publish_ready(False)
        self.status_pub.publish(String(data="idle"))

    def selection_state_callback(self, msg: String) -> None:
        self.selection_state = msg.data.strip()

    def target_mode_callback(self, msg: String) -> None:
        self.target_mode = msg.data.strip()

    def fused_target_callback(self, msg: PointStamped) -> None:
        self.fused_target = msg

    def fused_confidence_callback(self, msg: Float32) -> None:
        self.fused_confidence = float(msg.data)

    def bbox_callback(self, msg: CamDetection) -> None:
        self.latest_bbox = msg
        self.latest_bbox_received_at = self.get_clock().now()

    def image_callback(self, msg: Image) -> None:
        self.latest_image_width = int(msg.width)

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def reset_callback(self, msg: Bool) -> None:
        if bool(msg.data):
            self.publish_ready(False)
            self.stop_robot()
            self.status_pub.publish(String(data="idle"))

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

    def _bbox_is_fresh(self) -> bool:
        if self.latest_bbox is None or self.latest_bbox_received_at is None:
            return False
        age = (self.get_clock().now() - self.latest_bbox_received_at).nanoseconds / 1e9
        return age <= float(self.get_parameter("max_bbox_age_sec").value)

    def _image_error(self) -> Optional[float]:
        if self.latest_bbox is None or self.latest_image_width is None or not self._bbox_is_fresh():
            return None
        bag_center = 0.5 * (self.latest_bbox.x1 + self.latest_bbox.x2)
        image_center = 0.5 * self.latest_image_width
        return float(bag_center - image_center)

    def _scan_clearance(self) -> tuple[Optional[float], Optional[float]]:
        if self.latest_scan is None:
            return None, None
        front_half_angle = math.radians(float(self.get_parameter("obstacle_sector_half_angle_deg").value))
        min_front = None
        min_rotate = None
        angle = self.latest_scan.angle_min
        for rng in self.latest_scan.ranges:
            if not math.isfinite(rng) or rng < self.latest_scan.range_min or rng > self.latest_scan.range_max:
                angle += self.latest_scan.angle_increment
                continue
            if abs(angle) <= front_half_angle:
                min_front = rng if min_front is None else min(min_front, rng)
            if abs(angle) <= math.radians(8.0):
                min_rotate = rng if min_rotate is None else min(min_rotate, rng)
            angle += self.latest_scan.angle_increment
        return min_front, min_rotate

    def _should_run(self) -> bool:
        return self.selection_state == "APPROACH_TARGET"

    def _publish_status(self, status: str) -> None:
        self.status_pub.publish(String(data=status))

    def control_loop(self) -> None:
        if not self._should_run():
            if self.ready:
                self.publish_ready(False)
            self.stop_robot()
            self._publish_status("idle")
            return

        if self.target_mode == "target_lost":
            self.publish_ready(False)
            self.stop_robot()
            self._publish_status("target_lost")
            return

        if self.fused_target is None:
            self.publish_ready(False)
            self.stop_robot()
            self._publish_status("stale_target")
            return

        target_age = self._stamp_age_sec(self.fused_target.header.stamp)
        if target_age is not None and target_age > float(self.get_parameter("target_stale_timeout_sec").value):
            self.publish_ready(False)
            self.stop_robot()
            self._publish_status("stale_target")
            return

        if self.fused_confidence < float(self.get_parameter("min_tracking_confidence_to_move").value):
            self.publish_ready(False)
            self.stop_robot()
            self._publish_status("low_confidence")
            return

        target_x = float(self.fused_target.point.x)
        target_y = float(self.fused_target.point.y)
        distance = math.hypot(target_x, target_y)
        heading_error = math.atan2(target_y, target_x)
        image_error = self._image_error()
        image_tol = float(self.get_parameter("image_centering_tolerance_px").value)
        heading_tol = float(self.get_parameter("heading_tolerance_rad").value)

        obstacle_front, obstacle_rotate = self._scan_clearance()
        linear_blocked = (
            obstacle_front is not None and obstacle_front <= float(self.get_parameter("obstacle_stop_distance_m").value)
        )
        rotation_blocked = (
            obstacle_rotate is not None
            and obstacle_rotate <= float(self.get_parameter("obstacle_rotation_clearance_m").value)
        )

        # The controller becomes more vision-driven as the robot gets close.
        phase = "approach_target"
        if abs(heading_error) > heading_tol:
            phase = "turn_to_target"
        elif distance > float(self.get_parameter("prealign_distance_m").value):
            phase = "approach_target"
        elif image_error is None:
            phase = "waiting_for_visual"
        elif abs(image_error) > image_tol:
            phase = "visual_center"
        elif distance > float(self.get_parameter("final_stop_distance_m").value):
            phase = "final_approach"
        else:
            phase = "ready"

        cmd = Twist()
        if phase == "turn_to_target":
            if rotation_blocked:
                self.publish_ready(False)
                self.stop_robot()
                self._publish_status("blocked")
                return
            cmd.angular.z = clamp(
                float(self.get_parameter("turn_kp").value) * heading_error,
                -float(self.get_parameter("max_angular_speed_radps").value),
                float(self.get_parameter("max_angular_speed_radps").value),
            )
            self.publish_ready(False)
            self._publish_status("turn_to_target")

        elif phase == "approach_target":
            if linear_blocked:
                self.publish_ready(False)
                self.stop_robot()
                self._publish_status("blocked")
                return
            cmd.angular.z = clamp(
                float(self.get_parameter("approach_angular_kp").value) * heading_error,
                -float(self.get_parameter("max_angular_speed_radps").value),
                float(self.get_parameter("max_angular_speed_radps").value),
            )
            if abs(heading_error) <= float(self.get_parameter("approach_heading_deadband_rad").value):
                cmd.linear.x = clamp(
                    float(self.get_parameter("approach_linear_kp").value)
                    * (distance - float(self.get_parameter("prealign_distance_m").value)),
                    0.0,
                    float(self.get_parameter("max_forward_speed_mps").value),
                )
            self.publish_ready(False)
            self._publish_status("approach_target")

        elif phase == "visual_center":
            if rotation_blocked or image_error is None:
                self.publish_ready(False)
                self.stop_robot()
                self._publish_status("waiting_for_visual")
                return
            cmd.angular.z = clamp(
                -float(self.get_parameter("visual_center_kp").value) * image_error,
                -float(self.get_parameter("max_angular_speed_radps").value),
                float(self.get_parameter("max_angular_speed_radps").value),
            )
            self.publish_ready(False)
            self._publish_status("visual_center")

        elif phase == "final_approach":
            if linear_blocked:
                self.publish_ready(False)
                self.stop_robot()
                self._publish_status("blocked")
                return
            correction = heading_error
            if image_error is not None:
                correction = -float(self.get_parameter("visual_center_kp").value) * image_error
            cmd.angular.z = clamp(
                correction if image_error is not None else float(self.get_parameter("approach_angular_kp").value) * correction,
                -float(self.get_parameter("max_angular_speed_radps").value),
                float(self.get_parameter("max_angular_speed_radps").value),
            )
            cmd.linear.x = clamp(
                distance - float(self.get_parameter("final_stop_distance_m").value),
                0.0,
                float(self.get_parameter("final_approach_speed_mps").value),
            )
            self.publish_ready(False)
            self._publish_status("final_approach")

        elif phase == "waiting_for_visual":
            self.publish_ready(False)
            self.stop_robot()
            self._publish_status("waiting_for_visual")
            return

        else:
            self.stop_robot()
            self.publish_ready(True)
            self._publish_status("ready")
            return

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BagApproachControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
