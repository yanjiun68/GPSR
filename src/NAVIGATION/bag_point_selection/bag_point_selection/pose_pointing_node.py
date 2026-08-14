#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String

from putrabot_interfaces.msg import CamDetection, CamDetections

try:
    import mediapipe as mp
except ImportError:  # pragma: no cover - runtime dependency
    mp = None


Vec3 = Tuple[float, float, float]


def compute_angle_3d(a: Vec3, b: Vec3, c: Vec3) -> float:
    ab = (a[0] - b[0], a[1] - b[1], a[2] - b[2])
    cb = (c[0] - b[0], c[1] - b[1], c[2] - b[2])
    dot = ab[0] * cb[0] + ab[1] * cb[1] + ab[2] * cb[2]
    norm_ab = math.sqrt(ab[0] ** 2 + ab[1] ** 2 + ab[2] ** 2)
    norm_cb = math.sqrt(cb[0] ** 2 + cb[1] ** 2 + cb[2] ** 2)
    if norm_ab < 1e-6 or norm_cb < 1e-6:
        return 0.0
    value = max(-1.0, min(1.0, dot / (norm_ab * norm_cb)))
    return math.degrees(math.acos(value))


def point_to_ray_distance_2d(origin, direction_unit, point) -> tuple[float, float]:
    rel = (point[0] - origin[0], point[1] - origin[1])
    forward = rel[0] * direction_unit[0] + rel[1] * direction_unit[1]
    perp_x = rel[0] - forward * direction_unit[0]
    perp_y = rel[1] - forward * direction_unit[1]
    return math.hypot(perp_x, perp_y), forward


class PosePointingNode(Node):
    INDEX_MCP = 5
    INDEX_PIP = 6
    INDEX_DIP = 7
    INDEX_TIP = 8
    WRIST = 0

    def __init__(self):
        super().__init__("pose_pointing_node")
        self.declare_parameter("rgb_topic", "/bag_selection/image_enhanced")
        self.declare_parameter("pose_debug_enabled", False)
        self.declare_parameter("pose_debug_publish_image", False)
        self.declare_parameter("pose_debug_show_window", False)
        self.declare_parameter("pose_debug_window_name", "MediaPipe Pose Debug")
        self.declare_parameter("pose_debug_topic", "/bag_selection/pose_debug_image")
        self.declare_parameter("hand_max_num_hands", 1)
        self.declare_parameter("hand_min_detection_confidence", 0.7)
        self.declare_parameter("hand_min_tracking_confidence", 0.7)
        self.declare_parameter("index_straight_angle_min_deg", 160.0)
        self.declare_parameter("pointing_ray_forward_dot_min", 0.3)
        self.declare_parameter("pointing_ray_max_anchor_distance_px", 90.0)
        self.declare_parameter("pointing_target_confirm_frames", 8)
        self.declare_parameter("pointing_target_release_frames", 5)
        self.declare_parameter("pointing_min_image_plane_ratio", 0.35)
        self.declare_parameter("bag_detection_max_age_sec", 0.3)
        self.declare_parameter("camera_watchdog_sec", 2.0)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(String, "/pointing_direction_raw", 10)
        self.pointed_bag_pub = self.create_publisher(CamDetection, "/pointed_bag_bbox", 10)
        self.debug_enabled = bool(self.get_parameter("pose_debug_enabled").value)
        self.publish_debug_image = bool(self.get_parameter("pose_debug_publish_image").value)
        self.show_window = bool(self.get_parameter("pose_debug_show_window").value)
        self.window_name = str(self.get_parameter("pose_debug_window_name").value)
        self.debug_publisher = None
        if self.debug_enabled and self.publish_debug_image:
            self.debug_publisher = self.create_publisher(
                Image, str(self.get_parameter("pose_debug_topic").value), 10
            )

        self.subscription = self.create_subscription(
            Image,
            str(self.get_parameter("rgb_topic").value),
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(CamDetections, "/detected_bags", self.detections_callback, 10)

        self.latest_detections: list[CamDetection] = []
        self.latest_detections_stamp: Optional[Time] = None
        self.pending_target: Optional[CamDetection] = None
        self.pending_count = 0
        self.active_target: Optional[CamDetection] = None
        self.release_count = 0

        self.last_image_time = self.get_clock().now()
        watchdog_sec = float(self.get_parameter("camera_watchdog_sec").value)
        if watchdog_sec > 0.0:
            self.watchdog_sec = watchdog_sec
            self.create_timer(watchdog_sec, self._camera_watchdog)

        self.hands = None
        self.hand_connections = None
        self.drawing_utils = None
        if mp is None:
            self.get_logger().error("mediapipe is not installed. TODO: install mediapipe in this environment.")
        else:
            self.hands = mp.solutions.hands.Hands(
                static_image_mode=False,
                max_num_hands=int(self.get_parameter("hand_max_num_hands").value),
                min_detection_confidence=float(self.get_parameter("hand_min_detection_confidence").value),
                min_tracking_confidence=float(self.get_parameter("hand_min_tracking_confidence").value),
            )
            self.hand_connections = mp.solutions.hands.HAND_CONNECTIONS
            self.drawing_utils = mp.solutions.drawing_utils
            self.get_logger().info("MediaPipe Hands initialized on CPU.")
            if self.debug_enabled and self.debug_publisher is not None:
                self.get_logger().info(
                    f"Publishing pose debug images on {self.get_parameter('pose_debug_topic').value}"
                )
            if self.debug_enabled and self.show_window:
                self.get_logger().info(f"OpenCV pose window enabled: {self.window_name}")

    def detections_callback(self, msg: CamDetections) -> None:
        self.latest_detections = list(msg.detections)
        self.latest_detections_stamp = self.get_clock().now()

    def _detections_fresh(self) -> bool:
        if not self.latest_detections or self.latest_detections_stamp is None:
            return False
        max_age = float(self.get_parameter("bag_detection_max_age_sec").value)
        if max_age <= 0.0:
            return True
        age_sec = (self.get_clock().now() - self.latest_detections_stamp).nanoseconds / 1e9
        return age_sec <= max_age

    def _camera_watchdog(self) -> None:
        age_sec = (self.get_clock().now() - self.last_image_time).nanoseconds / 1e9
        if age_sec > self.watchdog_sec:
            self.get_logger().error(
                f"No frames on {self.get_parameter('rgb_topic').value} for {age_sec:.1f}s.",
                throttle_duration_sec=5.0,
            )

    def _landmark_xyz(self, hand_landmarks, idx) -> Vec3:
        lm = hand_landmarks.landmark[idx]
        return (float(lm.x), float(lm.y), float(lm.z))

    def _choose_hand(self, results) -> Optional[tuple]:
        hands = list(results.multi_hand_landmarks or [])
        if not hands:
            return None
        handedness = list(results.multi_handedness or [])
        best = None
        best_score = None
        for idx, hand_landmarks in enumerate(hands):
            wrist = self._landmark_xyz(hand_landmarks, self.WRIST)
            score = None
            if idx < len(handedness) and handedness[idx].classification:
                score = float(handedness[idx].classification[0].score)
            metric = score if score is not None else 1.0 - abs(wrist[0] - 0.5)
            if best is None or metric > best_score:
                best = (hand_landmarks, idx)
                best_score = metric
        return best

    def _is_index_extended(self, mcp: Vec3, pip: Vec3, dip: Vec3, tip: Vec3) -> bool:
        angle_pip = compute_angle_3d(mcp, pip, dip)
        angle_dip = compute_angle_3d(pip, dip, tip)
        threshold = float(self.get_parameter("index_straight_angle_min_deg").value)
        return angle_pip >= threshold and angle_dip >= threshold

    def _bag_anchor(self, det: CamDetection) -> tuple[float, float]:
        cx = 0.5 * (det.x1 + det.x2)
        height = max(1.0, float(det.y2 - det.y1))
        cy = float(det.y2) - 0.15 * height
        return cx, cy

    def _bag_matches(self, a: Optional[CamDetection], b: Optional[CamDetection]) -> bool:
        if a is None or b is None:
            return False
        ax, ay = self._bag_anchor(a)
        bx, by = self._bag_anchor(b)
        return math.hypot(ax - bx, ay - by) <= 35.0

    def _select_pointed_bag(self, direction_origin, direction_unit, image_width) -> Optional[CamDetection]:
        if not self._detections_fresh():
            return None

        best_det = None
        best_dist = None
        max_dist = float(self.get_parameter("pointing_ray_max_anchor_distance_px").value)
        min_forward_dot = float(self.get_parameter("pointing_ray_forward_dot_min").value)

        for det in self.latest_detections:
            anchor = self._bag_anchor(det)
            rel = (anchor[0] - direction_origin[0], anchor[1] - direction_origin[1])
            rel_norm = math.hypot(*rel)
            if rel_norm < 1e-6:
                continue
            rel_unit = (rel[0] / rel_norm, rel[1] / rel_norm)
            forward_dot = rel_unit[0] * direction_unit[0] + rel_unit[1] * direction_unit[1]
            if forward_dot < min_forward_dot:
                continue
            perp_dist, forward = point_to_ray_distance_2d(direction_origin, direction_unit, anchor)
            if forward <= 0.0 or perp_dist > max_dist:
                continue
            if best_det is None or perp_dist < best_dist:
                best_det = det
                best_dist = perp_dist
        return best_det

    def _update_target_state(self, candidate: Optional[CamDetection]) -> Optional[CamDetection]:
        confirm_frames = max(1, int(self.get_parameter("pointing_target_confirm_frames").value))
        release_frames = max(1, int(self.get_parameter("pointing_target_release_frames").value))

        if candidate is not None:
            if self._bag_matches(candidate, self.pending_target):
                self.pending_count += 1
            else:
                self.pending_target = candidate
                self.pending_count = 1
            self.release_count = 0
        else:
            self.pending_target = None
            self.pending_count = 0
            if self.active_target is not None:
                self.release_count += 1
                if self.release_count >= release_frames:
                    self.active_target = None
            return self.active_target

        if self.active_target is not None and self._bag_matches(candidate, self.active_target):
            self.active_target = candidate
            return self.active_target

        if self.pending_target is not None and self.pending_count >= confirm_frames:
            self.active_target = self.pending_target
            self.release_count = 0

        return self.active_target

    def _side_of_pointed_bag(self, pointed: CamDetection, image_width: int) -> str:
        if self._detections_fresh() and len(self.latest_detections) >= 2:
            sorted_bags = sorted(self.latest_detections, key=lambda d: d.center_u)
            if self._bag_matches(pointed, sorted_bags[0]):
                return "left"
            if self._bag_matches(pointed, sorted_bags[-1]):
                return "right"
        return "left" if pointed.center_u < 0.5 * max(1.0, float(image_width)) else "right"

    def _render_debug(
        self,
        frame,
        header,
        results,
        result_label,
        hand_landmarks,
        ray_start_px: Optional[tuple],
        ray_end_px: Optional[tuple],
        pointed_bag: Optional[CamDetection],
    ) -> None:
        if not self.debug_enabled:
            return

        debug_frame = frame.copy()
        if hand_landmarks is not None and self.drawing_utils is not None and self.hand_connections is not None:
            self.drawing_utils.draw_landmarks(
                debug_frame,
                hand_landmarks,
                self.hand_connections,
            )
        if ray_start_px is not None and ray_end_px is not None:
            cv2.arrowedLine(debug_frame, ray_start_px, ray_end_px, (255, 255, 0), 3, tipLength=0.2)
        if pointed_bag is not None:
            cv2.rectangle(
                debug_frame,
                (int(pointed_bag.x1), int(pointed_bag.y1)),
                (int(pointed_bag.x2), int(pointed_bag.y2)),
                (255, 128, 0),
                2,
            )
            cv2.putText(
                debug_frame,
                "pointed bag",
                (int(pointed_bag.x1), max(20, int(pointed_bag.y1) - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 128, 0),
                2,
                cv2.LINE_AA,
            )
        cv2.putText(
            debug_frame,
            f"Pointing: {result_label}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        if self.debug_publisher is not None:
            annotated = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
            annotated.header = header
            self.debug_publisher.publish(annotated)

        if self.show_window:
            cv2.imshow(self.window_name, debug_frame)
            cv2.waitKey(1)

    def image_callback(self, msg: Image) -> None:
        self.last_image_time = self.get_clock().now()
        result_label = "none"
        pointed_bag = None
        ray_start_px = None
        ray_end_px = None
        hand_landmarks = None

        if self.hands is None:
            self.publisher.publish(String(data=result_label))
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"cv_bridge failed: {exc}", throttle_duration_sec=2.0
            )
            self.publisher.publish(String(data=result_label))
            return

        try:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(rgb)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"MediaPipe Hands failed: {exc}", throttle_duration_sec=2.0
            )
            self.publisher.publish(String(data=result_label))
            return

        chosen = self._choose_hand(results)
        if chosen is not None:
            hand_landmarks, _ = chosen
            index_mcp = self._landmark_xyz(hand_landmarks, self.INDEX_MCP)
            index_pip = self._landmark_xyz(hand_landmarks, self.INDEX_PIP)
            index_dip = self._landmark_xyz(hand_landmarks, self.INDEX_DIP)
            index_tip = self._landmark_xyz(hand_landmarks, self.INDEX_TIP)

            if self._is_index_extended(index_mcp, index_pip, index_dip, index_tip):
                origin_3d = index_mcp
                direction_3d = (
                    index_tip[0] - index_mcp[0],
                    index_tip[1] - index_mcp[1],
                    index_tip[2] - index_mcp[2],
                )
                if math.sqrt(sum(c ** 2 for c in direction_3d)) < 1e-6:
                    origin_3d = index_pip
                    direction_3d = (
                        index_tip[0] - index_pip[0],
                        index_tip[1] - index_pip[1],
                        index_tip[2] - index_pip[2],
                    )

                length_3d = math.sqrt(sum(c ** 2 for c in direction_3d))
                length_2d = math.hypot(direction_3d[0], direction_3d[1])
                min_ratio = float(self.get_parameter("pointing_min_image_plane_ratio").value)

                if length_3d >= 1e-6 and length_2d / length_3d >= min_ratio:
                    direction_unit_2d = (direction_3d[0] / length_2d, direction_3d[1] / length_2d)
                    height, width = frame.shape[:2]
                    origin_px = (origin_3d[0] * width, origin_3d[1] * height)
                    candidate = self._select_pointed_bag(origin_px, direction_unit_2d, width)
                    pointed_bag = self._update_target_state(candidate)
                    if pointed_bag is not None:
                        result_label = self._side_of_pointed_bag(pointed_bag, width)
                        self.pointed_bag_pub.publish(pointed_bag)
                    ray_start_px = (int(origin_px[0]), int(origin_px[1]))
                    ray_end_px = (
                        int(origin_px[0] + direction_unit_2d[0] * width * 0.35),
                        int(origin_px[1] + direction_unit_2d[1] * width * 0.35),
                    )
                else:
                    self._update_target_state(None)
            else:
                self._update_target_state(None)
        else:
            self._update_target_state(None)

        self._render_debug(frame, msg.header, results, result_label, hand_landmarks, ray_start_px, ray_end_px, pointed_bag)
        self.publisher.publish(String(data=result_label))
        self.get_logger().debug(f"Pointing result: {result_label}")

    def destroy_node(self):
        if self.debug_enabled and self.show_window:
            cv2.destroyWindow(self.window_name)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PosePointingNode()
    try:
        rclpy.spin(node)
    finally:
        if getattr(node, "hands", None) is not None:
            node.hands.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
