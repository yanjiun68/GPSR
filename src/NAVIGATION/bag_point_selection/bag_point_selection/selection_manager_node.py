#!/usr/bin/env python3
from enum import Enum
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from putrabot_interfaces.msg import CamDetection, CamDetections


class SelectionState(str, Enum):
    IDLE = "IDLE"
    WAIT_FOR_TWO_BAGS = "WAIT_FOR_TWO_BAGS"
    WAIT_FOR_SELECTION = "WAIT_FOR_SELECTION"
    SELECT_BAG = "SELECT_BAG"
    EXTRACT_CAMERA_CLUSTER = "EXTRACT_CAMERA_CLUSTER"
    MATCH_OR_FUSE_TARGET = "MATCH_OR_FUSE_TARGET"
    APPROACH_TARGET = "APPROACH_TARGET"
    FINAL_ALIGN = "FINAL_ALIGN"
    READY_FOR_PICKUP = "READY_FOR_PICKUP"


class SelectionManagerNode(Node):
    def __init__(self):
        super().__init__("selection_manager_node")
        self.declare_parameter("required_bag_count", 2)
        self.declare_parameter("pointing_confirm_sec", 0.7)
        self.declare_parameter("pointing_timeout_sec", 5.0)
        self.declare_parameter("bag_detection_timeout_sec", 3.0)
        self.declare_parameter("camera_cluster_timeout_sec", 3.0)
        self.declare_parameter("fusion_timeout_sec", 2.0)
        self.declare_parameter("approach_timeout_sec", 30.0)
        self.declare_parameter("final_alignment_timeout_sec", 10.0)
        self.declare_parameter("max_pointed_bag_age_sec", 0.75)
        self.declare_parameter("min_camera_cluster_confidence", 0.35)
        self.declare_parameter("min_fused_confidence", 0.35)

        self.pointing_raw = "none"
        self.session_active = False
        self.confirm_side: Optional[str] = None
        self.confirm_started = None
        self.current_state_started = self.get_clock().now()
        self.state = SelectionState.IDLE

        self.detected_bags = []
        self.selected_bag: Optional[CamDetection] = None
        self.pointed_bag: Optional[CamDetection] = None
        self.pointed_bag_received_at = None
        self.camera_centroid: Optional[PointStamped] = None
        self.camera_confidence = 0.0
        self.fused_center: Optional[PointStamped] = None
        self.fused_confidence = 0.0
        self.tracking_mode = "target_lost"
        self.motion_status = "idle"
        self.bag_ready = False

        self.side_pub = self.create_publisher(String, "/selected_side", 10)
        self.state_pub = self.create_publisher(String, "/selection_state", 10)
        self.selected_bag_pub = self.create_publisher(CamDetection, "/selected_bag_bbox", 10)
        self.failure_pub = self.create_publisher(String, "/bag_selection/failure_reason", 10)

        self.create_subscription(String, "/pointing_direction_raw", self.pointing_callback, 10)
        self.create_subscription(CamDetections, "/detected_bags", self.bags_callback, 10)
        self.create_subscription(CamDetection, "/pointed_bag_bbox", self.pointed_bag_callback, 10)
        self.create_subscription(PointStamped, "/selected_bag_centroid_base", self.camera_centroid_callback, 10)
        self.create_subscription(Float32, "/selected_bag_cluster_confidence", self.camera_confidence_callback, 10)
        self.create_subscription(PointStamped, "/selected_bag_fused_center", self.fused_center_callback, 10)
        self.create_subscription(Float32, "/selected_bag_fused_confidence", self.fused_confidence_callback, 10)
        self.create_subscription(String, "/target_tracking_mode", self.tracking_mode_callback, 10)
        self.create_subscription(String, "/bag_motion_status", self.motion_status_callback, 10)
        self.create_subscription(Bool, "/bag_ready_for_pickup", self.ready_callback, 10)
        self.create_subscription(Bool, "/bag_selection/start", self.start_callback, 10)
        self.create_subscription(Bool, "/bag_selection/reset", self.reset_callback, 10)

        self.timer = self.create_timer(0.1, self.step)
        self.publish_state()

    def now(self):
        return self.get_clock().now()

    def seconds_in_state(self) -> float:
        return (self.now() - self.current_state_started).nanoseconds / 1e9

    def transition(self, state: SelectionState) -> None:
        if self.state == state:
            return
        self.state = state
        self.current_state_started = self.now()
        self.publish_state()

    def publish_state(self) -> None:
        self.state_pub.publish(String(data=self.state.value))

    def clear_session_data(self) -> None:
        self.pointing_raw = "none"
        self.confirm_side = None
        self.confirm_started = None
        self.detected_bags = []
        self.selected_bag = None
        self.pointed_bag = None
        self.pointed_bag_received_at = None
        self.camera_centroid = None
        self.camera_confidence = 0.0
        self.fused_center = None
        self.fused_confidence = 0.0
        self.tracking_mode = "target_lost"
        self.motion_status = "idle"
        self.bag_ready = False

    def publish_failure(self, reason: str) -> None:
        if reason:
            self.failure_pub.publish(String(data=reason))

    def reset(self, reason: str = "") -> None:
        self.session_active = False
        self.clear_session_data()
        if reason:
            self.publish_failure(reason)
        self.transition(SelectionState.IDLE)

    def start_session(self) -> None:
        self.clear_session_data()
        self.session_active = True
        self.transition(SelectionState.WAIT_FOR_TWO_BAGS)

    def timeout_for_state(self) -> Optional[float]:
        mapping = {
            SelectionState.WAIT_FOR_TWO_BAGS: float(self.get_parameter("bag_detection_timeout_sec").value),
            SelectionState.WAIT_FOR_SELECTION: float(self.get_parameter("pointing_timeout_sec").value),
            SelectionState.EXTRACT_CAMERA_CLUSTER: float(self.get_parameter("camera_cluster_timeout_sec").value),
            SelectionState.MATCH_OR_FUSE_TARGET: float(self.get_parameter("fusion_timeout_sec").value),
            SelectionState.APPROACH_TARGET: float(self.get_parameter("approach_timeout_sec").value),
            SelectionState.FINAL_ALIGN: float(self.get_parameter("final_alignment_timeout_sec").value),
        }
        return mapping.get(self.state)

    def required_bag_count(self) -> int:
        return max(1, int(self.get_parameter("required_bag_count").value))

    def has_required_bag_count(self) -> bool:
        return len(self.detected_bags) == self.required_bag_count()

    def pointing_callback(self, msg: String) -> None:
        self.pointing_raw = msg.data.strip().lower()

    def bags_callback(self, msg: CamDetections) -> None:
        self.detected_bags = sorted(list(msg.detections), key=lambda det: det.center_u)
        if self.session_active and self.confirm_side and self.has_required_bag_count():
            selected = self.fresh_pointed_bag()
            if selected is None:
                selected = self.detected_bags[0] if self.confirm_side == "left" else self.detected_bags[-1]
            self.selected_bag = selected
            self.selected_bag_pub.publish(selected)

    def pointed_bag_callback(self, msg: CamDetection) -> None:
        self.pointed_bag = msg
        self.pointed_bag_received_at = self.now()

    def fresh_pointed_bag(self) -> Optional[CamDetection]:
        if self.pointed_bag is None or self.pointed_bag_received_at is None:
            return None
        age_sec = (self.now() - self.pointed_bag_received_at).nanoseconds / 1e9
        if age_sec > float(self.get_parameter("max_pointed_bag_age_sec").value):
            return None
        return self.pointed_bag

    def camera_centroid_callback(self, msg: PointStamped) -> None:
        self.camera_centroid = msg

    def camera_confidence_callback(self, msg: Float32) -> None:
        self.camera_confidence = float(msg.data)

    def fused_center_callback(self, msg: PointStamped) -> None:
        self.fused_center = msg

    def fused_confidence_callback(self, msg: Float32) -> None:
        self.fused_confidence = float(msg.data)

    def tracking_mode_callback(self, msg: String) -> None:
        self.tracking_mode = msg.data.strip()

    def motion_status_callback(self, msg: String) -> None:
        self.motion_status = msg.data.strip()

    def ready_callback(self, msg: Bool) -> None:
        self.bag_ready = bool(msg.data)

    def start_callback(self, msg: Bool) -> None:
        if bool(msg.data):
            self.start_session()

    def reset_callback(self, msg: Bool) -> None:
        if bool(msg.data):
            self.reset()

    def timeout_reason(self) -> str:
        mapping = {
            SelectionState.WAIT_FOR_TWO_BAGS: "bag detection timeout",
            SelectionState.WAIT_FOR_SELECTION: "pointing timeout",
            SelectionState.EXTRACT_CAMERA_CLUSTER: "camera cluster timeout",
            SelectionState.MATCH_OR_FUSE_TARGET: "target fusion timeout",
            SelectionState.APPROACH_TARGET: "approach timeout",
            SelectionState.FINAL_ALIGN: "final alignment timeout",
        }
        return mapping.get(self.state, f"{self.state.value} timeout")

    def _update_selection_confirmation(self) -> bool:
        if self.pointing_raw not in ("left", "right"):
            self.confirm_started = None
            return False
        if self.confirm_side != self.pointing_raw or self.confirm_started is None:
            self.confirm_side = self.pointing_raw
            self.confirm_started = self.now()
            return False
        confirm_sec = float(self.get_parameter("pointing_confirm_sec").value)
        confirmed = (self.now() - self.confirm_started).nanoseconds / 1e9 >= confirm_sec
        if confirmed:
            self.side_pub.publish(String(data=self.confirm_side))
        return confirmed

    def step(self) -> None:
        if not self.session_active or self.state == SelectionState.IDLE:
            return

        timeout = self.timeout_for_state()
        if timeout is not None and self.seconds_in_state() > timeout:
            reason = self.timeout_reason()
            self.get_logger().warn(f"{self.state.value} timed out. Resetting.")
            self.reset(reason)
            return

        if self.selected_bag is not None and self.state != SelectionState.IDLE:
            self.selected_bag_pub.publish(self.selected_bag)

        if self.state == SelectionState.WAIT_FOR_TWO_BAGS:
            if self.has_required_bag_count():
                self.transition(SelectionState.WAIT_FOR_SELECTION)

        elif self.state == SelectionState.WAIT_FOR_SELECTION:
            if not self.has_required_bag_count():
                self.transition(SelectionState.WAIT_FOR_TWO_BAGS)
                return
            if self._update_selection_confirmation():
                self.transition(SelectionState.SELECT_BAG)

        elif self.state == SelectionState.SELECT_BAG:
            if not self.has_required_bag_count():
                self.transition(SelectionState.WAIT_FOR_TWO_BAGS)
                return
            direct_bag = self.fresh_pointed_bag()
            if direct_bag is not None:
                self.selected_bag = direct_bag
            elif self.selected_bag is None and self.confirm_side:
                self.selected_bag = self.detected_bags[0] if self.confirm_side == "left" else self.detected_bags[-1]
            if self.selected_bag is None:
                return
            self.selected_bag_pub.publish(self.selected_bag)
            self.transition(SelectionState.EXTRACT_CAMERA_CLUSTER)

        elif self.state == SelectionState.EXTRACT_CAMERA_CLUSTER:
            if self.camera_centroid is not None and self.camera_confidence >= float(
                self.get_parameter("min_camera_cluster_confidence").value
            ):
                self.transition(SelectionState.MATCH_OR_FUSE_TARGET)

        elif self.state == SelectionState.MATCH_OR_FUSE_TARGET:
            if (
                self.fused_center is not None
                and self.tracking_mode != "target_lost"
                and self.fused_confidence >= float(self.get_parameter("min_fused_confidence").value)
            ):
                self.transition(SelectionState.APPROACH_TARGET)

        elif self.state == SelectionState.APPROACH_TARGET:
            if self.bag_ready:
                self.transition(SelectionState.READY_FOR_PICKUP)
            elif self.motion_status in ("visual_center", "final_approach", "ready"):
                self.transition(SelectionState.FINAL_ALIGN)

        elif self.state == SelectionState.FINAL_ALIGN:
            if self.bag_ready:
                self.transition(SelectionState.READY_FOR_PICKUP)
            elif self.motion_status in ("turn_to_target", "approach_target"):
                self.transition(SelectionState.APPROACH_TARGET)

        elif self.state == SelectionState.READY_FOR_PICKUP:
            if not self.bag_ready:
                self.reset()


def main(args=None):
    rclpy.init(args=args)
    node = SelectionManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
