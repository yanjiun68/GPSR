#!/usr/bin/env python3
"""Section: Name-only find-object perception action server."""

from __future__ import annotations

import threading
import time
from typing import Any
import json

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from putrabot_interfaces.msg import ObjectDetection, ObjectDetections
from robot_ai_interfaces.action import FindObject
from voice_interfaces.srv import SayText


def normalize_name(value: Any) -> str:
    """Special function: Normalize target and detected object names for exact comparison."""
    return str(value or "").strip().lower().replace(" ", "_").replace("-", "_")


class FindObjectPerceptionServer(Node):
    """Special function: Compare detected object names with a requested target name."""

    def __init__(self) -> None:
        """Special function: Initialize detection subscription, TTS client, and FindObject action."""
        super().__init__("find_object_perception_server")

        # Section: Declare ROS interfaces and comparison parameters.
        self.declare_parameter("detections_topic", "/perception/raw_object_detections")
        self.declare_parameter("action_name", "/perception/find_object")
        self.declare_parameter("tts_service_name", "/voice/tts/say")
        self.declare_parameter("min_confidence", 0.35)
        self.declare_parameter("default_timeout_sec", 6.0)
        self.declare_parameter("speak_on_found", True)
        self.declare_parameter("target_object_name", "")
        self.declare_parameter("item_found_topic", "/perception/item_found")
        self.declare_parameter("item_found_cooldown_sec", 3.0)

        # Section: Keep the latest structured detection frame for action requests.
        self.latest_detections: list[ObjectDetection] = []
        self.detection_lock = threading.Lock()
        self.detection_event = threading.Event()
        self.callback_group = ReentrantCallbackGroup()
        self.last_item_found_time_sec = 0.0
        self.last_item_found_object_id = ""
        self.last_item_found_payload = ""

        # Section: Subscribe only to structured detections, never raw image inference.
        self.create_subscription(
            ObjectDetections,
            str(self.get_parameter("detections_topic").value),
            self.detections_callback,
            10,
            callback_group=self.callback_group,
        )

        # Section: Publish a simple status message when the preset target is recognized.
        self.item_found_pub = self.create_publisher(
            String,
            str(self.get_parameter("item_found_topic").value),
            10,
        )

        # Section: Use the existing TTS service to announce found targets.
        self.tts_client = self.create_client(
            SayText,
            str(self.get_parameter("tts_service_name").value),
            callback_group=self.callback_group,
        )

        # Section: Provide a simple FindObject action endpoint for perception-only lookup.
        self.action_server = ActionServer(
            self,
            FindObject,
            str(self.get_parameter("action_name").value),
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        # Section: Log startup wiring for launch diagnosis.
        self.get_logger().info(
            "Find object perception server ready: "
            f"action={self.get_parameter('action_name').value}, "
            f"detections={self.get_parameter('detections_topic').value}, "
            f"tts={self.get_parameter('tts_service_name').value}, "
            f"preset_target={self.get_parameter('target_object_name').value}, "
            f"item_found_topic={self.get_parameter('item_found_topic').value}"
        )

    def detections_callback(self, msg: ObjectDetections) -> None:
        """Special function: Cache detections and check the optional preset target."""
        with self.detection_lock:
            self.latest_detections = list(msg.detections)
        self.detection_event.set()
        self.handle_preset_target_detection()

    def goal_callback(self, goal_request: FindObject.Goal) -> GoalResponse:
        """Special function: Accept only goals with a target object name or category."""
        target_name = self.resolve_target_name(goal_request)
        if not target_name:
            self.get_logger().warn("Rejected FindObject goal because object_name/category is empty.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        """Special function: Allow cancellation of pending target-name searches."""
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle) -> FindObject.Result:
        """Special function: Wait for a matching detection name and announce success through TTS."""
        goal = goal_handle.request
        target_name = self.resolve_target_name(goal)
        target_display_name = target_name.replace("_", " ")
        timeout_sec = float(goal.timeout_sec) or float(self.get_parameter("default_timeout_sec").value)
        min_confidence = float(self.get_parameter("min_confidence").value)
        deadline = time.monotonic() + max(0.1, timeout_sec)

        # Section: Publish action feedback while waiting for matching detections.
        feedback = FindObject.Feedback()
        feedback.status = f"Searching for {target_display_name} by detection name."
        goal_handle.publish_feedback(feedback)

        # Section: Poll cached structured detections until target is found, canceled, or timed out.
        matched_detection = self.find_matching_detection(target_name, min_confidence)
        while matched_detection is None and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                result = self.build_result(False, target_name, None, "Find object goal canceled.")
                goal_handle.canceled()
                return result
            self.detection_event.wait(timeout=0.1)
            self.detection_event.clear()
            matched_detection = self.find_matching_detection(target_name, min_confidence)

        # Section: Return not-found if no detection name matched before timeout.
        if matched_detection is None:
            result = self.build_result(False, target_name, None, f"{target_display_name} was not found.")
            goal_handle.abort()
            return result

        # Section: Announce the found object and return action success.
        message = f"{target_display_name} is found."
        self.speak_text(f"The {target_display_name} is found.", "found-object announcement")
        result = self.build_result(True, target_name, matched_detection, message)
        goal_handle.succeed()
        return result

    def get_preset_target_name(self) -> str:
        """Special function: Return the normalized preset target object name."""
        return normalize_name(self.get_parameter("target_object_name").value)

    def resolve_target_name(self, goal: FindObject.Goal) -> str:
        """Special function: Choose object_name first, then category as a fallback target."""
        object_name = normalize_name(goal.object_name)
        if object_name:
            return object_name
        return normalize_name(goal.category)

    def find_matching_detection(self, target_name: str, min_confidence: float) -> ObjectDetection | None:
        """Special function: Return the highest-confidence detection whose name exactly matches the target."""
        with self.detection_lock:
            detections = list(self.latest_detections)
        matches = []
        for detection in detections:
            if normalize_name(detection.name) != target_name:
                continue
            if float(detection.confidence) < min_confidence:
                continue
            matches.append(detection)
        matches.sort(key=lambda detection: float(detection.confidence), reverse=True)
        return matches[0] if matches else None

    def handle_preset_target_detection(self) -> None:
        """Special function: Publish item-found status for the configured preset target."""
        target_name = self.get_preset_target_name()
        if not target_name:
            return

        min_confidence = float(self.get_parameter("min_confidence").value)
        best_detection = self.find_best_detection(min_confidence)
        matched_detection = self.find_matching_detection(target_name, min_confidence)
        if matched_detection is None:
            self.last_item_found_object_id = ""
            self.publish_item_found_status(best_detection, target_name, False)
            return

        object_id = self.build_object_id(target_name, matched_detection)
        now_sec = time.monotonic()
        cooldown_sec = max(0.0, float(self.get_parameter("item_found_cooldown_sec").value))
        if object_id == self.last_item_found_object_id and (now_sec - self.last_item_found_time_sec) < cooldown_sec:
            return

        self.publish_item_found_status(matched_detection, target_name, True)
        self.last_item_found_time_sec = now_sec
        self.last_item_found_object_id = object_id

    def find_best_detection(self, min_confidence: float) -> ObjectDetection | None:
        """Special function: Return the highest-confidence visible detection above threshold."""
        with self.detection_lock:
            detections = list(self.latest_detections)
        matches = [detection for detection in detections if float(detection.confidence) >= min_confidence]
        matches.sort(key=lambda detection: float(detection.confidence), reverse=True)
        return matches[0] if matches else None

    def build_item_found_payload(self, detection: ObjectDetection | None, target_name: str, itemfound: bool) -> str:
        """Special function: Build structured item-found topic JSON with detected item and truth value."""
        detected_item = normalize_name(detection.name) if detection is not None else ""
        payload = {
            "detected_item": detected_item,
            "itemfound": bool(itemfound),
            "target_object_name": str(target_name),
        }
        return json.dumps(payload, sort_keys=True)

    def publish_item_found_status(self, detection: ObjectDetection | None, target_name: str, itemfound: bool) -> None:
        """Special function: Publish one structured item-found status message."""
        payload = self.build_item_found_payload(detection, target_name, itemfound)
        if not itemfound and payload == self.last_item_found_payload:
            return
        msg = String()
        msg.data = payload
        self.item_found_pub.publish(msg)
        self.last_item_found_payload = payload

    def speak_text(self, text: str, context: str) -> None:
        """Special function: Send one non-blocking text request to the TTS service."""
        if not bool(self.get_parameter("speak_on_found").value):
            return
        if not self.tts_client.service_is_ready():
            self.get_logger().warn(f"TTS service is not ready; {context} skipped.")
            return
        request = SayText.Request()
        request.text = str(text)
        self.tts_client.call_async(request)

    def build_result(self, found: bool, target_name: str, detection: ObjectDetection | None, message: str) -> FindObject.Result:
        """Special function: Build the FindObject action result from a matched detection."""
        result = FindObject.Result()
        result.found = bool(found)
        result.object_pose = PoseStamped()
        result.object_id = self.build_object_id(target_name, detection) if detection is not None else ""
        result.message = str(message)
        return result

    def build_object_id(self, target_name: str, detection: ObjectDetection | None) -> str:
        """Special function: Build a stable object id from target name and image center."""
        if detection is None:
            return ""
        return f"{target_name}_{int(detection.center_u)}_{int(detection.center_v)}"


def main(args: list[str] | None = None) -> None:
    """Special function: Run the find-object perception action server."""
    rclpy.init(args=args)
    node = FindObjectPerceptionServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
