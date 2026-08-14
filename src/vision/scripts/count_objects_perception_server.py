#!/usr/bin/env python3
"""Section: GPSR count-object perception service server."""

from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import dataclass
from typing import Any

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from std_msgs.msg import String

from putrabot_interfaces.msg import ObjectDetections
from robot_ai_interfaces.srv import CountObjects


@dataclass(slots=True)
class SceneObject:
    """Special function: Stable object record derived from detector output."""

    # Section: Stable object key used to avoid duplicate counts in one scene window.
    object_id: str

    # Section: Detector object label such as cup, bottle, or apple.
    name: str

    # Section: Detection confidence from the source detector.
    confidence: float

    # Section: Pixel-space bounding box used for simple duplicate suppression.
    bbox: tuple[int, int, int, int]


class CountObjectsPerceptionServer(LifecycleNode):
    """Special function: Count stable detected objects matching a GPSR entity/category."""

    def __init__(self) -> None:
        """Special function: Initialize subscriptions, service, category map, and scene cache."""
        super().__init__("count_objects_perception_server")

        # Section: Declare ROS interface and matching parameters.
        self.declare_parameter("service_name", "/perception/count_objects")
        self.declare_parameter("detections_topic", "/perception/raw_object_detections")
        self.declare_parameter("object_quantities_topic", "/perception/object_quantities")
        self.declare_parameter("raw_object_quantities_topic", "/perception/object_quantities_raw")
        self.declare_parameter("min_confidence", 0.50)
        self.declare_parameter("stale_timeout_sec", 2.0)
        self.declare_parameter("enforce_location_filter", False)
        self.declare_parameter("confirmation_window_sec", 2.0)
        self.declare_parameter("confirmation_required_ratio", 0.10)
        self.declare_parameter("confirmation_confidence_threshold", 0.35)
        self.declare_parameter("confirmation_last_seen_timeout_sec", 1.0)
        self.declare_parameter("confirmation_min_frames", 3)

        # Section: Keep the latest raw detection set and a short temporal confirmation history.
        self.scene_objects: dict[str, SceneObject] = {}
        self.last_scene_time_sec = 0.0
        self.frame_history: deque[tuple[float, dict[str, SceneObject]]] = deque()

        # Section: Map GPSR category words to detector labels.
        self.category_map: dict[str, set[str]] = {
            "drink": {"cup", "bottle", "can"},
            "drinks": {"cup", "bottle", "can"},
            "fruit": {"apple", "banana", "orange"},
            "fruits": {"apple", "banana", "orange"},
            "snack": {"chips", "cracker", "cookie"},
            "snacks": {"chips", "cracker", "cookie"},
            "object": set(),
            "objects": set(),
            "thing": set(),
            "things": set(),
        }

        # Section: ROS resources remain empty until CONFIGURE.
        self.detections_subscription = None
        self.confirmed_quantities_pub = None
        self.raw_quantities_pub = None
        self.count_service = None
        self.skill_active = False
        self.get_logger().info("CountObjects perception lifecycle node is UNCONFIGURED.")
    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Allocate detection, publisher, and service interfaces."""
        try:
            # Section: Create resources while callbacks remain explicitly INACTIVE.
            self.detections_subscription = self.create_subscription(
                ObjectDetections,
                str(self.get_parameter("detections_topic").value),
                self.detections_callback,
                10,
            )
            self.confirmed_quantities_pub = self.create_lifecycle_publisher(
                String,
                str(self.get_parameter("object_quantities_topic").value),
                10,
            )
            self.raw_quantities_pub = self.create_lifecycle_publisher(
                String,
                str(self.get_parameter("raw_object_quantities_topic").value),
                10,
            )
            self.count_service = self.create_service(
                CountObjects,
                str(self.get_parameter("service_name").value),
                self.handle_count_objects,
            )
            self.reset_cache()
            self.get_logger().info("[GPSR_LIFECYCLE] CountObjects perception: configured")
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.get_logger().error(f"CountObjects perception configure failed: {exc}")
            self.release_resources()
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Special function: Activate publishers and allow detection/service callbacks."""
        result = super().on_activate(state)
        if result == TransitionCallbackReturn.SUCCESS:
            self.skill_active = True
        return result

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Special function: Stop consuming detections and serving count requests."""
        self.skill_active = False
        return super().on_deactivate(state)

    def on_cleanup(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Destroy configured interfaces and clear cached detections."""
        self.skill_active = False
        self.release_resources()
        self.reset_cache()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Release resources during lifecycle shutdown."""
        self.skill_active = False
        self.release_resources()
        self.reset_cache()
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Fail closed and return to a clean cache baseline."""
        self.skill_active = False
        self.release_resources()
        self.reset_cache()
        return TransitionCallbackReturn.SUCCESS

    def release_resources(self) -> None:
        """Special function: Destroy resources allocated during CONFIGURE."""
        if self.detections_subscription is not None:
            self.destroy_subscription(self.detections_subscription)
            self.detections_subscription = None
        if self.confirmed_quantities_pub is not None:
            self.destroy_publisher(self.confirmed_quantities_pub)
            self.confirmed_quantities_pub = None
        if self.raw_quantities_pub is not None:
            self.destroy_publisher(self.raw_quantities_pub)
            self.raw_quantities_pub = None
        if self.count_service is not None:
            self.destroy_service(self.count_service)
            self.count_service = None

    def reset_cache(self) -> None:
        """Special function: Reset perception state to a fresh UNCONFIGURED baseline."""
        self.scene_objects.clear()
        self.last_scene_time_sec = 0.0
        self.frame_history.clear()

    def detections_callback(self, msg: ObjectDetections) -> None:
        """Special function: Cache the latest object detections as countable scene objects."""
        if not self.skill_active:
            return
        now = self.now_sec()
        self.last_scene_time_sec = now
        next_scene: dict[str, SceneObject] = {}
        next_confirmation_frame: dict[str, SceneObject] = {}
        confirmation_confidence = float(self.get_parameter("confirmation_confidence_threshold").value)

        # Section: Convert every detector message into a normalized SceneObject.
        for index, det in enumerate(msg.detections):
            name = str(det.name).strip().lower()
            confidence = float(det.confidence)
            bbox = (int(det.x1), int(det.y1), int(det.x2), int(det.y2))
            if not name:
                continue

            # Section: Build a duplicate-resistant key from label and coarse bbox center.
            object_id = self.make_object_id(name, bbox, index)
            obj = SceneObject(object_id, name, confidence, bbox)
            next_scene[object_id] = obj
            if confidence >= confirmation_confidence:
                next_confirmation_frame[object_id] = obj

        # Section: Replace the cache atomically so service calls see one coherent scene.
        self.scene_objects = next_scene

        # Section: Update temporal history used for confirmed object presence.
        self.frame_history.append((now, next_confirmation_frame))
        self.prune_frame_history(now)

        # Section: Publish a compact summary topic after every detection update.
        self.publish_object_quantities()

    def handle_count_objects(self, request: CountObjects.Request, response: CountObjects.Response) -> CountObjects.Response:
        """Special function: Return the number of recognized objects matching the requested entity."""
        if not self.skill_active:
            response.success = False
            response.count = 0
            response.result_json = "{}"
            response.message = "CountObjects perception is not ACTIVE."
            return response
        filters = self.parse_filter_json(request.filter_json)
        target_names = self.resolve_target_names(request.category)
        min_confidence = float(filters.get("min_confidence", self.get_parameter("min_confidence").value))
        stale_timeout_sec = float(filters.get("stale_timeout_sec", self.get_parameter("stale_timeout_sec").value))
        scene_age_sec = self.now_sec() - self.last_scene_time_sec if self.last_scene_time_sec else float("inf")

        # Section: Reject stale perception so GPSR does not answer from old camera data.
        if scene_age_sec > stale_timeout_sec:
            response.success = False
            response.count = 0
            response.result_json = self.build_result_json(request, [], [], [], scene_age_sec, location_verified=False)
            response.message = f"No fresh object detections received within {stale_timeout_sec:.2f}s."
            return response

        # Section: Keep every confidence-filtered raw object for troubleshooting metadata.
        raw_visible_objects = []
        for obj in self.scene_objects.values():
            if obj.confidence < min_confidence:
                continue
            raw_visible_objects.append(obj)

        # Section: Use only temporally confirmed objects for GPSR count decisions.
        confirmed_objects = self.get_confirmed_objects(self.now_sec())

        # Section: Apply target entity filter to compute the requested count.
        matched = []
        for obj in confirmed_objects:
            if target_names and obj.name not in target_names:
                continue
            matched.append(obj)

        # Section: Apply location policy; ObjectDetections currently has no location field.
        requested_location = self.clean_text(request.location)
        enforce_location = bool(filters.get("enforce_location_filter", self.get_parameter("enforce_location_filter").value))
        location_verified = not requested_location
        if requested_location and enforce_location:
            matched = []

        # Section: Fill response with count and structured result details.
        response.success = True
        response.count = len(matched)
        response.result_json = self.build_result_json(request, matched, confirmed_objects, raw_visible_objects, scene_age_sec, location_verified)
        response.message = self.build_message(request.category, requested_location, len(matched), location_verified)
        return response

    def now_sec(self) -> float:
        """Special function: Return ROS time in seconds for stale-scene checks."""
        return self.get_clock().now().nanoseconds / 1e9

    def clean_text(self, value: Any) -> str:
        """Special function: Normalize empty ROS/CLI text values into an empty string."""
        text = str(value).strip()
        return "" if text.lower() in {"", "none", "null"} else text

    def make_object_id(self, name: str, bbox: tuple[int, int, int, int], index: int) -> str:
        """Special function: Build a stable-enough object key from label and coarse image position."""
        x1, y1, x2, y2 = bbox
        center_x_bucket = int(((x1 + x2) / 2.0) // 20)
        center_y_bucket = int(((y1 + y2) / 2.0) // 20)
        return f"{name}_{center_x_bucket}_{center_y_bucket}"

    def resolve_target_names(self, category: str) -> set[str]:
        """Special function: Convert GPSR category/entity text into detector class names."""
        target = str(category).strip().lower()
        if not target:
            return set()
        if target in self.category_map:
            return set(self.category_map[target])
        return {target}

    def parse_filter_json(self, raw_filter_json: str) -> dict[str, Any]:
        """Special function: Parse optional GPSR filter JSON without throwing in service callbacks."""
        text = str(raw_filter_json).strip()
        if not text:
            return {}
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid filter_json ignored: {text}")
            return {}
        return parsed if isinstance(parsed, dict) else {}

    def build_result_json(
        self,
        request: CountObjects.Request,
        matched: list[SceneObject],
        confirmed_objects: list[SceneObject],
        raw_visible_objects: list[SceneObject],
        scene_age_sec: float,
        location_verified: bool,
    ) -> str:
        """Special function: Serialize count details for action results, logs, and blackboard storage."""
        quantities = self.build_object_quantities(confirmed_objects)
        raw_quantities = self.build_object_quantities(raw_visible_objects)
        payload = {
            "category": str(request.category),
            "location": str(request.location),
            "location_verified": bool(location_verified),
            "count": len(matched),
            "total_detected_count": len(confirmed_objects),
            "raw_total_detected_count": len(raw_visible_objects),
            "object_quantities": quantities,
            "raw_object_quantities": raw_quantities,
            "confirmation": self.get_confirmation_settings(),
            "scene_age_sec": float(scene_age_sec),
            "all_detected_objects": [
                {
                    "object_id": obj.object_id,
                    "name": obj.name,
                    "confidence": obj.confidence,
                    "bbox": list(obj.bbox),
                }
                for obj in confirmed_objects
            ],
            "raw_detected_objects": [
                {
                    "object_id": obj.object_id,
                    "name": obj.name,
                    "confidence": obj.confidence,
                    "bbox": list(obj.bbox),
                }
                for obj in raw_visible_objects
            ],
            "matched_objects": [
                {
                    "object_id": obj.object_id,
                    "name": obj.name,
                    "confidence": obj.confidence,
                    "bbox": list(obj.bbox),
                }
                for obj in matched
            ],
        }
        return json.dumps(payload, sort_keys=True)

    def build_object_quantities(self, objects: list[SceneObject]) -> dict[str, int]:
        """Special function: Count every perceived object type in the current scene."""
        quantities: dict[str, int] = {}
        for obj in objects:
            quantities[obj.name] = int(quantities.get(obj.name, 0)) + 1
        return dict(sorted(quantities.items()))

    def publish_object_quantities(self) -> None:
        """Special function: Publish raw and confirmed per-type quantities as JSON text."""
        if (
            not self.skill_active
            or self.raw_quantities_pub is None
            or self.confirmed_quantities_pub is None
        ):
            return
        min_confidence = float(self.get_parameter("min_confidence").value)
        raw_visible_objects = [
            obj for obj in self.scene_objects.values()
            if obj.confidence >= min_confidence
        ]
        confirmed_objects = self.get_confirmed_objects(self.now_sec())

        # Section: Publish immediate raw detections for detector tuning and GUI correlation.
        raw_payload = {
            "stamp_sec": self.now_sec(),
            "confirmed": False,
            "total_detected_count": len(raw_visible_objects),
            "object_quantities": self.build_object_quantities(raw_visible_objects),
            "detected_types": sorted({obj.name for obj in raw_visible_objects}),
        }
        raw_msg = String()
        raw_msg.data = json.dumps(raw_payload, sort_keys=True)
        self.raw_quantities_pub.publish(raw_msg)

        # Section: Publish confirmed detections for GPSR decisions and stable monitoring.
        confirmed_payload = {
            "stamp_sec": self.now_sec(),
            "confirmed": True,
            "total_detected_count": len(confirmed_objects),
            "object_quantities": self.build_object_quantities(confirmed_objects),
            "detected_types": sorted({obj.name for obj in confirmed_objects}),
            "confirmation": self.get_confirmation_settings(),
        }
        msg = String()
        msg.data = json.dumps(confirmed_payload, sort_keys=True)
        self.confirmed_quantities_pub.publish(msg)

    def prune_frame_history(self, now: float) -> None:
        """Special function: Drop frames outside the temporal confirmation window."""
        window_sec = float(self.get_parameter("confirmation_window_sec").value)
        while self.frame_history and (now - self.frame_history[0][0]) > window_sec:
            self.frame_history.popleft()

    def get_confirmed_objects(self, now: float) -> list[SceneObject]:
        """Special function: Return object candidates that satisfy temporal confirmation rules."""
        self.prune_frame_history(now)
        frame_count = len(self.frame_history)
        min_frames = int(self.get_parameter("confirmation_min_frames").value)
        if frame_count < min_frames:
            return []

        required_ratio = float(self.get_parameter("confirmation_required_ratio").value)
        last_seen_timeout = float(self.get_parameter("confirmation_last_seen_timeout_sec").value)
        required_votes = max(1, int(math.ceil(required_ratio * frame_count)))
        votes: dict[str, int] = {}
        latest_objects: dict[str, SceneObject] = {}
        last_seen: dict[str, float] = {}

        # Section: Count each object candidate once per frame if it passed confidence threshold.
        for stamp_sec, frame_objects in self.frame_history:
            for object_id, obj in frame_objects.items():
                votes[object_id] = int(votes.get(object_id, 0)) + 1
                latest_objects[object_id] = obj
                last_seen[object_id] = stamp_sec

        # Section: Keep candidates with enough votes and recent last observation.
        confirmed = []
        for object_id, vote_count in votes.items():
            if vote_count < required_votes:
                continue
            if (now - last_seen.get(object_id, 0.0)) > last_seen_timeout:
                continue
            confirmed.append(latest_objects[object_id])
        confirmed.sort(key=lambda obj: (obj.name, obj.object_id))
        return confirmed

    def get_confirmation_settings(self) -> dict[str, float | int]:
        """Special function: Report active confirmation thresholds in result payloads."""
        return {
            "window_sec": float(self.get_parameter("confirmation_window_sec").value),
            "required_ratio": float(self.get_parameter("confirmation_required_ratio").value),
            "confidence_threshold": float(self.get_parameter("confirmation_confidence_threshold").value),
            "last_seen_timeout_sec": float(self.get_parameter("confirmation_last_seen_timeout_sec").value),
            "min_frames": int(self.get_parameter("confirmation_min_frames").value),
            "history_frames": len(self.frame_history),
        }

    def build_message(self, category: str, location: str, count: int, location_verified: bool) -> str:
        """Special function: Build a human-readable count summary."""
        target = str(category).strip() or "objects"
        if location and not location_verified:
            return f"Counted {count} {target}; requested location '{location}' was not verified by current detections."
        if location:
            return f"Counted {count} {target} at {location}."
        return f"Counted {count} {target}."


def main(args: list[str] | None = None) -> None:
    """Special function: Start the count-object perception service server."""
    rclpy.init(args=args)
    node = CountObjectsPerceptionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
