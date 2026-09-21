#!/usr/bin/env -S bash -c 'exec "${VISION_PYTHON:-${HOME}/ai/qwen3v1/.venv/bin/python}" "$0" "$@"'
"""Section: GPSR count-people perception service server."""

from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import dataclass
from typing import Any

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from std_msgs.msg import String

from putrabot_interfaces.msg import HumanSkeleton, HumanSkeletonKeypoint, HumanSkeletons, ObjectDetections
from robot_ai_interfaces.srv import CountPeople


@dataclass(slots=True)
class HumanEntity:
    """Special function: Stable human record derived from detector output."""

    # Section: Stable human key used to avoid duplicate counts in one scene window.
    entity_id: str

    # Section: Detection confidence from the source detector.
    confidence: float

    # Section: Pixel-space bounding box used for simple duplicate suppression.
    bbox: tuple[int, int, int, int]

    # Section: Coarse body pose label inferred from skeleton geometry.
    pose: str = "unknown"

    # Section: Pixel-space center used for deterministic left-to-right sorting.
    center_u: float = 0.0

    # Section: Pixel-space center used for JSON metadata and future policies.
    center_v: float = 0.0

    # Section: Optional skeleton keypoints serialized into result_json when available.
    keypoints: list[dict[str, Any]] | None = None

    # Section: Detection source used for debugging merge and fallback behavior.
    source: str = "box"


class CountPeoplePerceptionServer(LifecycleNode):
    """Special function: Count stable visible human entities from person detections."""

    # Section: Pose group priority used for deterministic count result ordering.
    POSE_SORT_PRIORITY = {"standing": 0, "sitting": 1, "lying": 2, "unknown": 3}

    def __init__(self) -> None:
        """Special function: Initialize subscriptions, service, and human scene cache."""
        super().__init__("count_people_perception_server")

        # Section: Declare ROS interface and matching parameters.
        self.declare_parameter("service_name", "/perception/count_people")
        self.declare_parameter("detections_topic", "/perception/raw_human_detections")
        self.declare_parameter("human_quantities_topic", "/perception/human_quantities")
        self.declare_parameter("raw_human_quantities_topic", "/perception/human_quantities_raw")
        self.declare_parameter("enable_pose_sorting", True)
        self.declare_parameter("skeletons_topic", "/perception/human_skeletons")
        self.declare_parameter("skeleton_stale_timeout_sec", 1.0)
        self.declare_parameter("min_skeleton_confidence", 0.35)
        self.declare_parameter("min_visible_skeleton_keypoints", 5)
        self.declare_parameter("box_skeleton_match_iou_threshold", 0.20)
        self.declare_parameter("box_skeleton_match_center_distance_px", 80.0)
        self.declare_parameter("min_confidence", 0.50)
        self.declare_parameter("stale_timeout_sec", 5.0)
        self.declare_parameter("enforce_location_filter", False)
        self.declare_parameter("confirmation_window_sec", 2.0)
        self.declare_parameter("confirmation_required_ratio", 0.60)
        self.declare_parameter("confirmation_confidence_threshold", 0.35)
        self.declare_parameter("confirmation_last_seen_timeout_sec", 1.0)
        self.declare_parameter("confirmation_min_frames", 3)

        # Section: Keep the latest raw human detections and temporal confirmation history.
        self.scene_humans: dict[str, HumanEntity] = {}
        self.last_scene_time_sec = 0.0
        self.last_box_time_sec = 0.0
        self.last_skeleton_time_sec = 0.0
        self.latest_box_humans: list[HumanEntity] = []
        self.latest_skeleton_humans: list[HumanEntity] = []
        self.frame_history: deque[tuple[float, dict[str, HumanEntity]]] = deque()

        # Section: ROS resources remain empty until CONFIGURE.
        self.detections_subscription = None
        self.skeletons_subscription = None
        self.confirmed_quantities_pub = None
        self.raw_quantities_pub = None
        self.count_service = None
        self.skill_active = False
        self.get_logger().info("CountPeople perception lifecycle node is UNCONFIGURED.")
    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Allocate human detection, pose, publisher, and service interfaces."""
        try:
            # Section: Create resources while callbacks remain explicitly INACTIVE.
            self.detections_subscription = self.create_subscription(
                ObjectDetections,
                str(self.get_parameter("detections_topic").value),
                self.detections_callback,
                10,
            )
            if bool(self.get_parameter("enable_pose_sorting").value):
                self.skeletons_subscription = self.create_subscription(
                    HumanSkeletons,
                    str(self.get_parameter("skeletons_topic").value),
                    self.skeletons_callback,
                    10,
                )
            self.confirmed_quantities_pub = self.create_lifecycle_publisher(
                String,
                str(self.get_parameter("human_quantities_topic").value),
                10,
            )
            self.raw_quantities_pub = self.create_lifecycle_publisher(
                String,
                str(self.get_parameter("raw_human_quantities_topic").value),
                10,
            )
            self.count_service = self.create_service(
                CountPeople,
                str(self.get_parameter("service_name").value),
                self.handle_count_people,
            )
            self.reset_cache()
            self.get_logger().info("[GPSR_LIFECYCLE] CountPeople perception: configured")
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.get_logger().error(f"CountPeople perception configure failed: {exc}")
            self.release_resources()
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Special function: Activate publishers and allow human perception callbacks."""
        result = super().on_activate(state)
        if result == TransitionCallbackReturn.SUCCESS:
            self.skill_active = True
        return result

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Special function: Stop consuming detections and serving count requests."""
        self.skill_active = False
        return super().on_deactivate(state)

    def on_cleanup(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Destroy configured interfaces and clear human caches."""
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
        if self.skeletons_subscription is not None:
            self.destroy_subscription(self.skeletons_subscription)
            self.skeletons_subscription = None
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
        self.scene_humans.clear()
        self.last_scene_time_sec = 0.0
        self.last_box_time_sec = 0.0
        self.last_skeleton_time_sec = 0.0
        self.latest_box_humans.clear()
        self.latest_skeleton_humans.clear()
        self.frame_history.clear()

    def detections_callback(self, msg: ObjectDetections) -> None:
        """Special function: Cache the latest person detections as countable human entities."""
        if not self.skill_active:
            return
        now = self.now_sec()
        self.last_box_time_sec = now
        next_box_humans: list[HumanEntity] = []

        # Section: Convert every person detector message into a normalized HumanEntity.
        for index, det in enumerate(msg.detections):
            name = str(det.name).strip().lower()
            confidence = float(det.confidence)
            bbox = (int(det.x1), int(det.y1), int(det.x2), int(det.y2))
            if name != "person":
                continue

            # Section: Build a duplicate-resistant key from coarse bbox center.
            entity_id = self.make_entity_id(bbox, index)
            center_u, center_v = self.bbox_center(bbox)
            entity = HumanEntity(entity_id, confidence, bbox, "unknown", center_u, center_v, None, "box")
            next_box_humans.append(entity)

        # Section: Store latest box-only humans and rebuild the merged scene.
        self.latest_box_humans = next_box_humans
        self.refresh_human_scene(now)

        if not self.skill_active:
            return
    def skeletons_callback(self, msg: HumanSkeletons) -> None:
        """Special function: Cache latest skeleton detections as pose-labeled human entities."""
        now = self.now_sec()
        self.last_skeleton_time_sec = now
        min_confidence = float(self.get_parameter("min_skeleton_confidence").value)
        min_visible = int(self.get_parameter("min_visible_skeleton_keypoints").value)
        next_skeleton_humans: list[HumanEntity] = []

        # Section: Convert every usable skeleton into a countable human candidate.
        for index, skeleton in enumerate(msg.skeletons):
            confidence = float(skeleton.confidence)
            visible_count = self.count_visible_keypoints(skeleton)
            if confidence < min_confidence or visible_count < min_visible:
                continue
            bbox = (int(skeleton.x1), int(skeleton.y1), int(skeleton.x2), int(skeleton.y2))
            if not self.is_valid_bbox(bbox):
                continue
            entity_id = self.make_entity_id(bbox, index)
            pose = self.clean_pose_label(skeleton.pose)
            keypoints = [self.keypoint_to_dict(keypoint) for keypoint in skeleton.keypoints if keypoint.visible]
            entity = HumanEntity(
                entity_id,
                confidence,
                bbox,
                pose,
                float(skeleton.center_u),
                float(skeleton.center_v),
                keypoints,
                "skeleton",
            )
            next_skeleton_humans.append(entity)

        # Section: Store latest skeleton humans and rebuild the merged scene.
        self.latest_skeleton_humans = next_skeleton_humans
        self.refresh_human_scene(now)

    def refresh_human_scene(self, now: float) -> None:
        """Special function: Merge latest box and skeleton observations into one count scene."""
        confirmation_confidence = float(self.get_parameter("confirmation_confidence_threshold").value)
        merged_humans = self.merge_box_and_skeleton_humans(now)
        next_scene = {entity.entity_id: entity for entity in merged_humans}
        next_confirmation_frame = {
            entity.entity_id: entity
            for entity in merged_humans
            if entity.confidence >= confirmation_confidence
        }

        # Section: Replace the cache atomically so service calls see one coherent scene.
        self.scene_humans = next_scene
        self.last_scene_time_sec = now

        # Section: Update temporal history used for confirmed human presence.
        self.frame_history.append((now, next_confirmation_frame))
        self.prune_frame_history(now)

        # Section: Publish compact summary topics after every perception update.
        self.publish_human_quantities()

    def merge_box_and_skeleton_humans(self, now: float) -> list[HumanEntity]:
        """Special function: Deduplicate box detections and skeleton detections into human entities."""
        boxes_fresh = self.boxes_are_fresh(now)
        skeletons_fresh = self.skeletons_are_fresh(now)
        merged: list[HumanEntity] = []
        used_skeleton_indexes: set[int] = set()

        # Section: Start with box detections and enrich matching boxes with pose skeletons.
        active_box_humans = self.latest_box_humans if boxes_fresh else []
        for box_entity in active_box_humans:
            match_index = self.find_matching_skeleton(box_entity, used_skeleton_indexes) if skeletons_fresh else None
            if match_index is None:
                merged.append(box_entity)
                continue
            skeleton_entity = self.latest_skeleton_humans[match_index]
            used_skeleton_indexes.add(match_index)
            merged.append(self.merge_entities(box_entity, skeleton_entity))

        # Section: Count skeleton-only people when pose data sees a human without a box.
        if skeletons_fresh:
            for index, skeleton_entity in enumerate(self.latest_skeleton_humans):
                if index not in used_skeleton_indexes:
                    merged.append(skeleton_entity)

        # Section: Keep count output stable across repeated service calls.
        merged.sort(key=self.entity_sort_key)
        return merged

    def boxes_are_fresh(self, now: float) -> bool:
        """Special function: Return true when person-box observations are recent enough to merge."""
        if not self.last_box_time_sec:
            return False
        stale_timeout = float(self.get_parameter("stale_timeout_sec").value)
        return (now - self.last_box_time_sec) <= stale_timeout

    def skeletons_are_fresh(self, now: float) -> bool:
        """Special function: Return true when skeleton observations are recent enough to merge."""
        if not bool(self.get_parameter("enable_pose_sorting").value):
            return False
        if not self.last_skeleton_time_sec:
            return False
        stale_timeout = float(self.get_parameter("skeleton_stale_timeout_sec").value)
        return (now - self.last_skeleton_time_sec) <= stale_timeout

    def find_matching_skeleton(self, box_entity: HumanEntity, used_indexes: set[int]) -> int | None:
        """Special function: Find the best unused skeleton matching one person box."""
        best_index = None
        best_score = -1.0
        iou_threshold = float(self.get_parameter("box_skeleton_match_iou_threshold").value)
        center_threshold = float(self.get_parameter("box_skeleton_match_center_distance_px").value)

        # Section: Prefer bbox overlap, then allow nearest-center matching for loose skeleton boxes.
        for index, skeleton_entity in enumerate(self.latest_skeleton_humans):
            if index in used_indexes:
                continue
            iou_score = self.bbox_iou(box_entity.bbox, skeleton_entity.bbox)
            center_distance = self.center_distance(box_entity, skeleton_entity)
            if iou_score < iou_threshold and center_distance > center_threshold:
                continue
            score = iou_score + max(0.0, (center_threshold - center_distance) / max(1.0, center_threshold))
            if score > best_score:
                best_index = index
                best_score = score
        return best_index

    def merge_entities(self, box_entity: HumanEntity, skeleton_entity: HumanEntity) -> HumanEntity:
        """Special function: Combine a person box with matching pose metadata without double-counting."""
        confidence = max(float(box_entity.confidence), float(skeleton_entity.confidence))
        return HumanEntity(
            box_entity.entity_id,
            confidence,
            box_entity.bbox,
            skeleton_entity.pose,
            box_entity.center_u,
            box_entity.center_v,
            skeleton_entity.keypoints,
            "box+skeleton",
        )

    def handle_count_people(self, request: CountPeople.Request, response: CountPeople.Response) -> CountPeople.Response:
        """Special function: Return the number of confirmed visible human entities."""
        if not self.skill_active:
            response.success = False
            response.count = 0
            response.result_json = "{}"
            response.message = "CountPeople perception is not ACTIVE."
            return response
        filters = self.parse_filter_json(request.filter_json)
        min_confidence = float(filters.get("min_confidence", self.get_parameter("min_confidence").value))
        stale_timeout_sec = float(filters.get("stale_timeout_sec", self.get_parameter("stale_timeout_sec").value))
        scene_age_sec = self.now_sec() - self.last_scene_time_sec if self.last_scene_time_sec else float("inf")

        # Section: Reject stale perception so GPSR does not answer from old camera data.
        if scene_age_sec > stale_timeout_sec:
            response.success = False
            response.count = 0
            response.result_json = self.build_result_json(request, [], [], scene_age_sec, location_verified=False)
            response.message = f"No fresh human detections received within {stale_timeout_sec:.2f}s."
            return response

        # Section: Keep every confidence-filtered raw human for troubleshooting metadata.
        raw_visible_humans = []
        for entity in self.scene_humans.values():
            if entity.confidence < min_confidence:
                continue
            raw_visible_humans.append(entity)
        raw_visible_humans.sort(key=self.entity_sort_key)

        # Section: Use only temporally confirmed humans for GPSR count decisions.
        confirmed_humans = self.get_confirmed_humans(self.now_sec())

        # Section: Apply location policy; ObjectDetections currently has no location field.
        requested_location = self.clean_text(request.location)
        enforce_location = bool(filters.get("enforce_location_filter", self.get_parameter("enforce_location_filter").value))
        location_verified = not requested_location
        matched_humans = sorted(confirmed_humans, key=self.entity_sort_key)
        if requested_location and enforce_location:
            matched_humans = []

        # Section: Fill response with count and structured result details.
        response.success = True
        response.count = len(matched_humans)
        response.result_json = self.build_result_json(request, matched_humans, raw_visible_humans, scene_age_sec, location_verified)
        response.message = self.build_message(requested_location, len(matched_humans), location_verified)
        return response

    def now_sec(self) -> float:
        """Special function: Return ROS time in seconds for stale-scene checks."""
        return self.get_clock().now().nanoseconds / 1e9

    def clean_text(self, value: Any) -> str:
        """Special function: Normalize empty ROS/CLI text values into an empty string."""
        text = str(value).strip()
        return "" if text.lower() in {"", "none", "null"} else text

    def make_entity_id(self, bbox: tuple[int, int, int, int], index: int) -> str:
        """Special function: Build a stable-enough human key from coarse image position."""
        x1, y1, x2, y2 = bbox
        center_x_bucket = int(((x1 + x2) / 2.0) // 20)
        center_y_bucket = int(((y1 + y2) / 2.0) // 20)
        return f"person_{center_x_bucket}_{center_y_bucket}"

    def bbox_center(self, bbox: tuple[int, int, int, int]) -> tuple[float, float]:
        """Special function: Return the center point for a pixel-space bbox."""
        x1, y1, x2, y2 = bbox
        return float((x1 + x2) / 2.0), float((y1 + y2) / 2.0)

    def is_valid_bbox(self, bbox: tuple[int, int, int, int]) -> bool:
        """Special function: Reject empty skeleton boxes before counting them as humans."""
        x1, y1, x2, y2 = bbox
        return x2 > x1 and y2 > y1

    def bbox_iou(self, first: tuple[int, int, int, int], second: tuple[int, int, int, int]) -> float:
        """Special function: Compute intersection-over-union between two person boxes."""
        first_x1, first_y1, first_x2, first_y2 = first
        second_x1, second_y1, second_x2, second_y2 = second
        inter_x1 = max(first_x1, second_x1)
        inter_y1 = max(first_y1, second_y1)
        inter_x2 = min(first_x2, second_x2)
        inter_y2 = min(first_y2, second_y2)
        inter_area = max(0, inter_x2 - inter_x1) * max(0, inter_y2 - inter_y1)
        first_area = max(0, first_x2 - first_x1) * max(0, first_y2 - first_y1)
        second_area = max(0, second_x2 - second_x1) * max(0, second_y2 - second_y1)
        union_area = first_area + second_area - inter_area
        return float(inter_area / union_area) if union_area > 0 else 0.0

    def center_distance(self, first: HumanEntity, second: HumanEntity) -> float:
        """Special function: Return Euclidean distance between two human entity centers."""
        return math.hypot(float(first.center_u) - float(second.center_u), float(first.center_v) - float(second.center_v))

    def entity_sort_key(self, entity: HumanEntity) -> tuple[int, float, str]:
        """Special function: Sort humans by pose group, then left-to-right, then stable id."""
        pose_priority = self.POSE_SORT_PRIORITY.get(entity.pose, self.POSE_SORT_PRIORITY["unknown"])
        return pose_priority, float(entity.center_u), entity.entity_id

    def clean_pose_label(self, pose: str) -> str:
        """Special function: Normalize skeleton pose labels to the supported coarse set."""
        normalized = str(pose).strip().lower()
        return normalized if normalized in self.POSE_SORT_PRIORITY else "unknown"

    def count_visible_keypoints(self, skeleton: HumanSkeleton) -> int:
        """Special function: Count visible keypoints in a skeleton message."""
        return sum(1 for keypoint in skeleton.keypoints if bool(keypoint.visible))

    def keypoint_to_dict(self, keypoint: HumanSkeletonKeypoint) -> dict[str, Any]:
        """Special function: Convert one skeleton keypoint into JSON-friendly metadata."""
        return {
            "name": str(keypoint.name),
            "index": int(keypoint.index),
            "x": float(keypoint.x),
            "y": float(keypoint.y),
            "confidence": float(keypoint.confidence),
        }

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
        request: CountPeople.Request,
        human_entities: list[HumanEntity],
        raw_human_entities: list[HumanEntity],
        scene_age_sec: float,
        location_verified: bool,
    ) -> str:
        """Special function: Serialize count details for action results, logs, and blackboard storage."""
        payload = {
            "location": str(request.location),
            "location_verified": bool(location_verified),
            "count": len(human_entities),
            "total_detected_count": len(human_entities),
            "raw_total_detected_count": len(raw_human_entities),
            "human_quantities": {"person": len(human_entities)} if human_entities else {},
            "raw_human_quantities": {"person": len(raw_human_entities)} if raw_human_entities else {},
            "pose_quantities": self.build_pose_quantities(human_entities),
            "raw_pose_quantities": self.build_pose_quantities(raw_human_entities),
            "confirmation": self.get_confirmation_settings(),
            "scene_age_sec": float(scene_age_sec),
            "pose_sorting_enabled": bool(self.get_parameter("enable_pose_sorting").value),
            "skeletons_fresh": self.skeletons_are_fresh(self.now_sec()),
            "human_entities": [self.entity_to_dict(entity) for entity in human_entities],
            "raw_human_entities": [self.entity_to_dict(entity) for entity in raw_human_entities],
        }
        return json.dumps(payload, sort_keys=True)

    def build_pose_quantities(self, entities: list[HumanEntity]) -> dict[str, int]:
        """Special function: Count confirmed human entities by coarse pose label."""
        quantities: dict[str, int] = {}
        for entity in entities:
            quantities[entity.pose] = int(quantities.get(entity.pose, 0)) + 1
        return {pose: quantities[pose] for pose in sorted(quantities, key=lambda label: self.POSE_SORT_PRIORITY.get(label, 99))}

    def entity_to_dict(self, entity: HumanEntity) -> dict[str, Any]:
        """Special function: Convert one human entity into JSON-friendly metadata."""
        payload = {
            "entity_id": entity.entity_id,
            "name": "person",
            "pose": entity.pose,
            "confidence": entity.confidence,
            "bbox": list(entity.bbox),
            "center_u": float(entity.center_u),
            "center_v": float(entity.center_v),
            "source": entity.source,
        }
        if entity.keypoints is not None:
            payload["keypoints"] = entity.keypoints
        return payload

    def publish_human_quantities(self) -> None:
        """Special function: Publish raw and confirmed human quantities as JSON text."""
        if (
            not self.skill_active
            or self.raw_quantities_pub is None
            or self.confirmed_quantities_pub is None
        ):
            return
        min_confidence = float(self.get_parameter("min_confidence").value)
        raw_visible_humans = [
            entity for entity in self.scene_humans.values()
            if entity.confidence >= min_confidence
        ]
        confirmed_humans = self.get_confirmed_humans(self.now_sec())

        # Section: Publish immediate raw detections for detector tuning and GUI correlation.
        raw_payload = {
            "stamp_sec": self.now_sec(),
            "confirmed": False,
            "total_detected_count": len(raw_visible_humans),
            "human_quantities": {"person": len(raw_visible_humans)} if raw_visible_humans else {},
            "pose_quantities": self.build_pose_quantities(raw_visible_humans),
            "detected_types": ["person"] if raw_visible_humans else [],
            "skeletons_fresh": self.skeletons_are_fresh(self.now_sec()),
        }
        raw_msg = String()
        raw_msg.data = json.dumps(raw_payload, sort_keys=True)
        self.raw_quantities_pub.publish(raw_msg)

        # Section: Publish confirmed detections for GPSR decisions and stable monitoring.
        confirmed_payload = {
            "stamp_sec": self.now_sec(),
            "confirmed": True,
            "total_detected_count": len(confirmed_humans),
            "human_quantities": {"person": len(confirmed_humans)} if confirmed_humans else {},
            "pose_quantities": self.build_pose_quantities(confirmed_humans),
            "detected_types": ["person"] if confirmed_humans else [],
            "confirmation": self.get_confirmation_settings(),
            "skeletons_fresh": self.skeletons_are_fresh(self.now_sec()),
        }
        msg = String()
        msg.data = json.dumps(confirmed_payload, sort_keys=True)
        self.confirmed_quantities_pub.publish(msg)

    def prune_frame_history(self, now: float) -> None:
        """Special function: Drop frames outside the temporal confirmation window."""
        window_sec = float(self.get_parameter("confirmation_window_sec").value)
        while self.frame_history and (now - self.frame_history[0][0]) > window_sec:
            self.frame_history.popleft()

    def get_confirmed_humans(self, now: float) -> list[HumanEntity]:
        """Special function: Return human candidates that satisfy temporal confirmation rules."""
        self.prune_frame_history(now)
        frame_count = len(self.frame_history)
        min_frames = int(self.get_parameter("confirmation_min_frames").value)
        if frame_count < min_frames:
            return []

        required_ratio = float(self.get_parameter("confirmation_required_ratio").value)
        last_seen_timeout = float(self.get_parameter("confirmation_last_seen_timeout_sec").value)
        required_votes = max(1, int(math.ceil(required_ratio * frame_count)))
        votes: dict[str, int] = {}
        latest_entities: dict[str, HumanEntity] = {}
        last_seen: dict[str, float] = {}

        # Section: Count each human candidate once per frame if it passed confidence threshold.
        for stamp_sec, frame_entities in self.frame_history:
            for entity_id, entity in frame_entities.items():
                votes[entity_id] = int(votes.get(entity_id, 0)) + 1
                latest_entities[entity_id] = entity
                last_seen[entity_id] = stamp_sec

        # Section: Keep candidates with enough votes and recent last observation.
        confirmed = []
        for entity_id, vote_count in votes.items():
            if vote_count < required_votes:
                continue
            if (now - last_seen.get(entity_id, 0.0)) > last_seen_timeout:
                continue
            confirmed.append(latest_entities[entity_id])
        confirmed.sort(key=self.entity_sort_key)
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

    def build_message(self, location: str, count: int, location_verified: bool) -> str:
        """Special function: Build a human-readable people count summary."""
        if location and not location_verified:
            return f"Counted {count} people; requested location '{location}' was not verified by current detections."
        if location:
            return f"Counted {count} people at {location}."
        return f"Counted {count} people."


def main(args: list[str] | None = None) -> None:
    """Special function: Start the count-people perception service server."""
    rclpy.init(args=args)
    node = CountPeoplePerceptionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
