#!/usr/bin/env -S bash -c 'exec "${VISION_PYTHON:-${HOME}/ai/qwen3v1/.venv/bin/python}" "$0" "$@"'
"""Section: GPSR count-object perception service server."""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from std_msgs.msg import String

from putrabot_interfaces.msg import ObjectDetections
from robot_ai_interfaces.srv import CountObjects
from robot_yasmin_tasks.object_knowledge import (
    ObjectKnowledge,
    ObjectKnowledgeError,
    TargetKind,
    normalize_object_symbol,
)


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

    # Section: Validated Qwen semantic evidence used for color, shape, and orientation filters.
    attributes: dict[str, str] = field(default_factory=dict)

    # Section: Source image dimensions support deterministic image-relative relation decisions.
    image_width: int = 0
    image_height: int = 0


class CountObjectsPerceptionServer(LifecycleNode):
    """Special function: Count stable detected objects matching a GPSR entity/category."""

    def __init__(self) -> None:
        """Special function: Initialize subscriptions, semantic query state, and scene cache."""
        super().__init__("count_objects_perception_server")

        # Section: Declare ROS interface and matching parameters.
        self.declare_parameter("service_name", "/perception/count_objects")
        self.declare_parameter("detections_topic", "/perception/raw_object_detections")
        self.declare_parameter("detector_query_topic", "/perception/object_detector_query")
        self.declare_parameter("object_knowledge_file", str(ObjectKnowledge.default_path()))
        self.declare_parameter("object_quantities_topic", "/perception/object_quantities")
        self.declare_parameter("raw_object_quantities_topic", "/perception/object_quantities_raw")
        self.declare_parameter("min_confidence", 0.50)
        self.declare_parameter("stale_timeout_sec", 2.0)
        self.declare_parameter("qwen_query_wait_timeout_sec", 0.0)
        self.declare_parameter("near_relation_distance_ratio", 0.25)
        self.declare_parameter("enforce_location_filter", False)
        self.declare_parameter("confirmation_window_sec", 2.0)
        self.declare_parameter("confirmation_required_ratio", 0.10)
        self.declare_parameter("confirmation_confidence_threshold", 0.35)
        self.declare_parameter("confirmation_last_seen_timeout_sec", 1.0)
        self.declare_parameter("confirmation_min_frames", 3)

        # Section: Protect each cached Qwen frame while a service request awaits a fresh query result.
        self.scene_lock = threading.RLock()
        self.scene_condition = threading.Condition(self.scene_lock)
        self.scene_generation = 0

        # Section: Keep the latest raw detection set and a short temporal confirmation history.
        self.scene_objects: dict[str, SceneObject] = {}
        self.last_scene_time_sec = 0.0
        self.frame_history: deque[tuple[float, dict[str, SceneObject]]] = deque()

        # Section: Resolve GPSR entities from the same authoritative labels used by FIND_OBJECT.
        self.object_knowledge: ObjectKnowledge | None = None

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
        self.detector_query_pub = None
        self.confirmed_quantities_pub = None
        self.raw_quantities_pub = None
        self.count_service = None
        self.skill_active = False
        self.get_logger().info("CountObjects perception lifecycle node is UNCONFIGURED.")
    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Allocate detection, query, publisher, and service interfaces."""
        try:
            # Section: Create resources while callbacks remain explicitly INACTIVE.
            knowledge_file = str(self.get_parameter("object_knowledge_file").value).strip()
            self.object_knowledge = ObjectKnowledge.from_file(knowledge_file)
            self.detections_subscription = self.create_subscription(
                ObjectDetections,
                str(self.get_parameter("detections_topic").value),
                self.detections_callback,
                10,
            )
            # Section: COUNT_OBJECTS uses the shared bounded Qwen query transport, never raw prompt text.
            self.detector_query_pub = self.create_publisher(
                String,
                str(self.get_parameter("detector_query_topic").value),
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
        if self.detector_query_pub is not None:
            self.destroy_publisher(self.detector_query_pub)
            self.detector_query_pub = None
        if self.confirmed_quantities_pub is not None:
            self.destroy_publisher(self.confirmed_quantities_pub)
            self.confirmed_quantities_pub = None
        if self.raw_quantities_pub is not None:
            self.destroy_publisher(self.raw_quantities_pub)
            self.raw_quantities_pub = None
        if self.count_service is not None:
            self.destroy_service(self.count_service)
            self.count_service = None
        self.object_knowledge = None

    def reset_cache(self) -> None:
        """Special function: Reset perception state to a fresh UNCONFIGURED baseline."""
        # Section: Wake any pending request so lifecycle teardown cannot leave it waiting on a frame.
        with self.scene_condition:
            self.scene_objects.clear()
            self.last_scene_time_sec = 0.0
            self.frame_history.clear()
            self.scene_generation += 1
            self.scene_condition.notify_all()

    def detections_callback(self, msg: ObjectDetections) -> None:
        """Special function: Cache the latest object detections as countable scene objects."""
        if not self.skill_active:
            return
        now = self.now_sec()
        next_scene: dict[str, SceneObject] = {}
        next_confirmation_frame: dict[str, SceneObject] = {}
        confirmation_confidence = float(self.get_parameter("confirmation_confidence_threshold").value)

        # Section: Convert every detector message into a normalized SceneObject.
        for index, det in enumerate(msg.detections):
            name = normalize_object_symbol(det.name)
            confidence = float(det.confidence)
            bbox = (int(det.x1), int(det.y1), int(det.x2), int(det.y2))
            if not name:
                continue

            # Section: Build a duplicate-resistant key from label and coarse bbox center.
            object_id = self.make_object_id(name, bbox, index)
            obj = SceneObject(
                object_id,
                name,
                confidence,
                bbox,
                self.detection_attributes(det),
                int(getattr(det, "image_width", 0) or 0),
                int(getattr(det, "image_height", 0) or 0),
            )
            next_scene[object_id] = obj
            if confidence >= confirmation_confidence:
                next_confirmation_frame[object_id] = obj

        # Section: Replace the cache atomically and release any query waiting for its next detector frame.
        with self.scene_condition:
            self.last_scene_time_sec = now
            self.scene_objects = next_scene
            self.frame_history.append((now, next_confirmation_frame))
            self.prune_frame_history(now)
            self.scene_generation += 1
            self.scene_condition.notify_all()

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
        semantic_filters = self.semantic_filters(filters)
        target_names = self.resolve_target_names(request.category)
        min_confidence = float(filters.get("min_confidence", self.get_parameter("min_confidence").value))
        stale_timeout_sec = float(filters.get("stale_timeout_sec", self.get_parameter("stale_timeout_sec").value))

        # Section: Send the count target to Qwen before reading its cache, then optionally await one new frame.
        with self.scene_condition:
            query_generation = self.scene_generation
        qwen_query, query_dispatched = self.publish_detector_query(request, semantic_filters)
        query_frame_received = self.wait_for_query_frame(query_generation) if query_dispatched else False

        # Section: Capture one coherent confirmed/raw scene after query dispatch for deterministic counting.
        with self.scene_condition:
            now = self.now_sec()
            scene_age_sec = now - self.last_scene_time_sec if self.last_scene_time_sec else float("inf")
            raw_visible_objects = [
                obj
                for obj in self.scene_objects.values()
                if obj.confidence >= min_confidence
            ]
            confirmed_objects = self.get_confirmed_objects_locked(now)

        # Section: Reject stale perception so GPSR does not answer from old camera data.
        if scene_age_sec > stale_timeout_sec:
            response.success = False
            response.count = 0
            response.result_json = self.build_result_json(
                request,
                [],
                [],
                [],
                scene_age_sec,
                location_verified=False,
                semantic_filters=semantic_filters,
                qwen_query=qwen_query,
                qwen_query_dispatched=query_dispatched,
                qwen_query_frame_received=query_frame_received,
            )
            response.message = f"No fresh object detections received within {stale_timeout_sec:.2f}s."
            return response

        # Section: Count only confirmed candidates satisfying target, Qwen attribute, and relation constraints.
        matched = self.filter_matching_objects(
            confirmed_objects,
            target_names,
            min_confidence,
            semantic_filters,
        )

        # Section: Apply location policy; ObjectDetections currently has no location field.
        requested_location = self.clean_text(request.location)
        enforce_location = bool(filters.get("enforce_location_filter", self.get_parameter("enforce_location_filter").value))
        location_verified = not requested_location
        if requested_location and enforce_location:
            matched = []

        # Section: Fill response with count and structured result details.
        response.success = True
        response.count = len(matched)
        response.result_json = self.build_result_json(
            request,
            matched,
            confirmed_objects,
            raw_visible_objects,
            scene_age_sec,
            location_verified,
            semantic_filters=semantic_filters,
            qwen_query=qwen_query,
            qwen_query_dispatched=query_dispatched,
            qwen_query_frame_received=query_frame_received,
        )
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
        """Special function: Convert a GPSR entity/category into authoritative detector labels."""
        # Section: Generic object words intentionally retain wildcard matching of every visible item.
        target = normalize_object_symbol(category)
        if not target:
            return set()
        if target in {"object", "objects", "thing", "things", "any", "any_object"}:
            return set()

        # Section: Exact aliases and configured categories share FIND_OBJECT's canonical vocabulary.
        if self.object_knowledge is not None:
            try:
                resolved = self.object_knowledge.resolve(object_name=target)
                if resolved.kind == TargetKind.EXPLICIT_WILDCARD:
                    return set()
                return {
                    normalize_object_symbol(label)
                    for label in resolved.detector_labels
                    if normalize_object_symbol(label)
                }
            except ObjectKnowledgeError:
                pass

        # Section: Retain pre-existing generic labels for deployments with an extended local vocabulary.
        if target in self.category_map:
            return {normalize_object_symbol(label) for label in self.category_map[target]}
        return {target}

    def query_target(self, category: str) -> dict[str, str]:
        """Special function: Encode one count target in the bounded Qwen object-query schema."""
        # Section: Preserve category semantics so Qwen returns all candidates instead of one named item.
        target = normalize_object_symbol(category)
        if not target:
            return {}
        if target in {"object", "objects", "thing", "things", "any", "any_object"}:
            return {"category": "object"}
        if self.object_knowledge is not None:
            try:
                resolved = self.object_knowledge.resolve(object_name=target)
                if resolved.kind == TargetKind.KNOWN_CATEGORY:
                    return {"category": resolved.canonical_name}
                if resolved.kind == TargetKind.EXPLICIT_WILDCARD:
                    return {"category": "object"}
                return {"object_name": resolved.canonical_name}
            except ObjectKnowledgeError:
                pass
        return {"object_name": target}

    def detection_attributes(self, detection: Any) -> dict[str, str]:
        """Special function: Decode safe Qwen attributes without trusting arbitrary detector JSON."""
        # Section: Interfaces built before attributes_json remain compatible and simply have no semantic evidence.
        raw_attributes = str(getattr(detection, "attributes_json", "") or "").strip()
        if not raw_attributes:
            return {}
        try:
            parsed = json.loads(raw_attributes)
        except json.JSONDecodeError:
            return {}
        if not isinstance(parsed, dict):
            return {}
        aliases = {
            "color": "color",
            "object_color": "color",
            "shape": "shape",
            "approximate_shape": "shape",
            "orientation": "orientation",
        }
        attributes: dict[str, str] = {}
        for source, destination in aliases.items():
            value = normalize_object_symbol(parsed.get(source, ""))
            if value:
                attributes[destination] = value
        return attributes

    def semantic_filters(self, filters: dict[str, Any]) -> list[dict[str, str]]:
        """Special function: Retain only count constraints that Qwen and local ROI logic can verify."""
        # Section: Control values such as confidence/timeouts stay outside the semantic Qwen prompt.
        raw_filters = filters.get("filters", [])
        if not isinstance(raw_filters, list):
            return []
        aliases = {
            "color": "object_color",
            "approximate_shape": "shape",
            "reference": "reference_object",
        }
        allowed_features = {
            "object_color",
            "shape",
            "orientation",
            "size_relation",
            "spatial_relation",
            "reference_object",
            "reference_category",
        }
        allowed_size_relations = {"smallest", "largest"}
        allowed_spatial_relations = {
            "left_of",
            "right_of",
            "above",
            "below",
            "near",
            "far_from",
        }
        normalized: list[dict[str, str]] = []
        for item in raw_filters:
            if not isinstance(item, dict):
                continue
            feature = aliases.get(
                normalize_object_symbol(item.get("feature", "")),
                normalize_object_symbol(item.get("feature", "")),
            )
            value = normalize_object_symbol(item.get("value", ""))
            if feature not in allowed_features or not value:
                continue
            if feature == "size_relation":
                value = {"biggest": "largest"}.get(value, value)
                if value not in allowed_size_relations:
                    continue
            if feature == "spatial_relation" and value not in allowed_spatial_relations:
                continue
            candidate = {"feature": feature, "operator": "equals", "value": value}
            if candidate not in normalized:
                normalized.append(candidate)
        return normalized

    @staticmethod
    def semantic_filter_value(filters: list[dict[str, str]], feature: str) -> str:
        """Special function: Read one first-match constraint from a normalized semantic filter list."""
        # Section: Scheduler filters are ordered, so the first valid instance stays deterministic.
        for item in filters:
            if item.get("feature") == feature:
                return str(item.get("value", ""))
        return ""

    def publish_detector_query(
        self,
        request: CountObjects.Request,
        semantic_filters: list[dict[str, str]],
    ) -> tuple[dict[str, Any], bool]:
        """Special function: Ask the shared Qwen detector for count-relevant ROI evidence."""
        # Section: The same normalized schema is consumed by FIND_OBJECT and blocks prompt injection.
        query: dict[str, Any] = {
            "target": self.query_target(request.category),
            "filters": list(semantic_filters),
            "requested_fields": ["image_position", "color", "shape", "orientation"],
        }
        if self.detector_query_pub is None:
            return query, False
        try:
            message = String()
            message.data = json.dumps(query, sort_keys=True)
            self.detector_query_pub.publish(message)
            return query, True
        except Exception as exc:
            self.get_logger().warn(f"Unable to publish COUNT_OBJECTS Qwen query: {exc}")
            return query, False

    def wait_for_query_frame(self, previous_generation: int) -> bool:
        """Special function: Optionally wait for one post-query Qwen detector frame without busy polling."""
        # Section: A zero timeout preserves non-Qwen/debug operation while production launches opt in.
        timeout_sec = max(0.0, float(self.get_parameter("qwen_query_wait_timeout_sec").value))
        if timeout_sec <= 0.0:
            return False
        deadline = time.monotonic() + timeout_sec
        with self.scene_condition:
            while self.skill_active and self.scene_generation <= previous_generation:
                remaining_sec = deadline - time.monotonic()
                if remaining_sec <= 0.0:
                    break
                self.scene_condition.wait(timeout=remaining_sec)
            return self.scene_generation > previous_generation

    @staticmethod
    def bbox_center(obj: SceneObject) -> tuple[float, float]:
        """Special function: Calculate a pixel-space ROI center for relationship checks."""
        # Section: Bounding boxes are the shared minimum geometry emitted by Qwen and YOLO backends.
        x1, y1, x2, y2 = obj.bbox
        return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

    @staticmethod
    def bbox_area(obj: SceneObject) -> float:
        """Special function: Return one finite image ROI area for optional count size constraints."""
        # Section: Image area is deliberately not a physical-size measurement.
        x1, y1, x2, y2 = obj.bbox
        return float(max(1, x2 - x1) * max(1, y2 - y1))

    def label_matches_target(self, label: str, target_names: set[str]) -> bool:
        """Special function: Match canonical labels while tolerating safe Qwen container suffixes."""
        # Section: Examples include coke/coke_can and soju/soju_bottle without broad substring matching.
        normalized_label = normalize_object_symbol(label)
        return any(
            normalized_label == target
            or normalized_label.startswith(f"{target}_")
            or target.startswith(f"{normalized_label}_")
            for target in target_names
        )

    @staticmethod
    def attribute_matches(observed: str, required: str) -> bool:
        """Special function: Match a normalized property while allowing compound Qwen descriptions."""
        # Section: A red_and_white item may satisfy a request for red, but unrelated substrings cannot.
        return observed == required or required in observed.split("_")

    def matches_semantic_attributes(
        self,
        obj: SceneObject,
        semantic_filters: list[dict[str, str]],
    ) -> bool:
        """Special function: Reject a candidate that lacks requested local Qwen attribute evidence."""
        # Section: Relations and size are evaluated only after ordinary semantic filters are proven.
        for feature, attribute_key in (
            ("object_color", "color"),
            ("shape", "shape"),
            ("orientation", "orientation"),
        ):
            required = self.semantic_filter_value(semantic_filters, feature)
            if required and not self.attribute_matches(obj.attributes.get(attribute_key, ""), required):
                return False
        return True

    def image_diagonal(self, objects: list[SceneObject]) -> float:
        """Special function: Derive a usable image diagonal for near/far count constraints."""
        # Section: Legacy messages without dimensions fall back to the furthest visible ROI edge.
        width = max([obj.image_width for obj in objects] + [obj.bbox[2] for obj in objects] + [1])
        height = max([obj.image_height for obj in objects] + [obj.bbox[3] for obj in objects] + [1])
        return math.hypot(float(width), float(height))

    def relation_matches(
        self,
        candidate: SceneObject,
        reference: SceneObject,
        relation: str,
        objects: list[SceneObject],
    ) -> bool:
        """Special function: Verify one requested image-space relation between two confirmed ROIs."""
        # Section: Left/right/above/below use centers, while near/far use an explicit image-relative threshold.
        candidate_x, candidate_y = self.bbox_center(candidate)
        reference_x, reference_y = self.bbox_center(reference)
        if relation == "left_of":
            return candidate_x < reference_x
        if relation == "right_of":
            return candidate_x > reference_x
        if relation == "above":
            return candidate_y < reference_y
        if relation == "below":
            return candidate_y > reference_y
        distance = math.hypot(candidate_x - reference_x, candidate_y - reference_y)
        threshold = self.image_diagonal(objects) * float(
            self.get_parameter("near_relation_distance_ratio").value
        )
        if relation == "near":
            return distance <= threshold
        if relation == "far_from":
            return distance > threshold
        return False

    def filter_matching_objects(
        self,
        confirmed_objects: list[SceneObject],
        target_names: set[str],
        min_confidence: float,
        semantic_filters: list[dict[str, str]],
    ) -> list[SceneObject]:
        """Special function: Locally verify count candidates after Qwen supplies semantic ROIs."""
        # Section: Target identity, confidence, and ordinary properties are mandatory for every counted item.
        candidates = [
            obj
            for obj in confirmed_objects
            if obj.confidence >= min_confidence
            and (not target_names or self.label_matches_target(obj.name, target_names))
            and self.matches_semantic_attributes(obj, semantic_filters)
        ]

        # Section: Spatial counts require a separately visible, configured reference object/category.
        relation = self.semantic_filter_value(semantic_filters, "spatial_relation")
        if relation:
            reference_object = self.semantic_filter_value(semantic_filters, "reference_object")
            reference_category = self.semantic_filter_value(semantic_filters, "reference_category")
            reference_names = self.resolve_target_names(reference_object or reference_category)
            if not (reference_object or reference_category):
                return []
            references = [
                obj
                for obj in confirmed_objects
                if obj.confidence >= min_confidence
                and (not reference_names or self.label_matches_target(obj.name, reference_names))
            ]
            candidates = [
                candidate
                for candidate in candidates
                if any(
                    candidate.object_id != reference.object_id
                    and self.relation_matches(candidate, reference, relation, confirmed_objects)
                    for reference in references
                )
            ]

        # Section: Extreme wording produces the tied smallest/largest visible count instead of a new skill.
        size_relation = self.semantic_filter_value(semantic_filters, "size_relation")
        if candidates and size_relation in {"smallest", "largest"}:
            areas = [self.bbox_area(candidate) for candidate in candidates]
            selected_area = min(areas) if size_relation == "smallest" else max(areas)
            candidates = [
                candidate
                for candidate in candidates
                if math.isclose(self.bbox_area(candidate), selected_area, abs_tol=1.0)
            ]
        return sorted(candidates, key=lambda obj: (obj.name, obj.object_id))

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

    def image_position(self, obj: SceneObject) -> str:
        """Special function: Derive one compact image-relative position from a detected ROI."""
        # Section: A legacy detector without image dimensions reports unknown instead of inventing geometry.
        if obj.image_width <= 0 or obj.image_height <= 0:
            return "unknown"
        center_x, center_y = self.bbox_center(obj)
        horizontal = "left" if center_x < obj.image_width / 3.0 else (
            "right" if center_x >= 2.0 * obj.image_width / 3.0 else "center"
        )
        vertical = "upper" if center_y < obj.image_height / 3.0 else (
            "lower" if center_y >= 2.0 * obj.image_height / 3.0 else "middle"
        )
        return f"{vertical}_{horizontal}"

    def object_payload(self, obj: SceneObject) -> dict[str, Any]:
        """Special function: Serialize one countable ROI with its validated Qwen evidence."""
        # Section: Preserve common fields for existing GPSR reporters while adding semantic traceability.
        return {
            "object_id": obj.object_id,
            "name": obj.name,
            "confidence": obj.confidence,
            "bbox": list(obj.bbox),
            "attributes": dict(obj.attributes),
            "image_position": self.image_position(obj),
        }

    def build_result_json(
        self,
        request: CountObjects.Request,
        matched: list[SceneObject],
        confirmed_objects: list[SceneObject],
        raw_visible_objects: list[SceneObject],
        scene_age_sec: float,
        location_verified: bool,
        *,
        semantic_filters: list[dict[str, str]] | None = None,
        qwen_query: dict[str, Any] | None = None,
        qwen_query_dispatched: bool = False,
        qwen_query_frame_received: bool = False,
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
            "filters": list(semantic_filters or []),
            "qwen_query": dict(qwen_query or {}),
            "qwen_query_dispatched": bool(qwen_query_dispatched),
            "qwen_query_frame_received": bool(qwen_query_frame_received),
            "all_detected_objects": [self.object_payload(obj) for obj in confirmed_objects],
            "raw_detected_objects": [self.object_payload(obj) for obj in raw_visible_objects],
            "matched_objects": [self.object_payload(obj) for obj in matched],
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
        # Section: Snapshot both views under one lock so publishing never mixes adjacent detector frames.
        with self.scene_condition:
            raw_visible_objects = [
                obj for obj in self.scene_objects.values()
                if obj.confidence >= min_confidence
            ]
            confirmed_objects = self.get_confirmed_objects_locked(self.now_sec())

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
        # Section: Callers hold scene_condition so history cannot be pruned during a count snapshot.
        window_sec = float(self.get_parameter("confirmation_window_sec").value)
        while self.frame_history and (now - self.frame_history[0][0]) > window_sec:
            self.frame_history.popleft()

    def get_confirmed_objects(self, now: float) -> list[SceneObject]:
        """Special function: Return object candidates that satisfy temporal confirmation rules."""
        # Section: Public callers receive a coherent temporal vote snapshot.
        with self.scene_condition:
            return self.get_confirmed_objects_locked(now)

    def get_confirmed_objects_locked(self, now: float) -> list[SceneObject]:
        """Special function: Evaluate temporal confirmation while the caller holds scene_condition."""
        # Section: Keep the vote calculation adjacent to cache mutation for race-free count results.
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
        # Section: History size is protected because service responses can overlap detector updates.
        with self.scene_condition:
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
    # Section: Two callback threads let a service wait for one detector frame without blocking subscriptions.
    rclpy.init(args=args)
    node = CountObjectsPerceptionServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
