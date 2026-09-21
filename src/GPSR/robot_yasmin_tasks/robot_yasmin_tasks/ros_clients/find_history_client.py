"""Section: Small optional client for the bounded GPSR find-history services."""

from __future__ import annotations

import hashlib
import json
import threading
from typing import Any

from geometry_msgs.msg import PoseStamped

from robot_ai_interfaces.msg import FoundEntityRecord
from robot_ai_interfaces.srv import LookupFoundEntity, SaveFoundEntity
from robot_yasmin_tasks.gpsr.action_types import (
    FIND_OBJECT,
    FIND_PERSON,
    FIND_PERSON_BY_ATTRIBUTE,
)
from robot_yasmin_tasks.gpsr.models import ActionStepData


def clean_key(value: Any) -> str:
    """Special function: Normalize one spoken identifier into a stable lookup token."""
    return "_".join(
        str(value or "").strip().casefold().replace("-", " ").split()
    )[:128]


def find_target_identity(action: ActionStepData) -> tuple[str, str] | None:
    """Special function: Derive an exact entity type and repeatable find target key."""
    if action.action_type == FIND_OBJECT:
        object_name = clean_key(
            action.target_object or action.parameters.get("object_name", "")
        )
        category = clean_key(
            action.target_category or action.parameters.get("category", "")
        )
        if object_name:
            return "object", f"name:{object_name}"
        if category:
            return "object", f"category:{category}"
        return None
    if action.action_type == FIND_PERSON:
        selector = action.parameters.get("selector", {})
        selector = dict(selector) if isinstance(selector, dict) else {}
        name = clean_key(
            selector.get("value")
            if str(selector.get("type", "")).casefold() == "name"
            else action.target_person
        )
        return ("person", f"name:{name}") if name else None
    if action.action_type == FIND_PERSON_BY_ATTRIBUTE:
        selector = action.parameters.get("selector", {})
        if not isinstance(selector, dict) or not selector:
            return None
        # Section: Canonical JSON makes equivalent structured selectors share one key.
        canonical = json.dumps(selector, separators=(",", ":"), sort_keys=True).casefold()
        digest = hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:32]
        return "person", f"selector:{digest}"
    return None


def pose_from_dict(payload: Any) -> PoseStamped:
    """Special function: Convert one JSON-safe pose result back into PoseStamped."""
    pose = PoseStamped()
    data = dict(payload) if isinstance(payload, dict) else {}
    pose.header.frame_id = str(data.get("frame_id", ""))
    stamp = data.get("stamp", {})
    stamp = dict(stamp) if isinstance(stamp, dict) else {}
    pose.header.stamp.sec = int(stamp.get("sec", 0) or 0)
    pose.header.stamp.nanosec = int(stamp.get("nanosec", 0) or 0)
    position = data.get("position", {})
    position = dict(position) if isinstance(position, dict) else {}
    pose.pose.position.x = float(position.get("x", 0.0) or 0.0)
    pose.pose.position.y = float(position.get("y", 0.0) or 0.0)
    pose.pose.position.z = float(position.get("z", 0.0) or 0.0)
    orientation = data.get("orientation", {})
    orientation = dict(orientation) if isinstance(orientation, dict) else {}
    pose.pose.orientation.x = float(orientation.get("x", 0.0) or 0.0)
    pose.pose.orientation.y = float(orientation.get("y", 0.0) or 0.0)
    pose.pose.orientation.z = float(orientation.get("z", 0.0) or 0.0)
    pose.pose.orientation.w = float(orientation.get("w", 0.0) or 0.0)
    return pose


def pose_to_dict(pose: PoseStamped) -> dict[str, Any]:
    """Special function: Convert a remembered map pose into action parameter data."""
    return {
        "frame_id": str(pose.header.frame_id),
        "stamp": {
            "sec": int(pose.header.stamp.sec),
            "nanosec": int(pose.header.stamp.nanosec),
        },
        "position": {
            "x": float(pose.pose.position.x),
            "y": float(pose.pose.position.y),
            "z": float(pose.pose.position.z),
        },
        "orientation": {
            "x": float(pose.pose.orientation.x),
            "y": float(pose.pose.orientation.y),
            "z": float(pose.pose.orientation.z),
            "w": float(pose.pose.orientation.w),
        },
    }


def record_to_hint(record: FoundEntityRecord) -> dict[str, Any]:
    """Special function: Expose only compact routing and identity fields to a find skill."""
    try:
        metadata = json.loads(str(record.metadata_json or "{}"))
    except json.JSONDecodeError:
        metadata = {}
    return {
        "entity_id": str(record.entity_id),
        "entity_type": str(record.entity_type),
        "target_key": str(record.target_key),
        "semantic_location": str(record.semantic_location),
        "viewpoint": str(record.viewpoint),
        "entity_pose": pose_to_dict(record.entity_pose),
        "observation_pose": pose_to_dict(record.observation_pose),
        "metadata": metadata if isinstance(metadata, dict) else {},
        "revision": int(record.revision),
    }


def compact_person_ref(payload: Any) -> dict[str, Any]:
    """Special function: Keep only identity fields needed by later person skills."""
    source = dict(payload) if isinstance(payload, dict) else {}
    allowed = (
        "person_id",
        "person_uuid",
        "profile_id",
        "name",
        "target_name",
        "tracking_id",
        "track_id",
        "tracking_session_id",
        "location",
        "selector_type",
        "selector_value",
        "pose",
        "gesture",
        "confidence",
        "source",
        "identification_source",
    )
    return {key: source[key] for key in allowed if key in source}


def build_found_record(
    action: ActionStepData,
    result_payload: dict[str, Any],
) -> FoundEntityRecord | None:
    """Special function: Build one compact record from a successful find result."""
    identity = find_target_identity(action)
    if identity is None:
        return None
    entity_type, target_key = identity
    record = FoundEntityRecord()
    record.entity_type = entity_type
    record.target_key = target_key
    if action.action_type == FIND_OBJECT:
        record.entity_id = str(result_payload.get("object_id", "") or target_key)
        record.entity_pose = pose_from_dict(result_payload.get("object_pose", {}))
        record.observation_pose = pose_from_dict(
            result_payload.get("observation_pose", {})
        )
        record.semantic_location = str(result_payload.get("found_room", ""))
        record.viewpoint = str(result_payload.get("found_viewpoint", ""))
        metadata = {
            "object_id": record.entity_id,
            "localized": bool(result_payload.get("localized", False)),
        }
    else:
        person_ref = compact_person_ref(result_payload.get("person_ref", {}))
        record.entity_id = str(
            person_ref.get("person_uuid")
            or person_ref.get("profile_id")
            or person_ref.get("person_id")
            or result_payload.get("memory_id", "")
            or target_key
        )
        record.semantic_location = str(person_ref.get("location", ""))
        metadata = {
            "person_ref": person_ref,
            "memory_id": str(result_payload.get("memory_id", "")),
        }
    record.metadata_json = json.dumps(
        metadata,
        separators=(",", ":"),
        sort_keys=True,
        default=str,
    )
    return record


class FindHistoryClient:
    """Special function: Perform bounded best-effort lookups without owning cached records."""

    def __init__(self, node: Any, timeout_sec: float = 0.25) -> None:
        """Special function: Create two lightweight clients on the existing executor node."""
        self.node = node
        self.timeout_sec = max(0.01, float(timeout_sec))
        self.lookup_client = node.create_client(
            LookupFoundEntity,
            "/gpsr/find_history/lookup",
        )
        self.save_client = node.create_client(
            SaveFoundEntity,
            "/gpsr/find_history/save",
        )

    def lookup(self, action: ActionStepData) -> FoundEntityRecord | None:
        """Special function: Return a matching record or continue when memory is unavailable."""
        identity = find_target_identity(action)
        if identity is None or not self.lookup_client.wait_for_service(timeout_sec=0.02):
            return None
        request = LookupFoundEntity.Request()
        request.entity_type, request.target_key = identity
        response = self._wait(self.lookup_client.call_async(request))
        return response.record if response is not None and bool(response.found) else None

    def remember(self, action: ActionStepData, result_payload: dict[str, Any]) -> bool:
        """Special function: Save successful find evidence without changing queue success."""
        record = build_found_record(action, result_payload)
        if record is None or not self.save_client.wait_for_service(timeout_sec=0.02):
            return False
        request = SaveFoundEntity.Request()
        request.record = record
        response = self._wait(self.save_client.call_async(request))
        return bool(response is not None and response.success)

    def _wait(self, future: Any) -> Any:
        """Special function: Wait briefly while the multithreaded executor services the future."""
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(timeout=self.timeout_sec):
            future.cancel()  # Release a stalled request instead of retaining pending futures.
            return None
        try:
            return future.result()
        except Exception:  # noqa: BLE001 - memory remains an optional optimization.
            return None

    def destroy(self) -> None:
        """Special function: Destroy ROS handles and remove retained client references."""
        for client_name in ("lookup_client", "save_client"):
            client = getattr(self, client_name, None)
            if client is not None:
                self.node.destroy_client(client)
                setattr(self, client_name, None)
        self.node = None
