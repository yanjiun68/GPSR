#!/usr/bin/env python3
"""Section: GPSR-only cross-node guest memory backed by a YASMIN blackboard."""

from __future__ import annotations

import copy
import json
import threading
import uuid
from typing import Any

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from yasmin import Blackboard

from robot_ai_interfaces.msg import GpsrGuestRecord
from robot_ai_interfaces.srv import GetGpsrGuest, ListGpsrGuests, SaveGpsrGuest


# Section: Stable keys make the service-owned blackboard contract explicit.
GPSR_SESSION_ID = "gpsr_session_id"
ACTIVE_GUEST_MEMORY_ID = "active_guest_memory_id"
GUEST_RECORDS = "guest_records"
MEMORY_REVISION = "memory_revision"


def parse_json_object(raw_value: str) -> dict[str, Any]:
    """Special function: Parse one JSON object without allowing malformed memory writes."""
    try:
        value = json.loads(str(raw_value or "{}"))
    except (json.JSONDecodeError, TypeError):
        return {}
    return value if isinstance(value, dict) else {}


def merge_nonempty(existing: Any, incoming: Any) -> Any:
    """Special function: Recursively merge meaningful values without erasing known evidence."""
    if isinstance(existing, dict) and isinstance(incoming, dict):
        merged = dict(existing)
        for key, value in incoming.items():
            merged[key] = merge_nonempty(merged.get(key), value)
        return merged
    # Section: UNKNOWN values are semantic absence regardless of source casing.
    if incoming is None or incoming == "" or (
        isinstance(incoming, str) and incoming.strip().casefold() == "unknown"
    ):
        return existing
    if isinstance(incoming, (list, tuple)) and not incoming:
        return existing
    return incoming


def observation_stamp_ns(payload: dict[str, Any]) -> int:
    """Special function: Convert a serialized observation stamp to a comparable integer."""
    stamp = payload.get("stamp", {})
    if not isinstance(stamp, dict):
        return 0
    return int(stamp.get("sec", 0) or 0) * 1_000_000_000 + int(
        stamp.get("nanosec", 0) or 0
    )


def merge_appearance_evidence(
    existing: dict[str, Any],
    incoming: dict[str, Any],
) -> dict[str, Any]:
    """Special function: Accept only known appearance evidence at equal/greater confidence."""
    merged: dict[str, Any] = {}
    for field_name in ("glasses", "hat", "face_mask", "shirt_color"):
        confidence_name = f"{field_name}_confidence"
        if field_name in existing:
            merged[field_name] = existing[field_name]
        if confidence_name in existing:
            merged[confidence_name] = existing[confidence_name]
        incoming_value = incoming.get(field_name)
        if incoming_value in {None, ""} or (
            isinstance(incoming_value, str)
            and incoming_value.strip().casefold() == "unknown"
        ):
            continue
        incoming_confidence = float(incoming.get(confidence_name, 0.0) or 0.0)
        existing_confidence = float(existing.get(confidence_name, 0.0) or 0.0)
        existing_value = existing.get(field_name)
        if (
            existing_value in {None, "", "unknown", "UNKNOWN"}
            or incoming_confidence >= existing_confidence
        ):
            merged[field_name] = incoming_value
            merged[confidence_name] = incoming_confidence
    return merged


class GpsrGuestMemoryStore:
    """Special function: Own and merge GPSR guest records inside one synchronized blackboard."""

    def __init__(self, now_callback: Any, session_id: str = "") -> None:
        """Special function: Initialize one RAM-only GPSR session memory."""
        # Section: Only this store touches the blackboard; callers use synchronized methods.
        self._lock = threading.RLock()
        self._now_callback = now_callback
        self.blackboard = Blackboard()
        self.blackboard[GPSR_SESSION_ID] = session_id or uuid.uuid4().hex
        self.blackboard[ACTIVE_GUEST_MEMORY_ID] = ""
        self.blackboard[GUEST_RECORDS] = {}
        self.blackboard[MEMORY_REVISION] = 0

    @property
    def session_id(self) -> str:
        """Special function: Return the immutable tracking session identifier."""
        return str(self.blackboard[GPSR_SESSION_ID])

    def save(self, incoming: GpsrGuestRecord) -> GpsrGuestRecord:
        """Special function: Idempotently create or merge one guest record."""
        with self._lock:
            # Section: Derive a stable identity before consulting the current record map.
            memory_id = self._resolve_memory_id(incoming)
            records = dict(self.blackboard[GUEST_RECORDS])
            existing = records.get(memory_id)
            merged = self._merge_record(existing, incoming, memory_id)
            records[memory_id] = copy.deepcopy(merged)
            self.blackboard[GUEST_RECORDS] = records
            self.blackboard[ACTIVE_GUEST_MEMORY_ID] = memory_id
            self.blackboard[MEMORY_REVISION] = int(merged.revision)
            return copy.deepcopy(merged)

    def get(self, memory_id: str = "") -> GpsrGuestRecord | None:
        """Special function: Return a detached record by id or select the active guest."""
        with self._lock:
            selected_id = str(memory_id or self.blackboard[ACTIVE_GUEST_MEMORY_ID])
            record = dict(self.blackboard[GUEST_RECORDS]).get(selected_id)
            return copy.deepcopy(record) if record is not None else None

    def list_records(self) -> list[GpsrGuestRecord]:
        """Special function: Return detached records in deterministic insertion order."""
        with self._lock:
            return [copy.deepcopy(record) for record in dict(self.blackboard[GUEST_RECORDS]).values()]

    def clear(self) -> None:
        """Special function: Clear records while retaining the current GPSR session identity."""
        with self._lock:
            self.blackboard[ACTIVE_GUEST_MEMORY_ID] = ""
            self.blackboard[GUEST_RECORDS] = {}
            self.blackboard[MEMORY_REVISION] = 0

    def active_memory_id(self) -> str:
        """Special function: Return the currently selected memory identifier."""
        with self._lock:
            return str(self.blackboard[ACTIVE_GUEST_MEMORY_ID])

    def _resolve_memory_id(self, incoming: GpsrGuestRecord) -> str:
        """Special function: Prefer profile identity and otherwise scope a track to its session."""
        requested_id = str(incoming.memory_id).strip()
        if requested_id:
            return requested_id
        profile_id = str(incoming.profile.profile_id).strip()
        if profile_id:
            return f"profile:{profile_id.casefold()}"
        person_ref = parse_json_object(incoming.person_ref_json)
        referenced_profile = str(person_ref.get("profile_id", "")).strip()
        if referenced_profile:
            return f"profile:{referenced_profile.casefold()}"
        tracking_session = str(
            incoming.tracking_session_id
            or person_ref.get("tracking_session_id", "")
            or self.session_id
        ).strip()
        track_id = int(incoming.track_id or person_ref.get("track_id", 0) or 0)
        return f"anonymous:{tracking_session}:{track_id}"

    def _merge_record(
        self,
        existing: GpsrGuestRecord | None,
        incoming: GpsrGuestRecord,
        memory_id: str,
    ) -> GpsrGuestRecord:
        """Special function: Merge one request without replacing known fields with empty data."""
        now = self._now_callback()
        merged = copy.deepcopy(existing) if existing is not None else GpsrGuestRecord()
        merged.memory_id = memory_id
        merged.command_id = str(incoming.command_id or merged.command_id)
        merged.source_step_id = str(incoming.source_step_id or merged.source_step_id)
        merged.tracking_session_id = str(
            incoming.tracking_session_id or merged.tracking_session_id or self.session_id
        )
        if int(incoming.track_id) != 0 or int(merged.track_id) == 0:
            merged.track_id = int(incoming.track_id)
        # Section: JSON evidence is merged field-wise to retain earlier known values.
        existing_ref = parse_json_object(merged.person_ref_json)
        incoming_ref = parse_json_object(incoming.person_ref_json)
        person_ref = merge_nonempty(existing_ref, incoming_ref)
        person_ref["appearance"] = merge_appearance_evidence(
            existing_ref.get("appearance", {})
            if isinstance(existing_ref.get("appearance"), dict) else {},
            incoming_ref.get("appearance", {})
            if isinstance(incoming_ref.get("appearance"), dict) else {},
        )
        existing_info = parse_json_object(merged.guest_info_json)
        incoming_info = parse_json_object(incoming.guest_info_json)
        guest_info = merge_nonempty(existing_info, incoming_info)
        guest_info.update(merge_appearance_evidence(existing_info, incoming_info))
        # Section: Delayed observations cannot replace newer pose or gesture evidence.
        incoming_stamp = observation_stamp_ns(incoming_ref)
        existing_stamp = observation_stamp_ns(existing_ref)
        if existing_stamp and incoming_stamp and incoming_stamp < existing_stamp:
            for field_name in (
                "pose", "pose_confidence", "gesture", "gesture_confidence",
            ):
                if field_name in existing_ref:
                    person_ref[field_name] = existing_ref[field_name]
                if field_name in existing_info:
                    guest_info[field_name] = existing_info[field_name]
        merged.person_ref_json = json.dumps(person_ref, sort_keys=True)
        merged.guest_info_json = json.dumps(guest_info, sort_keys=True)
        merged.profile = self._merge_profile(merged.profile, incoming.profile)
        if existing is None:
            merged.created_at = now
        merged.updated_at = now
        merged.revision = int(merged.revision) + 1
        return merged

    @staticmethod
    def _merge_profile(existing: Any, incoming: Any) -> Any:
        """Special function: Merge profile and confidence-bearing appearance evidence."""
        merged = copy.deepcopy(existing)
        # Section: Non-empty semantic profile fields update the stored profile.
        for field_name in (
            "profile_id", "role", "name", "gender", "favorite_drink", "hometown",
            "seat_position", "clothes_color",
        ):
            incoming_value = getattr(incoming, field_name)
            if incoming_value and str(incoming_value).strip().casefold() != "unknown":
                setattr(merged, field_name, incoming_value)
        if int(incoming.age) > 0:
            merged.age = int(incoming.age)
        if list(incoming.face_embedding):
            merged.face_embedding = list(incoming.face_embedding)
        merged.is_confirmed = bool(incoming.is_confirmed or merged.is_confirmed)
        merged.confidence_score = max(float(merged.confidence_score), float(incoming.confidence_score))
        # Section: Only known, stronger appearance values replace existing evidence.
        for state_name, confidence_name in (
            ("glasses", "glasses_confidence"),
            ("hat", "hat_confidence"),
            ("face_mask", "face_mask_confidence"),
        ):
            incoming_state = int(getattr(incoming.appearance, state_name))
            incoming_confidence = float(getattr(incoming.appearance, confidence_name))
            current_state = int(getattr(merged.appearance, state_name))
            current_confidence = float(getattr(merged.appearance, confidence_name))
            if incoming_state != int(incoming.appearance.UNKNOWN) and (
                current_state == int(merged.appearance.UNKNOWN) or incoming_confidence >= current_confidence
            ):
                setattr(merged.appearance, state_name, incoming_state)
                setattr(merged.appearance, confidence_name, incoming_confidence)
        if incoming.appearance.shirt_color and (
            not merged.appearance.shirt_color
            or float(incoming.appearance.shirt_color_confidence)
            >= float(merged.appearance.shirt_color_confidence)
        ):
            merged.appearance.shirt_color = incoming.appearance.shirt_color
            merged.appearance.shirt_color_confidence = incoming.appearance.shirt_color_confidence
        return merged


class GpsrGuestMemoryNode(Node):
    """Special function: Expose the service-owned GPSR blackboard to other ROS nodes."""

    def __init__(self) -> None:
        """Special function: Create session-scoped save, get, list, and clear services."""
        super().__init__("gpsr_guest_memory_node")
        self.store = GpsrGuestMemoryStore(lambda: self.get_clock().now().to_msg())
        # Section: GPSR namespace prevents accidental receptionist-memory mutation.
        self.save_service = self.create_service(
            SaveGpsrGuest, "/gpsr/guest_memory/save", self.save_callback
        )
        self.get_service = self.create_service(
            GetGpsrGuest, "/gpsr/guest_memory/get", self.get_callback
        )
        self.list_service = self.create_service(
            ListGpsrGuests, "/gpsr/guest_memory/list", self.list_callback
        )
        self.clear_service = self.create_service(
            Trigger, "/gpsr/guest_memory/clear", self.clear_callback
        )
        self.get_logger().info(
            f"GPSR guest memory ready for session {self.store.session_id}."
        )

    def save_callback(self, request: Any, response: Any) -> Any:
        """Special function: Merge one record and return its authoritative revision."""
        try:
            response.record = self.store.save(request.record)
            response.success = True
            response.message = f"Stored GPSR guest {response.record.memory_id}."
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"GPSR guest save failed: {exc}"
        return response

    def get_callback(self, request: Any, response: Any) -> Any:
        """Special function: Resolve one stored record for a remote GPSR node."""
        record = self.store.get(request.memory_id)
        response.success = record is not None
        if record is not None:
            response.record = record
            response.message = f"Retrieved GPSR guest {record.memory_id}."
        else:
            response.message = f"GPSR guest '{request.memory_id or 'active'}' was not found."
        return response

    def list_callback(self, _request: Any, response: Any) -> Any:
        """Special function: Return all session records to a remote GPSR node."""
        response.success = True
        response.records = self.store.list_records()
        response.active_memory_id = self.store.active_memory_id()
        response.message = f"Returned {len(response.records)} GPSR guest records."
        return response

    def clear_callback(self, _request: Any, response: Any) -> Any:
        """Special function: Clear session records only through the explicit memory reset."""
        self.store.clear()
        response.success = True
        response.message = "GPSR guest memory cleared."
        return response


def main(args: list[str] | None = None) -> None:
    """Special function: Run the persistent GPSR session memory node."""
    rclpy.init(args=args)
    node = GpsrGuestMemoryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
