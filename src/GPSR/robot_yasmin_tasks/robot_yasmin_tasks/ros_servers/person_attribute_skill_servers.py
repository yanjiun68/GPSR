#!/usr/bin/env python3
"""Section: Lifecycle YASMIN servers for GPSR person attributes and guest information."""

from __future__ import annotations

import copy
import json
import threading
import time
from collections import deque
from typing import Any

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State as LifecycleState, TransitionCallbackReturn
from receptionist_interfaces.msg import GuestAppearance, PersonProfile, TrackedPersonAppearance, TrackedPersonAppearanceArray
from receptionist_interfaces.srv import GetProfile
from yasmin import Blackboard, State, StateMachine

from robot_ai_interfaces.action import ExtractGuestInfo, FindPersonByAttribute
from robot_ai_interfaces.msg import GpsrGuestRecord
from robot_ai_interfaces.srv import GetGpsrGuest, SaveGpsrGuest


# Section: Internal mission outcomes remain independent from top-level queue outcomes.
READY = "ready"
MATCHED = "matched"
SAVED = "saved"
COMPLETE = "complete"
SUCCESS = "success"
INVALID_INPUT = "invalid_input"
NOT_FOUND = "not_found"
TIMEOUT = "timeout"
CANCELED = "canceled"
FAILED = "failed"

# Section: Supported fields match observable appearance plus registered guest data.
SUPPORTED_SELECTOR_TYPES = {
    "any_person", "pose", "gesture", "shirt_color", "glasses", "hat", "face_mask",
}
SUPPORTED_INFO_FIELDS = {
    "name", "age", "gender", "favorite_drink", "hometown", "shirt_color",
    "glasses", "hat", "face_mask", "pose", "gesture", "profile_id", "person_id",
    "track_id",
}


def parse_json_object(raw_value: str) -> dict[str, Any]:
    """Special function: Parse a JSON object without propagating malformed action input."""
    try:
        value = json.loads(str(raw_value or "{}"))
    except (json.JSONDecodeError, TypeError):
        return {}
    return value if isinstance(value, dict) else {}


def blackboard_value(blackboard: Blackboard, key: str, default: Any = None) -> Any:
    """Special function: Read an optional YASMIN key without using dict-only APIs."""
    return blackboard[key] if key in blackboard else default


def normalize_selector(
    raw_value: str,
    enabled_wearables: set[str] | None = None,
) -> tuple[list[dict[str, str]], str]:
    """Special function: Normalize legacy single selectors and conjunction selectors."""
    payload = parse_json_object(raw_value)
    predicates = payload.get("all") if isinstance(payload.get("all"), list) else [payload]
    normalized: list[dict[str, str]] = []
    for predicate in predicates:
        if not isinstance(predicate, dict):
            return [], "Every person selector must be an object."
        selector_type = str(predicate.get("type", "")).strip().casefold()
        operator = str(predicate.get("operator", "equals")).strip().casefold()
        value = str(predicate.get("value", "")).strip().casefold().replace(" ", "_")
        if selector_type not in SUPPORTED_SELECTOR_TYPES:
            return [], f"Unsupported person selector: {selector_type or 'empty'}."
        if operator not in {"equals", "is"}:
            return [], "Only positive equality person selectors are supported."
        if selector_type in {"glasses", "hat", "face_mask"} and value not in {"yes", "true", "wearing", "1"}:
            return [], f"Negative {selector_type} matching is not supported."
        # Section: Hat and mask are rejected until a validated positive model is enabled.
        if (
            selector_type in {"glasses", "hat", "face_mask"}
            and enabled_wearables is not None
            and selector_type not in enabled_wearables
        ):
            return [], f"{selector_type} matching has no configured positive-class model."
        if selector_type != "any_person" and not value:
            return [], f"Selector {selector_type} requires a value."
        normalized.append({"type": selector_type, "operator": "equals", "value": value})
    return normalized, "" if normalized else "At least one person selector is required."


def appearance_dict(appearance: GuestAppearance) -> dict[str, Any]:
    """Special function: Convert positive-only appearance evidence into JSON-safe values."""
    def state_text(value: int) -> str:
        """Special function: Preserve tri-state semantics for one wearable attribute."""
        if int(value) == int(GuestAppearance.YES):
            return "yes"
        if int(value) == int(GuestAppearance.NO):
            return "no"
        return "unknown"

    return {
        "glasses": state_text(appearance.glasses),
        "glasses_confidence": float(appearance.glasses_confidence),
        "hat": state_text(appearance.hat),
        "hat_confidence": float(appearance.hat_confidence),
        "face_mask": state_text(appearance.face_mask),
        "face_mask_confidence": float(appearance.face_mask_confidence),
        "shirt_color": str(appearance.shirt_color or "unknown"),
        "shirt_color_confidence": float(appearance.shirt_color_confidence),
    }


def observation_dict(observation: TrackedPersonAppearance) -> dict[str, Any]:
    """Special function: Serialize one typed tracked observation for durable memory."""
    return {
        "tracking_session_id": str(observation.tracking_session_id),
        "track_id": int(observation.track_id),
        "person_id": int(observation.person_id),
        "profile_id": str(observation.profile_id),
        "bbox_xyxy": [float(value) for value in observation.bbox_xyxy],
        "appearance": appearance_dict(observation.appearance),
        "pose": str(observation.pose or "unknown"),
        "pose_confidence": float(observation.pose_confidence),
        "gesture": str(observation.gesture or "none"),
        "gesture_confidence": float(observation.gesture_confidence),
        "stamp": {
            "sec": int(observation.header.stamp.sec),
            "nanosec": int(observation.header.stamp.nanosec),
        },
    }


def wait_for_future(future: Any, timeout_sec: float) -> Any:
    """Special function: Wait for a ROS future without recursively spinning its executor."""
    completed = threading.Event()
    future.add_done_callback(lambda _future: completed.set())
    if not completed.wait(timeout=max(0.05, float(timeout_sec))):
        return None
    return future.result()


def predicate_score(observation: TrackedPersonAppearance, predicate: dict[str, str]) -> float:
    """Special function: Return positive match confidence or a negative score on mismatch."""
    selector_type = predicate["type"]
    value = predicate["value"]
    if selector_type == "any_person":
        return 1.0
    if selector_type == "pose":
        expected = "lying_down" if value == "lying" else value
        return float(observation.pose_confidence) if str(observation.pose).casefold() == expected else -1.0
    if selector_type == "gesture":
        actual = str(observation.gesture).casefold()
        aliases = {
            "right_hand_raised": {"right_hand_raised", "raising_right_arm"},
            "left_hand_raised": {"left_hand_raised", "raising_left_arm"},
            "waving": {"waving", "left_arm_waving", "right_arm_waving", "both_arms_waving"},
        }
        return float(observation.gesture_confidence) if actual in aliases.get(value, {value}) else -1.0
    if selector_type == "shirt_color":
        return (
            float(observation.appearance.shirt_color_confidence)
            if str(observation.appearance.shirt_color).casefold() == value
            else -1.0
        )
    state_value = int(getattr(observation.appearance, selector_type))
    confidence = float(getattr(observation.appearance, f"{selector_type}_confidence"))
    return confidence if state_value == int(GuestAppearance.YES) else -1.0


def match_score(observation: TrackedPersonAppearance, predicates: list[dict[str, str]]) -> float:
    """Special function: Match every selector and return the weakest predicate confidence."""
    scores = [predicate_score(observation, predicate) for predicate in predicates]
    return min(scores) if scores and all(score >= 0.0 for score in scores) else -1.0


def visible_area(observation: TrackedPersonAppearance) -> float:
    """Special function: Calculate deterministic nearest-person tie-breaking area."""
    x1, y1, x2, y2 = [float(value) for value in observation.bbox_xyxy]
    return max(0.0, x2 - x1) * max(0.0, y2 - y1)


class ValidateFindGoalState(State):
    """Special function: Validate and normalize a find-person action goal."""

    def __init__(self, server: "FindPersonByAttributeSkillServer") -> None:
        """Special function: Declare find validation outcomes."""
        super().__init__([READY, INVALID_INPUT, CANCELED])
        self.server = server

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Reject malformed, negative, or canceled selector goals."""
        goal_handle = blackboard["goal_handle"]
        if goal_handle.is_cancel_requested:
            blackboard["message"] = "Find-person goal canceled before validation."
            return CANCELED
        predicates, error = normalize_selector(
            goal_handle.request.selector_json,
            self.server.enabled_wearables,
        )
        if error:
            blackboard["message"] = error
            return INVALID_INPUT
        blackboard["predicates"] = predicates
        # Section: One server-side absolute deadline covers matching and mandatory persistence.
        timeout_sec = max(
            0.1,
            float(goal_handle.request.timeout_sec or self.server.default_timeout_sec),
        )
        blackboard["deadline"] = time.monotonic() + timeout_sec
        return READY


class WaitForTrackedMatchState(State):
    """Special function: Wait for stable evidence from the tracked appearance stream."""

    def __init__(self, server: "FindPersonByAttributeSkillServer") -> None:
        """Special function: Store the lifecycle server that owns observation history."""
        super().__init__([MATCHED, NOT_FOUND, TIMEOUT, CANCELED, FAILED])
        self.server = server

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Confirm one canonical track across repeated observations."""
        goal_handle = blackboard["goal_handle"]
        timeout_sec = max(
            0.1,
            float(goal_handle.request.timeout_sec or self.server.default_timeout_sec),
        )
        required = max(
            1,
            int(goal_handle.request.confirmation_min_frames or self.server.confirmation_min_frames),
        )
        deadline = float(
            blackboard_value(blackboard, "deadline", time.monotonic() + timeout_sec)
        )
        with self.server.observation_condition:
            cursor = self.server.observation_sequence
        confirmations: dict[str, int] = {}
        saw_frames = False
        saw_people = False
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested or not self.server.skill_active:
                blackboard["message"] = "Find-person goal canceled."
                return CANCELED
            frames = self.server.frames_after(cursor, deadline)
            if not frames:
                continue
            for sequence, frame in frames:
                saw_frames = True
                cursor = max(cursor, sequence)
                people = list(frame.people)
                saw_people = saw_people or bool(people)
                ranked: list[tuple[float, float, int, TrackedPersonAppearance]] = []
                for person in people:
                    score = match_score(person, blackboard["predicates"])
                    if score >= 0.0:
                        ranked.append((score, visible_area(person), -int(person.track_id), person))
                feedback = FindPersonByAttribute.Feedback()
                feedback.status = "confirming" if ranked else "searching"
                feedback.visible_count = len(people)
                feedback.matching_count = len(ranked)
                goal_handle.publish_feedback(feedback)
                matched_keys: set[str] = set()
                for _score, _area, _negative_id, person in sorted(ranked, reverse=True):
                    key = f"{person.tracking_session_id}:{int(person.track_id)}"
                    matched_keys.add(key)
                    confirmations[key] = int(confirmations.get(key, 0)) + 1
                    if confirmations[key] >= required:
                        blackboard["observation"] = copy.deepcopy(person)
                        return MATCHED
                for key in list(confirmations):
                    if key not in matched_keys:
                        confirmations[key] = 0
        blackboard["message"] = (
            "People were visible, but none matched every requested attribute."
            if saw_people
            else (
                "Fresh tracked-person frames arrived, but no people were visible."
                if saw_frames
                else "No fresh tracked-person observations arrived before the deadline."
            )
        )
        # Section: An observed empty scene is a miss; stream silence is infrastructure failure.
        return NOT_FOUND if saw_frames else FAILED


class SaveFoundGuestState(State):
    """Special function: Persist a confirmed match before allowing skill success."""

    def __init__(self, server: "FindPersonByAttributeSkillServer") -> None:
        """Special function: Store the server-owned guest-memory client."""
        super().__init__([SAVED, FAILED, CANCELED, TIMEOUT])
        self.server = server

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Save the observation and authoritative person reference."""
        goal_handle = blackboard["goal_handle"]
        if goal_handle.is_cancel_requested:
            blackboard["message"] = "Find-person goal canceled before memory save."
            return CANCELED
        absolute_deadline = float(blackboard_value(blackboard, "deadline", 0.0))
        remaining = max(0.0, absolute_deadline - time.monotonic())
        if remaining <= 0.0:
            blackboard["message"] = "Find-person deadline expired before guest-memory save."
            return TIMEOUT
        observation = blackboard["observation"]
        snapshot = observation_dict(observation)
        person_ref = dict(snapshot)
        person_ref["location"] = str(goal_handle.request.location)
        person_ref["selector"] = {"all": list(blackboard["predicates"])}
        request = SaveGpsrGuest.Request()
        request.record.command_id = str(goal_handle.request.command_id)
        request.record.source_step_id = str(goal_handle.request.source_step_id)
        request.record.tracking_session_id = str(observation.tracking_session_id)
        request.record.track_id = int(observation.track_id)
        request.record.person_ref_json = json.dumps(person_ref, sort_keys=True)
        request.record.guest_info_json = json.dumps(
            {
                **snapshot["appearance"],
                "pose": snapshot["pose"],
                "gesture": snapshot["gesture"],
                "profile_id": snapshot["profile_id"],
                "person_id": snapshot["person_id"],
                "track_id": snapshot["track_id"],
            },
            sort_keys=True,
        )
        request.record.profile = copy.deepcopy(observation.profile)
        response = self.server.call_save(
            request,
            min(self.server.memory_timeout_sec, remaining),
        )
        if time.monotonic() >= absolute_deadline:
            blackboard["message"] = "Find-person deadline expired during guest-memory save."
            return TIMEOUT
        if response is None or not response.success:
            blackboard["message"] = (
                response.message if response is not None else "GPSR guest-memory save timed out."
            )
            return FAILED
        blackboard["saved_record"] = copy.deepcopy(response.record)
        blackboard["person_ref"] = person_ref
        blackboard["observation_json"] = json.dumps(snapshot, sort_keys=True)
        blackboard["message"] = "Person matching the requested attributes was found and stored."
        return SAVED


class FindPersonByAttributeMissionSM(StateMachine):
    """Special function: Own the real find-person-by-attribute mission inside its server."""

    def __init__(self, server: "FindPersonByAttributeSkillServer") -> None:
        """Special function: Build validation, matching, and mandatory persistence states."""
        super().__init__(outcomes=[SUCCESS, INVALID_INPUT, NOT_FOUND, TIMEOUT, CANCELED, FAILED])
        self.set_name("FIND_PERSON_BY_ATTRIBUTE_SERVER_SM")
        self.add_state("VALIDATE_SELECTORS", ValidateFindGoalState(server), transitions={READY: "WAIT_FOR_MATCH", INVALID_INPUT: INVALID_INPUT, CANCELED: CANCELED})
        self.add_state("WAIT_FOR_MATCH", WaitForTrackedMatchState(server), transitions={MATCHED: "SAVE_GUEST", NOT_FOUND: NOT_FOUND, TIMEOUT: TIMEOUT, CANCELED: CANCELED, FAILED: FAILED})
        self.add_state("SAVE_GUEST", SaveFoundGuestState(server), transitions={SAVED: SUCCESS, FAILED: FAILED, CANCELED: CANCELED, TIMEOUT: TIMEOUT})


class FindPersonByAttributeSkillServer(LifecycleNode):
    """Special function: Expose the lifecycle-owned attribute search action."""

    def __init__(self) -> None:
        """Special function: Declare interfaces while remaining resource-free in UNCONFIGURED."""
        super().__init__("find_person_by_attribute_skill_server")
        self.cb_group = ReentrantCallbackGroup()
        self.declare_parameter("action_name", "/gpsr/find_person_by_attribute")
        self.declare_parameter("appearance_topic", "/appearance/tracked_people")
        self.declare_parameter("memory_save_service", "/gpsr/guest_memory/save")
        self.declare_parameter("default_timeout_sec", 20.0)
        self.declare_parameter("confirmation_min_frames", 3)
        self.declare_parameter("memory_timeout_sec", 2.0)
        self.declare_parameter("enabled_wearable_predicates", ["glasses"])
        self.skill_active = False
        self.default_timeout_sec = 20.0
        self.confirmation_min_frames = 3
        self.memory_timeout_sec = 2.0
        self.enabled_wearables = {"glasses"}
        self.observation_condition = threading.Condition()
        self.observation_sequence = 0
        self.observation_history: deque[tuple[int, TrackedPersonAppearanceArray]] = deque(maxlen=120)
        self.appearance_sub = None
        self.memory_client = None
        self.action_server = None
        # Section: Admission is reserved atomically before ActionServer starts execution.
        self.goal_lock = threading.Lock()
        self.goal_admitted = False
        self.active_goal_handle = None

    def on_configure(self, _state: LifecycleState) -> TransitionCallbackReturn:
        """Special function: Allocate observation, memory, and action resources."""
        try:
            self.default_timeout_sec = float(self.get_parameter("default_timeout_sec").value)
            self.confirmation_min_frames = max(1, int(self.get_parameter("confirmation_min_frames").value))
            self.memory_timeout_sec = max(0.1, float(self.get_parameter("memory_timeout_sec").value))
            self.enabled_wearables = {
                str(value).strip().casefold()
                for value in self.get_parameter("enabled_wearable_predicates").value
                if str(value).strip()
            }
            self.observation_sequence = 0
            self.observation_history = deque(maxlen=120)
            self.appearance_sub = self.create_subscription(
                TrackedPersonAppearanceArray,
                str(self.get_parameter("appearance_topic").value),
                self.appearance_callback,
                10,
                callback_group=self.cb_group,
            )
            self.memory_client = self.create_client(
                SaveGpsrGuest,
                str(self.get_parameter("memory_save_service").value),
                callback_group=self.cb_group,
            )
            if not self.memory_client.wait_for_service(timeout_sec=self.memory_timeout_sec):
                raise RuntimeError("GPSR guest-memory save service is unavailable")
            self.action_server = ActionServer(
                self,
                FindPersonByAttribute,
                str(self.get_parameter("action_name").value),
                execute_callback=self.execute_callback,
                goal_callback=self.goal_callback,
                cancel_callback=self.cancel_callback,
                callback_group=self.cb_group,
            )
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Find-person attribute configure failed: {exc}")
            self.release_resources()
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, _state: LifecycleState) -> TransitionCallbackReturn:
        """Special function: Admit attribute-search goals only while ACTIVE."""
        self.skill_active = True
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, _state: LifecycleState) -> TransitionCallbackReturn:
        """Special function: Stop admission and wake active searches for cancellation."""
        self.skill_active = False
        with self.observation_condition:
            self.observation_condition.notify_all()
        return TransitionCallbackReturn.SUCCESS if not self.has_active_goal() else TransitionCallbackReturn.FAILURE

    def on_cleanup(self, _state: LifecycleState) -> TransitionCallbackReturn:
        """Special function: Release action, subscriptions, clients, and observation history."""
        if self.has_active_goal():
            return TransitionCallbackReturn.FAILURE
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _state: LifecycleState) -> TransitionCallbackReturn:
        """Special function: Release configured resources during shutdown."""
        self.skill_active = False
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def release_resources(self) -> None:
        """Special function: Destroy every resource allocated by configure."""
        if self.action_server is not None:
            self.action_server.destroy()
            self.action_server = None
        if self.appearance_sub is not None:
            self.destroy_subscription(self.appearance_sub)
            self.appearance_sub = None
        if self.memory_client is not None:
            self.destroy_client(self.memory_client)
            self.memory_client = None
        self.observation_history.clear()
        with self.goal_lock:
            self.goal_admitted = False
            self.active_goal_handle = None

    def appearance_callback(self, message: TrackedPersonAppearanceArray) -> None:
        """Special function: Cache frame-consistent observations only while ACTIVE."""
        if not self.skill_active:
            return
        with self.observation_condition:
            self.observation_sequence += 1
            self.observation_history.append((self.observation_sequence, copy.deepcopy(message)))
            self.observation_condition.notify_all()

    def frames_after(self, cursor: int, deadline: float) -> list[tuple[int, Any]]:
        """Special function: Return newly received appearance frames after a bounded wait."""
        with self.observation_condition:
            frames = [item for item in self.observation_history if item[0] > cursor]
            if frames:
                return frames
            remaining = max(0.0, deadline - time.monotonic())
            if remaining > 0.0:
                self.observation_condition.wait(timeout=min(0.10, remaining))
            return [item for item in self.observation_history if item[0] > cursor]

    def call_save(self, request: Any, timeout_sec: float) -> Any:
        """Special function: Perform one mandatory guest-memory service call."""
        if self.memory_client is None or not self.memory_client.service_is_ready():
            return None
        return wait_for_future(self.memory_client.call_async(request), timeout_sec)

    def goal_callback(self, _goal_request: Any) -> GoalResponse:
        """Special function: Reject goals unless lifecycle is ACTIVE and idle."""
        with self.goal_lock:
            if not self.skill_active or self.goal_admitted:
                return GoalResponse.REJECT
            self.goal_admitted = True
            return GoalResponse.ACCEPT

    def has_active_goal(self) -> bool:
        """Special function: Read reserved or executing goal ownership atomically."""
        with self.goal_lock:
            return bool(self.goal_admitted)

    def cancel_callback(self, _goal_handle: Any) -> CancelResponse:
        """Special function: Accept cancellation so the YASMIN wait state can exit."""
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: Any) -> Any:
        """Special function: Run one server-owned find-person YASMIN state machine."""
        with self.goal_lock:
            self.active_goal_handle = goal_handle
        blackboard = Blackboard()
        blackboard["goal_handle"] = goal_handle
        try:
            outcome = FindPersonByAttributeMissionSM(self)(blackboard)
            result = FindPersonByAttribute.Result()
            result.success = outcome == SUCCESS
            result.outcome = outcome
            result.message = str(blackboard_value(blackboard, "message", ""))
            if outcome == SUCCESS:
                record = blackboard["saved_record"]
                result.person_ref_json = json.dumps(blackboard["person_ref"], sort_keys=True)
                result.memory_id = str(record.memory_id)
                result.memory_revision = int(record.revision)
                result.observation_json = str(blackboard["observation_json"])
                goal_handle.succeed()
            elif outcome == CANCELED:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result
        finally:
            with self.goal_lock:
                self.active_goal_handle = None
                self.goal_admitted = False


def profile_info(profile: PersonProfile) -> dict[str, Any]:
    """Special function: Convert known registered profile fields into extraction data."""
    values: dict[str, Any] = {}
    for field_name in ("profile_id", "name", "gender", "favorite_drink", "hometown"):
        value = getattr(profile, field_name)
        if value:
            values[field_name] = value
    if int(profile.age) > 0:
        values["age"] = int(profile.age)
    values.update({key: value for key, value in appearance_dict(profile.appearance).items() if not key.endswith("_confidence") and value != "unknown"})
    return values


def merge_person_profiles(existing: PersonProfile, incoming: PersonProfile) -> PersonProfile:
    """Special function: Refresh known profile fields without erasing earlier values."""
    merged = copy.deepcopy(existing)
    for field_name in (
        "profile_id", "role", "name", "gender", "favorite_drink", "hometown",
    ):
        value = getattr(incoming, field_name)
        if value and str(value).strip().casefold() != "unknown":
            setattr(merged, field_name, value)
    if int(incoming.age) > 0:
        merged.age = int(incoming.age)
    merged.is_confirmed = bool(merged.is_confirmed or incoming.is_confirmed)
    merged.confidence_score = max(
        float(merged.confidence_score),
        float(incoming.confidence_score),
    )
    merged.appearance = copy.deepcopy(incoming.appearance)
    return merged


class ValidateExtractGoalState(State):
    """Special function: Validate extraction reference and requested fields."""

    def __init__(self) -> None:
        """Special function: Declare extraction validation outcomes."""
        super().__init__([READY, INVALID_INPUT, CANCELED])

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Normalize aliases and require one usable guest reference."""
        goal_handle = blackboard["goal_handle"]
        if goal_handle.is_cancel_requested:
            blackboard["message"] = "Guest extraction canceled before validation."
            return CANCELED
        person_ref = parse_json_object(goal_handle.request.person_ref_json)
        if not str(goal_handle.request.memory_id).strip() and not person_ref:
            blackboard["message"] = "Extraction requires memory_id or person_ref_json."
            return INVALID_INPUT
        aliases = {"favourite_drink": "favorite_drink", "shirt_colour": "shirt_color"}
        fields = [aliases.get(str(field).strip().casefold(), str(field).strip().casefold()) for field in goal_handle.request.fields]
        unsupported = sorted(set(fields) - SUPPORTED_INFO_FIELDS)
        if unsupported:
            blackboard["message"] = f"Unsupported guest information fields: {unsupported}."
            return INVALID_INPUT
        blackboard["person_ref"] = person_ref
        blackboard["fields"] = fields
        return READY


class LoadGuestRecordState(State):
    """Special function: Load the service-owned guest record or seed one from PersonRef."""

    def __init__(self, server: "ExtractGuestInfoSkillServer") -> None:
        """Special function: Store the server that owns memory clients."""
        super().__init__([READY, NOT_FOUND, FAILED, CANCELED])
        self.server = server

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Call shared memory before using any local snapshot."""
        goal_handle = blackboard["goal_handle"]
        if goal_handle.is_cancel_requested:
            return CANCELED
        memory_id = str(goal_handle.request.memory_id).strip()
        if memory_id:
            request = GetGpsrGuest.Request()
            request.memory_id = memory_id
            response = self.server.call_get(request, self.server.memory_timeout_sec)
            if response is None:
                blackboard["message"] = "GPSR guest-memory get timed out."
                return FAILED
            if response.success:
                blackboard["record"] = copy.deepcopy(response.record)
                return READY
            if not blackboard["person_ref"]:
                blackboard["message"] = response.message
                return NOT_FOUND
        # Section: A direct PersonRef can seed a new record before the mandatory save.
        person_ref = blackboard["person_ref"]
        record = GpsrGuestRecord()
        record.memory_id = memory_id
        record.command_id = str(goal_handle.request.command_id)
        record.source_step_id = str(goal_handle.request.source_step_id)
        record.tracking_session_id = str(person_ref.get("tracking_session_id", ""))
        record.track_id = int(person_ref.get("track_id", 0) or 0)
        record.person_ref_json = json.dumps(person_ref, sort_keys=True)
        blackboard["record"] = record
        return READY


class MergeGuestInfoState(State):
    """Special function: Merge saved, live, and profile values into requested information."""

    def __init__(self, server: "ExtractGuestInfoSkillServer") -> None:
        """Special function: Store the server's live observation cache."""
        super().__init__([READY, CANCELED])
        self.server = server

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Prefer fresh observable values and registered semantic fields."""
        goal_handle = blackboard["goal_handle"]
        if goal_handle.is_cancel_requested:
            return CANCELED
        record = blackboard["record"]
        # Section: The latest accepted action step becomes the record's update provenance.
        record.command_id = str(goal_handle.request.command_id or record.command_id)
        record.source_step_id = str(goal_handle.request.source_step_id or record.source_step_id)
        person_ref = parse_json_object(record.person_ref_json) or blackboard["person_ref"]
        info = parse_json_object(record.guest_info_json)
        info.update(profile_info(record.profile))
        # Section: Saved PersonRef remains valid across lifecycle cleanup and track resets.
        appearance = person_ref.get("appearance", {})
        if isinstance(appearance, dict):
            info.update({key: value for key, value in appearance.items() if not key.endswith("_confidence") and value not in {"", "unknown", None}})
        for field_name in ("pose", "gesture", "profile_id", "person_id", "track_id"):
            value = person_ref.get(field_name)
            if value not in {None, "", "unknown"}:
                info[field_name] = value
        live = blackboard_value(blackboard, "live_observation")
        if live is not None:
            live_data = observation_dict(live)
            info.update({key: value for key, value in live_data["appearance"].items() if not key.endswith("_confidence") and value != "unknown"})
            for field_name in ("pose", "gesture", "profile_id", "person_id", "track_id"):
                value = live_data.get(field_name)
                if value not in {None, "", "unknown"}:
                    info[field_name] = value
            record.profile = merge_person_profiles(record.profile, live.profile)
            record.person_ref_json = json.dumps(live_data, sort_keys=True)
        fields = list(blackboard["fields"])
        requested = fields or sorted(SUPPORTED_INFO_FIELDS)
        missing = [field for field in requested if info.get(field) in {None, "", "unknown", 0}]
        blackboard["info"] = info
        blackboard["missing"] = missing
        blackboard["final_outcome"] = NOT_FOUND if fields and missing else SUCCESS if info else NOT_FOUND
        record.guest_info_json = json.dumps(info, sort_keys=True)
        blackboard["record"] = record
        return READY


class RefreshLiveObservationState(State):
    """Special function: Refresh a referenced track without requiring it to survive cleanup."""

    def __init__(self, server: "ExtractGuestInfoSkillServer") -> None:
        """Special function: Retain access to the lifecycle server's fresh track cache."""
        super().__init__([READY, CANCELED])
        self.server = server

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Cache an exact live match or continue with the saved snapshot."""
        goal_handle = blackboard["goal_handle"]
        if goal_handle.is_cancel_requested:
            return CANCELED
        record = blackboard["record"]
        person_ref = parse_json_object(record.person_ref_json) or blackboard["person_ref"]
        blackboard["live_observation"] = self.server.latest_matching(person_ref)
        return READY


class LookupRegisteredProfileState(State):
    """Special function: Retrieve current registered fields through a lifecycle ROS service."""

    def __init__(self, server: "ExtractGuestInfoSkillServer") -> None:
        """Special function: Retain the server that owns the profile service client."""
        super().__init__([READY, CANCELED])
        self.server = server

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Merge a known profile when lookup is available and succeeds."""
        goal_handle = blackboard["goal_handle"]
        if goal_handle.is_cancel_requested:
            return CANCELED
        record = blackboard["record"]
        live = blackboard_value(blackboard, "live_observation")
        profile_id = str(
            (live.profile.profile_id if live is not None else "")
            or record.profile.profile_id
            or parse_json_object(record.person_ref_json).get("profile_id", "")
        )
        if not profile_id:
            return READY
        request = GetProfile.Request()
        request.profile_id = profile_id
        response = self.server.call_profile(request, self.server.memory_timeout_sec)
        if response is not None and response.success:
            # Section: Preserve live appearance while refreshing semantic profile fields.
            response.profile.appearance = copy.deepcopy(record.profile.appearance)
            record.profile = merge_person_profiles(record.profile, response.profile)
            blackboard["record"] = record
        return READY


class SaveExtractedGuestState(State):
    """Special function: Persist enriched guest information before returning an outcome."""

    def __init__(self, server: "ExtractGuestInfoSkillServer") -> None:
        """Special function: Store the mandatory guest-memory save client owner."""
        super().__init__([COMPLETE, FAILED, CANCELED])
        self.server = server

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Save enriched data and retain the authoritative revision."""
        goal_handle = blackboard["goal_handle"]
        if goal_handle.is_cancel_requested:
            return CANCELED
        request = SaveGpsrGuest.Request()
        request.record = copy.deepcopy(blackboard["record"])
        response = self.server.call_save(request, self.server.memory_timeout_sec)
        if response is None or not response.success:
            blackboard["message"] = response.message if response is not None else "GPSR guest-memory save timed out."
            return FAILED
        blackboard["saved_record"] = copy.deepcopy(response.record)
        blackboard["message"] = (
            "Requested guest information extracted and stored."
            if blackboard["final_outcome"] == SUCCESS
            else f"Guest information is incomplete; missing {blackboard['missing']}."
        )
        return COMPLETE


class ExtractGuestInfoMissionSM(StateMachine):
    """Special function: Own guest-memory retrieval, merge, and persistence inside the server."""

    def __init__(self, server: "ExtractGuestInfoSkillServer") -> None:
        """Special function: Build the complete lifecycle extraction graph."""
        super().__init__(outcomes=[COMPLETE, INVALID_INPUT, NOT_FOUND, CANCELED, FAILED])
        self.set_name("EXTRACT_PERSON_INFO_SERVER_SM")
        self.add_state("VALIDATE_REFERENCE", ValidateExtractGoalState(), transitions={READY: "GET_GUEST_MEMORY", INVALID_INPUT: INVALID_INPUT, CANCELED: CANCELED})
        self.add_state("GET_GUEST_MEMORY", LoadGuestRecordState(server), transitions={READY: "REFRESH_LIVE_OBSERVATION", NOT_FOUND: NOT_FOUND, FAILED: FAILED, CANCELED: CANCELED})
        self.add_state("REFRESH_LIVE_OBSERVATION", RefreshLiveObservationState(server), transitions={READY: "LOOKUP_REGISTERED_PROFILE", CANCELED: CANCELED})
        self.add_state("LOOKUP_REGISTERED_PROFILE", LookupRegisteredProfileState(server), transitions={READY: "MERGE_REQUESTED_FIELDS", CANCELED: CANCELED})
        self.add_state("MERGE_REQUESTED_FIELDS", MergeGuestInfoState(server), transitions={READY: "SAVE_GUEST_MEMORY", CANCELED: CANCELED})
        self.add_state("SAVE_GUEST_MEMORY", SaveExtractedGuestState(server), transitions={COMPLETE: COMPLETE, FAILED: FAILED, CANCELED: CANCELED})


class ExtractGuestInfoSkillServer(LifecycleNode):
    """Special function: Expose lifecycle-owned guest information extraction."""

    def __init__(self) -> None:
        """Special function: Declare extraction interfaces without allocating resources."""
        super().__init__("extract_person_info_skill_server")
        self.cb_group = ReentrantCallbackGroup()
        self.declare_parameter("action_name", "/gpsr/extract_guest_info")
        self.declare_parameter("appearance_topic", "/appearance/tracked_people")
        self.declare_parameter("memory_get_service", "/gpsr/guest_memory/get")
        self.declare_parameter("memory_save_service", "/gpsr/guest_memory/save")
        self.declare_parameter("profile_get_service", "/gpsr/identity/get_profile")
        self.declare_parameter("memory_timeout_sec", 2.0)
        self.declare_parameter("live_stale_timeout_sec", 1.0)
        self.skill_active = False
        self.memory_timeout_sec = 2.0
        self.live_stale_timeout_sec = 1.0
        self.latest_people: dict[str, tuple[TrackedPersonAppearance, float]] = {}
        self.appearance_sub = None
        self.get_client = None
        self.save_client = None
        self.profile_client = None
        self.action_server = None
        # Section: A boolean reservation supports unhashable ROS goal-handle test doubles.
        self.goal_lock = threading.Lock()
        self.goal_admitted = False
        self.active_goal_handle = None

    def on_configure(self, _state: LifecycleState) -> TransitionCallbackReturn:
        """Special function: Allocate live perception, memory clients, and action server."""
        try:
            self.memory_timeout_sec = max(0.1, float(self.get_parameter("memory_timeout_sec").value))
            self.live_stale_timeout_sec = max(0.1, float(self.get_parameter("live_stale_timeout_sec").value))
            self.latest_people = {}
            self.appearance_sub = self.create_subscription(
                TrackedPersonAppearanceArray,
                str(self.get_parameter("appearance_topic").value),
                self.appearance_callback,
                10,
                callback_group=self.cb_group,
            )
            self.get_client = self.create_client(GetGpsrGuest, str(self.get_parameter("memory_get_service").value), callback_group=self.cb_group)
            self.save_client = self.create_client(SaveGpsrGuest, str(self.get_parameter("memory_save_service").value), callback_group=self.cb_group)
            self.profile_client = self.create_client(
                GetProfile,
                str(self.get_parameter("profile_get_service").value),
                callback_group=self.cb_group,
            )
            if not self.get_client.wait_for_service(timeout_sec=self.memory_timeout_sec):
                raise RuntimeError("GPSR guest-memory get service is unavailable")
            if not self.save_client.wait_for_service(timeout_sec=self.memory_timeout_sec):
                raise RuntimeError("GPSR guest-memory save service is unavailable")
            self.action_server = ActionServer(
                self,
                ExtractGuestInfo,
                str(self.get_parameter("action_name").value),
                execute_callback=self.execute_callback,
                goal_callback=self.goal_callback,
                cancel_callback=self.cancel_callback,
                callback_group=self.cb_group,
            )
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Extract-person-info configure failed: {exc}")
            self.release_resources()
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, _state: LifecycleState) -> TransitionCallbackReturn:
        """Special function: Admit extraction goals only while ACTIVE."""
        self.skill_active = True
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, _state: LifecycleState) -> TransitionCallbackReturn:
        """Special function: Stop extraction admission before cleanup."""
        self.skill_active = False
        return TransitionCallbackReturn.SUCCESS if not self.has_active_goal() else TransitionCallbackReturn.FAILURE

    def on_cleanup(self, _state: LifecycleState) -> TransitionCallbackReturn:
        """Special function: Release clients, subscriptions, actions, and live caches."""
        if self.has_active_goal():
            return TransitionCallbackReturn.FAILURE
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _state: LifecycleState) -> TransitionCallbackReturn:
        """Special function: Release configured resources during shutdown."""
        self.skill_active = False
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def release_resources(self) -> None:
        """Special function: Destroy every extraction resource allocated by configure."""
        if self.action_server is not None:
            self.action_server.destroy()
            self.action_server = None
        if self.appearance_sub is not None:
            self.destroy_subscription(self.appearance_sub)
            self.appearance_sub = None
        for name in ("get_client", "save_client", "profile_client"):
            client = getattr(self, name)
            if client is not None:
                self.destroy_client(client)
                setattr(self, name, None)
        self.latest_people = {}
        with self.goal_lock:
            self.goal_admitted = False
            self.active_goal_handle = None

    def appearance_callback(self, message: TrackedPersonAppearanceArray) -> None:
        """Special function: Cache fresh tracked observations for optional extraction refresh."""
        if not self.skill_active:
            return
        received_at = time.monotonic()
        for person in message.people:
            key = f"{person.tracking_session_id}:{int(person.track_id)}"
            self.latest_people[key] = (copy.deepcopy(person), received_at)

    def latest_matching(self, person_ref: dict[str, Any]) -> TrackedPersonAppearance | None:
        """Special function: Return only the exact fresh canonical track referenced by memory."""
        key = f"{person_ref.get('tracking_session_id', '')}:{int(person_ref.get('track_id', 0) or 0)}"
        item = self.latest_people.get(key)
        if item is None or time.monotonic() - item[1] > self.live_stale_timeout_sec:
            return None
        return copy.deepcopy(item[0])

    def call_get(self, request: Any, timeout_sec: float) -> Any:
        """Special function: Retrieve one guest record through the shared memory service."""
        if self.get_client is None or not self.get_client.service_is_ready():
            return None
        return wait_for_future(self.get_client.call_async(request), timeout_sec)

    def call_save(self, request: Any, timeout_sec: float) -> Any:
        """Special function: Persist one enriched record through the shared memory service."""
        if self.save_client is None or not self.save_client.service_is_ready():
            return None
        return wait_for_future(self.save_client.call_async(request), timeout_sec)

    def call_profile(self, request: Any, timeout_sec: float) -> Any:
        """Special function: Retrieve registered fields without making lookup mandatory."""
        if self.profile_client is None or not self.profile_client.service_is_ready():
            return None
        return wait_for_future(self.profile_client.call_async(request), timeout_sec)

    def goal_callback(self, _goal_request: Any) -> GoalResponse:
        """Special function: Reject extraction unless lifecycle is ACTIVE and idle."""
        with self.goal_lock:
            if not self.skill_active or self.goal_admitted:
                return GoalResponse.REJECT
            self.goal_admitted = True
            return GoalResponse.ACCEPT

    def has_active_goal(self) -> bool:
        """Special function: Read reserved or executing extraction ownership atomically."""
        with self.goal_lock:
            return bool(self.goal_admitted)

    def cancel_callback(self, _goal_handle: Any) -> CancelResponse:
        """Special function: Accept cancellation for state-machine polling."""
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: Any) -> Any:
        """Special function: Run one server-owned extraction YASMIN state machine."""
        with self.goal_lock:
            self.active_goal_handle = goal_handle
        blackboard = Blackboard()
        blackboard["goal_handle"] = goal_handle
        try:
            sm_outcome = ExtractGuestInfoMissionSM(self)(blackboard)
            outcome = blackboard_value(blackboard, "final_outcome", sm_outcome) if sm_outcome == COMPLETE else sm_outcome
            result = ExtractGuestInfo.Result()
            result.success = outcome == SUCCESS
            result.outcome = str(outcome)
            result.message = str(blackboard_value(blackboard, "message", ""))
            result.info_json = json.dumps(blackboard_value(blackboard, "info", {}), sort_keys=True)
            result.missing_fields = list(blackboard_value(blackboard, "missing", []))
            if "saved_record" in blackboard:
                record = blackboard["saved_record"]
                result.memory_id = str(record.memory_id)
                result.memory_revision = int(record.revision)
            if outcome == SUCCESS:
                goal_handle.succeed()
            elif outcome == CANCELED:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result
        finally:
            with self.goal_lock:
                self.active_goal_handle = None
                self.goal_admitted = False


def run_server(node_type: Any, args: list[str] | None = None) -> None:
    """Special function: Run one lifecycle skill server with concurrent callbacks."""
    rclpy.init(args=args)
    node = node_type()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def main_find(args: list[str] | None = None) -> None:
    """Special function: Run the find-person-by-attribute lifecycle server."""
    run_server(FindPersonByAttributeSkillServer, args)


def main_extract(args: list[str] | None = None) -> None:
    """Special function: Run the extract-person-info lifecycle server."""
    run_server(ExtractGuestInfoSkillServer, args)
