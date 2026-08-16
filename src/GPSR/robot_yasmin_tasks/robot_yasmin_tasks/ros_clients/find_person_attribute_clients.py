"""Section: Real pose-summary adapter for attribute-based person search."""

from __future__ import annotations

import json
import threading
import time
from collections import deque
from typing import Any

from std_msgs.msg import String

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import ActionStepData, make_skill_result
from robot_yasmin_tasks.gpsr.outcomes import CANCELED, FAILED, NOT_FOUND, SUCCESS, TIMEOUT


class FindPersonAttributeClients:
    """Special function: Match stable person attributes from real MediaPipe pose summaries."""

    def __init__(
        self,
        node: Any | None = None,
        pose_summary_topic: str = "/perception/human_poses",
    ) -> None:
        """Special function: Subscribe once and retain a bounded sequence of pose observations."""
        # Section: Persistent executor ownership avoids creating subscriptions inside skill calls.
        self.node = node
        self.pose_summary_topic = str(pose_summary_topic)
        self.condition = threading.Condition()
        self.sequence = 0
        self.history: deque[tuple[int, dict[str, Any]]] = deque(maxlen=120)
        self.subscription = (
            node.create_subscription(String, self.pose_summary_topic, self._pose_summary_cb, 10)
            if node is not None
            else None
        )

    def _pose_summary_cb(self, message: String) -> None:
        """Special function: Cache one valid pose-summary frame and wake active searches."""
        try:
            payload = json.loads(str(message.data))
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(payload, dict):
            return
        with self.condition:
            self.sequence += 1
            self.history.append((self.sequence, payload))
            self.condition.notify_all()

    @staticmethod
    def _is_canceled(blackboard: Any) -> bool:
        """Special function: Read queue cancellation without owning the action goal."""
        callback = bb_get(blackboard, bb.CANCEL_REQUESTED, None)
        return bool(callback()) if callable(callback) else False

    def find_person(self, blackboard: Any, action: ActionStepData) -> Any:
        """Special function: Wait for one temporally confirmed person matching the selector."""
        if self.node is None or self.subscription is None:
            return make_skill_result(
                action.action_type,
                FAILED,
                "Attribute person perception interface is unavailable.",
                recoverable=True,
            )

        # Section: Use only frames received after this skill owns its lifecycle resources.
        with self.condition:
            cursor = self.sequence
        selector = dict(action.parameters.get("selector", {}))
        required_frames = max(1, int(action.parameters.get("confirmation_min_frames", 3) or 3))
        timeout_sec = max(0.05, float(action.timeout_sec or action.parameters.get("timeout_sec", 20.0) or 20.0))
        deadline = time.monotonic() + timeout_sec
        confirmations: dict[str, int] = {}
        saw_person = False

        # Section: Match repeated tracked-person evidence until success, cancellation, or timeout.
        while time.monotonic() < deadline:
            if self._is_canceled(blackboard):
                return make_skill_result(action.action_type, CANCELED, "Attribute person search canceled.", fatal=True)
            frames = self._frames_after(cursor, deadline)
            if not frames:
                continue
            for sequence, payload in frames:
                cursor = max(cursor, sequence)
                people = payload.get("people", [])
                if isinstance(people, list) and people:
                    saw_person = True
                matched_ids: set[str] = set()
                for person in people if isinstance(people, list) else []:
                    if not isinstance(person, dict) or not self._matches(person, selector):
                        continue
                    person_id = str(person.get("person_id", "")).strip() or "untracked_person"
                    matched_ids.add(person_id)
                    confirmations[person_id] = int(confirmations.get(person_id, 0)) + 1
                    if confirmations[person_id] >= required_frames:
                        return self._success_result(blackboard, action, selector, person)
                # Section: Require consecutive evidence for each candidate to reject flicker.
                for person_id in list(confirmations):
                    if person_id not in matched_ids:
                        confirmations[person_id] = 0

        if saw_person:
            return make_skill_result(
                action.action_type,
                NOT_FOUND,
                "People were detected, but none matched the requested attribute.",
                {"selector": selector},
                recoverable=True,
            )
        return make_skill_result(
            action.action_type,
            TIMEOUT,
            "No fresh person pose observations arrived before the deadline.",
            {"selector": selector, "topic": self.pose_summary_topic},
            recoverable=True,
        )

    def _frames_after(self, cursor: int, deadline: float) -> list[tuple[int, dict[str, Any]]]:
        """Special function: Return newly received frames after a bounded condition wait."""
        with self.condition:
            frames = [(sequence, payload) for sequence, payload in self.history if sequence > cursor]
            if frames:
                return frames
            remaining = max(0.0, deadline - time.monotonic())
            if remaining > 0.0:
                self.condition.wait(timeout=min(0.10, remaining))
            return [(sequence, payload) for sequence, payload in self.history if sequence > cursor]

    def _matches(self, person: dict[str, Any], selector: dict[str, Any]) -> bool:
        """Special function: Compare one normalized pose-summary person with one selector."""
        selector_type = str(selector.get("type", "")).strip().casefold()
        value = str(selector.get("value", "")).strip().casefold()
        status = str(person.get("status", "")).strip().upper()
        if status not in {"DETECTED", "UNRELIABLE"}:
            return False
        if selector_type == "any_person":
            return status == "DETECTED"
        if selector_type == "pose":
            expected_pose = "lying_down" if value == "lying" else value
            return status == "DETECTED" and str(person.get("pose_label", "")).strip().casefold() == expected_pose
        if selector_type != "gesture":
            return False
        gesture = str(person.get("gesture", "")).strip().casefold()
        raised_state = str(person.get("raised_arm_state", "")).strip().upper()
        wave_state = str(person.get("wave_state", "")).strip().upper()
        if value == "right_hand_raised":
            return gesture == "raising_right_arm" or raised_state in {"RIGHT_ARM_RAISED", "BOTH_ARMS_RAISED"}
        if value == "left_hand_raised":
            return gesture == "raising_left_arm" or raised_state in {"LEFT_ARM_RAISED", "BOTH_ARMS_RAISED"}
        if value == "waving":
            return gesture == "waving" or wave_state in {"LEFT_ARM_WAVING", "RIGHT_ARM_WAVING", "BOTH_ARMS_WAVING"}
        return False

    def _success_result(
        self,
        blackboard: Any,
        action: ActionStepData,
        selector: dict[str, Any],
        person: dict[str, Any],
    ) -> Any:
        """Special function: Commit a normalized attribute-backed person reference."""
        # Section: Gesture confidence is authoritative when present; pose status otherwise uses one.
        confidence = float(person.get("gesture_confidence", 0.0) or 0.0)
        if str(selector.get("type", "")) in {"pose", "any_person"}:
            confidence = max(confidence, 1.0 if str(person.get("status", "")).upper() == "DETECTED" else 0.0)
        person_ref = {
            "person_id": str(person.get("person_id", "untracked_person")),
            "tracking_id": person.get("person_id"),
            "location": str(action.target_location or action.parameters.get("location", "")),
            "selector_type": str(selector.get("type", "")),
            "selector_value": str(selector.get("value", "")),
            "pose": str(person.get("pose_label", "")),
            "gesture": str(person.get("gesture", "")),
            "confidence": confidence,
            "source": "POSE_ATTRIBUTE",
        }
        blackboard[bb.PERSON_REF] = person_ref
        blackboard[bb.TARGET_PERSON] = person_ref["person_id"]
        return make_skill_result(
            action.action_type,
            SUCCESS,
            "Person matching the requested attribute found.",
            {"person_ref": person_ref, "observation": dict(person)},
        )
