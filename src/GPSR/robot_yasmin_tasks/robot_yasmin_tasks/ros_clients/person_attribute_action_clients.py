"""Section: Thin executor clients for lifecycle-owned person skill servers."""

from __future__ import annotations

import json
import threading
import time
from typing import Any

from rclpy.action import ActionClient

from robot_ai_interfaces.action import ExtractGuestInfo, FindPersonByAttribute
from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import ActionStepData, make_skill_result
from robot_yasmin_tasks.gpsr.outcomes import CANCELED, FAILED, INVALID_INPUT, NOT_FOUND, SUCCESS, TIMEOUT


def wait_for_future(future: Any, timeout_sec: float, cancel_requested: Any = None) -> Any:
    """Special function: Wait for one ROS future while honoring queue cancellation."""
    completed = threading.Event()
    future.add_done_callback(lambda _future: completed.set())
    deadline = time.monotonic() + max(0.1, float(timeout_sec))
    while time.monotonic() < deadline:
        if completed.wait(timeout=0.05):
            return future.result()
        if callable(cancel_requested) and cancel_requested():
            return None
    return future.result() if future.done() else None


def normalized_outcome(raw_outcome: str) -> str:
    """Special function: Restrict remote server outcomes to the shared skill contract."""
    outcome = str(raw_outcome).strip().casefold()
    return outcome if outcome in {SUCCESS, FAILED, INVALID_INPUT, NOT_FOUND, TIMEOUT, CANCELED} else FAILED


class FindPersonByAttributeActionClient:
    """Special function: Forward one ActionStep to the lifecycle find-person server."""

    def __init__(self, node: Any, action_name: str = "/gpsr/find_person_by_attribute") -> None:
        """Special function: Create one persistent ROS action client owned by the executor."""
        self.action_name = str(action_name)
        self.client = ActionClient(node, FindPersonByAttribute, self.action_name)

    def find_person(self, blackboard: Any, action: ActionStepData) -> Any:
        """Special function: Execute attribute search and mirror its memory reference locally."""
        timeout_sec = max(0.1, float(action.timeout_sec or action.parameters.get("timeout_sec", 20.0) or 20.0))
        if not self.client.wait_for_server(timeout_sec=min(timeout_sec, 5.0)):
            return make_skill_result(action.action_type, FAILED, f"Find-person action unavailable: {self.action_name}", recoverable=True)
        selector = action.parameters.get("selector", {})
        goal = FindPersonByAttribute.Goal()
        goal.selector_json = json.dumps(selector, sort_keys=True)
        goal.location = str(action.target_location or action.parameters.get("location", ""))
        goal.command_id = str(action.task_id)
        goal.source_step_id = str(action.step_id)
        goal.timeout_sec = timeout_sec
        goal.confirmation_min_frames = int(action.parameters.get("confirmation_min_frames", 3) or 3)
        cancel_requested = bb_get(blackboard, bb.CANCEL_REQUESTED, None)
        goal_handle = wait_for_future(self.client.send_goal_async(goal), timeout_sec, cancel_requested)
        if goal_handle is None:
            return make_skill_result(action.action_type, CANCELED if callable(cancel_requested) and cancel_requested() else TIMEOUT, "Find-person goal acceptance timed out.", recoverable=True)
        if not goal_handle.accepted:
            return make_skill_result(action.action_type, FAILED, "Find-person goal was rejected.", recoverable=True)
        wrapped = wait_for_future(goal_handle.get_result_async(), timeout_sec, cancel_requested)
        if wrapped is None:
            goal_handle.cancel_goal_async()
            outcome = CANCELED if callable(cancel_requested) and cancel_requested() else TIMEOUT
            return make_skill_result(action.action_type, outcome, "Find-person action did not finish before its deadline.", recoverable=outcome == TIMEOUT, fatal=outcome == CANCELED)
        result = wrapped.result
        outcome = normalized_outcome(result.outcome)
        person_ref = self.parse_json(result.person_ref_json)
        observation = self.parse_json(result.observation_json)
        if person_ref:
            blackboard[bb.PERSON_REF] = person_ref
            blackboard[bb.TARGET_PERSON] = str(person_ref.get("profile_id") or person_ref.get("track_id", ""))
        if result.memory_id:
            blackboard[bb.GPSR_GUEST_MEMORY_ID] = str(result.memory_id)
            blackboard[bb.GPSR_GUEST_MEMORY_REVISION] = int(result.memory_revision)
        payload = {
            "person_ref": person_ref,
            "observation": observation,
            "memory_id": str(result.memory_id),
            "memory_revision": int(result.memory_revision),
        }
        return make_skill_result(
            action.action_type,
            outcome,
            str(result.message),
            payload,
            recoverable=outcome in {FAILED, NOT_FOUND, TIMEOUT},
            fatal=outcome in {CANCELED, INVALID_INPUT},
        )

    @staticmethod
    def parse_json(raw_value: str) -> dict[str, Any]:
        """Special function: Parse one remote JSON result into a blackboard dictionary."""
        try:
            value = json.loads(str(raw_value or "{}"))
        except json.JSONDecodeError:
            return {}
        return value if isinstance(value, dict) else {}


class ExtractGuestInfoActionClient:
    """Special function: Forward extraction to the lifecycle guest-information server."""

    def __init__(self, node: Any, action_name: str = "/gpsr/extract_guest_info") -> None:
        """Special function: Create one persistent extraction action client."""
        self.action_name = str(action_name)
        self.client = ActionClient(node, ExtractGuestInfo, self.action_name)

    def extract_person_info(self, blackboard: Any, action: ActionStepData) -> Any:
        """Special function: Extract requested fields and mirror service-owned memory metadata."""
        timeout_sec = max(0.1, float(action.timeout_sec or action.parameters.get("timeout_sec", 10.0) or 10.0))
        if not self.client.wait_for_server(timeout_sec=min(timeout_sec, 5.0)):
            return make_skill_result(action.action_type, FAILED, f"Extract-guest action unavailable: {self.action_name}", recoverable=True)
        fields = action.parameters.get("fields", [])
        if not fields and action.parameters.get("field"):
            fields = [action.parameters.get("field")]
        person_ref = action.parameters.get("person_ref") or bb_get(blackboard, bb.PERSON_REF, {})
        memory_id = action.parameters.get("memory_id") or bb_get(blackboard, bb.GPSR_GUEST_MEMORY_ID, "")
        goal = ExtractGuestInfo.Goal()
        goal.memory_id = str(memory_id or "")
        goal.person_ref_json = json.dumps(person_ref if isinstance(person_ref, dict) else {}, sort_keys=True)
        goal.command_id = str(action.task_id)
        goal.source_step_id = str(action.step_id)
        goal.fields = [str(field) for field in fields]
        goal.timeout_sec = timeout_sec
        cancel_requested = bb_get(blackboard, bb.CANCEL_REQUESTED, None)
        goal_handle = wait_for_future(self.client.send_goal_async(goal), timeout_sec, cancel_requested)
        if goal_handle is None or not goal_handle.accepted:
            return make_skill_result(action.action_type, FAILED, "Extract-guest goal was rejected or timed out.", recoverable=True)
        wrapped = wait_for_future(goal_handle.get_result_async(), timeout_sec, cancel_requested)
        if wrapped is None:
            goal_handle.cancel_goal_async()
            outcome = CANCELED if callable(cancel_requested) and cancel_requested() else TIMEOUT
            return make_skill_result(action.action_type, outcome, "Extract-guest action timed out.", recoverable=outcome == TIMEOUT, fatal=outcome == CANCELED)
        result = wrapped.result
        outcome = normalized_outcome(result.outcome)
        info = FindPersonByAttributeActionClient.parse_json(result.info_json)
        blackboard[bb.EXTRACTED_PERSON_INFO] = info
        if result.memory_id:
            blackboard[bb.GPSR_GUEST_MEMORY_ID] = str(result.memory_id)
            blackboard[bb.GPSR_GUEST_MEMORY_REVISION] = int(result.memory_revision)
        payload = {
            **info,
            "guest_info": info,
            "memory_id": str(result.memory_id),
            "memory_revision": int(result.memory_revision),
            "missing_fields": list(result.missing_fields),
        }
        return make_skill_result(
            action.action_type,
            outcome,
            str(result.message),
            payload,
            recoverable=outcome in {FAILED, NOT_FOUND, TIMEOUT},
            fatal=outcome in {CANCELED, INVALID_INPUT},
        )
