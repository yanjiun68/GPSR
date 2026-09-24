"""Section: Perception client wrappers with ROS action runtime and deterministic fallback."""

import json
import threading
import time
from typing import Any

from rclpy.action import ActionClient

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import ActionStepData, make_skill_result
from robot_yasmin_tasks.gpsr.outcomes import CANCELED, FAILED, NOT_FOUND, SUCCESS

from robot_ai_interfaces.action import CountObjects, CountPeople


class PerceptionClients:
    """Special function: Wrap object/person perception and counting."""

    def __init__(
        self,
        node: Any | None = None,
        simulate: bool = True,
        count_objects_action_name: str = "/gpsr/count_objects",
        count_people_action_name: str = "/gpsr/count_people",
    ) -> None:
        """Special function: Store optional ROS node, simulation mode, and action clients."""
        self.node = node
        self.simulate = simulate
        self.count_objects_action_name = count_objects_action_name
        self.count_people_action_name = count_people_action_name
        self.count_objects_client = ActionClient(node, CountObjects, count_objects_action_name) if node is not None else None
        self.count_people_client = ActionClient(node, CountPeople, count_people_action_name) if node is not None else None

    def find_object(self, blackboard: Any, action: ActionStepData) -> Any:
        """Special function: Store target object and pose after object search."""
        target = action.target_object or action.target_category
        if action.parameters.get("simulate_not_found", False):
            return make_skill_result(action.action_type, NOT_FOUND, "Object not found.", recoverable=True)
        object_id = action.parameters.get("object_id", target or "object")
        pose = action.parameters.get("object_pose") or bb_get(blackboard, bb.TARGET_POSE, None) or f"simulated_pose:{object_id}"
        memory = dict(bb_get(blackboard, bb.ENTITY_MEMORY, {}))
        memory[object_id] = {"type": "object", "pose": pose}
        blackboard[bb.ENTITY_MEMORY] = memory
        blackboard[bb.TARGET_OBJECT] = object_id
        blackboard[bb.TARGET_POSE] = pose
        return make_skill_result(action.action_type, SUCCESS, "Object found.", {"object_id": object_id})

    def find_person(self, blackboard: Any, action: ActionStepData) -> Any:
        """Special function: Store target person after person search."""
        person_id = action.target_person or action.parameters.get("description", "person")
        blackboard[bb.TARGET_PERSON] = person_id
        return make_skill_result(action.action_type, SUCCESS, "Person found.", {"person_id": person_id})

    def count_objects(self, blackboard: Any, action: ActionStepData) -> Any:
        """Special function: Trigger CountObjects action and store the counted entity result."""
        if self.node is None or self.count_objects_client is None or self.simulate:
            return self.count_objects_simulated(blackboard, action)

        # Section: Wait briefly for the GPSR-facing count action server.
        timeout_sec = float(action.timeout_sec or action.parameters.get("timeout_sec", 5.0) or 5.0)
        if not self.count_objects_client.wait_for_server(timeout_sec=timeout_sec):
            return make_skill_result(action.action_type, FAILED, f"CountObjects action unavailable: {self.count_objects_action_name}", recoverable=True)

        # Section: Build action goal from normalized GPSR action fields.
        goal = CountObjects.Goal()
        goal.category = str(action.target_category or action.target_object or action.parameters.get("category", "")).strip()
        goal.location = str(action.target_location or action.parameters.get("location", "")).strip()
        goal.filter_json = self.build_filter_json(action)
        goal.timeout_sec = timeout_sec

        # Section: Send the action and wait without taking ownership of the executor.
        cancel_requested = lambda: self.is_cancel_requested(blackboard)
        goal_handle = self.wait_for_future(
            self.count_objects_client.send_goal_async(goal),
            timeout_sec,
            cancel_requested,
        )
        if self.is_cancel_requested(blackboard):
            return make_skill_result(action.action_type, CANCELED, "CountObjects canceled before goal acceptance.", fatal=True)
        if goal_handle is None or not goal_handle.accepted:
            return make_skill_result(action.action_type, FAILED, "CountObjects action goal was rejected or timed out.", recoverable=True)

        # Section: Wait for the action result and copy it into blackboard-friendly form.
        result_future = goal_handle.get_result_async()
        wrapped_result = self.wait_for_future(result_future, timeout_sec, cancel_requested)
        if self.is_cancel_requested(blackboard):
            goal_handle.cancel_goal_async()
            return make_skill_result(action.action_type, CANCELED, "CountObjects action canceled.", fatal=True)
        if wrapped_result is None:
            return make_skill_result(action.action_type, FAILED, "CountObjects action result timed out.", recoverable=True)

        result = wrapped_result.result
        result_payload = self.parse_result_json(result.result_json)
        blackboard[bb.COUNT_RESULT] = int(result.count)
        blackboard[bb.COUNT_OBJECTS_RESULT_JSON] = str(result.result_json)
        blackboard[bb.OBJECT_QUANTITIES] = dict(result_payload.get("object_quantities", {}))
        blackboard[bb.DETECTED_OBJECTS] = list(result_payload.get("all_detected_objects", []))
        if bool(result.success):
            return make_skill_result(
                action.action_type,
                SUCCESS,
                result.message,
                {
                    "count": int(result.count),
                    "object_quantities": dict(result_payload.get("object_quantities", {})),
                    "detected_objects": list(result_payload.get("all_detected_objects", [])),
                    "result_json": result_payload,
                },
            )
        return make_skill_result(
            action.action_type,
            FAILED,
            result.message,
            {
                "count": int(result.count),
                "object_quantities": dict(result_payload.get("object_quantities", {})),
                "detected_objects": list(result_payload.get("all_detected_objects", [])),
                "result_json": result_payload,
            },
            recoverable=True,
        )

    def count_objects_simulated(self, blackboard: Any, action: ActionStepData) -> Any:
        """Special function: Keep deterministic unit-test fallback when no ROS node is provided."""
        count = int(action.parameters.get("simulated_count", 0))
        object_quantities = dict(action.parameters.get("simulated_object_quantities", {}))
        detected_objects = list(action.parameters.get("simulated_detected_objects", []))
        blackboard[bb.COUNT_RESULT] = count
        blackboard[bb.OBJECT_QUANTITIES] = object_quantities
        blackboard[bb.DETECTED_OBJECTS] = detected_objects
        return make_skill_result(
            action.action_type,
            SUCCESS,
            "Objects counted.",
            {"count": count, "object_quantities": object_quantities, "detected_objects": detected_objects},
        )

    def build_filter_json(self, action: ActionStepData) -> str:
        """Special function: Build perception filter JSON from GPSR action parameters."""
        if "filter_json" in action.parameters:
            return str(action.parameters.get("filter_json") or "{}")
        # Section: Preserve the scheduler's semantic filter list for Qwen-backed object counting.
        filters: dict[str, Any] = {}
        raw_filters = action.parameters.get("filters", {})
        if isinstance(raw_filters, list):
            filters["filters"] = [item for item in raw_filters if isinstance(item, dict)]
        elif isinstance(raw_filters, dict):
            filters = dict(raw_filters)
        # Section: Legacy count-control fields coexist with typed visual constraints in the same envelope.
        for key in ("min_confidence", "stale_timeout_sec", "enforce_location_filter"):
            if key in action.parameters:
                filters[key] = action.parameters[key]
        return json.dumps(filters, sort_keys=True)

    def parse_result_json(self, raw_result_json: str) -> dict[str, Any]:
        """Special function: Parse action result JSON for blackboard storage."""
        try:
            parsed = json.loads(str(raw_result_json)) if str(raw_result_json).strip() else {}
        except json.JSONDecodeError:
            return {}
        return parsed if isinstance(parsed, dict) else {"value": parsed}

    def wait_for_future(self, future: Any, timeout_sec: float, cancel_requested: Any = None) -> Any:
        """Special function: Wait for a ROS future while polling top-level cancellation."""
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        deadline = time.monotonic() + max(0.1, timeout_sec)
        while time.monotonic() < deadline:
            if done.wait(timeout=0.05):
                return future.result()
            if callable(cancel_requested) and cancel_requested():
                return None
        if not future.done():
            return None
        return future.result()

    @staticmethod
    def is_cancel_requested(blackboard: Any) -> bool:
        """Special function: Read the queue cancellation callback from blackboard state."""
        callback = bb_get(blackboard, bb.CANCEL_REQUESTED, None)
        return bool(callback()) if callable(callback) else False

    def count_people(self, blackboard: Any, action: ActionStepData) -> Any:
        """Special function: Trigger CountPeople action and store the counted people result."""
        if self.node is None or self.count_people_client is None or self.simulate:
            return self.count_people_simulated(blackboard, action)

        # Section: Wait briefly for the GPSR-facing count people action server.
        timeout_sec = float(action.timeout_sec or action.parameters.get("timeout_sec", 5.0) or 5.0)
        if not self.count_people_client.wait_for_server(timeout_sec=timeout_sec):
            return make_skill_result(action.action_type, FAILED, f"CountPeople action unavailable: {self.count_people_action_name}", recoverable=True)

        # Section: Build action goal from normalized GPSR action fields.
        goal = CountPeople.Goal()
        goal.location = str(action.target_location or action.parameters.get("location", "")).strip()
        goal.filter_json = self.build_filter_json(action)
        goal.timeout_sec = timeout_sec

        # Section: Send the action and wait without taking ownership of the executor.
        goal_handle = self.wait_for_future(self.count_people_client.send_goal_async(goal), timeout_sec)
        if goal_handle is None or not goal_handle.accepted:
            return make_skill_result(action.action_type, FAILED, "CountPeople action goal was rejected or timed out.", recoverable=True)

        # Section: Wait for the action result and copy it into blackboard-friendly form.
        wrapped_result = self.wait_for_future(goal_handle.get_result_async(), timeout_sec)
        if wrapped_result is None:
            return make_skill_result(action.action_type, FAILED, "CountPeople action result timed out.", recoverable=True)

        result = wrapped_result.result
        result_payload = self.parse_result_json(result.result_json)
        detected_people = list(result_payload.get("human_entities", []))
        blackboard[bb.COUNT_RESULT] = int(result.count)
        blackboard[bb.COUNT_PEOPLE_RESULT_JSON] = str(result.result_json)
        blackboard[bb.DETECTED_PEOPLE] = detected_people
        if bool(result.success):
            return make_skill_result(
                action.action_type,
                SUCCESS,
                result.message,
                {
                    "count": int(result.count),
                    "detected_people": detected_people,
                    "result_json": result_payload,
                },
            )
        return make_skill_result(
            action.action_type,
            FAILED,
            result.message,
            {
                "count": int(result.count),
                "detected_people": detected_people,
                "result_json": result_payload,
            },
            recoverable=True,
        )

    def count_people_simulated(self, blackboard: Any, action: ActionStepData) -> Any:
        """Special function: Keep deterministic people-count fallback when no ROS node is provided."""
        count = int(action.parameters.get("simulated_count", 0))
        detected_people = list(action.parameters.get("simulated_detected_people", []))
        result_json = dict(action.parameters.get("simulated_result_json", {}))
        blackboard[bb.COUNT_RESULT] = count
        blackboard[bb.COUNT_PEOPLE_RESULT_JSON] = json.dumps(result_json, sort_keys=True)
        blackboard[bb.DETECTED_PEOPLE] = detected_people
        return make_skill_result(
            action.action_type,
            SUCCESS,
            "People counted.",
            {"count": count, "detected_people": detected_people, "result_json": result_json},
        )

    def compare_objects(self, blackboard: Any, action: ActionStepData) -> Any:
        """Special function: Store selected object from comparison."""
        selected = action.parameters.get("selected_object", action.target_object or "selected_object")
        blackboard[bb.SELECTED_OBJECT] = selected
        blackboard[bb.TARGET_OBJECT] = selected
        return make_skill_result(action.action_type, SUCCESS, "Objects compared.", {"selected_object": selected})

    def extract_person_info(self, blackboard: Any, action: ActionStepData) -> Any:
        """Special function: Store extracted person information."""
        field = action.parameters.get("field", "name")
        value = action.parameters.get("value", action.target_person)
        blackboard[bb.EXTRACTED_PERSON_INFO] = {field: value}
        return make_skill_result(action.action_type, SUCCESS, "Person info extracted.", {field: value})
