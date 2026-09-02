"""Section: Read-only, structured GPSR action queue execution status."""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import Any

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import normalize_action_step
from robot_yasmin_tasks.gpsr.outcomes import CANCELED, SUCCESS, is_recoverable


# Section: Status callback contract shared by executor states and the ROS wrapper.
QueueStatusCallback = Callable[[Any, str, str], None]


def _attempt_for(blackboard: Any, index: int, action: Any) -> int:
    """Special function: Resolve the executor's retry counter for one queue item."""
    attempts = dict(bb_get(blackboard, bb.RECOVERY_ATTEMPTS, {}))
    key = action.step_id or f"{index}:{action.action_type}"
    return int(attempts.get(key, 0) or 0)


def _action_to_dict(blackboard: Any, index: int, raw_action: Any) -> dict[str, Any]:
    """Special function: Preserve every structured ActionStep field for read-only clients."""
    action = normalize_action_step(raw_action)
    return {
        "index": index,
        "step_id": action.step_id,
        "action_type": action.action_type,
        "task_id": action.task_id,
        "task_family": action.task_family,
        "target_object": action.target_object,
        "target_person": action.target_person,
        "target_location": action.target_location,
        "target_category": action.target_category,
        "parameters": dict(action.parameters),
        "timeout_sec": action.timeout_sec,
        "max_retries": action.max_retries,
        "attempt": _attempt_for(blackboard, index, action),
        "requires_success": list(action.requires_success),
        "requires_data": list(action.requires_data),
        "result_variable": action.result_variable,
    }


def build_queue_status(
    blackboard: Any,
    phase: str,
    outcome: str = "",
    *,
    timestamp: float | None = None,
) -> dict[str, Any]:
    """Special function: Build an authoritative snapshot from executor blackboard state."""
    # Section: Read queue state once so every item is classified from one consistent cursor.
    queue = list(bb_get(blackboard, bb.ACTION_QUEUE, []))
    current_index = int(bb_get(blackboard, bb.CURRENT_ACTION_INDEX, 0) or 0)
    normalized_phase = str(phase or "UNKNOWN").upper()
    normalized_outcome = str(outcome or "")
    last_result = dict(bb_get(blackboard, bb.LAST_SKILL_RESULT, {}) or {})
    skill_results = list(bb_get(blackboard, bb.SKILL_RESULTS, []) or [])
    dispatch_outcome = str(bb_get(blackboard, bb.DISPATCH_OUTCOME, normalized_outcome) or "")
    failure_reason = str(bb_get(blackboard, bb.FAILURE_REASON, "") or "")
    status_message = str(last_result.get("message", "")) or failure_reason
    if normalized_phase == "QUEUE_FINISHED" and normalized_outcome != SUCCESS and skill_results:
        # Section: Keep the original client failure visible when recovery reports only exhaustion.
        execution_message = str(dict(skill_results[-1]).get("message", ""))
        if execution_message and execution_message != status_message:
            status_message = f"{execution_message}; recovery: {status_message}" if status_message else execution_message

    # Section: Classify the overall queue without inferring state from human language or logs.
    if normalized_phase == "QUEUE_FINISHED":
        if normalized_outcome == SUCCESS:
            queue_state = "DONE"
        elif normalized_outcome == CANCELED:
            queue_state = "CANCELED"
        else:
            queue_state = "FAILED"
    elif not queue:
        queue_state = "DONE"
    elif normalized_phase == "ACTION_RETRYING" or is_recoverable(dispatch_outcome):
        queue_state = "RETRYING"
    else:
        queue_state = "RUNNING"

    # Section: Derive item states only from the executor cursor, outcome, and recovery counters.
    items: list[dict[str, Any]] = []
    for index, raw_action in enumerate(queue):
        item = _action_to_dict(blackboard, index, raw_action)
        item_state = "DONE" if index < current_index else "PENDING"
        action_is_active = not (
            normalized_phase == "ACTION_FINISHED" and dispatch_outcome == SUCCESS
        )
        if normalized_phase == "QUEUE_FINISHED":
            if normalized_outcome == SUCCESS:
                item_state = "DONE"
            elif normalized_outcome == CANCELED and index >= current_index:
                item_state = "CANCELED"
            elif index == current_index:
                item_state = "FAILED"
        elif index == current_index and index < len(queue) and action_is_active:
            if normalized_phase == "ACTION_RETRYING" or is_recoverable(dispatch_outcome):
                item_state = "RETRYING"
            elif normalized_phase in {"ACTION_SELECTED", "ACTION_FINISHED"}:
                if dispatch_outcome == CANCELED:
                    item_state = "CANCELED"
                elif normalized_phase == "ACTION_FINISHED" and dispatch_outcome not in {"", SUCCESS}:
                    item_state = "FAILED"
                else:
                    item_state = "RUNNING"
        item["state"] = item_state
        item["raw_state"] = dispatch_outcome if index == current_index else item_state.lower()
        item["current"] = (
            index == current_index
            and current_index < len(queue)
            and action_is_active
            and not (normalized_phase == "QUEUE_FINISHED" and normalized_outcome == SUCCESS)
        )
        item["message"] = status_message if item["current"] else ""
        items.append(item)

    # Section: Expose the selected action and exact terminal/retry reason when available.
    current_action_type = ""
    if normalized_phase == "ACTION_FINISHED" and dispatch_outcome == SUCCESS:
        current_action_type = str(last_result.get("action_type", ""))
    elif 0 <= current_index < len(items):
        current_action_type = str(items[current_index]["action_type"])
    elif normalized_phase == "ACTION_FINISHED":
        current_action_type = str(last_result.get("action_type", ""))
    current_attempt = int(items[current_index]["attempt"]) if 0 <= current_index < len(items) else 0
    current_limit = int(items[current_index]["max_retries"]) if 0 <= current_index < len(items) else 0
    return {
        "schema_version": 1,
        "timestamp": time.time() if timestamp is None else float(timestamp),
        "task_id": str(bb_get(blackboard, bb.CURRENT_TASK_ID, "")),
        "phase": normalized_phase,
        "queue_state": queue_state,
        "executor_outcome": normalized_outcome,
        "dispatch_outcome": dispatch_outcome,
        "current_index": current_index,
        "queue_length": len(items),
        "current_action_type": current_action_type,
        "retry_attempt": current_attempt,
        "retry_limit": current_limit,
        "message": status_message,
        "skill_results": skill_results,
        "items": items,
    }


def emit_queue_status(
    callback: QueueStatusCallback | None,
    blackboard: Any,
    phase: str,
    outcome: str = "",
) -> None:
    """Special function: Isolate observer failures from safety-critical queue execution."""
    if callback is None:
        return
    try:
        callback(blackboard, phase, outcome)
    except Exception:
        # Section: Monitoring is strictly read-only and must never alter executor outcomes.
        return
