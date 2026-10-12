"""Section: Structured GPSR command and action queue monitoring."""

from __future__ import annotations

import json
from typing import Any, Mapping

from ..event_model import MonitorEvent


# Section: Canonical primitive domains are metadata, never natural-language inference.
ACTION_DOMAINS = {
    "NAVIGATE": "NAVIGATION",
    "FOLLOW_PERSON": "NAVIGATION",
    "GUIDE_PERSON": "NAVIGATION",
    "FIND_OBJECT": "VISION",
    "FIND_PERSON": "VISION",
    "FIND_PERSON_BY_ATTRIBUTE": "VISION",
    "COUNT_OBJECTS": "VISION",
    "COUNT_PEOPLE": "VISION",
    "EXTRACT_PERSON_INFO": "VISION",
    "PICK": "MANIPULATION",
    "PICK_OBJECT": "MANIPULATION",
    "PLACE": "MANIPULATION",
    "PLACE_OBJECT": "MANIPULATION",
    "HANDOVER": "MANIPULATION",
    "HANDOVER_OBJECT": "MANIPULATION",
    "SAY": "VOICE",
    "ASK_AND_LISTEN": "VOICE",
}


# Section: Preflight aliases normalize only structured producer values, never command text.
PREFLIGHT_STATE_ALIASES = {
    "ready": "READY",
    "available": "READY",
    "runnable": "READY",
    "unavailable": "UNAVAILABLE",
    "missing": "UNAVAILABLE",
    "disabled": "UNAVAILABLE",
    "unsupported": "UNAVAILABLE",
    "blocked": "BLOCKED",
    "dependency_blocked": "BLOCKED",
}

# Section: Only the validation owner may populate these normalized queue fields.
PREFLIGHT_ITEM_FIELDS = {
    "preflight_present",
    "preflight_state",
    "raw_preflight_state",
    "preflight_reason",
    "preflight_message",
    "preflight_resource",
    "blocked_by",
    "raw_validation_item",
}


def normalize_preflight_state(raw_state: Any) -> str:
    """Special function: Normalize structured skill-readiness values for display only."""
    value = str(raw_state or "").strip().lower().replace("-", "_").replace(" ", "_")
    return PREFLIGHT_STATE_ALIASES.get(value, "UNKNOWN")


def normalize_validation_state(raw_state: Any) -> str:
    """Special function: Normalize the producer's overall validation result."""
    value = str(raw_state or "").strip().lower().replace("-", "_").replace(" ", "_")
    if value in {"ready", "available", "runnable", "success", "succeeded", "complete", "completed"}:
        return "READY"
    if value in {"partial", "partially_runnable", "degraded"}:
        return "PARTIAL"
    if value in {"failed", "failure", "unavailable", "not_runnable"}:
        return "FAILED"
    return "UNKNOWN"


def _safe_nonnegative_int(value: Any, default: int = 0) -> int:
    """Special function: Parse untrusted JSON counters without raising conversion errors."""
    try:
        parsed = int(value)
    except (TypeError, ValueError, OverflowError):
        return default
    return parsed if parsed >= 0 else default


def _safe_int(value: Any, default: int = 0) -> int:
    """Special function: Parse an integer field that legitimately permits sentinels."""
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return default


def _optional_nonnegative_int(value: Any) -> int | None:
    """Special function: Distinguish an absent or invalid producer count from zero."""
    if value is None or isinstance(value, bool):
        return None
    try:
        parsed = int(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return parsed if parsed >= 0 else None


def _command_id(payload: Mapping[str, Any]) -> str:
    """Special function: Read only command identity aliases already used by GPSR owners."""
    for key in ("command_id", "active_command_id", "task_id"):
        value = str(payload.get(key, "") or "").strip()
        if value:
            return value
    return ""


def _blocked_by(value: Any) -> list[str]:
    """Special function: Normalize dependency identifiers while tolerating optional input."""
    if isinstance(value, (list, tuple, set)):
        return [str(item) for item in value if str(item)]
    if value not in (None, ""):
        return [str(value)]
    return []


def _validation_items(payload: Mapping[str, Any]) -> list[dict[str, Any]]:
    """Special function: Namespace validation fields so they cannot overwrite execution state."""
    raw_items = payload.get("items", [])
    source = raw_items if isinstance(raw_items, list) else []
    normalized: list[dict[str, Any]] = []
    for position, raw_item in enumerate(source):
        if not isinstance(raw_item, Mapping):
            continue
        raw_state = raw_item.get("raw_preflight_state", raw_item.get("preflight_state", ""))
        item: dict[str, Any] = {
            "index": _safe_nonnegative_int(raw_item.get("index", position), position),
            "preflight_present": True,
            "preflight_state": normalize_preflight_state(raw_state),
            "raw_preflight_state": str(raw_state or ""),
            "preflight_reason": str(raw_item.get("preflight_reason", raw_item.get("reason_code", "")) or ""),
            "preflight_message": str(raw_item.get("preflight_message", raw_item.get("message", "")) or ""),
            "preflight_resource": str(raw_item.get("preflight_resource", raw_item.get("resource", "")) or ""),
            "blocked_by": _blocked_by(raw_item.get("blocked_by", [])),
        }
        # Section: Optional validation identity fields must not erase scheduler values.
        step_id = str(raw_item.get("step_id", "") or "")
        action_type = str(raw_item.get("action_type", raw_item.get("skill", "")) or "")
        if step_id:
            item["step_id"] = step_id
        if action_type:
            item["action_type"] = action_type
        # Section: Preserve unknown producer diagnostics without letting execution keys leak through.
        item["raw_validation_item"] = dict(raw_item)
        normalized.append(item)
    return normalized


def _without_preflight_fields(items: Any) -> list[Any]:
    """Special function: Prevent scheduler or executor fields from impersonating validation."""
    source = items if isinstance(items, list) else []
    sanitized: list[Any] = []
    for raw_item in source:
        if not isinstance(raw_item, Mapping):
            sanitized.append(raw_item)
            continue
        sanitized.append({
            key: value
            for key, value in raw_item.items()
            if key not in PREFLIGHT_ITEM_FIELDS
        })
    return sanitized


def retained_json_qos():
    """Special function: Request the latest retained snapshot from optional topic producers."""
    # Section: Delay ROS imports so the normalization model remains hardware-free in tests.
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    # Section: Match reliable transient-local scheduler, validation, and executor publishers.
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def normalize_queue_state(raw_state: Any) -> str:
    """Special function: Normalize structured producer states while retaining raw state separately."""
    value = str(raw_state or "").strip().lower().replace("-", "_").replace(" ", "_")
    if value in {"success", "succeeded", "done", "complete", "completed"}:
        return "DONE"
    if value in {"executing", "active", "running", "accepted", "started"}:
        return "RUNNING"
    if value in {"queued", "waiting", "not_started", "pending", "in_queue"}:
        return "PENDING"
    if value in {"skip", "skipped"}:
        return "SKIPPED"
    if value in {"retry", "retrying", "retry_wait", "recovering", "recoverable_failure"}:
        return "RETRYING"
    if value in {"abort", "aborted", "fail", "failed", "error", "fatal_failure", "invalid_input", "not_found", "skill_unavailable", "timeout"}:
        return "FAILED"
    if value in {"canceled", "cancelled"}:
        return "CANCELED"
    return "UNKNOWN"


def action_domain(action_type: Any) -> str:
    """Special function: Map a canonical structured action identifier to its subsystem."""
    return ACTION_DOMAINS.get(str(action_type or "").upper(), "OTHER")


def action_target(item: Mapping[str, Any]) -> str:
    """Special function: Select a display target from explicit ActionStep target fields."""
    # Section: Scheduler plans retain primitive targets inside arguments while ROS
    # transport can retain the same values inside parameters_json.
    arguments = action_parameters(item)
    for key in ("target_location", "target_object", "target_person", "target_category"):
        value = str(item.get(key, arguments.get(key, "")) or "").strip()
        if value:
            return value
    # Section: Semantic plan fields use object_name and category before ActionStep conversion.
    for key in ("object_name", "category"):
        value = str(arguments.get(key, "") or "").strip()
        if value:
            return value
    return "-"


def _parameter_mapping(value: Any) -> dict[str, Any]:
    """Special function: Decode one structured or serialized parameter object safely."""
    # Section: Native dictionaries are copied so normalization never mutates producer data.
    if isinstance(value, Mapping):
        return dict(value)
    # Section: ActionStep transport may expose the same parameters as serialized JSON.
    if isinstance(value, str) and value.strip():
        try:
            decoded = json.loads(value)
        except json.JSONDecodeError:
            return {}
        return dict(decoded) if isinstance(decoded, Mapping) else {}
    return {}


def action_parameters(item: Mapping[str, Any]) -> dict[str, Any]:
    """Special function: Preserve the complete displayable parameter map for one action."""
    # Section: Merge transport JSON, normalized parameters, and scheduler arguments in
    # increasing order of authority.
    parameters: dict[str, Any] = {}
    for key in ("parameters_json", "parameters", "arguments"):
        parameters.update(_parameter_mapping(item.get(key)))
    # Section: ActionStep target fields remain parameters from the operator's point of
    # view even when the producer transports them in dedicated message fields.
    for key in (
        "target_location",
        "target_object",
        "target_person",
        "target_category",
    ):
        value = item.get(key)
        if value not in (None, ""):
            parameters.setdefault(key, value)
    return parameters


def merge_queue_items(planned: Any, runtime: Any) -> list[Any]:
    """Special function: Overlay owner data by step ID, then conservative index fallback."""
    planned_items = planned if isinstance(planned, list) else []
    runtime_items = runtime if isinstance(runtime, list) else []
    if not runtime_items:
        return list(planned_items)

    if not planned_items:
        return list(runtime_items)

    # Section: Prefer stable step identifiers, then fall back to queue position for
    # older owner snapshots while retaining base items not repeated by the overlay.
    planned_by_step = {
        str(item.get("step_id")): index
        for index, item in enumerate(planned_items)
        if isinstance(item, Mapping) and str(item.get("step_id", ""))
    }
    merged_items: list[Any] = [dict(item) if isinstance(item, Mapping) else item for item in planned_items]
    for index, runtime_item in enumerate(runtime_items):
        if not isinstance(runtime_item, Mapping):
            continue
        step_id = str(runtime_item.get("step_id", ""))
        target_index = planned_by_step.get(step_id) if step_id else None
        if target_index is None:
            candidate_index = _optional_nonnegative_int(runtime_item.get("index"))
            target_index = candidate_index if candidate_index is not None else index
        if target_index >= len(merged_items):
            merged_items.append(dict(runtime_item))
            continue
        base_item = merged_items[target_index]
        base = base_item if isinstance(base_item, Mapping) else {}
        merged = dict(base or {})
        merged.update(runtime_item)
        parameters = action_parameters(base or {})
        parameters.update(action_parameters(runtime_item))
        if parameters:
            merged["parameters"] = parameters
        merged_items[target_index] = merged
    return merged_items


def _derived_validation_state(counts: Mapping[str, int]) -> str:
    """Special function: Derive a conservative display state when the owner omits one."""
    ready = int(counts.get("READY", 0))
    unavailable = int(counts.get("UNAVAILABLE", 0))
    blocked = int(counts.get("BLOCKED", 0))
    unknown = int(counts.get("UNKNOWN", 0))
    if ready > 0 and unavailable == 0 and blocked == 0 and unknown == 0:
        return "READY"
    if ready > 0 and unavailable + blocked + unknown > 0:
        return "PARTIAL"
    if ready == 0 and unavailable + blocked > 0:
        return "FAILED"
    return "UNKNOWN"


def _validation_summary(
    payload: Mapping[str, Any],
    items: list[dict[str, Any]],
    *,
    present: bool,
    report_present: bool,
    stale: bool,
) -> dict[str, Any]:
    """Special function: Build the compact, command-safe preflight summary."""
    derived_counts = {
        state: sum(1 for item in items if item.get("preflight_state") == state)
        for state in ("READY", "UNAVAILABLE", "BLOCKED", "UNKNOWN")
    }
    count_keys = {
        "READY": "ready_count",
        "UNAVAILABLE": "unavailable_count",
        "BLOCKED": "blocked_count",
        "UNKNOWN": "unknown_count",
    }
    counts: dict[str, int] = {}
    for state, key in count_keys.items():
        explicit = _optional_nonnegative_int(payload.get(key))
        counts[state] = explicit if explicit is not None else derived_counts[state]

    raw_overall = payload.get("validation_state", payload.get("state", ""))
    overall = normalize_validation_state(raw_overall)
    if overall == "UNKNOWN":
        overall = _derived_validation_state(counts)
    fully_runnable_raw = payload.get("fully_runnable")
    fully_runnable = fully_runnable_raw if isinstance(fully_runnable_raw, bool) else overall == "READY"
    return {
        "present": bool(present),
        "report_present": bool(report_present),
        "stale": bool(stale),
        "command_id": _command_id(payload),
        "state": overall if present else "UNKNOWN",
        "raw_state": str(raw_overall or "") if present else "",
        "fully_runnable": bool(fully_runnable) if present else False,
        "ready_count": counts["READY"] if present else 0,
        "unavailable_count": counts["UNAVAILABLE"] if present else 0,
        "blocked_count": counts["BLOCKED"] if present else 0,
        "unknown_count": counts["UNKNOWN"] if present else 0,
        "item_count": len(items) if present else 0,
        "message": str(payload.get("message", "") or "") if present else "",
    }


def task_display_status(state: Any, queue_state: Any = "") -> str:
    """Special function: Map executor states to concise operator-facing task statuses."""
    normalized = normalize_queue_state(state)
    terminal_queue = normalize_queue_state(queue_state)
    if normalized == "DONE":
        return "COMPLETED"
    if normalized in {"FAILED", "CANCELED"}:
        return "ABORTED"
    # Section: A terminal queue abort means every unstarted remainder can no longer run.
    if terminal_queue in {"FAILED", "CANCELED"} and normalized in {"PENDING", "UNKNOWN"}:
        return "ABORTED"
    if normalized == "RUNNING":
        return "RUNNING"
    if normalized == "RETRYING":
        return "RETRYING"
    if normalized == "SKIPPED":
        return "SKIPPED"
    return "IN QUEUE"


class GpsrStateModel:
    """Special function: Merge scheduler, validation, and executor-owned snapshots."""

    def __init__(self) -> None:
        """Special function: Initialize an empty startup-safe GPSR model."""
        self.plan: dict[str, Any] = {}
        self.scheduler: dict[str, Any] = {}
        self.executor: dict[str, Any] = {}
        self.validation: dict[str, Any] = {}
        self.validation_received = False
        self.scheduler_present = False
        self.executor_present = False

    def set_plan(self, payload: Mapping[str, Any]) -> None:
        """Special function: Store a validated scheduler plan payload."""
        self.plan = dict(payload)

    def set_scheduler(self, payload: Mapping[str, Any]) -> None:
        """Special function: Store a validated scheduler cycle payload."""
        self.scheduler = dict(payload)

    def set_executor(self, payload: Mapping[str, Any]) -> None:
        """Special function: Store a validated executor queue payload."""
        self.executor = dict(payload)

    def set_validation(self, payload: Mapping[str, Any]) -> None:
        """Special function: Store the latest valid owner-side preflight report."""
        self.validation = dict(payload)
        self.validation_received = True

    def set_presence(self, scheduler_present: bool, executor_present: bool) -> None:
        """Special function: Update scheduler and executor graph presence independently."""
        self.scheduler_present = bool(scheduler_present)
        self.executor_present = bool(executor_present)

    def snapshot(self) -> dict[str, Any]:
        """Special function: Produce normalized command, queue, and current-action state."""
        context = self.plan.get("command_context", {})
        context = context if isinstance(context, Mapping) else {}
        command_id = str(self.plan.get("command_id", context.get("command_id", _command_id(self.scheduler))) or "")
        executor_task_id = str(self.executor.get("task_id", ""))
        executor_matches = bool(self.executor) and (
            not command_id or not executor_task_id or executor_task_id == command_id
        )
        # Section: Resolve the scheduler-owned parameter-rich queue before applying
        # executor-owned live state.
        planned_items = context.get("action_queue", [])
        if not isinstance(planned_items, list) or not planned_items:
            planned_items = self.plan.get("steps", [])
        planned_items = _without_preflight_fields(planned_items)
        # Section: A retained validation report is applied only when its owner identity
        # is compatible with the current scheduler command.
        validation_command_id = _command_id(self.validation)
        validation_matches = self.validation_received and (
            not command_id or not validation_command_id or validation_command_id == command_id
        )
        normalized_validation_items = _validation_items(self.validation) if validation_matches else []
        validation_merged_items = merge_queue_items(planned_items, normalized_validation_items)
        runtime_items = _without_preflight_fields(self.executor.get("items")) if executor_matches else None
        source_items = merge_queue_items(validation_merged_items, runtime_items)
        source_items = source_items if isinstance(source_items, list) else []
        current_index = _safe_int(self.executor.get("current_index", -1), -1) if executor_matches else -1
        queue_state_raw = self.executor.get("queue_state", "IN_QUEUE") if executor_matches else (
            "IN_QUEUE" if bool(self.plan.get("valid")) and source_items else self.scheduler.get("state", "UNKNOWN")
        )
        queue_state = normalize_queue_state(queue_state_raw)

        # Section: Normalize each structured queue item and preserve its raw producer state.
        items: list[dict[str, Any]] = []
        for index, raw_item in enumerate(source_items):
            item = dict(raw_item) if isinstance(raw_item, Mapping) else {}
            raw_state = item.get("raw_state", item.get("state", "PENDING"))
            state = normalize_queue_state(item.get("state", raw_state))
            item.update({
                "index": _safe_nonnegative_int(item.get("index", index), index),
                "domain": action_domain(item.get("action_type", item.get("skill_name", ""))),
                "skill": str(item.get("action_type", item.get("skill_name", ""))),
                "target": action_target(item),
                "parameters": action_parameters(item),
                # Section: Preflight readiness remains orthogonal to executor state.
                "preflight_present": bool(item.get("preflight_present", False)),
                "preflight_state": normalize_preflight_state(item.get("preflight_state", "")),
                "raw_preflight_state": str(item.get("raw_preflight_state", item.get("preflight_state", "")) or ""),
                "preflight_reason": str(item.get("preflight_reason", "") or ""),
                "preflight_message": str(item.get("preflight_message", "") or ""),
                "preflight_resource": str(item.get("preflight_resource", "") or ""),
                "blocked_by": _blocked_by(item.get("blocked_by", [])),
                "state": state,
                "task_status": task_display_status(state, queue_state),
                "raw_state": str(raw_state),
                "attempt": _safe_nonnegative_int(item.get("attempt", 0)),
                "max_retries": _safe_nonnegative_int(item.get("max_retries", item.get("retry_count", 0))),
                "current": bool(item.get("current", index == current_index)),
            })
            items.append(item)

        current = next((item for item in items if item.get("current")), None)
        # Section: Counts let the Overview page report the whole sequence at a glance.
        status_counts = {
            status: sum(1 for item in items if item.get("task_status") == status)
            for status in ("IN QUEUE", "RUNNING", "RETRYING", "COMPLETED", "ABORTED", "SKIPPED")
        }
        completed_count = _safe_nonnegative_int(
            self.executor.get("completed_count", status_counts["COMPLETED"]),
            status_counts["COMPLETED"],
        ) if executor_matches else status_counts["COMPLETED"]
        preflight = _validation_summary(
            self.validation,
            normalized_validation_items,
            present=validation_matches,
            report_present=self.validation_received,
            stale=self.validation_received and not validation_matches,
        )
        return {
            "scheduler_present": self.scheduler_present,
            "executor_present": self.executor_present,
            "command_id": command_id,
            "raw_command": str(context.get("raw_text", "")),
            "normalized_command": str(context.get("normalized_text", "")),
            "scheduler_state": str(self.scheduler.get("state", "UNKNOWN")),
            "queue_state": queue_state,
            "raw_queue_state": str(queue_state_raw),
            "current_index": current_index,
            "queue_length": len(items),
            "completed_count": min(len(items), max(0, completed_count)),
            "task_status_counts": status_counts,
            "preflight": preflight,
            "current_domain": str(current.get("domain", "")) if current else "",
            "current_skill": str(current.get("skill", "")) if current else "",
            "retry_attempt": _safe_nonnegative_int(self.executor.get("retry_attempt", current.get("attempt", 0) if current else 0)) if executor_matches else 0,
            "retry_limit": _safe_nonnegative_int(self.executor.get("retry_limit", current.get("max_retries", 0) if current else 0)) if executor_matches else 0,
            "message": str(self.executor.get("message", "")) if executor_matches else "",
            "queue": items,
        }


class GpsrTransitionTracker:
    """Special function: Convert normalized snapshots into one event per GPSR transition."""

    def __init__(self) -> None:
        """Special function: Initialize empty command, queue, and action transition memory."""
        self.last_command_id = ""
        self.last_queue_state = "UNKNOWN"
        self.last_item_states: dict[str, str] = {}
        self.last_preflight_state = "UNKNOWN"
        self.last_preflight_item_states: dict[str, str] = {}

    def update(self, snapshot: Mapping[str, Any]) -> list[MonitorEvent]:
        """Special function: Return new command/queue/action events without duplicate polling noise."""
        emitted: list[MonitorEvent] = []
        preflight = snapshot.get("preflight", {})
        preflight = preflight if isinstance(preflight, Mapping) else {}
        command_id = str(snapshot.get("command_id", "") or preflight.get("command_id", ""))
        if command_id and command_id != self.last_command_id:
            emitted.append(MonitorEvent("INFO", "GPSR", "GPSR_COMMAND_RECEIVED", "structured GPSR command received", gpsr_command=command_id))
            queue = snapshot.get("queue", [])
            if isinstance(queue, list) and queue:
                emitted.append(MonitorEvent("INFO", "GPSR", "GPSR_QUEUE_CREATED", f"queue contains {len(queue)} actions", gpsr_command=command_id))
            self.last_command_id = command_id
            # Section: Identical step IDs represent new transitions under a new command.
            self.last_queue_state = "UNKNOWN"
            self.last_item_states.clear()
            self.last_preflight_state = "UNKNOWN"
            self.last_preflight_item_states.clear()

        # Section: Emit one overall preflight event only when the normalized result changes.
        if bool(preflight.get("present")):
            preflight_state = str(preflight.get("state", "UNKNOWN"))
            if preflight_state != self.last_preflight_state and preflight_state in {"READY", "PARTIAL", "FAILED"}:
                severity = "INFO" if preflight_state == "READY" else "WARN" if preflight_state == "PARTIAL" else "ERROR"
                emitted.append(MonitorEvent(
                    severity,
                    "GPSR",
                    f"GPSR_PREFLIGHT_{preflight_state}",
                    str(preflight.get("message", "")) or f"pre-execution validation is {preflight_state.lower()}",
                    gpsr_command=command_id,
                    details={
                        "ready_count": preflight.get("ready_count", 0),
                        "unavailable_count": preflight.get("unavailable_count", 0),
                        "blocked_count": preflight.get("blocked_count", 0),
                        "unknown_count": preflight.get("unknown_count", 0),
                    },
                ))
            self.last_preflight_state = preflight_state

        # Section: Compare step identity and state so repeated snapshots do not duplicate events.
        queue = snapshot.get("queue", [])
        for item in queue if isinstance(queue, list) else []:
            if not isinstance(item, Mapping):
                continue
            identity = str(item.get("step_id") or f"{item.get('index')}:{item.get('skill')}")
            preflight_state = str(item.get("preflight_state", "UNKNOWN"))
            previous_preflight = self.last_preflight_item_states.get(identity)
            if bool(item.get("preflight_present")) and preflight_state != previous_preflight and preflight_state in {"UNAVAILABLE", "BLOCKED"}:
                severity = "ERROR" if preflight_state == "UNAVAILABLE" else "WARN"
                event_code = "GPSR_SKILL_UNAVAILABLE" if preflight_state == "UNAVAILABLE" else "GPSR_SKILL_BLOCKED"
                emitted.append(MonitorEvent(
                    severity,
                    "GPSR",
                    event_code,
                    str(item.get("preflight_message", "")) or f"{item.get('skill', 'action')} is {preflight_state.lower()}",
                    gpsr_command=command_id,
                    gpsr_skill=str(item.get("skill", "")),
                    details={
                        "index": item.get("index"),
                        "step_id": item.get("step_id", ""),
                        "reason_code": item.get("preflight_reason", ""),
                        "resource": item.get("preflight_resource", ""),
                        "blocked_by": item.get("blocked_by", []),
                        "message": item.get("preflight_message", ""),
                    },
                ))
            if bool(item.get("preflight_present")):
                self.last_preflight_item_states[identity] = preflight_state
            state = str(item.get("state", "UNKNOWN"))
            previous = self.last_item_states.get(identity)
            if state != previous and state in {"RUNNING", "DONE", "SKIPPED", "RETRYING", "FAILED", "CANCELED"}:
                severity = "ERROR" if state == "FAILED" else "WARN" if state in {"RETRYING", "CANCELED"} else "INFO"
                message = str(item.get("message", "")) or f"{item.get('skill', 'action')} is {state.lower()}"
                event_state = "STARTED" if state == "RUNNING" else state
                emitted.append(MonitorEvent(severity, "GPSR", f"GPSR_ACTION_{event_state}", message, gpsr_command=command_id, gpsr_skill=str(item.get("skill", "")), details={"index": item.get("index"), "raw_state": item.get("raw_state"), "attempt": item.get("attempt")}))
            self.last_item_states[identity] = state

        queue_state = str(snapshot.get("queue_state", "UNKNOWN"))
        if queue_state != self.last_queue_state and queue_state in {"DONE", "FAILED", "CANCELED"}:
            severity = "INFO" if queue_state == "DONE" else "ERROR"
            event_code = "GPSR_QUEUE_COMPLETED" if queue_state == "DONE" else "GPSR_QUEUE_FAILED"
            emitted.append(MonitorEvent(severity, "GPSR", event_code, str(snapshot.get("message", "")) or f"queue {queue_state.lower()}", gpsr_command=command_id))
        self.last_queue_state = queue_state
        return emitted


class GpsrSource:
    """Special function: Consume scheduler, validation, and executor owner snapshots."""

    def __init__(self, node, cache, events, plan_topic: str, state_topic: str, queue_topic: str, validation_topic: str, rate_hz: float) -> None:
        """Special function: Create structured JSON subscriptions and a graph presence timer."""
        self.node = node
        self.cache = cache
        self.events = events
        self.model = GpsrStateModel()
        self.transition_tracker = GpsrTransitionTracker()
        # Section: Import ROS-only QoS and messages only when constructing the runtime source.
        from std_msgs.msg import String
        # Section: Retained plan delivery lets this optional monitor start after command acceptance.
        self.plan_sub = node.create_subscription(
            String,
            plan_topic,
            self._on_plan,
            retained_json_qos(),
        )
        self.state_sub = node.create_subscription(String, state_topic, self._on_scheduler, 10)
        # Section: Retained executor delivery restores the latest live queue without a handshake.
        self.queue_sub = node.create_subscription(
            String,
            queue_topic,
            self._on_executor,
            retained_json_qos(),
        )
        # Section: Validation is optional, retained, read-only owner data like the plan snapshot.
        self.validation_sub = node.create_subscription(
            String,
            validation_topic,
            self._on_validation,
            retained_json_qos(),
        )
        self.timer = node.create_timer(1.0 / max(0.1, float(rate_hz)), self.poll)
        self.cache.set("gpsr", self.model.snapshot())

    def _decode(self, msg: Any, source: str) -> dict[str, Any] | None:
        """Special function: Decode one JSON object without allowing malformed data to stop monitoring."""
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError) as exc:
            self.events.emit(MonitorEvent("WARN", "GPSR", "GPSR_STATUS_INVALID_JSON", f"{source}: {exc}"))
            return None
        if not isinstance(payload, Mapping):
            self.events.emit(MonitorEvent("WARN", "GPSR", "GPSR_STATUS_INVALID_JSON", f"{source}: expected JSON object"))
            return None
        return dict(payload)

    def _on_plan(self, msg: Any) -> None:
        """Special function: Update the command and planned queue from scheduler output."""
        payload = self._decode(msg, "task_plan")
        if payload is None:
            return
        self.model.set_plan(payload)
        self._publish_and_emit()

    def _on_scheduler(self, msg: Any) -> None:
        """Special function: Update the GPSR cycle state from scheduler output."""
        payload = self._decode(msg, "scheduler_state")
        if payload is None:
            return
        self.model.set_scheduler(payload)
        self._publish_and_emit()

    def _on_executor(self, msg: Any) -> None:
        """Special function: Update live per-item execution state from the queue owner."""
        payload = self._decode(msg, "executor_queue")
        if payload is None:
            return
        self.model.set_executor(payload)
        self._publish_and_emit()

    def _on_validation(self, msg: Any) -> None:
        """Special function: Update pre-execution readiness without changing runtime state."""
        payload = self._decode(msg, "action_queue_validation")
        if payload is None:
            return
        self.model.set_validation(payload)
        self._publish_and_emit()

    def poll(self) -> None:
        """Special function: Refresh scheduler and executor graph presence without startup failure."""
        try:
            names = {
                f"{str(namespace).rstrip('/')}/{name}" if str(namespace).rstrip("/") else f"/{name}"
                for name, namespace in self.node.get_node_names_and_namespaces()
            }
        except Exception as exc:
            self.events.emit(MonitorEvent("WARN", "GPSR", "GPSR_GRAPH_QUERY_FAILED", str(exc), node=self.node.get_name()))
            return
        self.model.set_presence("/gpsr_task_scheduler" in names, "/gpsr_executor" in names)
        self.cache.set("gpsr", self.model.snapshot())

    def _publish_and_emit(self) -> None:
        """Special function: Cache current state and emit only meaningful GPSR transitions."""
        snapshot = self.model.snapshot()
        self.cache.set("gpsr", snapshot)
        for event in self.transition_tracker.update(snapshot):
            self.events.emit(event)
