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


def retained_json_qos():
    """Special function: Request the latest retained snapshot from optional topic producers."""
    # Section: Delay ROS imports so the normalization model remains hardware-free in tests.
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    # Section: Match reliable transient-local scheduler and executor snapshot publishers.
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
    """Special function: Overlay executor state without discarding scheduler parameters."""
    planned_items = planned if isinstance(planned, list) else []
    runtime_items = runtime if isinstance(runtime, list) else []
    if not runtime_items:
        return list(planned_items)

    # Section: Prefer stable step identifiers, then fall back to queue position for
    # older executor snapshots.
    planned_by_step = {
        str(item.get("step_id")): item
        for item in planned_items
        if isinstance(item, Mapping) and str(item.get("step_id", ""))
    }
    merged_items: list[Any] = []
    for index, runtime_item in enumerate(runtime_items):
        if not isinstance(runtime_item, Mapping):
            merged_items.append(runtime_item)
            continue
        step_id = str(runtime_item.get("step_id", ""))
        base = planned_by_step.get(step_id)
        if base is None and index < len(planned_items):
            candidate = planned_items[index]
            base = candidate if isinstance(candidate, Mapping) else None
        merged = dict(base or {})
        merged.update(runtime_item)
        parameters = action_parameters(base or {})
        parameters.update(action_parameters(runtime_item))
        if parameters:
            merged["parameters"] = parameters
        merged_items.append(merged)
    return merged_items


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
    """Special function: Merge scheduler and executor-owned structured snapshots."""

    def __init__(self) -> None:
        """Special function: Initialize an empty startup-safe GPSR model."""
        self.plan: dict[str, Any] = {}
        self.scheduler: dict[str, Any] = {}
        self.executor: dict[str, Any] = {}
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

    def set_presence(self, scheduler_present: bool, executor_present: bool) -> None:
        """Special function: Update scheduler and executor graph presence independently."""
        self.scheduler_present = bool(scheduler_present)
        self.executor_present = bool(executor_present)

    def snapshot(self) -> dict[str, Any]:
        """Special function: Produce normalized command, queue, and current-action state."""
        context = self.plan.get("command_context", {})
        context = context if isinstance(context, Mapping) else {}
        command_id = str(self.plan.get("command_id", context.get("command_id", self.scheduler.get("active_command_id", ""))))
        executor_task_id = str(self.executor.get("task_id", ""))
        executor_matches = bool(self.executor) and (
            not command_id or not executor_task_id or executor_task_id == command_id
        )
        # Section: Resolve the scheduler-owned parameter-rich queue before applying
        # executor-owned live state.
        planned_items = context.get("action_queue", [])
        if not isinstance(planned_items, list) or not planned_items:
            planned_items = self.plan.get("steps", [])
        runtime_items = self.executor.get("items") if executor_matches else None
        source_items = merge_queue_items(planned_items, runtime_items)
        source_items = source_items if isinstance(source_items, list) else []
        current_index = int(self.executor.get("current_index", -1) or 0) if executor_matches else -1
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
                "index": int(item.get("index", index) or 0),
                "domain": action_domain(item.get("action_type", item.get("skill_name", ""))),
                "skill": str(item.get("action_type", item.get("skill_name", ""))),
                "target": action_target(item),
                "parameters": action_parameters(item),
                "state": state,
                "task_status": task_display_status(state, queue_state),
                "raw_state": str(raw_state),
                "attempt": int(item.get("attempt", 0) or 0),
                "max_retries": int(item.get("max_retries", item.get("retry_count", 0)) or 0),
                "current": bool(item.get("current", index == current_index)),
            })
            items.append(item)

        current = next((item for item in items if item.get("current")), None)
        # Section: Counts let the Overview page report the whole sequence at a glance.
        status_counts = {
            status: sum(1 for item in items if item.get("task_status") == status)
            for status in ("IN QUEUE", "RUNNING", "RETRYING", "COMPLETED", "ABORTED", "SKIPPED")
        }
        completed_count = int(self.executor.get("completed_count", status_counts["COMPLETED"]) or 0) if executor_matches else status_counts["COMPLETED"]
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
            "current_domain": str(current.get("domain", "")) if current else "",
            "current_skill": str(current.get("skill", "")) if current else "",
            "retry_attempt": int(self.executor.get("retry_attempt", current.get("attempt", 0) if current else 0) or 0) if executor_matches else 0,
            "retry_limit": int(self.executor.get("retry_limit", current.get("max_retries", 0) if current else 0) or 0) if executor_matches else 0,
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

    def update(self, snapshot: Mapping[str, Any]) -> list[MonitorEvent]:
        """Special function: Return new command/queue/action events without duplicate polling noise."""
        emitted: list[MonitorEvent] = []
        command_id = str(snapshot.get("command_id", ""))
        if command_id and command_id != self.last_command_id:
            emitted.append(MonitorEvent("INFO", "GPSR", "GPSR_COMMAND_RECEIVED", "structured GPSR command received", gpsr_command=command_id))
            queue = snapshot.get("queue", [])
            if isinstance(queue, list) and queue:
                emitted.append(MonitorEvent("INFO", "GPSR", "GPSR_QUEUE_CREATED", f"queue contains {len(queue)} actions", gpsr_command=command_id))
            self.last_command_id = command_id

        # Section: Compare step identity and state so repeated snapshots do not duplicate events.
        queue = snapshot.get("queue", [])
        for item in queue if isinstance(queue, list) else []:
            if not isinstance(item, Mapping):
                continue
            identity = str(item.get("step_id") or f"{item.get('index')}:{item.get('skill')}")
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
    """Special function: Consume scheduler plan/state and executor queue status topics."""

    def __init__(self, node, cache, events, plan_topic: str, state_topic: str, queue_topic: str, rate_hz: float) -> None:
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
