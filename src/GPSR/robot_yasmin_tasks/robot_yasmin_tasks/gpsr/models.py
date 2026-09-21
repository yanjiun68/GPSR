"""Section: Dataclass models for GPSR execution."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any

from robot_yasmin_tasks.gpsr.action_types import normalize_primitive_name
from robot_yasmin_tasks.gpsr.greetings import DEFAULT_GREETING  # Keep legacy queue greetings independent of human traits.


@dataclass(slots=True)
class ActionStepData:
    """Special function: Normalized symbolic GPSR action."""

    action_type: str
    task_id: str = ""
    task_family: str = ""
    target_object: str = ""
    target_person: str = ""
    target_location: str = ""
    target_category: str = ""
    parameters: dict[str, Any] = field(default_factory=dict)
    timeout_sec: float = 0.0
    step_id: str = ""
    max_retries: int = 1
    requires_success: list[str] = field(default_factory=list)
    requires_data: list[str] = field(default_factory=list)
    result_variable: str = ""


@dataclass(slots=True)
class SkillResultData:
    """Special function: Normalized deterministic skill result."""

    action_type: str
    outcome: str
    message: str = ""
    result: dict[str, Any] = field(default_factory=dict)
    recoverable: bool = False
    fatal: bool = False

    def to_dict(self) -> dict[str, Any]:
        """Special function: Convert result to plain dict for blackboard storage."""
        return {"action_type": self.action_type, "outcome": self.outcome, "message": self.message, "result": self.result, "recoverable": self.recoverable, "fatal": self.fatal}


def format_skill_failure(result: dict[str, Any]) -> str:
    """Special function: Preserve the exact failing action, outcome, reason, and details."""
    # Section: Normalize mandatory diagnostic fields without hiding missing producer data.
    action_type = str(result.get("action_type", "") or "unknown_action")
    outcome = str(result.get("outcome", "") or "unknown_outcome")
    reason = str(result.get("message", "") or "no failure message provided")
    details = result.get("result", {})

    # Section: Include structured client details when they carry additional failure evidence.
    details_text = ""
    if details not in ({}, None, ""):
        try:
            details_text = f" details={json.dumps(details, sort_keys=True)}"
        except (TypeError, ValueError):
            details_text = f" details={details!r}"
    return f"action={action_type} outcome={outcome} reason={reason}{details_text}"


def summarize_action_queue_failure(
    queue_outcome: str,
    results: list[dict[str, Any]],
) -> str:
    """Special function: Select and format the executor's terminal failed SkillResult."""
    # Section: Prefer the last non-success result because it terminates queue execution.
    failures = [
        result
        for result in results
        if str(result.get("outcome", "")).strip().lower() != "success"
    ]
    if failures:
        return format_skill_failure(failures[-1])

    # Section: Retain the queue outcome when an abnormal executor returns no skill result.
    return (
        f"action=unknown_action outcome={str(queue_outcome or 'unknown_outcome')} "
        "reason=executor returned no failed SkillResult"
    )


def parse_parameters(raw: Any) -> dict[str, Any]:
    """Special function: Parse JSON parameter payloads deterministically."""
    if isinstance(raw, dict):
        return dict(raw)
    if not raw:
        return {}
    try:
        parsed = json.loads(str(raw))
    except json.JSONDecodeError:
        return {"raw": str(raw)}
    return parsed if isinstance(parsed, dict) else {"value": parsed}


def normalize_action_step(raw: Any) -> ActionStepData:
    """Special function: Normalize dict or ROS message action steps."""
    if isinstance(raw, ActionStepData):
        return canonicalize_action_step(raw)
    if isinstance(raw, dict):
        params = parse_parameters(raw.get("parameters", raw.get("parameters_json", {})))
        # Section: Preserve unknown fields inside parameters without hiding execution metadata.
        known = {
            "action_type", "task_id", "task_family", "target_object",
            "target_person", "target_location", "target_category", "parameters",
            "parameters_json", "timeout_sec", "step_id", "max_retries",
            "requires_success", "requires_data", "result_variable",
        }
        for key, value in raw.items():
            if key not in known:
                params.setdefault(key, value)
        return canonicalize_action_step(ActionStepData(
            action_type=str(raw.get("action_type", "")).upper(),
            task_id=str(raw.get("task_id", "")),
            task_family=str(raw.get("task_family", "")),
            target_object=str(raw.get("target_object", "")),
            target_person=str(raw.get("target_person", "")),
            target_location=str(raw.get("target_location", "")),
            target_category=str(raw.get("target_category", "")),
            parameters=params,
            timeout_sec=float(raw.get("timeout_sec", 0.0) or 0.0),
            step_id=str(raw.get("step_id", "")),
            max_retries=max(0, int(raw.get("max_retries", 1) or 0)),
            requires_success=list(raw.get("requires_success", [])),
            requires_data=list(raw.get("requires_data", [])),
            result_variable=str(raw.get("result_variable", "")),
        ))
    return canonicalize_action_step(ActionStepData(
        action_type=str(getattr(raw, "action_type", "")).upper(),
        task_id=str(getattr(raw, "task_id", "")),
        task_family=str(getattr(raw, "task_family", "")),
        target_object=str(getattr(raw, "target_object", "")),
        target_person=str(getattr(raw, "target_person", "")),
        target_location=str(getattr(raw, "target_location", "")),
        target_category=str(getattr(raw, "target_category", "")),
        parameters=parse_parameters(getattr(raw, "parameters_json", "")),
        timeout_sec=float(getattr(raw, "timeout_sec", 0.0) or 0.0),
        step_id=str(getattr(raw, "step_id", "")),
        max_retries=max(0, int(getattr(raw, "max_retries", 0) or 0)),
        requires_success=list(getattr(raw, "requires_success", [])),
        requires_data=list(getattr(raw, "requires_data", [])),
        result_variable=str(getattr(raw, "result_variable", "")),
    ))


def canonicalize_action_step(action: ActionStepData) -> ActionStepData:
    """Special function: Translate temporary legacy queue identifiers into primitives."""
    canonical, legacy = normalize_primitive_name(action.action_type)
    action.action_type = canonical
    if not legacy:
        return action

    # Section: Preserve deprecation evidence for executor logging and result diagnostics.
    action.parameters.setdefault("_legacy_action_type", legacy)
    action.parameters.setdefault("_canonical_action_type", canonical)
    if legacy == "RETURN_TO_INSTRUCTION_POINT":
        action.target_location = action.target_location or str(
            action.parameters.get("target_location", "instruction_point")
        )
        action.parameters.setdefault("target_location", action.target_location)
    elif legacy == "GREET":
        if not str(action.parameters.get("text") or "").strip():  # Supply wording even when legacy callers send an empty text field.
            action.parameters["text"] = DEFAULT_GREETING  # Do not speak raw person identifiers or descriptions.
    return action


def make_skill_result(action_type: str, outcome: str, message: str = "", result: dict[str, Any] | None = None, recoverable: bool = False, fatal: bool = False) -> SkillResultData:
    """Special function: Create a normalized skill result."""
    return SkillResultData(action_type, outcome, message, result or {}, recoverable, fatal)
