"""Section: Dataclass models for GPSR execution."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any


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
        return raw
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
        return ActionStepData(
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
        )
    return ActionStepData(
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
    )


def make_skill_result(action_type: str, outcome: str, message: str = "", result: dict[str, Any] | None = None, recoverable: bool = False, fatal: bool = False) -> SkillResultData:
    """Special function: Create a normalized skill result."""
    return SkillResultData(action_type, outcome, message, result or {}, recoverable, fatal)
