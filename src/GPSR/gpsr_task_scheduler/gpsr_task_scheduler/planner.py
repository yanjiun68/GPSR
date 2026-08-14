"""Section: Deterministic GPSR task planner."""

from __future__ import annotations

import json
from typing import Any

from gpsr_task_scheduler.config import SchedulerConfig
from gpsr_task_scheduler.models import SchedulerResult, SkillStep, TaskRequest, ValidationIssue


TASK_TO_SKILL = {
    "find_object": "FIND_OBJECT",
    "find_person": "FIND_PERSON",
    "count_objects": "COUNT_OBJECTS",
    "count_people": "COUNT_PEOPLE",
}


def plan_task(request: TaskRequest, config: SchedulerConfig) -> SchedulerResult:
    """Special function: Build a physical task order from semantic roles."""
    issues = _validate_request(request)
    if issues:
        return SchedulerResult(request.command_id, request, [], issues)
    steps: list[SkillStep] = []
    # Section: FIND_OBJECT is a compound mission and owns every search navigation.
    if request.task_type == "find_object":
        execute_step = _execute_step(request, None, config)
        steps.append(execute_step)
    else:
        steps.append(_step("go_to_task_location", "NAVIGATE", {"target_location": request.task_location}, [], config))
        execute_step = _execute_step(request, steps[-1].step_id, config)
        steps.append(execute_step)
    if request.report_location:
        steps.append(_step("go_to_report_location", "NAVIGATE", {"target_location": request.report_location}, [execute_step.step_id], config))
    if request.recipient:
        dependency = steps[-1].step_id
        steps.append(_step("find_recipient", "FIND_PERSON", {"target_person": request.recipient.get("name", ""), "target_location": request.report_location or ""}, [dependency], config, "recipient"))
    if request.output_template:
        dependency = steps[-1].step_id
        steps.append(_step("tts_say_output", "SAY", {"text": request.output_template, "substitution": "task_result"}, [dependency], config))
    if request.return_after_completion and request.return_location:
        dependency = steps[-1].step_id
        steps.append(_step("return_to_instruction_point", "RETURN_TO_INSTRUCTION_POINT", {"target_location": request.return_location}, [dependency], config))
    return SchedulerResult(request.command_id, request, steps, [])


def _validate_request(request: TaskRequest) -> list[ValidationIssue]:
    """Special function: Reject unresolved or contradictory semantic plans."""
    issues: list[ValidationIssue] = []
    if request.task_type not in TASK_TO_SKILL:
        issues.append(ValidationIssue("unknown_task", f"Unsupported task type: {request.task_type}."))
    if request.task_type != "find_object" and not request.task_location:
        issues.append(ValidationIssue("missing_task_location", "Task location is required."))
    if request.recipient and not request.output_template:
        issues.append(ValidationIssue("recipient_without_output", "Named recipient requires an output template."))
    if request.report_location is None and request.recipient:
        issues.append(ValidationIssue("missing_report_location", "Named recipient delivery requires a report location."))
    return issues


def _execute_step(request: TaskRequest, dependency: str | None, config: SchedulerConfig) -> SkillStep:
    """Special function: Build the task execution step for the requested task family."""
    skill_name = TASK_TO_SKILL[request.task_type]
    arguments: dict[str, Any] = {"target_location": request.task_location, "filters": request.filters}
    arguments.update(request.target)
    # Section: Explicit FIND_OBJECT locations are strict boundaries; omission means global.
    if request.task_type == "find_object":
        arguments["location"] = request.task_location or ""
        arguments["search_scope"] = "STRICT_LOCATION" if request.task_location else "GLOBAL"
        arguments["approach_target"] = True
    dependencies = [dependency] if dependency else []
    return _step("execute_task", skill_name, arguments, dependencies, config, "task_result")


def _step(step_id: str, skill_name: str, arguments: dict[str, Any], dependencies: list[str], config: SchedulerConfig, result_variable: str | None = None) -> SkillStep:
    """Special function: Create one skill step with configured timeout and retries."""
    return SkillStep(step_id, skill_name, arguments, dependencies, config.timeout_for(skill_name), config.retries_for(skill_name), result_variable)


def step_to_action_step_dict(step: SkillStep, command_id: str) -> dict[str, Any]:
    """Special function: Convert a scheduler step into robot_ai_interfaces/ActionStep-compatible data."""
    args = dict(step.arguments)
    return {
        "step_id": step.step_id,
        "action_type": step.skill_name,
        "task_id": command_id,
        "task_family": "gpsr_scheduler",
        "target_object": str(args.get("object_name", "")),
        "target_person": str(args.get("target_person", "")),
        "target_location": str(args.get("target_location", "")),
        "target_category": str(args.get("category", "")),
        "parameters_json": json.dumps(args, sort_keys=True),
        "timeout_sec": float(step.timeout),
        "max_retries": int(step.retry_count),
        "requires_success": list(step.dependencies),
        "requires_data": list(args.get("requires_data", [])),
        "result_variable": str(step.result_variable or ""),
    }


def plan_to_dict(result: SchedulerResult) -> dict[str, Any]:
    """Special function: Convert a scheduler result into deterministic JSON-friendly data."""
    return {
        "command_id": result.command_id,
        "valid": result.valid,
        "task_request": None if result.task_request is None else {
            "command_id": result.task_request.command_id,
            "task_type": result.task_request.task_type,
            "task_location": result.task_request.task_location,
            "report_location": result.task_request.report_location,
            "return_location": result.task_request.return_location,
            "target": result.task_request.target,
            "filters": result.task_request.filters,
            "recipient": result.task_request.recipient,
            "output_template": result.task_request.output_template,
            "return_after_completion": result.task_request.return_after_completion,
        },
        "steps": [
            {
                "step_id": step.step_id,
                "skill_name": step.skill_name,
                "arguments": step.arguments,
                "dependencies": step.dependencies,
                "timeout": step.timeout,
                "retry_count": step.retry_count,
                "result_variable": step.result_variable,
            }
            for step in result.steps
        ],
        "issues": [{"code": issue.code, "message": issue.message} for issue in result.issues],
    }
