"""Section: Deterministic GPSR task planner."""

from __future__ import annotations

import json
from typing import Any

from gpsr_task_scheduler.config import SchedulerConfig
from gpsr_task_scheduler.models import SchedulerResult, SkillStep, TaskRequest, ValidationIssue
from robot_yasmin_tasks.gpsr.action_types import unavailable_primitives


TASK_TO_SKILL = {
    "find_object": "FIND_OBJECT",
    "find_person": "FIND_PERSON_BY_ATTRIBUTE",
    "count_objects": "COUNT_OBJECTS",
    "count_people": "COUNT_PEOPLE",
    "navigate": "NAVIGATE",
    "follow_person": "FOLLOW_PERSON",
    "guide_person": "GUIDE_PERSON",
    "say": "SAY",
    "ask_and_listen": "ASK_AND_LISTEN",
}

# Section: Compound parser tasks are compiler-only and never become ActionQueue identifiers.
COMPOUND_TASK_TYPES = {"acquire_object", "place_object", "deliver_object"}


def plan_task(request: TaskRequest, config: SchedulerConfig) -> SchedulerResult:
    """Special function: Build a physical task order from semantic roles."""
    issues = _validate_request(request)
    if issues:
        return SchedulerResult(request.command_id, request, [], issues)
    steps: list[SkillStep] = []
    # Section: Operator-relative reports execute only after the scheduler-owned return phase.
    post_return_steps: list[SkillStep] = []
    # Section: Standalone executable primitives bypass task-family helper expansion.
    if request.task_type == "navigate":
        steps.append(_step("navigate", "NAVIGATE", {"target_location": request.task_location}, [], config))
        return _validated_capability_result(request, steps, config)
    if request.task_type == "acquire_object":
        steps.extend(_acquire_steps(request, config))
        return _validated_capability_result(request, steps, config)
    if request.task_type == "place_object":
        # Section: Placement always acquires the named item before attempting to place it.
        steps.extend(_place_steps(request, config))
        return _validated_capability_result(request, steps, config)
    if request.task_type == "deliver_object":
        steps.extend(_delivery_steps(request, config))
        return _validated_capability_result(request, steps, config)
    if request.task_type in {"follow_person", "guide_person"}:
        action_type = TASK_TO_SKILL[request.task_type]
        steps.append(_step(
            request.task_type,
            action_type,
            {"target_person": request.target.get("target_person", ""), "target_location": request.task_location},
            [],
            config,
        ))
        return _validated_capability_result(request, steps, config)
    if request.task_type in {"say", "ask_and_listen"}:
        steps.extend(_communication_steps(request, config))
        return _validated_capability_result(request, steps, config)
    # Section: Every FIND skill is a compound mission and owns its search navigation.
    if request.task_type in {"find_object", "find_person"}:
        execute_step = _execute_step(request, None, config)
        steps.append(execute_step)
    else:
        steps.append(_step("go_to_task_location", "NAVIGATE", {"target_location": request.task_location}, [], config))
        execute_step = _execute_step(request, steps[-1].step_id, config)
        steps.append(execute_step)
    # Section: Attribute search stores PersonRef first, then extraction enriches the same GPSR memory record.
    task_result_step = execute_step
    if request.task_type == "find_person" and request.requested_person_fields and execute_step.skill_name == "FIND_PERSON_BY_ATTRIBUTE":
        task_result_step = _step(
            "extract_person_info",
            "EXTRACT_PERSON_INFO",
            {"fields": list(request.requested_person_fields), "requires_data": ["person_ref", "gpsr_guest_memory_id"]},
            [execute_step.step_id],
            config,
            "task_result",
        )
        steps.append(task_result_step)
    if request.report_location:
        steps.append(_step("go_to_report_location", "NAVIGATE", {"target_location": request.report_location}, [task_result_step.step_id], config))
    if request.recipient:
        dependency = steps[-1].step_id
        steps.append(_step("find_recipient", "FIND_PERSON", {"target_person": request.recipient.get("name", ""), "target_location": request.report_location or ""}, [dependency], config, "recipient"))
    if request.output_template:
        dependency = steps[-1].step_id
        report_step = _step(
            "tts_say_output",
            "SAY",
            {"text": request.output_template, "substitution": "task_result"},
            [dependency],
            config,
        )
        if request.report_after_return:
            post_return_steps.append(report_step)
        else:
            steps.append(report_step)
    if request.return_after_completion and request.return_location:
        dependency = steps[-1].step_id
        steps.append(_step("return_to_instruction_point", "NAVIGATE", {"target_location": request.return_location}, [dependency], config))
    return _validated_capability_result(
        request,
        steps,
        config,
        post_return_steps=post_return_steps,
    )


def _validate_request(request: TaskRequest) -> list[ValidationIssue]:
    """Special function: Reject unresolved or contradictory semantic plans."""
    issues: list[ValidationIssue] = []
    if request.task_type not in TASK_TO_SKILL and request.task_type not in COMPOUND_TASK_TYPES:
        issues.append(ValidationIssue("unknown_task", f"Unsupported task type: {request.task_type}."))
    if request.task_type in {"count_objects", "count_people", "navigate", "guide_person", "place_object"} and not request.task_location:
        issues.append(ValidationIssue("missing_task_location", "Task location is required."))
    if request.recipient and not request.output_template:
        issues.append(ValidationIssue("recipient_without_output", "Named recipient requires an output template."))
    if request.report_location is None and request.recipient:
        issues.append(ValidationIssue("missing_report_location", "Named recipient delivery requires a report location."))
    return issues


def _validated_capability_result(
    request: TaskRequest,
    steps: list[SkillStep],
    config: SchedulerConfig,
    post_return_steps: list[SkillStep] | None = None,
) -> SchedulerResult:
    """Special function: Retain proposed primitives while rejecting unavailable capabilities."""
    # Section: Capability admission covers both the task queue and deferred reporting queue.
    deferred_steps = list(post_return_steps or [])
    unavailable = unavailable_primitives(
        [step.skill_name for step in [*steps, *deferred_steps]],
        config.primitive_capabilities,
        config.allow_simulated_primitives,
    )
    issues = [
        ValidationIssue(
            "primitive_unavailable",
            f"Required executable primitives are unavailable: {unavailable}.",
        )
    ] if unavailable else []
    return SchedulerResult(
        request.command_id,
        request,
        steps,
        issues,
        deferred_steps,
    )


def _acquire_steps(request: TaskRequest, config: SchedulerConfig) -> list[SkillStep]:
    """Special function: Expand object acquisition into find then pick primitives."""
    target = str(request.target.get("object_name", ""))
    return _object_acquisition_steps(target, request.task_location, config)


def _object_acquisition_steps(
    target: str,
    search_location: str | None,
    config: SchedulerConfig,
) -> list[SkillStep]:
    """Special function: Build dependency-linked find and pick steps for one target item."""
    # Section: A source location constrains the search; otherwise FIND_OBJECT performs global search.
    find_arguments = {
        "object_name": target,
        "location": search_location or "",
        "target_location": search_location,
        "search_scope": "STRICT_LOCATION" if search_location else "GLOBAL",
        "approach_target": True,
    }
    find_step = _step("find_object", "FIND_OBJECT", find_arguments, [], config, "found_object")
    pick_step = _step(
        "pick_object",
        "PICK_OBJECT",
        {"object_name": target, "requires_data": ["found_object"]},
        [find_step.step_id],
        config,
        "picked_object",
    )
    return [find_step, pick_step]


def _place_steps(request: TaskRequest, config: SchedulerConfig) -> list[SkillStep]:
    """Special function: Expand placement into global acquisition followed by dependent placement."""
    target = str(request.target.get("object_name", ""))
    # Section: The parsed location is the placement destination, not an assumed pickup location.
    steps = _object_acquisition_steps(target, None, config)
    pick_step = steps[-1]
    # Section: PLACE_OBJECT may execute only after PICK_OBJECT produced the held-item result.
    place_step = _step(
        "place_object",
        "PLACE_OBJECT",
        {
            "object_name": target,
            "target_location": request.task_location,
            "requires_data": ["picked_object"],
        },
        [pick_step.step_id],
        config,
        "placed_object",
    )
    steps.append(place_step)
    return steps


def _delivery_steps(request: TaskRequest, config: SchedulerConfig) -> list[SkillStep]:
    """Special function: Expand bring/deliver into explicit acquisition and handover primitives."""
    steps = _acquire_steps(request, config)
    target_person = str(request.target.get("target_person", "operator") or "operator")
    dependency = steps[-1].step_id
    if target_person == "operator":
        destination_step = _step(
            "navigate_to_recipient",
            "NAVIGATE",
            {"target_location": "operator"},
            [dependency],
            config,
        )
    else:
        destination_step = _step(
            "find_recipient",
            "FIND_PERSON",
            {"target_person": target_person},
            [dependency],
            config,
            "recipient",
        )
    steps.append(destination_step)
    steps.append(_step(
        "handover_object",
        "HANDOVER_OBJECT",
        {"object_name": request.target.get("object_name", ""), "target_person": target_person},
        [destination_step.step_id],
        config,
    ))
    return steps


def _communication_steps(request: TaskRequest, config: SchedulerConfig) -> list[SkillStep]:
    """Special function: Compile literal communication into optional recipient search plus one voice primitive."""
    action_type = TASK_TO_SKILL[request.task_type]
    target_person = str(request.target.get("target_person", ""))
    arguments = dict(request.target)
    steps: list[SkillStep] = []
    dependencies: list[str] = []
    if target_person:
        recipient = _step(
            "find_recipient",
            "FIND_PERSON",
            {"target_person": target_person},
            [],
            config,
            "recipient",
        )
        steps.append(recipient)
        dependencies = [recipient.step_id]
    steps.append(_step("communicate", action_type, arguments, dependencies, config, "dialogue_result" if action_type == "ASK_AND_LISTEN" else None))
    return steps


def _execute_step(request: TaskRequest, dependency: str | None, config: SchedulerConfig) -> SkillStep:
    """Special function: Build the task execution step for the requested task family."""
    # Section: Person selector type determines an explicit, lifecycle-visible action route.
    skill_name = (
        "FIND_PERSON"
        if request.task_type == "find_person" and request.person_selector.get("type") == "name"
        else TASK_TO_SKILL[request.task_type]
    )
    arguments: dict[str, Any] = {"target_location": request.task_location, "filters": request.filters}
    arguments.update(request.target)
    if request.task_type == "find_person":
        arguments["selector"] = dict(request.person_selector)
        arguments["location"] = request.task_location or ""
        arguments["search_scope"] = "STRICT_LOCATION" if request.task_location else "GLOBAL"
        if request.person_selector.get("type") == "name":
            arguments["target_person"] = str(request.person_selector.get("value", ""))
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
        "target_object": _string_or_empty(args.get("object_name")),
        "target_person": _string_or_empty(args.get("target_person")),
        "target_location": _string_or_empty(args.get("target_location")),
        "target_category": _string_or_empty(args.get("category")),
        "parameters_json": json.dumps(args, sort_keys=True),
        "timeout_sec": float(step.timeout),
        "max_retries": int(step.retry_count),
        "requires_success": list(step.dependencies),
        "requires_data": list(args.get("requires_data", [])),
        "result_variable": str(step.result_variable or ""),
    }


def _string_or_empty(value: Any) -> str:
    """Special function: Convert optional ROS string fields without leaking literal None."""
    return "" if value is None else str(value)


def plan_to_dict(result: SchedulerResult) -> dict[str, Any]:
    """Special function: Convert a scheduler result into deterministic JSON-friendly data."""
    return {
        "command_id": result.command_id,
        "valid": result.valid,
        "command_context": (
            result.command_context.to_dict()
            if result.command_context is not None
            else None
        ),
        "task_request": None if result.task_request is None else {
            "command_id": result.task_request.command_id,
            "task_type": result.task_request.task_type,
            "task_location": result.task_request.task_location,
            "report_location": result.task_request.report_location,
            "return_location": result.task_request.return_location,
            "target": result.task_request.target,
            "filters": result.task_request.filters,
            "person_selector": result.task_request.person_selector,
            "requested_person_fields": result.task_request.requested_person_fields,
            "recipient": result.task_request.recipient,
            "output_template": result.task_request.output_template,
            "return_after_completion": result.task_request.return_after_completion,
            "report_after_return": result.task_request.report_after_return,
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
        "post_return_steps": [
            {
                "step_id": step.step_id,
                "skill_name": step.skill_name,
                "arguments": step.arguments,
                "dependencies": step.dependencies,
                "timeout": step.timeout,
                "retry_count": step.retry_count,
                "result_variable": step.result_variable,
            }
            for step in result.post_return_steps
        ],
        "issues": [{"code": issue.code, "message": issue.message} for issue in result.issues],
    }
