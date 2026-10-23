"""Section: Best-effort ActionQueue dependency, reporting, and compiler regressions."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from gpsr_task_scheduler.config import default_config
from gpsr_task_scheduler.models import TaskRequest
from gpsr_task_scheduler.planner import plan_task
from gpsr_task_scheduler.qwen_action_queue_bridge import parse_qwen_action_queue
from gpsr_task_scheduler.scheduler_node import GpsrTaskSchedulerNode
from gpsr_task_scheduler.semantic_compiler import compile_command
from robot_yasmin_tasks.gpsr.models import (
    action_queue_result_statistics,
    summarize_action_queue_results,
)
from robot_yasmin_tasks.gpsr.outcomes import (
    PARTIAL_SUCCESS,
    SKIPPED_DEPENDENCY_FAILED,
    is_fatal,
    is_recoverable,
)


# Section: Locate the compiled executor source for policy-order regression guards.
EXECUTOR_SOURCE = (
    Path(__file__).resolve().parents[2]
    / "robot_yasmin_tasks"
    / "src"
    / "nodes"
    / "orchestration"
    / "gpsr_executor.cpp"
)


def _config():
    """Special function: Build the production-default deterministic compiler configuration."""
    return default_config()


def _steps(command: str):
    """Special function: Compile one multi-clause command and expose its generated steps."""
    return compile_command("best_effort_test", command, _config()).steps


def test_room_arrival_guards_strict_object_search():
    steps = _steps("Go to the living room and find the cola.")
    assert [step.skill_name for step in steps] == ["NAVIGATE", "FIND_OBJECT"]
    assert steps[1].dependencies == [steps[0].step_id]


@pytest.mark.parametrize(
    ("command", "object_name", "phrase"),
    [
        (
            "Go to the living room, find the cola, and tell me where it is relative to the furniture.",
            "coke",
            "I found the {name} {furniture_relative_phrase}.",
        ),
        (
            "Go to the dining room, find the bowl, and say where it is relative to the furniture.",
            "bowl",
            "The {name} {furniture_relative_phrase}.",
        ),
    ],
)
def test_furniture_relative_report_consumes_grounded_find_result(command, object_name, phrase):
    steps = _steps(command)
    assert [step.skill_name for step in steps] == ["NAVIGATE", "FIND_OBJECT", "SAY"]
    navigate, find_object, say = steps
    assert find_object.arguments["object_name"] == object_name
    assert find_object.dependencies == [navigate.step_id]
    assert "furniture_relative_location" in find_object.arguments["query"]["requested_fields"]
    assert find_object.result_variable == "found_object"
    assert say.dependencies == [find_object.step_id]
    assert say.arguments == {
        "text_template": phrase,
        "substitution": "found_object",
        "requires_data": ["found_object"],
    }


def test_independent_clause_has_no_artificial_success_dependency():
    """Special function: Verify queue order does not become a cross-clause dependency."""
    steps = _steps("find coke then count people in the living room")
    assert [step.skill_name for step in steps] == [
        "FIND_OBJECT",
        "NAVIGATE",
        "COUNT_PEOPLE",
    ]
    assert steps[1].dependencies == []
    assert steps[2].dependencies == [steps[1].step_id]


def test_planner_keeps_genuine_find_pick_place_dependencies():
    """Special function: Verify manipulation consumers retain their semantic producers."""
    request = TaskRequest(
        command_id="genuine_dependency",
        task_type="place_object",
        task_location="kitchen",
        report_location=None,
        return_location="instruction_point",
        target={"object_name": "coke"},
    )
    steps = plan_task(request, _config()).steps
    assert [step.skill_name for step in steps] == [
        "FIND_OBJECT",
        "PICK_OBJECT",
        "PLACE_OBJECT",
    ]
    assert steps[1].dependencies == [steps[0].step_id]
    assert steps[2].dependencies == [steps[1].step_id]


def test_compiler_reuse_points_to_actual_object_producers():
    """Special function: Verify pruned find/pick macros keep explicit earlier prerequisites."""
    find_then_pick = _steps("find coke then pick it up")
    assert [step.skill_name for step in find_then_pick] == ["FIND_OBJECT", "PICK_OBJECT"]
    assert find_then_pick[1].dependencies == [find_then_pick[0].step_id]
    assert find_then_pick[1].arguments["requires_data"] == ["found_object"]

    pick_then_deliver = _steps("pick up coke then deliver it to operator")
    pick_step = next(step for step in pick_then_deliver if step.skill_name == "PICK_OBJECT")
    delivery_navigation = next(
        step
        for step in pick_then_deliver
        if step.step_id.startswith("semantic_2_") and step.skill_name == "NAVIGATE"
    )
    assert delivery_navigation.dependencies == [pick_step.step_id]


def test_named_person_reuse_keeps_follow_dependency():
    """Special function: Verify a follow action still requires its earlier person producer."""
    steps = _steps("find john then follow him")
    assert [step.skill_name for step in steps] == ["FIND_PERSON", "FOLLOW_PERSON"]
    assert steps[1].dependencies == [steps[0].step_id]


def test_return_to_instruction_point_is_ordered_last_but_independent():
    """Special function: Verify ordinary task failure cannot suppress mission-final return."""
    config = _config()
    config.return_after_completion = True
    result = compile_command(
        "return_finalization",
        "find coke then count people in the living room",
        config,
    )
    assert result.steps[-1].step_id == "return_to_instruction_point"
    assert result.steps[-1].skill_name == "NAVIGATE"
    assert result.steps[-1].dependencies == []


def test_qwen_omitted_dependency_stays_empty_and_explicit_dependency_survives():
    """Special function: Verify Qwen array order and semantic edges remain separate."""
    payload = json.dumps({
        "task_id": "qwen_dependency_test",
        "actions": [
            {"step_id": "a", "skill": "SAY", "params": {"text": "one"}},
            {"step_id": "b", "skill": "SAY", "params": {"text": "two"}},
            {
                "step_id": "c",
                "skill": "SAY",
                "params": {"text": "three"},
                "requires_success": ["a"],
            },
        ],
    })
    queue = parse_qwen_action_queue(
        payload,
        default_timeout_sec=5.0,
        default_retries=0,
    )
    assert [list(step.requires_success) for step in queue.steps] == [[], [], ["a"]]


def test_qwen_compiles_extreme_object_report_into_find_and_say():
    """Special function: Execute semantic observation/report actions with existing primitives."""
    payload = json.dumps({
        "task_id": "qwen_extreme_report",
        "actions": [
            {"skill": "MOVE_TO", "params": {"location": "kitchen_table"}},
            {"skill": "OBSERVE", "params": {"location": "kitchen_table"}},
            {
                "skill": "FIND_EXTREME_OBJECT",
                "params": {"attribute": "size", "mode": "largest"},
            },
            {"skill": "SPEAK_RESULT", "params": {}},
        ],
    })

    queue = parse_qwen_action_queue(
        payload,
        default_timeout_sec=30.0,
        default_retries=0,
    )

    assert [step.action_type for step in queue.steps] == ["NAVIGATE", "FIND_OBJECT", "SAY"]
    find_step, say_step = queue.steps[1:]
    find_params = json.loads(find_step.parameters_json)
    say_params = json.loads(say_step.parameters_json)
    assert find_step.target_category == "objects"
    assert find_step.result_variable == "qwen_result_2"
    assert {
        "feature": "size_relation",
        "operator": "equals",
        "value": "largest",
    } in find_params["filters"]
    assert say_params == {
        "text_template": "The {name} is the largest object I perceived.",
        "substitution": "qwen_result_2",
    }
    assert list(say_step.requires_success) == [find_step.step_id]
    assert list(say_step.requires_data) == ["qwen_result_2"]


def test_qwen_compiles_perceived_color_report_and_requests_color_evidence():
    """Special function: Bind a color report to validated FIND_OBJECT evidence."""
    payload = json.dumps({
        "actions": [
            {"skill": "FIND_OBJECT", "params": {"object": "bottle"}},
            {"skill": "SPEAK_RESULT", "params": {"attribute": "color"}},
        ],
    })

    queue = parse_qwen_action_queue(
        payload,
        default_timeout_sec=30.0,
        default_retries=0,
    )

    find_step, say_step = queue.steps
    find_params = json.loads(find_step.parameters_json)
    say_params = json.loads(say_step.parameters_json)
    assert find_params["query"]["requested_fields"] == ["color"]
    assert say_params["text_template"] == (
        "The perceived color of the {name} is {attributes.color}."
    )
    assert say_params["substitution"] == find_step.result_variable


@pytest.mark.parametrize(
    ("mode", "relation"),
    [("nearest", "near"), ("furthest", "far_from")],
)
def test_qwen_extreme_distance_requires_and_preserves_reference(mode: str, relation: str):
    """Special function: Keep near/far claims explicitly relative and image-space."""
    payload = json.dumps({
        "actions": [{
            "skill": "FIND_EXTREME_OBJECT",
            "params": {"mode": mode, "reference_object": "cup"},
        }],
    })

    queue = parse_qwen_action_queue(
        payload,
        default_timeout_sec=30.0,
        default_retries=0,
    )
    params = json.loads(queue.steps[0].parameters_json)
    assert {
        "feature": "spatial_relation",
        "operator": "equals",
        "value": relation,
    } in params["filters"]
    assert {
        "feature": "reference_object",
        "operator": "equals",
        "value": "cup",
    } in params["filters"]


def test_qwen_rejects_result_speech_without_a_producer():
    """Special function: Prevent result speech from fabricating absent perception data."""
    with pytest.raises(ValueError, match="earlier result-producing action"):
        parse_qwen_action_queue(
            json.dumps({"actions": [{"skill": "SPEAK_RESULT", "params": {}}]}),
            default_timeout_sec=30.0,
            default_retries=0,
        )


@pytest.mark.parametrize("dependency", ["missing", "future"])
def test_qwen_rejects_unknown_or_future_dependencies(dependency: str):
    """Special function: Verify explicit malformed Qwen dependencies remain admission errors."""
    actions = [
        {
            "step_id": "a",
            "skill": "SAY",
            "params": {"text": "one"},
            "requires_success": [dependency],
        },
        {"step_id": "future", "skill": "SAY", "params": {"text": "two"}},
    ]
    with pytest.raises(ValueError, match="unknown or future"):
        parse_qwen_action_queue(
            json.dumps({"actions": actions}),
            default_timeout_sec=5.0,
            default_retries=0,
        )


def test_scheduler_counts_only_success_and_preserves_all_diagnostics():
    """Special function: Verify partial completion accounting and operator summary content."""
    results = [
        {"action_type": "NAVIGATE", "outcome": "success", "message": "arrived"},
        {"action_type": "FIND_OBJECT", "outcome": "not_found", "message": "no coke"},
        {
            "action_type": "PICK_OBJECT",
            "outcome": SKIPPED_DEPENDENCY_FAILED,
            "message": "find failed",
        },
        {"action_type": "COUNT_PEOPLE", "outcome": "success", "message": "counted"},
    ]
    assert action_queue_result_statistics(results) == {
        "successful_steps": 2,
        "failed_steps": 1,
        "skipped_steps": 1,
        "total_steps": 4,
    }
    summary = summarize_action_queue_results(PARTIAL_SUCCESS, results)
    assert summary.startswith("Queue partially completed.")
    assert "FIND_OBJECT: not_found - no coke" in summary
    assert "PICK_OBJECT: skipped_dependency_failed - find failed" in summary
    assert "COUNT_PEOPLE: success - counted" in summary
    assert not is_recoverable(SKIPPED_DEPENDENCY_FAILED)
    assert not is_fatal(SKIPPED_DEPENDENCY_FAILED)


def test_failed_result_variable_is_not_filled_from_later_same_type_success():
    """Special function: Keep best-effort result payloads correlated after an early failure."""
    planned = {
        "steps": [
            {
                "skill_name": "FIND_OBJECT",
                "result_variable": "first_object",
            },
            {
                "skill_name": "FIND_OBJECT",
                "result_variable": "second_object",
            },
        ]
    }
    task_result = {
        "results": [
            {"action_type": "SAY", "outcome": "success", "result": {}},
            {"action_type": "FIND_OBJECT", "outcome": "not_found", "result": {}},
            {
                "action_type": "FIND_OBJECT",
                "outcome": "success",
                "result": {"object": "water"},
            },
        ]
    }
    assert GpsrTaskSchedulerNode.extract_result_variables(planned, task_result) == {
        "second_object": {"object": "water"}
    }


def test_executor_source_preserves_best_effort_and_fatal_lifecycle_barriers():
    """Special function: Guard the compiled executor's dispatch ordering and fatal barriers."""
    source = EXECUTOR_SOURCE.read_text(encoding="utf-8")
    # Section: A dependency skip must be decided before lifecycle preparation.
    assert source.index('if (!failed_dependencies.empty())') < source.index(
        "lifecycle_->start(action.action_type)"
    )
    assert '"STEP_SKIPPED"' in source
    assert "SKIPPED_DEPENDENCY_FAILED" in source
    # Section: Cleanup and recovery uncertainty must still produce fatal queue termination.
    assert "const auto cleaned = lifecycle_->stop(action.action_type);" in source
    assert "const auto recovered = lifecycle_->recover(action.action_type);" in source
    assert source.count("FATAL_FAILURE") >= 3
    # Section: Normal aggregate results use ROS success while actual cancellation is distinct.
    assert "aggregate_queue_outcome(results)" in source
    assert "goal_handle->succeed(action_result)" in source
    assert "terminal == CANCELED && goal_handle->is_canceling()" in source
    # Section: FIND_OBJECT retains its compound-search-owned NOT_FOUND retry policy.
    assert "object_miss_owned_by_search_policy" in source
