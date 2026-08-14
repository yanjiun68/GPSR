"""Section: Deterministic scheduler unit tests."""

from __future__ import annotations

import itertools

import pytest

from gpsr_task_scheduler.config import default_config
from gpsr_task_scheduler.planner import step_to_action_step_dict
from gpsr_task_scheduler.scheduler import schedule_command


def skill_names(instruction: str) -> list[str]:
    """Special function: Return ordered skill names for one deterministic command."""
    result = schedule_command("test_command", instruction, default_config())
    assert result.valid, [issue.message for issue in result.issues]
    return [step.skill_name for step in result.steps]


def planned_result(instruction: str):
    """Special function: Return one valid scheduler result for detailed assertions."""
    result = schedule_command("test_command", instruction, default_config())
    assert result.valid, [issue.message for issue in result.issues]
    return result


@pytest.mark.parametrize(
    "parts",
    itertools.permutations(["count people", "in the kitchen", "tell me"], 3),
)
def test_single_location_permutations_generate_task_then_tts(parts: tuple[str, str, str]) -> None:
    """Special function: All six phrase orders still produce task-location execution before TTS."""
    instruction = " ".join(parts)
    assert skill_names(instruction) == ["NAVIGATE", "COUNT_PEOPLE", "SAY"]


@pytest.mark.parametrize(
    "parts",
    itertools.permutations(["count people", "in the kitchen", "go to the living room", "tell me"], 4),
)
def test_two_location_permutations_generate_report_navigation(parts: tuple[str, str, str, str]) -> None:
    """Special function: All twenty-four phrase orders still navigate to report location after task execution."""
    instruction = " ".join(parts)
    assert skill_names(instruction) == ["NAVIGATE", "COUNT_PEOPLE", "NAVIGATE", "SAY"]


def test_named_recipient_inserts_find_person_before_tts() -> None:
    """Special function: Recipient delivery inserts person finding before speech."""
    assert skill_names("count people in the kitchen go to the living room tell john") == ["NAVIGATE", "COUNT_PEOPLE", "NAVIGATE", "FIND_PERSON", "SAY"]


def test_multiple_object_features_are_structured_filters() -> None:
    """Special function: Multiple feature words become structured perception filters."""
    result = planned_result("find coke with red and blue features in the kitchen tell me")
    filters = result.task_request.filters
    assert {"feature": "shirt_color", "operator": "equals", "value": "red"} in filters
    assert {"feature": "object_color", "operator": "equals", "value": "blue"} in filters


def test_human_pose_filter_is_structured() -> None:
    """Special function: Human pose phrases become structured pose filters."""
    result = planned_result("find person with right hand raised in the kitchen tell me")
    assert {"feature": "pose", "operator": "equals", "value": "right_hand_raised"} in result.task_request.filters


def test_unresolved_pronoun_is_rejected() -> None:
    """Special function: Ambiguous pronoun-only commands are rejected deterministically."""
    result = schedule_command("bad", "find it in the kitchen", default_config())
    assert not result.valid
    assert "unresolved_pronoun" in {issue.code for issue in result.issues}


def test_unknown_location_is_rejected() -> None:
    """Special function: Unsupported locations are rejected instead of guessed."""
    result = schedule_command("bad", "count people in the garage tell me", default_config())
    assert not result.valid
    assert "unknown_location" in {issue.code for issue in result.issues}


def test_missing_task_location_is_rejected() -> None:
    """Special function: Location-dependent tasks require a task location."""
    result = schedule_command("bad", "count people tell me", default_config())
    assert not result.valid
    assert "missing_task_location" in {issue.code for issue in result.issues}


def test_missing_report_location_is_rejected_when_destination_requested() -> None:
    """Special function: Report destination requests require a known report location."""
    result = schedule_command("bad", "count people in the kitchen go to the garage tell me", default_config())
    assert not result.valid
    assert "unknown_location" in {issue.code for issue in result.issues}


def test_task_result_substitution_is_passed_to_tts_arguments() -> None:
    """Special function: TTS step carries deterministic task-result substitution metadata."""
    result = planned_result("count people in the kitchen go to the living room tell me")
    assert result.steps[-1].skill_name == "SAY"
    assert result.steps[-1].arguments["substitution"] == "task_result"


def test_return_after_success_appends_instruction_point() -> None:
    """Special function: Mission policy appends return-to-instruction-point after success."""
    config = default_config()
    config.return_after_completion = True
    result = schedule_command("returning", "count people in the kitchen tell me", config)
    assert result.valid
    assert [step.skill_name for step in result.steps] == ["NAVIGATE", "COUNT_PEOPLE", "SAY", "RETURN_TO_INSTRUCTION_POINT"]


def test_action_step_conversion_preserves_timeout_and_arguments() -> None:
    """Special function: Scheduler steps convert into existing ActionStep-compatible dictionaries."""
    result = planned_result("find coke in the kitchen tell me")
    converted = step_to_action_step_dict(result.steps[0], result.command_id)
    assert converted["action_type"] == "FIND_OBJECT"
    assert converted["target_object"] == "coke"
    assert converted["timeout_sec"] == 300.0


def test_find_chairs_builds_global_compound_skill() -> None:
    """Special function: A location-free object command becomes one global FIND_OBJECT."""
    result = planned_result("find chairs")
    assert [step.skill_name for step in result.steps] == ["FIND_OBJECT"]
    assert result.task_request.task_location is None
    assert result.steps[0].arguments["object_name"] == "chair"
    assert result.steps[0].arguments["search_scope"] == "GLOBAL"
    assert result.steps[0].dependencies == []


def test_find_chairs_in_entrance_builds_strict_compound_skill() -> None:
    """Special function: An explicit location stays inside the FIND_OBJECT goal boundary."""
    result = planned_result("find chairs in entrance")
    assert [step.skill_name for step in result.steps] == ["FIND_OBJECT"]
    assert result.steps[0].arguments["location"] == "entrance"
    assert result.steps[0].arguments["search_scope"] == "STRICT_LOCATION"


def test_find_object_never_receives_automatic_prenavigate() -> None:
    """Special function: Planner must not split explicit-location FIND_OBJECT into two skills."""
    assert skill_names("find bottle in the kitchen") == ["FIND_OBJECT"]


def test_dry_run_plan_is_complete_without_ros_hardware() -> None:
    """Special function: Core dry-run planning remains pure Python and hardware-independent."""
    result = planned_result("count people in the kitchen tell me")
    assert len(result.steps) == 3
    assert result.steps[0].dependencies == []
    assert result.steps[1].dependencies == ["go_to_task_location"]
