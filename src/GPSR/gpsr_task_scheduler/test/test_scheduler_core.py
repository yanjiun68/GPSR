"""Section: Deterministic scheduler unit tests."""

# Section: Test fixtures, deterministic fakes, and regression scenarios.

from __future__ import annotations

import itertools

import pytest

from gpsr_task_scheduler.config import default_config
from gpsr_task_scheduler.models import SkillStep
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
    """Special function: Multiple feature words become object-Qwen perception filters."""
    result = planned_result("find coke with red and blue features in the kitchen tell me")
    filters = result.task_request.filters
    assert {"feature": "object_color", "operator": "equals", "value": "red"} in filters
    assert {"feature": "object_color", "operator": "equals", "value": "blue"} in filters


def test_human_pose_filter_is_structured() -> None:
    """Special function: Human pose phrases become structured pose filters."""
    result = planned_result("find person with right hand raised in the kitchen tell me")
    assert {"feature": "pose", "operator": "equals", "value": "right_hand_raised"} in result.task_request.filters
    assert result.task_request.person_selector == {"all": [{"type": "gesture", "operator": "equals", "value": "right_hand_raised"}]}
    assert [step.skill_name for step in result.steps] == ["FIND_PERSON_BY_ATTRIBUTE", "SAY"]
    assert result.steps[0].arguments["search_scope"] == "STRICT_LOCATION"


@pytest.mark.parametrize(
    "phrase",
    ["right hand raised", "right arm raised", "raising right hand", "raising right arm"],
)
def test_right_arm_stt_variants_select_attribute_route(phrase: str) -> None:
    """Special function: Supported STT arm variants produce the same attribute action."""
    result = planned_result(f"find a person {phrase} in the living room")
    assert result.task_request.person_selector == {"all": [{"type": "gesture", "operator": "equals", "value": "right_hand_raised"}]}
    assert result.steps[0].skill_name == "FIND_PERSON_BY_ATTRIBUTE"


def test_unspecified_waving_selects_arm_agnostic_gesture() -> None:
    """Special function: Waving without a named side remains one arm-agnostic selector."""
    result = planned_result("find a person waving in the living room")
    assert result.task_request.person_selector == {
        "all": [{"type": "gesture", "operator": "equals", "value": "waving"}]
    }
    assert result.steps[0].skill_name == "FIND_PERSON_BY_ATTRIBUTE"


def test_named_person_selects_identity_route() -> None:
    """Special function: A configured searched name produces only FIND_PERSON."""
    result = planned_result("find john in the living room")
    assert result.task_request.person_selector == {"type": "name", "value": "john"}
    assert [step.skill_name for step in result.steps] == ["FIND_PERSON"]
    assert result.steps[0].arguments["target_person"] == "john"
    assert result.steps[0].arguments["search_scope"] == "STRICT_LOCATION"


def test_location_free_person_search_builds_one_global_compound_skill() -> None:
    """Special function: Missing location selects YAML waypoint search instead of rejection."""
    result = planned_result("find a person")
    assert [step.skill_name for step in result.steps] == ["FIND_PERSON_BY_ATTRIBUTE"]
    assert result.steps[0].arguments["search_scope"] == "GLOBAL"
    assert result.steps[0].arguments["target_location"] is None


def test_location_free_named_person_search_builds_one_global_identity_skill() -> None:
    """Special function: A known name can be searched across every configured waypoint."""
    result = planned_result("find john")
    assert [step.skill_name for step in result.steps] == ["FIND_PERSON"]
    assert result.steps[0].arguments["search_scope"] == "GLOBAL"


def test_name_and_attribute_combination_is_rejected() -> None:
    """Special function: Mixed selectors cannot silently choose one person state machine."""
    result = schedule_command(
        "conflicting_person",
        "find john with right hand raised in the living room",
        default_config(),
    )
    assert not result.valid
    assert {issue.code for issue in result.issues} == {"conflicting_person_selectors"}


def test_any_person_selects_attribute_route() -> None:
    """Special function: Generic person search becomes an explicit any-person selector."""
    result = planned_result("find a person in the living room")
    assert result.task_request.person_selector == {"type": "any_person", "value": ""}
    assert result.steps[0].skill_name == "FIND_PERSON_BY_ATTRIBUTE"


def test_clothing_attribute_selects_tracked_appearance_route() -> None:
    """Special function: Shirt-color requests enter the lifecycle appearance state machine."""
    result = planned_result("find a person wearing red in the living room")
    assert result.task_request.person_selector == {
        "all": [{"type": "shirt_color", "operator": "equals", "value": "red"}]
    }
    assert result.steps[0].skill_name == "FIND_PERSON_BY_ATTRIBUTE"


def test_combined_attributes_and_guest_fields_create_extract_step() -> None:
    """Special function: Combined positive predicates are followed by guest extraction."""
    result = planned_result(
        "find a person wearing red with glasses in the kitchen tell me their name and favorite drink"
    )
    assert result.task_request.person_selector == {
        "all": [
            {"type": "shirt_color", "operator": "equals", "value": "red"},
            {"type": "glasses", "operator": "equals", "value": "yes"},
        ]
    }
    assert result.task_request.requested_person_fields == ["name", "favorite_drink"]
    assert [step.skill_name for step in result.steps] == [
        "FIND_PERSON_BY_ATTRIBUTE", "EXTRACT_PERSON_INFO", "SAY",
    ]
    assert result.steps[1].arguments["fields"] == ["name", "favorite_drink"]


def test_unconfigured_hat_model_is_rejected_during_planning() -> None:
    """Special function: Prevent an impossible positive wearable search from executing."""
    result = schedule_command(
        "missing_hat_model",
        "find a person wearing a hat in the kitchen",
        default_config(),
    )
    assert not result.valid
    assert {issue.code for issue in result.issues} == {
        "unavailable_person_attribute_model",
    }


def test_unresolved_pronoun_is_rejected() -> None:
    """Special function: Unresolved target references request clarification without a queue."""
    result = schedule_command("bad", "find it in the kitchen", default_config())
    assert not result.valid
    assert {issue.code for issue in result.issues} == {"CLARIFICATION_REQUIRED"}
    assert result.command_context.validation["status"] == "MISSING_TARGET"


def test_unknown_location_is_rejected() -> None:
    """Special function: Unsupported locations are rejected instead of guessed."""
    result = schedule_command("bad", "count people in the garage tell me", default_config())
    assert not result.valid
    assert "unknown_location" in {issue.code for issue in result.issues}


def test_missing_task_location_is_rejected() -> None:
    """Special function: Location-dependent tasks request a missing room."""
    result = schedule_command("bad", "count people tell me", default_config())
    assert not result.valid
    assert {issue.code for issue in result.issues} == {"CLARIFICATION_REQUIRED"}
    assert result.command_context.validation["status"] == "MISSING_LOCATION"


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


def test_report_back_to_me_is_deferred_until_after_scheduler_return() -> None:
    """Special function: Keep operator-relative reporting outside the pre-return queue."""
    result = planned_result("find bottles and report back to me")
    assert result.task_request.report_after_return is True
    assert [step.skill_name for step in result.steps] == ["FIND_OBJECT"]
    assert [step.skill_name for step in result.post_return_steps] == ["SAY"]
    assert result.post_return_steps[0].arguments["substitution"] == "task_result"


def test_missing_navigation_location_serializes_as_empty_string() -> None:
    """Special function: Prevent optional ROS string fields from becoming literal None."""
    converted = step_to_action_step_dict(
        SkillStep("raw_pose", "NAVIGATE", {"target_location": None}),
        "command_1",
    )
    assert converted["target_location"] == ""


def test_return_after_success_appends_instruction_point() -> None:
    """Special function: Mission policy appends return-to-instruction-point after success."""
    config = default_config()
    config.return_after_completion = True
    result = schedule_command("returning", "count people in the kitchen tell me", config)
    assert result.valid
    assert [step.skill_name for step in result.steps] == ["NAVIGATE", "COUNT_PEOPLE", "SAY", "NAVIGATE"]
    assert result.steps[-1].step_id == "return_to_instruction_point"


def test_standalone_go_compiles_to_navigation_primitive() -> None:
    """Special function: Surface navigation verbs never enter ActionQueue identifiers."""
    result = planned_result("go to the kitchen")
    assert [step.skill_name for step in result.steps] == ["NAVIGATE"]
    assert result.steps[0].arguments["target_location"] == "kitchen"


def test_take_expands_then_rejects_unavailable_pick() -> None:
    """Special function: Acquisition exposes its complete proposed primitive plan safely."""
    result = schedule_command("take", "take the coke", default_config())
    assert not result.valid
    assert [step.skill_name for step in result.steps] == ["FIND_OBJECT", "PICK_OBJECT"]
    assert {issue.code for issue in result.issues} == {"primitive_unavailable"}
    assert "PICK_OBJECT" in result.issues[0].message


def test_place_acquires_target_before_dependent_place() -> None:
    """Special function: Placement must find and pick its target before invoking PLACE_OBJECT."""
    result = schedule_command(
        "place",
        "place the coke in the kitchen",
        default_config(),
    )
    # Section: Missing manipulation implementations retain the proposed queue for diagnostics.
    assert not result.valid
    assert [step.skill_name for step in result.steps] == [
        "FIND_OBJECT", "PICK_OBJECT", "PLACE_OBJECT",
    ]
    assert result.steps[0].arguments["search_scope"] == "GLOBAL"
    assert result.steps[1].dependencies == ["find_object"]
    assert result.steps[1].arguments["requires_data"] == ["found_object"]
    assert result.steps[2].dependencies == ["pick_object"]
    assert result.steps[2].arguments["requires_data"] == ["picked_object"]
    assert result.steps[2].arguments["target_location"] == "kitchen"
    assert {issue.code for issue in result.issues} == {"primitive_unavailable"}
    assert "PICK_OBJECT" in result.issues[0].message
    assert "PLACE_OBJECT" in result.issues[0].message


def test_bring_expands_then_rejects_unavailable_manipulation() -> None:
    """Special function: Delivery expansion is centralized in the scheduler compiler."""
    # Section: A specified source satisfies the new semantic completeness gate.
    result = schedule_command("bring", "bring me the coke from the kitchen", default_config())
    assert not result.valid
    assert [step.skill_name for step in result.steps] == [
        "FIND_OBJECT", "PICK_OBJECT", "NAVIGATE", "HANDOVER_OBJECT",
    ]
    assert "HANDOVER_OBJECT" in result.issues[0].message
    assert "PICK_OBJECT" in result.issues[0].message


def test_unknown_capitalized_find_target_reports_unknown_person() -> None:
    """Special function: Unknown proper names do not fall through to object search."""
    result = schedule_command("unknown_person", "find Alex", default_config())
    assert not result.valid
    assert {issue.code for issue in result.issues} == {"unknown_person"}


def test_unknown_table_reports_location_or_surface() -> None:
    """Special function: On-surface phrases receive a precise deterministic diagnostic."""
    result = schedule_command("unknown_surface", "count the drinks on the table", default_config())
    assert not result.valid
    assert "unknown_location_or_surface" in {issue.code for issue in result.issues}


def test_follow_known_person_uses_enabled_real_capability() -> None:
    """Special function: A known person now schedules through the production follow adapter."""
    result = schedule_command("follow", "follow john", default_config())
    assert result.valid
    assert [step.skill_name for step in result.steps] == ["FOLLOW_PERSON"]
    assert result.steps[0].arguments["target_person"] == "john"
    assert result.issues == []


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
