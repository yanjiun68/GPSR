"""Section: Hardware-free canonical semantic compiler acceptance tests."""

from __future__ import annotations

from dataclasses import replace

import pytest

from gpsr_task_scheduler.config import default_config
from gpsr_task_scheduler.scheduler import schedule_command
from robot_yasmin_tasks.gpsr.action_types import CANONICAL_PRIMITIVES


def _semantic_signature(command: str):
    """Special function: Return intent and slots while ignoring surface wording."""
    result = schedule_command("semantic_test", command, default_config())
    assert result.command_context is not None
    assert result.command_context.validation["valid"], result.command_context.validation
    return [
        (step.intent, step.arguments, step.attributes)
        for step in result.command_context.semantic_steps
    ]


def _all_capabilities_config():
    """Special function: Mark registered simulated/missing primitives available for compiler tests."""
    config = default_config()
    config.primitive_capabilities = {
        name: replace(
            capability,
            implementation="real",
            production_enabled=True,
        )
        for name, capability in config.primitive_capabilities.items()
    }
    return config


# Section: Synonym invariance uses one canonical semantic intent for every phrase family.
@pytest.mark.parametrize(
    "command",
    ["Find John.", "Locate John.", "Look for John.", "Search for John."],
)
def test_find_person_synonyms_are_semantically_identical(command: str) -> None:
    assert _semantic_signature(command) == _semantic_signature("Find John.")


@pytest.mark.parametrize(
    "command",
    ["Find the Coke.", "Locate the Coke.", "Look for the Coke.", "Search for the Coke."],
)
def test_find_object_synonyms_are_semantically_identical(command: str) -> None:
    assert _semantic_signature(command) == _semantic_signature("Find the Coke.")


@pytest.mark.parametrize(
    "command",
    [
        "Go to the kitchen.",
        "Navigate to the kitchen.",
        "Head to the kitchen.",
        "Proceed to the kitchen.",
    ],
)
def test_navigation_synonyms_are_semantically_identical(command: str) -> None:
    assert _semantic_signature(command) == _semantic_signature("Go to the kitchen.")


def test_llm_rephrase_retains_raw_and_normalized_text_separately() -> None:
    """Special function: Preserve ASR evidence while compiling only normalized language."""
    raw = "could you uh go kitchen and find coke"
    normalized = "Go to the kitchen and find the Coke."
    result = schedule_command(
        "rephrased",
        raw,
        default_config(),
        normalized_instruction=normalized,
    )

    assert result.valid
    assert result.command_context.raw_text == raw
    assert result.command_context.normalized_text == normalized
    assert [step.intent for step in result.command_context.semantic_steps] == [
        "NAVIGATE", "FIND_OBJECT",
    ]


def test_llm_critical_attribute_drift_is_rejected() -> None:
    """Special function: Prevent a changed LLM color from reaching queue compilation."""
    result = schedule_command(
        "drift",
        "bring the red cup to John",
        default_config(),
        normalized_instruction="bring the blue cup to John",
    )

    assert not result.valid
    assert {issue.code for issue in result.issues} == {"LLM_SEMANTIC_DRIFT"}
    assert result.steps == []


def test_color_attribute_is_preserved_in_semantics_and_queue_filters() -> None:
    """Special function: Change only the requested color between equivalent object searches."""
    red = schedule_command("red", "Find the red cup.", default_config())
    blue = schedule_command("blue", "Find the blue cup.", default_config())

    assert red.command_context.semantic_steps[0].attributes == {"color": "red"}
    assert blue.command_context.semantic_steps[0].attributes == {"color": "blue"}
    assert {item["value"] for item in red.steps[0].arguments["filters"]} >= {"red"}
    assert {item["value"] for item in blue.steps[0].arguments["filters"]} >= {"blue"}


@pytest.mark.parametrize("relation", ["largest", "smallest"])
def test_size_relation_is_preserved(relation: str) -> None:
    """Special function: Retain superlative object constraints for downstream perception."""
    result = schedule_command(
        relation,
        f"Find the {relation} drink.",
        default_config(),
    )

    assert result.command_context.semantic_steps[0].attributes["size_relation"] == relation
    assert {
        (item["feature"], item["value"])
        for item in result.steps[0].arguments["filters"]
    } >= {("size_relation", relation)}


def test_find_disambiguates_person_and_object_registries() -> None:
    """Special function: Resolve FIND from the authoritative entity type."""
    assert _semantic_signature("Find John.")[0][0] == "FIND_PERSON"
    assert _semantic_signature("Find Coke.")[0][0] == "FIND_OBJECT"


def test_person_pronoun_resolves_to_previous_named_person() -> None:
    """Special function: Reuse an explicit person antecedent for FOLLOW_PERSON."""
    result = schedule_command(
        "person_reference",
        "Find John and follow him.",
        default_config(),
    )

    assert result.valid
    assert [step.intent for step in result.command_context.semantic_steps] == [
        "FIND_PERSON", "FOLLOW_PERSON",
    ]
    assert result.command_context.semantic_steps[1].arguments["target_person"] == "john"
    assert result.command_context.references["clause_2"] == {"him": "john"}


def test_object_and_operator_references_resolve_without_guessing() -> None:
    """Special function: Resolve IT from history and ME from the instruction-person alias."""
    result = schedule_command(
        "object_reference",
        "Find the Coke and bring it to me.",
        _all_capabilities_config(),
    )

    assert result.valid
    delivery = result.command_context.semantic_steps[1]
    assert delivery.intent == "DELIVER_OBJECT"
    assert delivery.arguments["object_name"] == "coke"
    assert delivery.arguments["target_person"] == "operator"
    assert result.command_context.references["clause_2"] == {
        "it": "coke",
        "me": "operator",
    }


def test_contextual_take_forms_compile_to_distinct_semantics() -> None:
    """Special function: Resolve ambiguous TAKE using object/person and destination slots."""
    config = _all_capabilities_config()
    pick = schedule_command("pick", "Take the Coke.", config)
    deliver = schedule_command("deliver", "Take the Coke to John.", config)
    guide = schedule_command("guide", "Take John to the kitchen.", config)

    assert pick.command_context.semantic_steps[0].intent == "PICK_OBJECT"
    assert deliver.command_context.semantic_steps[0].intent == "DELIVER_OBJECT"
    assert guide.command_context.semantic_steps[0].intent == "GUIDE_PERSON"
    assert [step.skill_name for step in pick.steps] == ["FIND_OBJECT", "PICK_OBJECT"]
    assert [step.skill_name for step in deliver.steps][-1] == "HANDOVER_OBJECT"
    assert [step.skill_name for step in guide.steps] == ["GUIDE_PERSON"]


def test_clause_order_changes_the_executable_queue() -> None:
    """Special function: Preserve explicit chronological order instead of using an intent bag."""
    find_first = schedule_command(
        "find_first",
        "Find John, then go to the kitchen.",
        default_config(),
    )
    navigate_first = schedule_command(
        "navigate_first",
        "Go to the kitchen, then find John.",
        default_config(),
    )

    assert [step.skill_name for step in find_first.steps] == ["FIND_PERSON", "NAVIGATE"]
    assert [step.skill_name for step in navigate_first.steps] == ["NAVIGATE", "FIND_PERSON"]


def test_leading_before_clause_is_reordered_chronologically() -> None:
    """Special function: Execute the subordinate action after the explicit BEFORE action."""
    result = schedule_command(
        "before",
        "Before going to the kitchen, find John.",
        default_config(),
    )

    assert [step.intent for step in result.command_context.semantic_steps] == [
        "FIND_PERSON", "NAVIGATE",
    ]


def test_leading_after_clause_preserves_chronological_order() -> None:
    """Special function: Normalize an AFTER gerund without reversing its chronology."""
    result = schedule_command(
        "after",
        "After finding John, go to the kitchen.",
        default_config(),
    )

    assert [step.intent for step in result.command_context.semantic_steps] == [
        "FIND_PERSON", "NAVIGATE",
    ]


def test_while_connector_retains_both_requested_steps_deterministically() -> None:
    """Special function: Preserve both clauses sequentially when no concurrent queue exists."""
    result = schedule_command(
        "while",
        "Find John while say hello.",
        default_config(),
    )

    assert [step.intent for step in result.command_context.semantic_steps] == [
        "FIND_PERSON", "SAY",
    ]
    assert result.command_context.semantic_steps[1].relationship == "while"


def test_quantity_and_location_aliases_are_canonicalized() -> None:
    """Special function: Preserve quantity and map a configured lounge alias to living_room."""
    result = schedule_command(
        "aliases",
        "Find 2 cups in the lounge.",
        default_config(),
    )

    semantic = result.command_context.semantic_steps[0]
    assert semantic.arguments["object_name"] == "cup"
    assert semantic.arguments["target_location"] == "living_room"
    assert semantic.attributes["quantity"] == 2


@pytest.mark.parametrize(
    "command",
    ["Bring it to John.", "Go there."],
)
def test_unresolved_references_block_compilation(command: str) -> None:
    """Special function: Reject missing antecedents without inventing an entity."""
    result = schedule_command("unresolved", command, default_config())

    assert not result.valid
    assert {issue.code for issue in result.issues} == {"UNRESOLVED_REFERENCE"}
    assert result.steps == []


def test_delivery_macro_uses_existing_planner_and_only_registered_primitives() -> None:
    """Special function: Expand delivery through authoritative acquisition/handover behavior."""
    result = schedule_command(
        "delivery",
        "Bring the Coke from the kitchen to John.",
        _all_capabilities_config(),
    )

    assert result.valid
    semantic = result.command_context.semantic_steps[0]
    assert semantic.intent == "DELIVER_OBJECT"
    assert semantic.arguments["source_location"] == "kitchen"
    assert [step.skill_name for step in result.steps] == [
        "FIND_OBJECT", "PICK_OBJECT", "FIND_PERSON", "HANDOVER_OBJECT",
    ]
    assert all(step.skill_name in CANONICAL_PRIMITIVES for step in result.steps)
    assert all(step.skill_name not in {"DELIVER_OBJECT", "PICK"} for step in result.steps)


def test_longest_phrase_matching_keeps_pick_up_as_one_intent() -> None:
    """Special function: Match PICK UP before the shorter PICK alias."""
    result = schedule_command(
        "longest",
        "Pick up the Coke.",
        _all_capabilities_config(),
    )

    assert result.command_context.semantic_steps[0].intent == "PICK_OBJECT"
    assert [step.skill_name for step in result.steps] == ["FIND_OBJECT", "PICK_OBJECT"]
