"""Section: Voice command parser tests."""

# Section: Test fixtures, deterministic fakes, and regression scenarios.

from robot_yasmin_tasks.gpsr.action_types import (
    COUNT_PEOPLE,
    FIND_OBJECT,
    FIND_PERSON_BY_ATTRIBUTE,
    HANDOVER_OBJECT,
    NAVIGATE,
    PICK_OBJECT,
    SAY,
)
from robot_yasmin_tasks.gpsr.voice_command_parser import fallback_parse_transcript, parse_llm_pairs


def test_llm_pairs_to_steps() -> None:
    """Special function: LLM pairs convert into GPSR symbolic steps."""
    steps = parse_llm_pairs(["navigate|kitchen", "find_obj|coke,table", "grasp|coke", "handover|operator"])
    assert [step.action_type for step in steps] == [NAVIGATE, FIND_OBJECT, PICK_OBJECT, HANDOVER_OBJECT]


def test_llm_count_people_pair_to_navigation_sequence() -> None:
    """Special function: LLM count_people pairs navigate before counting a named location."""
    steps = parse_llm_pairs(["count_people|kitchen"])
    assert [step.action_type for step in steps] == [NAVIGATE, COUNT_PEOPLE]
    assert steps[0].target_location == "kitchen"
    assert steps[1].target_location == "kitchen"


def test_fallback_count_people_current_view() -> None:
    """Special function: Fallback parser supports current-view people count commands."""
    steps = fallback_parse_transcript("how many people are here")
    assert [step.action_type for step in steps] == [COUNT_PEOPLE]
    assert steps[0].target_location == ""


def test_fallback_count_people_location_sequence() -> None:
    """Special function: Fallback parser navigates before counting people in a room."""
    steps = fallback_parse_transcript("count people in the kitchen")
    assert [step.action_type for step in steps] == [NAVIGATE, COUNT_PEOPLE]
    assert steps[0].target_location == "kitchen"
    assert steps[1].target_location == "kitchen"


def test_fallback_say() -> None:
    """Special function: Fallback parser supports simple say command."""
    steps = fallback_parse_transcript("say hello")
    assert steps[0].action_type == SAY


def test_fallback_post_noun_right_arm_description_is_side_specific() -> None:
    """Special function: A post-noun right-arm phrase selects only the right raised-arm gesture."""
    steps = fallback_parse_transcript("find a person raising right arm in the living room")
    assert [step.action_type for step in steps] == [FIND_PERSON_BY_ATTRIBUTE]
    assert steps[0].parameters["selector"] == {"type": "gesture", "value": "right_hand_raised"}
    assert steps[0].target_location == "living_room"


def test_fallback_post_noun_waving_description_is_arm_agnostic() -> None:
    """Special function: Waving without a side selects the generic either-arm gesture."""
    steps = fallback_parse_transcript("find a person waving")
    assert [step.action_type for step in steps] == [FIND_PERSON_BY_ATTRIBUTE]
    assert steps[0].parameters["selector"] == {"type": "gesture", "value": "waving"}
