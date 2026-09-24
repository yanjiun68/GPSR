"""Section: Tests for the isolated Qwen JSON ActionQueue adapter."""

# Section: JSON and pytest support for parser/validation contract tests.
import json

import pytest

# Section: Import the bridge's pure queue conversion boundary without starting a ROS node.
from gpsr_task_scheduler.qwen_action_queue_bridge import parse_qwen_action_queue


def test_qwen_aliases_convert_to_canonical_executor_actions() -> None:
    """Special function: Verify common Qwen aliases become typed canonical actions."""
    payload = {
        "task_id": "qwen_test_001",
        "requires_clarification": False,
        "actions": [
            {"skill": "MOVE_TO", "params": {"location": "kitchen"}},
            {"skill": "FIND_OBJECT", "params": {"object": "coke"}},
            {"skill": "PICK", "params": {"object": "coke"}},
            {"skill": "HANDOVER", "params": {"object": "coke", "person": "requester"}},
        ],
    }

    queue = parse_qwen_action_queue(json.dumps(payload), default_timeout_sec=30.0, default_retries=0)

    assert queue.task_id == "qwen_test_001"
    assert [step.action_type for step in queue.steps] == [
        "NAVIGATE",
        "FIND_OBJECT",
        "PICK_OBJECT",
        "HANDOVER_OBJECT",
    ]
    assert queue.steps[1].target_object == "coke"
    assert queue.steps[3].target_person == "requester"
    assert queue.steps[3].requires_success == [queue.steps[2].step_id]


def test_criterion_person_search_uses_attribute_primitive() -> None:
    """Special function: Convert Qwen criteria searches to FIND_PERSON_BY_ATTRIBUTE."""
    payload = {
        "actions": [
            {
                "skill": "FIND_PERSON",
                "params": {"criteria": {"gesture": "waving"}},
            }
        ]
    }

    queue = parse_qwen_action_queue(json.dumps(payload), default_timeout_sec=30.0, default_retries=0)

    assert queue.steps[0].action_type == "FIND_PERSON_BY_ATTRIBUTE"
    assert json.loads(queue.steps[0].parameters_json)["selector"] == {"gesture": "waving"}


def test_qwen_extreme_search_maps_to_unified_find_object_constraints() -> None:
    """Special function: Preserve largest/smallest intent without creating a feature-specific skill."""
    payload = {
        "actions": [
            {
                "skill": "FIND_EXTREME_OBJECT",
                "params": {"object": "object", "mode": "largest"},
            }
        ]
    }

    queue = parse_qwen_action_queue(json.dumps(payload), default_timeout_sec=30.0, default_retries=0)

    assert queue.steps[0].action_type == "FIND_OBJECT"
    assert queue.steps[0].target_object == "object"
    assert json.loads(queue.steps[0].parameters_json)["filters"] == [
        {"feature": "size_relation", "operator": "equals", "value": "largest"},
    ]


def test_qwen_count_object_constraints_use_the_shared_perception_filter_list() -> None:
    """Special function: Preserve Qwen count attributes so the local server can query and verify ROIs."""
    # Section: COUNT_OBJECTS stays aggregate while sharing color/shape semantics with the unified finder.
    payload = {
        "actions": [
            {
                "skill": "COUNT_OBJECTS",
                "params": {
                    "object": "cup",
                    "attributes": {"color": "red", "approximate_shape": "cylinder"},
                },
            }
        ]
    }

    queue = parse_qwen_action_queue(json.dumps(payload), default_timeout_sec=30.0, default_retries=0)
    params = json.loads(queue.steps[0].parameters_json)

    assert queue.steps[0].action_type == "COUNT_OBJECTS"
    assert queue.steps[0].target_object == "cup"
    assert params["filters"] == [
        {"feature": "object_color", "operator": "equals", "value": "red"},
        {"feature": "shape", "operator": "equals", "value": "cylinder"},
    ]
