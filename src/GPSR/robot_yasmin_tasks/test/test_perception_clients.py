"""Section: Perception client tests."""

# Section: Test fixtures, deterministic fakes, and regression scenarios.

import json

from yasmin import Blackboard

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.models import ActionStepData
from robot_yasmin_tasks.gpsr.outcomes import SUCCESS
from robot_yasmin_tasks.ros_clients.perception_clients import PerceptionClients


def test_count_people_simulated_stores_blackboard_contract() -> None:
    """Special function: Simulated CountPeople client stores count and people metadata."""
    blackboard = Blackboard()
    detected_people = [{"entity_id": "person_1_1", "confidence": 0.9}]
    action = ActionStepData(
        "COUNT_PEOPLE",
        parameters={
            "simulated_count": 1,
            "simulated_detected_people": detected_people,
            "simulated_result_json": {"count": 1},
        },
    )

    result = PerceptionClients().count_people(blackboard, action)

    assert result.outcome == SUCCESS
    assert blackboard[bb.COUNT_RESULT] == 1
    assert blackboard[bb.DETECTED_PEOPLE] == detected_people
    assert blackboard[bb.COUNT_PEOPLE_RESULT_JSON] == '{"count": 1}'


def test_count_object_filter_list_reaches_qwen_perception_contract() -> None:
    """Special function: Keep scheduler semantic filters inside the COUNT_OBJECTS service payload."""
    # Section: The C++ production client and Python compatibility client share this JSON envelope.
    action = ActionStepData(
        "COUNT_OBJECTS",
        parameters={
            "filters": [
                {"feature": "object_color", "operator": "equals", "value": "red"},
                {"feature": "shape", "operator": "equals", "value": "cylinder"},
            ],
            "min_confidence": 0.70,
        },
    )

    payload = json.loads(PerceptionClients().build_filter_json(action))

    assert payload["filters"] == action.parameters["filters"]
    assert payload["min_confidence"] == 0.70
