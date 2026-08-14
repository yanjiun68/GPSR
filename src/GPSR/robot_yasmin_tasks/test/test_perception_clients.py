"""Section: Perception client tests."""

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
