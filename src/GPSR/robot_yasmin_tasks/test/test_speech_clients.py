"""Section: Deferred speech substitution regression tests."""

# Section: Test fixtures, deterministic fakes, and regression scenarios.

from yasmin import Blackboard

from robot_yasmin_tasks.gpsr.models import ActionStepData
from robot_yasmin_tasks.gpsr.outcomes import SUCCESS
from robot_yasmin_tasks.ros_clients.speech_clients import SpeechClients


def test_explicit_substitution_data_crosses_queue_boundary() -> None:
    """Special function: Render deferred speech from explicit prior-queue result data."""
    action = ActionStepData(
        "SAY",
        parameters={
            "text": "I counted {result.count} bottles.",
            "substitution": "task_result",
            "substitution_data": {"count": 4},
        },
    )
    result = SpeechClients(simulate=True).say(Blackboard(), action)
    assert result.outcome == SUCCESS
    assert result.result["text"] == "I counted 4 bottles."
