"""Section: Dispatcher mapping tests."""

from yasmin import Blackboard

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.action_dispatcher_state import ACTION_TYPE_TO_SKILL, ActionDispatcherState
from robot_yasmin_tasks.gpsr.action_types import ALLOWED_ACTION_TYPES
from robot_yasmin_tasks.gpsr.models import make_skill_result
from robot_yasmin_tasks.gpsr.outcomes import INVALID_INPUT, SUCCESS


class MockSkill:
    """Special function: Mock skill that records a success result."""
    def __init__(self, action_type: str) -> None:
        """Special function: Store action type."""
        self.action_type = action_type
    def __call__(self, blackboard: Blackboard) -> str:
        """Special function: Execute mock skill."""
        blackboard[bb.LAST_SKILL_RESULT] = make_skill_result(self.action_type, SUCCESS).to_dict()
        return SUCCESS


def test_dispatcher_maps_all_allowed_actions() -> None:
    """Special function: Ensure every allowed action is dispatchable."""
    assert set(ACTION_TYPE_TO_SKILL) == ALLOWED_ACTION_TYPES


def test_dispatcher_rejects_unknown_action() -> None:
    """Special function: Ensure unknown actions are rejected."""
    blackboard = Blackboard()
    blackboard[bb.CURRENT_ACTION] = {"action_type": "DANCE"}
    assert ActionDispatcherState({}).execute(blackboard) == INVALID_INPUT


def test_dispatcher_executes_mock_skill() -> None:
    """Special function: Ensure dispatcher executes injected factories."""
    blackboard = Blackboard()
    blackboard[bb.CURRENT_ACTION] = {"action_type": "SAY"}
    assert ActionDispatcherState({"SAY": lambda: MockSkill("SAY")}).execute(blackboard) == SUCCESS
