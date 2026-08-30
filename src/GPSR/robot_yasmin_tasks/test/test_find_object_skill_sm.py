"""Section: Thin FIND_OBJECT action-wrapper tests."""

# Section: Test fixtures, deterministic fakes, and regression scenarios.

from yasmin import Blackboard

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.outcomes import RECOVERABLE_FAILURE, SUCCESS
from robot_yasmin_tasks.skills.find_object_skill_sm import (
    BuildGoalState,
    FIND_OBJECT_GOAL,
    FindObjectSkillSM,
)


def test_find_object_skill_is_thin_action_wrapper() -> None:
    """Special function: Persistent executor owns no search/navigation decisions."""
    assert set(FindObjectSkillSM().get_states()) == {
        "BUILD_GOAL",
        "SEND_GOAL",
        "WAIT_RESULT",
    }


def test_build_goal_enforces_strict_explicit_location() -> None:
    """Special function: Explicit command location becomes a strict action boundary."""
    blackboard = Blackboard()
    blackboard[bb.CURRENT_ACTION] = {
        "action_type": "FIND_OBJECT",
        "target_object": "bottle",
        "target_location": "kitchen",
        "timeout_sec": 42.0,
    }
    assert BuildGoalState().execute(blackboard) == SUCCESS
    goal = blackboard[FIND_OBJECT_GOAL]
    assert goal.object_name == "bottle"
    assert goal.location == "kitchen"
    assert goal.search_scope == "STRICT_LOCATION"
    assert goal.timeout_sec == 42.0


def test_wrapper_without_ros_node_reports_infrastructure_failure() -> None:
    """Special function: Missing action transport is recoverable, not fake NOT_FOUND."""
    blackboard = Blackboard()
    blackboard[bb.CURRENT_ACTION] = {
        "action_type": "FIND_OBJECT",
        "target_object": "cup",
        "max_retries": 0,
    }
    assert FindObjectSkillSM()(blackboard) == RECOVERABLE_FAILURE
    assert blackboard[bb.LAST_SKILL_RESULT]["outcome"] == RECOVERABLE_FAILURE
