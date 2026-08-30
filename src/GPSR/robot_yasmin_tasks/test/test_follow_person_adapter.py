"""Section: FOLLOW_PERSON GPSR result-classification tests."""

# Section: Test fixtures, deterministic fakes, and regression scenarios.

from action_msgs.msg import GoalStatus

from robot_yasmin_tasks.gpsr.outcomes import (
    CANCELED,
    FAILED,
    NOT_FOUND,
    PERSON_LOST,
    SUCCESS,
)
from robot_yasmin_tasks.ros_clients.follow_person_action_client import (
    FollowPersonActionClient,
)
def test_action_result_prefixes_map_to_shared_gpsr_outcomes() -> None:
    """Special function: Compact adapter reasons remain deterministic at the executor boundary."""
    classify = FollowPersonActionClient.classify_result
    assert classify(True, "follow_complete: done", GoalStatus.STATUS_SUCCEEDED) == SUCCESS
    assert classify(False, "person_lost: stale", GoalStatus.STATUS_ABORTED) == PERSON_LOST
    assert classify(False, "person_not_found: none", GoalStatus.STATUS_ABORTED) == NOT_FOUND
    assert classify(False, "stack_unavailable: missing", GoalStatus.STATUS_ABORTED) == FAILED
    assert classify(False, "canceled: user", GoalStatus.STATUS_CANCELED) == CANCELED
