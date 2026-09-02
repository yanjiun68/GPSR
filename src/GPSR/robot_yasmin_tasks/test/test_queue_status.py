"""Section: Executor-owned action queue status tests."""

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.queue_status import build_queue_status, emit_queue_status


def _blackboard() -> dict:
    """Special function: Build a two-action executor blackboard fixture."""
    return {
        bb.CURRENT_TASK_ID: "gpsr_001",
        bb.ACTION_QUEUE: [
            {"step_id": "s1", "action_type": "NAVIGATE", "target_location": "living_room", "max_retries": 1},
            {"step_id": "s2", "action_type": "FIND_OBJECT", "target_object": "bottle", "max_retries": 2},
        ],
        bb.CURRENT_ACTION_INDEX: 1,
        bb.RECOVERY_ATTEMPTS: {"s2": 1},
        bb.DISPATCH_OUTCOME: "recoverable_failure",
        bb.LAST_SKILL_RESULT: {"action_type": "FIND_OBJECT", "outcome": "recoverable_failure", "message": "camera unavailable"},
    }


def test_retry_snapshot_exposes_full_queue_and_exact_reason() -> None:
    """Special function: Retry status includes cursor, attempt limit, targets, and reason."""
    status = build_queue_status(_blackboard(), "ACTION_RETRYING", "success", timestamp=123.0)
    assert status["timestamp"] == 123.0
    assert status["task_id"] == "gpsr_001"
    assert status["queue_state"] == "RETRYING"
    assert status["retry_attempt"] == 1
    assert status["retry_limit"] == 2
    assert [item["state"] for item in status["items"]] == ["DONE", "RETRYING"]
    assert status["items"][1]["target_object"] == "bottle"
    assert status["message"] == "camera unavailable"


def test_terminal_success_marks_every_item_done() -> None:
    """Special function: Successful queue completion has no pending item residue."""
    blackboard = _blackboard()
    blackboard[bb.CURRENT_ACTION_INDEX] = 2
    blackboard[bb.DISPATCH_OUTCOME] = "success"
    status = build_queue_status(blackboard, "QUEUE_FINISHED", "success")
    assert status["queue_state"] == "DONE"
    assert all(item["state"] == "DONE" for item in status["items"])


def test_terminal_failure_overrides_retry_state_and_keeps_client_reason() -> None:
    """Special function: Final failure remains FAILED and preserves the original client detail."""
    blackboard = _blackboard()
    blackboard[bb.SKILL_RESULTS] = [{"action_type": "FIND_OBJECT", "outcome": "recoverable_failure", "message": "camera unavailable"}]
    blackboard[bb.LAST_SKILL_RESULT] = {"action_type": "FIND_OBJECT", "outcome": "fatal_failure", "message": "Recovery exhausted after 1 retries."}
    status = build_queue_status(blackboard, "QUEUE_FINISHED", "fatal_failure")
    assert status["queue_state"] == "FAILED"
    assert status["items"][1]["state"] == "FAILED"
    assert "camera unavailable" in status["message"]
    assert "Recovery exhausted" in status["message"]


def test_observer_exception_cannot_change_execution() -> None:
    """Special function: A failed read-only observer is isolated from executor control flow."""
    def broken_callback(_blackboard, _phase, _outcome):
        """Special function: Simulate an unavailable ROS publisher."""
        raise RuntimeError("publisher unavailable")

    emit_queue_status(broken_callback, _blackboard(), "QUEUE_STARTED")
