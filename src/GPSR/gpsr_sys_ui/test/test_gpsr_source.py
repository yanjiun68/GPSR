"""Section: Hardware-free GPSR UI normalization tests."""

from gpsr_sys_ui.sources.gpsr_source import (
    GpsrStateModel,
    GpsrTransitionTracker,
    action_domain,
    action_parameters,
    merge_queue_items,
    normalize_queue_state,
    retained_json_qos,
    task_display_status,
)


def test_monitor_requests_retained_topic_snapshots() -> None:
    """Special function: A late-starting optional TUI receives the latest plan and queue state."""
    # Section: Verify the subscriber contract without launching ROS nodes or robot hardware.
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
    qos = retained_json_qos()
    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 1
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


def _plan() -> dict:
    """Special function: Build a scheduler-owned command fixture."""
    return {
        "command_id": "gpsr_001",
        "command_context": {
            "raw_text": "Go to living room and find bottles",
            "normalized_text": "go to the living room and find bottles",
            "action_queue": [
                {"step_id": "s1", "action_type": "NAVIGATE", "target_location": "living_room"},
                {"step_id": "s2", "action_type": "FIND_OBJECT", "target_object": "bottle"},
            ],
        },
    }


def test_plan_only_startup_preserves_commands_and_pending_queue() -> None:
    """Special function: A plan is useful before the executor status publisher appears."""
    model = GpsrStateModel()
    model.set_plan(_plan())
    state = model.snapshot()
    assert state["raw_command"].startswith("Go to")
    assert state["normalized_command"].startswith("go to")
    assert [item["state"] for item in state["queue"]] == ["PENDING", "PENDING"]
    assert state["queue"][0]["domain"] == "NAVIGATION"
    assert state["queue"][1]["target"] == "bottle"
    assert [item["task_status"] for item in state["queue"]] == ["IN QUEUE", "IN QUEUE"]
    assert state["task_status_counts"]["IN QUEUE"] == 2


def test_public_plan_steps_supply_sequence_and_nested_targets() -> None:
    """Special function: Show accepted scheduler steps when context queue metadata is absent."""
    model = GpsrStateModel()
    model.set_plan({
        "command_id": "gpsr_steps",
        "valid": True,
        "command_context": {"raw_text": "bring coke", "normalized_text": "bring coke"},
        "steps": [
            {"step_id": "find", "skill_name": "FIND_OBJECT", "arguments": {"object_name": "coke"}},
            {"step_id": "go", "skill_name": "NAVIGATE", "arguments": {"target_location": "kitchen"}},
        ],
    })
    state = model.snapshot()
    assert [item["skill"] for item in state["queue"]] == ["FIND_OBJECT", "NAVIGATE"]
    assert [item["target"] for item in state["queue"]] == ["coke", "kitchen"]
    assert state["queue"][0]["parameters"] == {"object_name": "coke"}
    assert state["queue"][1]["parameters"] == {"target_location": "kitchen"}
    assert [item["task_status"] for item in state["queue"]] == ["IN QUEUE", "IN QUEUE"]


def test_executor_state_overlay_preserves_plan_parameters() -> None:
    """Special function: Status-only executor items must not erase scheduler arguments."""
    planned = [{
        "step_id": "find",
        "skill_name": "FIND_OBJECT",
        "arguments": {
            "object_name": "coke",
            "search_scope": "STRICT_LOCATION",
            "target_location": "kitchen",
        },
    }]
    runtime = [{
        "step_id": "find",
        "action_type": "FIND_OBJECT",
        "state": "running",
        "current": True,
    }]
    merged = merge_queue_items(planned, runtime)
    assert merged[0]["parameters"] == planned[0]["arguments"]


def test_serialized_action_parameters_include_explicit_targets() -> None:
    """Special function: ActionStep JSON and dedicated target fields share one UI map."""
    assert action_parameters({
        "parameters_json": '{"search_scope":"STRICT_LOCATION"}',
        "target_location": "living_room",
        "target_object": "bottle",
    }) == {
        "search_scope": "STRICT_LOCATION",
        "target_location": "living_room",
        "target_object": "bottle",
    }


def test_executor_snapshot_preserves_skipped_retry_and_current_action() -> None:
    """Special function: Explicit producer states survive normalization without inference."""
    model = GpsrStateModel()
    model.set_plan(_plan())
    model.set_executor({
        "task_id": "gpsr_001",
        "queue_state": "retrying",
        "current_index": 1,
        "completed_count": 1,
        "retry_attempt": 2,
        "retry_limit": 3,
        "items": [
            {"index": 0, "step_id": "s1", "action_type": "NAVIGATE", "state": "skipped", "raw_state": "policy_skip"},
            {"index": 1, "step_id": "s2", "action_type": "FIND_OBJECT", "state": "retrying", "attempt": 2, "max_retries": 3, "current": True},
        ],
    })
    state = model.snapshot()
    assert state["queue"][0]["state"] == "SKIPPED"
    assert state["queue"][0]["raw_state"] == "policy_skip"
    assert state["queue"][1]["state"] == "RETRYING"
    assert state["current_skill"] == "FIND_OBJECT"
    assert state["current_domain"] == "VISION"
    assert state["current_index"] == 1
    assert state["queue_length"] == 2
    assert state["completed_count"] == 1
    assert state["retry_attempt"] == 2
    assert state["retry_limit"] == 3


def test_running_queue_uses_executor_index_for_current_skill() -> None:
    """Special function: Current skill comes from queue index rather than command text."""
    model = GpsrStateModel()
    model.set_plan(_plan())
    model.set_executor({
        "task_id": "gpsr_001",
        "queue_state": "running",
        "current_index": 1,
        "items": [
            {"index": 0, "action_type": "NAVIGATE", "state": "done"},
            {"index": 1, "action_type": "FIND_OBJECT", "state": "running", "current": True},
            {"index": 2, "action_type": "PICK_OBJECT", "state": "pending"},
        ],
    })
    state = model.snapshot()
    assert [item["state"] for item in state["queue"]] == ["DONE", "RUNNING", "PENDING"]
    assert state["current_index"] == 1
    assert state["current_skill"] == "FIND_OBJECT"
    assert state["completed_count"] == 1
    assert [item["task_status"] for item in state["queue"]] == ["COMPLETED", "RUNNING", "IN QUEUE"]


def test_terminal_failure_marks_failed_and_unstarted_tasks_aborted() -> None:
    """Special function: Do not leave impossible remainder tasks labelled in queue."""
    model = GpsrStateModel()
    model.set_plan(_plan())
    model.set_executor({
        "task_id": "gpsr_001",
        "queue_state": "failed",
        "current_index": 1,
        "items": [
            {"index": 0, "action_type": "NAVIGATE", "state": "done"},
            {"index": 1, "action_type": "FIND_OBJECT", "state": "failed"},
            {"index": 2, "action_type": "SAY", "state": "pending"},
        ],
    })
    state = model.snapshot()
    assert [item["task_status"] for item in state["queue"]] == [
        "COMPLETED", "ABORTED", "ABORTED",
    ]
    assert state["task_status_counts"]["COMPLETED"] == 1
    assert state["task_status_counts"]["ABORTED"] == 2


def test_new_command_does_not_display_previous_executor_queue() -> None:
    """Special function: Task identifiers prevent stale cross-command queue mixing."""
    model = GpsrStateModel()
    model.set_executor({
        "task_id": "gpsr_old",
        "queue_state": "failed",
        "current_index": 0,
        "items": [{"action_type": "SAY", "state": "failed"}],
    })
    model.set_plan(_plan())
    state = model.snapshot()
    assert [item["skill"] for item in state["queue"]] == ["NAVIGATE", "FIND_OBJECT"]
    assert state["current_skill"] == ""
    assert state["queue_state"] != "FAILED"


def test_queue_terminal_and_domain_normalization() -> None:
    """Special function: Terminal states and canonical domains remain deterministic."""
    assert normalize_queue_state("fatal_failure") == "FAILED"
    assert normalize_queue_state("cancelled") == "CANCELED"
    assert normalize_queue_state("completed") == "DONE"
    assert action_domain("SAY") == "VOICE"
    assert action_domain("PICK") == "MANIPULATION"
    assert action_domain("PICK_OBJECT") == "MANIPULATION"
    assert action_domain("custom_skill") == "OTHER"
    assert task_display_status("pending") == "IN QUEUE"
    assert task_display_status("done") == "COMPLETED"
    assert task_display_status("failed") == "ABORTED"


def test_presence_can_change_after_absent_startup() -> None:
    """Special function: Late scheduler/executor discovery does not require a restart."""
    model = GpsrStateModel()
    assert model.snapshot()["scheduler_present"] is False
    model.set_presence(True, True)
    state = model.snapshot()
    assert state["scheduler_present"] is True
    assert state["executor_present"] is True


def test_failed_action_and_queue_events_are_emitted_once() -> None:
    """Special function: Repeated executor snapshots do not flood failure events."""
    tracker = GpsrTransitionTracker()
    snapshot = {
        "command_id": "gpsr_001",
        "queue_state": "FAILED",
        "message": "camera unavailable",
        "queue": [{"index": 0, "step_id": "s1", "skill": "FIND_OBJECT", "state": "FAILED", "raw_state": "fatal_failure", "message": "camera unavailable"}],
    }
    first_codes = [event.event_code for event in tracker.update(snapshot)]
    assert first_codes == ["GPSR_COMMAND_RECEIVED", "GPSR_QUEUE_CREATED", "GPSR_ACTION_FAILED", "GPSR_QUEUE_FAILED"]
    assert tracker.update(snapshot) == []
