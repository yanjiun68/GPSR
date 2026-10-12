"""Section: GPSR pre-execution validation model and transition contracts."""

from __future__ import annotations

from dataclasses import dataclass

import pytest

from gpsr_sys_ui.sources.gpsr_source import (
    GpsrSource,
    GpsrStateModel,
    GpsrTransitionTracker,
    normalize_preflight_state,
)


def _plan(command_id: str = "command_A") -> dict:
    """Special function: Build a scheduler-owned two-step plan fixture."""
    return {
        "command_id": command_id,
        "valid": True,
        "command_context": {
            "raw_text": "go and find",
            "normalized_text": "navigate then find object",
            "action_queue": [
                {"index": 0, "step_id": "step_1", "action_type": "NAVIGATE", "arguments": {"target_location": "kitchen"}},
                {"index": 1, "step_id": "step_2", "action_type": "FIND_OBJECT", "arguments": {"target_object": "coke"}},
            ],
        },
    }


def _validation(command_id: str = "command_A") -> dict:
    """Special function: Build a reordered owner-side validation fixture."""
    return {
        "command_id": command_id,
        "validation_state": "PARTIAL",
        "fully_runnable": False,
        "items": [
            {
                "index": 1,
                "step_id": "step_2",
                "action_type": "FIND_OBJECT",
                "preflight_state": "unavailable",
                "reason_code": "LIFECYCLE_NODE_UNAVAILABLE",
                "resource": "/find_object_skill_server",
                "message": "server unavailable",
            },
            {
                "index": 0,
                "step_id": "step_1",
                "action_type": "NAVIGATE",
                "preflight_state": "available",
                "reason_code": "AVAILABLE",
            },
        ],
    }


@pytest.mark.parametrize(
    ("raw", "expected"),
    [
        ("available", "READY"),
        ("ready", "READY"),
        ("missing", "UNAVAILABLE"),
        ("unavailable", "UNAVAILABLE"),
        ("dependency_blocked", "BLOCKED"),
        ("unexpected value", "UNKNOWN"),
    ],
)
def test_preflight_state_normalization(raw: str, expected: str) -> None:
    """Special function: Verify canonical structured readiness aliases."""
    assert normalize_preflight_state(raw) == expected


def test_validation_merges_by_step_id_without_reordering_plan() -> None:
    """Special function: Match reordered validation data by stable step identity."""
    model = GpsrStateModel()
    model.set_plan(_plan())
    model.set_validation(_validation())

    snapshot = model.snapshot()

    assert [item["step_id"] for item in snapshot["queue"]] == ["step_1", "step_2"]
    assert [item["preflight_state"] for item in snapshot["queue"]] == ["READY", "UNAVAILABLE"]
    assert snapshot["queue"][1]["parameters"]["target_object"] == "coke"


def test_validation_uses_index_fallback_without_stable_ids() -> None:
    """Special function: Apply legacy validation items by explicit queue index."""
    model = GpsrStateModel()
    model.set_plan(_plan())
    model.set_validation({
        "command_id": "command_A",
        "items": [
            {"index": 1, "preflight_state": "blocked", "blocked_by": ["step_1"]},
            {"index": 0, "preflight_state": "ready"},
        ],
    })

    snapshot = model.snapshot()

    assert snapshot["queue"][0]["preflight_state"] == "READY"
    assert snapshot["queue"][1]["preflight_state"] == "BLOCKED"
    assert snapshot["queue"][1]["blocked_by"] == ["step_1"]
    assert snapshot["preflight"]["state"] == "PARTIAL"
    assert snapshot["preflight"]["ready_count"] == 1
    assert snapshot["preflight"]["blocked_count"] == 1
    assert "GPSR_SKILL_BLOCKED" in {
        event.event_code for event in GpsrTransitionTracker().update(snapshot)
    }


def test_unavailable_preflight_does_not_change_pending_execution() -> None:
    """Special function: Keep a preflight failure separate from pending execution."""
    model = GpsrStateModel()
    model.set_plan(_plan())
    model.set_validation(_validation())
    model.set_executor({
        "task_id": "command_A",
        "queue_state": "PENDING",
        "items": [
            {"index": 0, "step_id": "step_1", "state": "PENDING"},
            {"index": 1, "step_id": "step_2", "state": "PENDING", "preflight_state": "READY"},
        ],
    })

    find_object = model.snapshot()["queue"][1]

    assert find_object["preflight_state"] == "UNAVAILABLE"
    assert find_object["state"] == "PENDING"
    assert find_object["task_status"] == "IN QUEUE"


def test_runtime_failure_remains_separate_from_ready_preflight() -> None:
    """Special function: Preserve healthy preflight evidence after runtime failure."""
    model = GpsrStateModel()
    model.set_plan(_plan())
    model.set_validation({
        "command_id": "command_A",
        "validation_state": "READY",
        "items": [{"step_id": "step_1", "preflight_state": "READY"}],
    })
    model.set_executor({
        "task_id": "command_A",
        "queue_state": "FAILED",
        "items": [{"step_id": "step_1", "state": "FAILED"}],
    })

    navigate = model.snapshot()["queue"][0]

    assert navigate["preflight_state"] == "READY"
    assert navigate["state"] == "FAILED"
    assert navigate["task_status"] == "ABORTED"


def test_stale_validation_is_not_applied_to_new_command() -> None:
    """Special function: Reject a retained report owned by a previous command."""
    model = GpsrStateModel()
    model.set_plan(_plan("command_B"))
    model.set_validation(_validation("command_A"))

    snapshot = model.snapshot()

    assert snapshot["preflight"]["present"] is False
    assert snapshot["preflight"]["stale"] is True
    assert all(item["preflight_state"] == "UNKNOWN" for item in snapshot["queue"])
    assert all(item["preflight_present"] is False for item in snapshot["queue"])


def test_no_validation_report_is_startup_safe() -> None:
    """Special function: Preserve normal startup when the optional publisher is absent."""
    snapshot = GpsrStateModel().snapshot()

    assert snapshot["preflight"]["present"] is False
    assert snapshot["preflight"]["state"] == "UNKNOWN"
    assert snapshot["queue"] == []


def test_empty_validation_object_is_a_present_unknown_report() -> None:
    """Special function: Distinguish an empty received report from an absent publisher."""
    model = GpsrStateModel()
    model.set_plan(_plan())
    model.set_validation({})

    snapshot = model.snapshot()

    assert snapshot["preflight"]["present"] is True
    assert snapshot["preflight"]["report_present"] is True
    assert snapshot["preflight"]["state"] == "UNKNOWN"


@dataclass
class _Message:
    """Section: Minimal std_msgs/String-compatible test message."""

    data: str


class _RecordingEvents:
    """Section: Minimal event sink used to verify malformed JSON handling."""

    def __init__(self) -> None:
        """Special function: Initialize an empty recorded-event list."""
        self.items = []

    def emit(self, event) -> None:
        """Special function: Record one monitor event without ROS transport."""
        self.items.append(event)


def test_invalid_validation_json_warns_and_preserves_good_state() -> None:
    """Special function: Reject malformed validation data without clearing the model."""
    source = GpsrSource.__new__(GpsrSource)
    source.events = _RecordingEvents()
    source.model = GpsrStateModel()
    source.model.set_validation(_validation())
    previous = dict(source.model.validation)

    source._on_validation(_Message("{not-json"))

    assert source.model.validation == previous
    assert len(source.events.items) == 1
    assert source.events.items[0].severity == "WARN"
    assert "action_queue_validation" in source.events.items[0].message


def test_preflight_events_are_deduplicated_and_reset_for_new_command() -> None:
    """Special function: Emit failure transitions once per command and state change."""
    tracker = GpsrTransitionTracker()
    model = GpsrStateModel()
    model.set_plan(_plan("command_A"))
    model.set_validation(_validation("command_A"))
    first_codes = [event.event_code for event in tracker.update(model.snapshot())]
    repeated_codes = [event.event_code for event in tracker.update(model.snapshot())]

    model.set_plan(_plan("command_B"))
    model.set_validation(_validation("command_B"))
    second_codes = [event.event_code for event in tracker.update(model.snapshot())]

    assert first_codes.count("GPSR_PREFLIGHT_PARTIAL") == 1
    assert first_codes.count("GPSR_SKILL_UNAVAILABLE") == 1
    assert "GPSR_PREFLIGHT_PARTIAL" not in repeated_codes
    assert "GPSR_SKILL_UNAVAILABLE" not in repeated_codes
    assert second_codes.count("GPSR_PREFLIGHT_PARTIAL") == 1
    assert second_codes.count("GPSR_SKILL_UNAVAILABLE") == 1
