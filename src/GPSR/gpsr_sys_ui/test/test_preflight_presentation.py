"""Section: Hardware-free preflight presentation and page-contract tests."""

from __future__ import annotations

from gpsr_sys_ui.monitor_tui import MonitorTui
from gpsr_sys_ui.presentation import PAGE_TABS, Column, fit_columns, queue_rows, validation_rows


def _queue_fixture() -> list[dict]:
    """Special function: Build one row containing independent validation and execution state."""
    return [{
        "index": 0,
        "step_id": "step_1",
        "skill": "FIND_OBJECT",
        "domain": "VISION",
        "target": "coke",
        "parameters": {"target_object": "coke"},
        "preflight_state": "UNAVAILABLE",
        "preflight_reason": "LIFECYCLE_NODE_UNAVAILABLE",
        "preflight_resource": "/find_object_skill_server/with/a/long/name",
        "preflight_message": "Required lifecycle server is unavailable",
        "blocked_by": [],
        "task_status": "IN QUEUE",
        "attempt": 0,
        "max_retries": 1,
    }]


def test_queue_rows_expose_preflight_and_execution_separately() -> None:
    """Special function: Keep both state dimensions visible in shared table rows."""
    row = queue_rows(_queue_fixture())[0]

    assert row["preflight"] == "UNAVAILABLE"
    assert row["state"] == "IN QUEUE"


def test_validation_rows_expose_diagnostic_fields() -> None:
    """Special function: Render reason, resource, and message from normalized fields."""
    row = validation_rows(_queue_fixture())[0]

    assert row["readiness"] == "UNAVAILABLE"
    assert row["reason"] == "LIFECYCLE_NODE_UNAVAILABLE"
    assert row["resource"].startswith("/find_object_skill_server")
    assert "unavailable" in row["message"]


def test_nine_page_contract_is_unchanged() -> None:
    """Special function: Prevent an accidental tenth preflight page."""
    assert PAGE_TABS == (
        "Overview",
        "GPSR",
        "Voice",
        "Nodes",
        "Lifecycle",
        "Nav2",
        "Arm",
        "Events",
        "System Telemetry",
    )
    assert len(PAGE_TABS) == 9
    assert MonitorTui.TABS == PAGE_TABS


def test_validation_columns_fit_a_narrow_terminal() -> None:
    """Special function: Shrink long validation columns without exceeding display width."""
    available = 40
    columns = fit_columns(
        [
            Column("number", "#", 4, 2),
            Column("skill", "SKILL", 20, 8, grow=1),
            Column("readiness", "READINESS", 12, 8),
            Column("reason", "REASON", 28, 10, grow=2),
            Column("resource", "RESOURCE / BLOCKED BY", 30, 12, grow=2),
            Column("message", "MESSAGE", 34, 12, grow=3),
        ],
        available,
    )

    assert sum(column.width for column in columns) + 2 * (len(columns) - 1) <= available
    assert all(column.width >= 1 for column in columns)
