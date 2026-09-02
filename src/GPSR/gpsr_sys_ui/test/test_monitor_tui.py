"""Section: Presentation helper regression tests."""

from gpsr_sys_ui.presentation import PAGE_TABS, Column, fit_columns, queue_rows, truncate, wrap_text


def test_page_order_and_no_energy_page() -> None:
    """Special function: Preserve the required nine-page navigation contract."""
    assert PAGE_TABS == (
        "Overview", "GPSR", "Voice", "Nodes", "Lifecycle", "Nav2", "Arm", "Events", "System Telemetry",
    )
    assert "Energy" not in PAGE_TABS


def test_text_truncation_wrapping_and_column_fit() -> None:
    """Special function: Narrow terminals remain stable with long operator text."""
    assert truncate("abcdefgh", 5) == "abcd…"
    assert wrap_text("one two three four", 7) == ["one two", "three", "four"]
    fitted = fit_columns([Column("a", "A", 20, 5), Column("b", "B", 20, 5)], 18)
    assert sum(column.width for column in fitted) + 2 == 18


def test_queue_rows_use_one_based_display_and_current_marker() -> None:
    """Special function: Queue rows remain aligned and highlight the current index."""
    rows = queue_rows([
        {"index": 0, "skill": "NAVIGATE", "state": "DONE", "attempt": 0, "max_retries": 1},
        {"index": 1, "skill": "FIND_OBJECT", "state": "RUNNING", "attempt": 1, "max_retries": 2, "current": True},
    ])
    assert rows[0]["number"].strip() == "1"
    assert rows[1]["number"] == "> 2"
    assert rows[1]["attempt"] == "1/2"
