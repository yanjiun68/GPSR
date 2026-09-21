"""Section: Dry-run GPSR planning for the expanded Incheon object vocabulary."""

from __future__ import annotations

import pytest

from gpsr_task_scheduler.config import default_config
from gpsr_task_scheduler.scheduler import schedule_command


@pytest.mark.parametrize(
    ("instruction", "target"),
    [
        ("find the red bull can in the kitchen", "red_bull"),
        ("find the Rubik's cube in the living room", "rubiks_cube"),
        ("find the yellow bell pepper in the kitchen", "yellow_bellpepper"),
        ("find the mangosteen in the dining room", "mangostane"),
        ("find the grey shirt in the bedroom", "grey_shirt"),
    ],
)
def test_official_object_phrases_reach_action_queue(
    instruction: str,
    target: str,
) -> None:
    """Special function: Ensure official names and aliases survive planning."""
    result = schedule_command("incheon_object", instruction, default_config())
    # Section: A valid dry-run command must retain the canonical object target.
    assert result.valid, [issue.message for issue in result.issues]
    assert result.command_context is not None
    assert any(
        step["action_type"] == "FIND_OBJECT"
        and step["arguments"].get("object_name") == target
        and not step["arguments"].get("filters")
        for step in result.command_context.action_queue
    )


def test_explicit_color_outside_object_name_remains_a_filter() -> None:
    """Special function: Preserve a requested color after removing named objects."""
    result = schedule_command(
        "colored_apple",
        "find the red apple in the kitchen",
        default_config(),
    )
    # Section: The apple name is distinct from the explicit red attribute.
    assert result.valid, [issue.message for issue in result.issues]
    assert result.command_context is not None
    filters = result.command_context.action_queue[-1]["arguments"]["filters"]
    assert {"feature": "object_color", "operator": "equals", "value": "red"} in filters
