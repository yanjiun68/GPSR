"""Section: Static regression checks for the two generated graph definitions."""

from pathlib import Path


# Section: Source inspection avoids requiring live ROS servers during package tests.
PACKAGE_ROOT = Path(__file__).parents[1] / "carry_my_luggage_yasmin"


def test_phase1_graph_contains_bag_and_confirmation_stages() -> None:
    """Guard the phase-one-only states against accidental removal."""
    source = (PACKAGE_ROOT / "phase1_sm.py").read_text(encoding="utf-8")
    assert "ST_WAIT_FOR_BUTTON_PRESS" in source
    assert "ST_SELECT_BAG" in source
    assert "ST_ASK_PASS_BAG_CONFIRMATION" in source


def test_follow_demo_graph_uses_unmapped_destination_leg() -> None:
    """Guard the streamlined demo's active destination transition."""
    source = (PACKAGE_ROOT / "follow_demo_sm.py").read_text(encoding="utf-8")
    assert 'message.data = "unmapped"' in source
    assert "ST_GOTO_DESTINATION" in source
    assert "ST_ASK_PASS_BAG_CONFIRMATION" not in source
