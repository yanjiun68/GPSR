"""Section: Hardware-free scheduler topic contract tests."""

from pathlib import Path

from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from gpsr_task_scheduler.scheduler_node import retained_plan_qos


def test_plan_publication_is_retained_for_optional_late_subscribers() -> None:
    """Special function: Verify plan publishing never needs a live TUI subscriber."""
    # Section: Transient-local DDS publication stores one plan without subscriber handshakes.
    qos = retained_plan_qos()
    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 1
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_scheduler_package_has_no_tui_runtime_dependency() -> None:
    """Special function: Keep the TUI an optional one-way observer of scheduler topics."""
    # Section: Package metadata must never force monitor installation or startup.
    package_xml = (Path(__file__).resolve().parents[1] / "package.xml").read_text()
    assert "gpsr_sys_ui" not in package_xml
