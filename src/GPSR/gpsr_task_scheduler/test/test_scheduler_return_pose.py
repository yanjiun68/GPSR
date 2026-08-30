"""Section: Per-command AMCL return-target regression tests."""

from __future__ import annotations

import threading
import time
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped

from gpsr_task_scheduler.models import ReturnTarget
from gpsr_task_scheduler.scheduler_node import GpsrTaskSchedulerNode


class _Param:
    """Special function: Carry one scheduler parameter value."""

    def __init__(self, value) -> None:
        """Special function: Store one fake parameter value."""
        self.value = value


class _Logger:
    """Special function: Accept scheduler diagnostics in isolated tests."""

    def info(self, _message: str) -> None:
        """Special function: Accept an informational diagnostic."""

    def warn(self, _message: str) -> None:
        """Special function: Accept a warning diagnostic."""


class _CaptureHarness:
    """Special function: Exercise return capture without constructing a ROS node."""

    def __init__(self, timeout: float = 0.0) -> None:
        """Special function: Initialize localization cache and return policy values."""
        self.amcl_pose_condition = threading.Condition()
        self.latest_amcl_pose = None
        self.latest_amcl_pose_receipt_time = 0.0
        self.config = SimpleNamespace(instruction_point="instruction_point")
        self.values = {
            "return_target_mode": "command_start_amcl",
            "return_fallback_named_location": "instruction_point",
            "amcl_pose_capture_timeout_sec": timeout,
            "amcl_pose_max_age_sec": 1.0,
        }
        self.logger = _Logger()

    def get_parameter(self, name: str) -> _Param:
        """Special function: Return one fake scheduler parameter."""
        return _Param(self.values[name])

    def get_logger(self) -> _Logger:
        """Special function: Return the inert diagnostic logger."""
        return self.logger

    def named_fallback_target(self) -> ReturnTarget:
        """Special function: Reuse production named-fallback resolution."""
        return GpsrTaskSchedulerNode.named_fallback_target(self)

    def validated_amcl_target_pose(self, message, receipt_time):
        """Special function: Reuse production AMCL validation and normalization."""
        return GpsrTaskSchedulerNode.validated_amcl_target_pose(
            self,
            message,
            receipt_time,
        )

    def capture(self, dry_run: bool = False) -> ReturnTarget:
        """Special function: Reuse production bounded return-target capture."""
        return GpsrTaskSchedulerNode.capture_return_target(
            self,
            "command_1",
            dry_run,
        )


def _pose(frame_id: str = "map", quaternion_w: float = 2.0) -> PoseWithCovarianceStamped:
    """Special function: Build one deterministic AMCL observation."""
    message = PoseWithCovarianceStamped()
    message.header.frame_id = frame_id
    message.pose.pose.position.x = 1.25
    message.pose.pose.position.y = -0.5
    message.pose.pose.orientation.w = quaternion_w
    return message


def test_fresh_amcl_pose_is_normalized_and_frozen() -> None:
    """Special function: Capture one immutable normalized command-start pose."""
    harness = _CaptureHarness()
    harness.latest_amcl_pose = _pose()
    harness.latest_amcl_pose_receipt_time = time.monotonic()
    target = harness.capture()
    harness.latest_amcl_pose.pose.pose.position.x = 99.0
    assert target.source == "command_start_amcl"
    assert target.target_pose["position"]["x"] == pytest.approx(1.25)
    assert target.target_pose["orientation"]["w"] == pytest.approx(1.0)


@pytest.mark.parametrize(
    "message,receipt_time",
    [
        (None, 0.0),
        (_pose("odom"), time.monotonic()),
        (_pose("map", 0.0), time.monotonic()),
        (_pose("map"), time.monotonic() - 2.0),
    ],
)
def test_invalid_or_stale_amcl_pose_uses_named_fallback(
    message: PoseWithCovarianceStamped | None,
    receipt_time: float,
) -> None:
    """Special function: Reject missing, non-map, zero-quaternion, and stale poses."""
    harness = _CaptureHarness()
    harness.latest_amcl_pose = message
    harness.latest_amcl_pose_receipt_time = receipt_time
    target = harness.capture()
    assert target.source == "instruction_point_fallback"
    assert target.target_location == "instruction_point"
    assert target.target_pose is None


def test_dry_run_identifies_policy_without_waiting_or_capturing() -> None:
    """Special function: Dry-run returns immediately with policy-only diagnostics."""
    harness = _CaptureHarness(timeout=3.0)
    started = time.monotonic()
    target = harness.capture(dry_run=True)
    assert time.monotonic() - started < 0.2
    assert target.source == "dry_run_command_start_amcl"
    assert target.target_pose is None


def test_return_plan_selects_raw_pose_or_named_location_exclusively() -> None:
    """Special function: Serialize exactly one navigation target form per return queue."""
    harness = SimpleNamespace(
        config=SimpleNamespace(
            timeout_for=lambda _skill: 30.0,
            retries_for=lambda _skill: 1,
        )
    )
    raw = ReturnTarget(
        source="command_start_amcl",
        target_pose={
            "frame_id": "map",
            "position": {"x": 1.0, "y": 2.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
    )
    named = ReturnTarget(source="instruction_point_fallback", target_location="instruction_point")
    raw_arguments = GpsrTaskSchedulerNode.build_return_plan(harness, raw)["steps"][0]["arguments"]
    named_arguments = GpsrTaskSchedulerNode.build_return_plan(harness, named)["steps"][0]["arguments"]
    assert "target_pose" in raw_arguments and "target_location" not in raw_arguments
    assert "target_location" in named_arguments and "target_pose" not in named_arguments


def test_not_found_report_is_target_specific() -> None:
    """Special function: Render deterministic object-specific speech after a failed search."""
    planned = {
        "task_request": {
            "task_type": "find_object",
            "target": {"object_name": "bottle"},
        }
    }
    task_result = {"results": [{"outcome": "not_found"}]}
    text = GpsrTaskSchedulerNode.failure_report_text(planned, task_result)
    assert text == "I could not find the bottle."


def test_not_found_category_report_is_target_specific() -> None:
    """Special function: Name a requested category when no object name was supplied."""
    # Section: Build a category-only FIND_OBJECT result from an exhaustive search.
    planned = {
        "task_request": {
            "task_type": "find_object",
            "target": {"category": "drink"},
        }
    }
    task_result = {"results": [{"outcome": "not_found"}]}

    # Section: Preserve the requested category in the spoken failure report.
    text = GpsrTaskSchedulerNode.failure_report_text(planned, task_result)
    assert text == "I could not find the drink."


def test_not_found_builds_automatic_post_return_speech() -> None:
    """Special function: Build a SAY queue even when the command did not request reporting."""
    # Section: Supply only the scheduler configuration required by the synthesized SAY step.
    harness = SimpleNamespace(
        config=SimpleNamespace(
            timeout_for=lambda skill: 8.0 if skill == "SAY" else 30.0,
            retries_for=lambda _skill: 0,
        ),
    )
    planned = {
        "task_request": {
            "task_type": "find_object",
            "target": {"object_name": "bottle"},
        },
        "steps": [],
        "post_return_steps": [],
    }
    task_result = {
        "success": False,
        "results": [{"action_type": "FIND_OBJECT", "outcome": "not_found"}],
    }

    # Section: Verify the automatically generated report is target-specific and dependency-free.
    report = GpsrTaskSchedulerNode.build_post_return_plan(harness, planned, task_result)
    assert len(report["steps"]) == 1
    assert report["steps"][0]["skill_name"] == "SAY"
    assert report["steps"][0]["arguments"]["text"] == "I could not find the bottle."
    assert report["steps"][0]["dependencies"] == []


def test_other_terminal_failure_uses_generic_report() -> None:
    """Special function: Avoid claiming a target-specific miss for other failures."""
    planned = {
        "task_request": {
            "task_type": "find_object",
            "target": {"object_name": "bottle"},
        }
    }
    task_result = {"results": [{"outcome": "timeout"}]}
    text = GpsrTaskSchedulerNode.failure_report_text(planned, task_result)
    assert text == "I could not complete the requested task."
