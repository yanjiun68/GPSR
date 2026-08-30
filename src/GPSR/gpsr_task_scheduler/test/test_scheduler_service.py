"""Section: GPSR scheduler service callback tests."""

# Section: Test fixtures, deterministic fakes, and regression scenarios.

from __future__ import annotations

import json

from gpsr_task_scheduler.scheduler_node import GpsrTaskSchedulerNode


class _Param:
    """Special function: Carry one fake ROS parameter value."""

    def __init__(self, value):
        """Special function: Store the supplied fake parameter value."""
        self.value = value


class _Config:
    """Special function: Carry fake scheduler execution timeout policy."""

    default_skill_timeout_sec = 30.0


class _Logger:
    """Special function: Capture fake scheduler warning messages."""

    def __init__(self):
        """Special function: Initialize captured warning storage."""
        self.warnings = []

    def warn(self, message: str) -> None:
        """Special function: Store one emitted warning message."""
        self.warnings.append(message)


class _StringMsg:
    """Special function: Carry one fake std_msgs/String payload."""

    def __init__(self, data: str):
        """Special function: Store the command text payload."""
        self.data = data


class _Request:
    """Special function: Carry one fake ScheduleCommand request."""

    def __init__(self, instruction: str, *, dry_run: bool, execute: bool):
        """Special function: Store fields consumed by the service callback."""
        self.command_id = "service_test"
        self.instruction = instruction
        self.dry_run = dry_run
        self.execute = execute
        self.overall_timeout_sec = 30.0


class _Response:
    """Special function: Carry one fake ScheduleCommand response."""

    def __init__(self):
        """Special function: Initialize response fields used by generated ROS service messages."""
        self.accepted = False
        self.success = False
        self.message = ""
        self.plan_json = ""
        self.execution_log_json = ""


class _FakeScheduler:
    """Special function: Provide only the node methods needed by the service callback."""

    def __init__(self, *, global_dry_run: bool = False, valid: bool = True):
        """Special function: Configure fake planning and execution behavior."""
        self.global_dry_run = global_dry_run
        self.valid = valid
        self.submitted = False
        self.scheduled_dry_run = None
        self.config = _Config()
        self.logger = _Logger()

    def get_parameter(self, _name: str) -> _Param:
        """Special function: Return the fake global dry-run parameter."""
        return _Param(self.global_dry_run)

    def get_logger(self) -> _Logger:
        """Special function: Return the fake logger used by topic callbacks."""
        return self.logger

    def schedule(self, command_id: str, instruction: str, dry_run: bool) -> dict:
        """Special function: Return a deterministic fake plan."""
        self.scheduled_dry_run = dry_run
        return {
            "command_id": command_id,
            "instruction": instruction,
            "valid": self.valid,
            "issues": [] if self.valid else [{"code": "invalid", "message": "bad command"}],
            "steps": [{"step_id": "say", "skill_name": "SAY"}] if self.valid else [],
            "dry_run": dry_run,
        }

    def submit_action_queue(self, command_id: str, planned: dict, timeout_sec: float) -> dict:
        """Special function: Record execution dispatch and return success."""
        self.submitted = True
        return {
            "success": True,
            "completed_steps": len(planned["steps"]),
            "message": f"executed {command_id} in {timeout_sec:.1f}s",
            "outcome": "success",
        }

    def process_one_command(
        self,
        command_id: str,
        instruction: str,
        dry_run: bool,
        timeout_sec: float,
    ) -> dict:
        """Special function: Emulate guarded cycle results for callback unit tests."""
        planned = self.schedule(command_id, instruction, dry_run)
        if not planned["valid"]:
            return {
                "accepted": False,
                "success": False,
                "message": json.dumps(planned["issues"]),
                "plan": planned,
            }
        if dry_run:
            return {
                "accepted": True,
                "success": True,
                "message": "Plan generated; execution skipped because dry_run is true.",
                "plan": planned,
            }
        executed = self.submit_action_queue(command_id, planned, timeout_sec)
        return {
            "accepted": True,
            "success": executed["success"],
            "message": executed["message"],
            "completed_steps": executed["completed_steps"],
            "plan": planned,
            "execution": executed,
        }


def test_schedule_service_dry_run_returns_plan_without_dispatch() -> None:
    """Special function: Dry-run service calls generate a plan and skip executor dispatch."""
    node = _FakeScheduler()
    response = GpsrTaskSchedulerNode.schedule_service_callback(
        node,
        _Request("count people in the kitchen tell me", dry_run=True, execute=False),
        _Response(),
    )
    plan = json.loads(response.plan_json)
    assert response.accepted is True
    assert response.success is True
    assert plan["valid"] is True
    assert node.scheduled_dry_run is True
    assert node.submitted is False


def test_schedule_service_execute_dispatches_valid_plan() -> None:
    """Special function: Execute service calls submit valid plans to the existing action queue."""
    node = _FakeScheduler()
    response = GpsrTaskSchedulerNode.schedule_service_callback(
        node,
        _Request("count people in the kitchen tell me", dry_run=False, execute=True),
        _Response(),
    )
    execution = json.loads(response.execution_log_json)
    assert response.accepted is True
    assert response.success is True
    assert execution["completed_steps"] == 1
    assert node.scheduled_dry_run is False
    assert node.submitted is True


def test_schedule_service_rejects_invalid_plan_without_dispatch() -> None:
    """Special function: Invalid service commands return planning issues and skip dispatch."""
    node = _FakeScheduler(valid=False)
    response = GpsrTaskSchedulerNode.schedule_service_callback(
        node,
        _Request("find it in the kitchen", dry_run=False, execute=True),
        _Response(),
    )
    issues = json.loads(response.message)
    assert response.accepted is False
    assert response.success is False
    assert issues[0]["code"] == "invalid"
    assert node.submitted is False


def test_schedule_service_respects_global_dry_run_guard() -> None:
    """Special function: Global dry-run prevents execution even when the service requests execution."""
    node = _FakeScheduler(global_dry_run=True)
    response = GpsrTaskSchedulerNode.schedule_service_callback(
        node,
        _Request("count people in the kitchen tell me", dry_run=False, execute=True),
        _Response(),
    )
    assert response.accepted is True
    assert response.success is True
    assert node.scheduled_dry_run is True
    assert node.submitted is False


def test_topic_command_dispatches_when_global_dry_run_is_false() -> None:
    """Special function: Topic commands submit valid plans when the scheduler is not in dry-run mode."""
    node = _FakeScheduler(global_dry_run=False)
    GpsrTaskSchedulerNode.command_callback(node, _StringMsg("count people in the kitchen tell me"))
    assert node.scheduled_dry_run is False
    assert node.submitted is True


def test_topic_command_skips_dispatch_when_global_dry_run_is_true() -> None:
    """Special function: Topic commands remain plan-only while global dry-run is enabled."""
    node = _FakeScheduler(global_dry_run=True)
    GpsrTaskSchedulerNode.command_callback(node, _StringMsg("count people in the kitchen tell me"))
    assert node.scheduled_dry_run is True
    assert node.submitted is False


def test_topic_command_rejects_invalid_plan_without_dispatch() -> None:
    """Special function: Invalid topic commands log the planning issue and skip executor dispatch."""
    node = _FakeScheduler(global_dry_run=False, valid=False)
    GpsrTaskSchedulerNode.command_callback(node, _StringMsg("find it in the kitchen"))
    assert node.submitted is False
    assert "GPSR command rejected" in node.logger.warnings[0]
