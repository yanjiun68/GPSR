"""Section: GPSR single-command competition-cycle regression tests."""

from __future__ import annotations

from gpsr_task_scheduler.cycle import GpsrCycleController, SchedulerState
from gpsr_task_scheduler.scheduler_node import GpsrTaskSchedulerNode


class _Publisher:
    """Special function: Capture published scheduler messages for assertions."""

    def __init__(self) -> None:
        """Special function: Initialize an empty message list."""
        self.messages = []

    def publish(self, message) -> None:
        """Special function: Store one published message."""
        self.messages.append(message)


class _Param:
    """Special function: Carry one fake recovery parameter value."""

    def __init__(self, value) -> None:
        """Special function: Store a supplied fake parameter value."""
        self.value = value


class _TriggerResponse:
    """Special function: Carry guarded reset service response fields."""

    def __init__(self) -> None:
        """Special function: Initialize an unsuccessful empty reset response."""
        self.success = False
        self.message = ""


class _Logger:
    """Special function: Capture scheduler recovery logs without a ROS node."""

    def __init__(self) -> None:
        """Special function: Initialize recovery log storage."""
        self.messages = []

    def info(self, message: str) -> None:
        """Special function: Capture one recovery success message."""
        self.messages.append(("info", message))

    def warn(self, message: str) -> None:
        """Special function: Capture one recovery warning message."""
        self.messages.append(("warn", message))

    def error(self, message: str) -> None:
        """Special function: Capture one exhausted-recovery message."""
        self.messages.append(("error", message))


class _RecoveryHarness:
    """Special function: Exercise automatic ERROR recovery without a ROS graph."""

    def __init__(self, return_outcomes: list[bool], max_attempts: int = 3) -> None:
        """Special function: Initialize a failed startup and deterministic return results."""
        self.cycle = GpsrCycleController(max_commands=3)
        self.cycle.start_initial_return()
        self.cycle.finish_initial_return(False)
        self.return_outcomes = list(return_outcomes)
        self.error_recovery_attempts = 0
        self.error_recovery_next_attempt_time = 0.0
        self.error_recovery_exhaustion_logged = False
        self.max_attempts = max_attempts
        self.states = []
        self.logger = _Logger()

    def get_parameter(self, name: str) -> _Param:
        """Special function: Supply automatic-recovery policy parameters."""
        values = {
            "enable_error_recovery": True,
            "error_recovery_max_attempts": self.max_attempts,
            "error_recovery_retry_delay_sec": 0.1,
        }
        return _Param(values[name])

    def publish_cycle_state(self) -> None:
        """Special function: Capture externally visible recovery transitions."""
        self.states.append(self.cycle.snapshot().state)

    def get_logger(self) -> _Logger:
        """Special function: Return the fake recovery logger."""
        return self.logger

    def submit_return_to_instruction_point(self, _command_id: str) -> dict:
        """Special function: Return the next deterministic recovery-navigation result."""
        success = self.return_outcomes.pop(0)
        return {
            "success": success,
            "message": "returned" if success else "navigation unavailable",
        }

    def recover(self) -> None:
        """Special function: Invoke the production automatic recovery callback."""
        GpsrTaskSchedulerNode.recover_from_error(self)


class _Timer:
    """Special function: Model the one-shot scheduler initialization timer."""

    def __init__(self) -> None:
        """Special function: Initialize the timer as active."""
        self.canceled = False

    def cancel(self) -> None:
        """Special function: Record cancellation of the initialization callback."""
        self.canceled = True


class _InitializationHarness:
    """Special function: Exercise dry-run initialization without commanding navigation."""

    def __init__(self) -> None:
        """Special function: Initialize a dry-run scheduler before its startup transition."""
        self.cycle = GpsrCycleController(max_commands=3)
        self.initialization_started = False
        self.initialization_timer = _Timer()
        self.navigation_submitted = False

    def get_parameter(self, name: str) -> _Param:
        """Special function: Enable both dry-run and the normally enabled startup return."""
        return _Param({
            "dry_run": True,
            "navigate_to_instruction_point_on_start": True,
        }[name])

    def publish_cycle_state(self) -> None:
        """Special function: Accept startup state publications in the pure unit test."""
        return None

    def submit_return_to_instruction_point(self, _command_id: str) -> dict:
        """Special function: Detect any prohibited physical navigation submission."""
        self.navigation_submitted = True
        return {"success": True, "message": "unexpected navigation"}


class _CycleHarness:
    """Special function: Exercise node cycle logic without ROS graph dependencies."""

    def __init__(self, skills: list[str], task_success: bool = True, max_commands: int = 3) -> None:
        """Special function: Configure a ready scheduler with deterministic queue outcomes."""
        self.cycle = GpsrCycleController(max_commands)
        self.cycle.start_initial_return()
        self.cycle.finish_initial_return(True)
        self.skills = list(skills)
        self.task_success = task_success
        self.command_result_pub = _Publisher()
        self.states = []
        self.submissions = []
        self.second_command_accepted = None

    def publish_cycle_state(self) -> None:
        """Special function: Capture each externally visible lifecycle transition."""
        self.states.append(self.cycle.snapshot().state)

    def schedule(self, command_id: str, _instruction: str, dry_run: bool) -> dict:
        """Special function: Return one valid multi-skill plan for the active command."""
        return {
            "command_id": command_id,
            "valid": True,
            "dry_run": dry_run,
            "steps": [
                {
                    "step_id": f"step_{index}",
                    "skill_name": skill,
                    "arguments": {},
                    "dependencies": [] if index == 0 else [f"step_{index - 1}"],
                    "timeout": 10.0,
                    "retry_count": 0,
                    "result_variable": None,
                }
                for index, skill in enumerate(self.skills)
            ],
            "issues": [],
        }

    def without_embedded_return(self, planned: dict) -> dict:
        """Special function: Reuse production removal of embedded return steps."""
        return GpsrTaskSchedulerNode.without_embedded_return(self, planned)

    def submit_action_queue(self, command_id: str, planned: dict, _timeout_sec: float) -> dict:
        """Special function: Capture task submission and inject a concurrent-command attempt."""
        skills = [step["skill_name"] for step in planned["steps"]]
        self.submissions.append((command_id, skills))
        if len(self.submissions) == 1:
            self.second_command_accepted = self.cycle.try_accept_command("command_2")
        return {
            "success": self.task_success,
            "completed_steps": len(skills),
            "message": "success" if self.task_success else "task failed",
            "outcome": "success" if self.task_success else "fatal_failure",
        }

    def submit_return_to_instruction_point(self, command_id: str) -> dict:
        """Special function: Record the return request after the task queue terminates."""
        self.submissions.append((f"{command_id}_return", ["RETURN_TO_INSTRUCTION_POINT"]))
        return {
            "success": True,
            "completed_steps": 1,
            "message": "returned",
            "outcome": "success",
        }

    def process(self, command_id: str) -> dict:
        """Special function: Invoke production command-cycle orchestration."""
        return GpsrTaskSchedulerNode.process_one_command(
            self,
            command_id,
            "test instruction",
            False,
            30.0,
        )


def test_initial_navigation_gates_command_intake() -> None:
    """Special function: Accept no command before reaching the Instruction Point."""
    cycle = GpsrCycleController(max_commands=3)
    assert cycle.try_accept_command("too_early") is False
    assert cycle.start_initial_return() is True
    assert cycle.try_accept_command("still_returning") is False
    assert cycle.finish_initial_return(True) is True
    assert cycle.try_accept_command("command_1") is True


def test_dry_run_skips_physical_initial_navigation() -> None:
    """Special function: Dry-run startup reaches waiting without submitting a return goal."""
    harness = _InitializationHarness()
    GpsrTaskSchedulerNode.initialize_competition_cycle(harness)
    assert harness.navigation_submitted is False
    assert harness.initialization_timer.canceled is True
    assert harness.cycle.snapshot().state == SchedulerState.WAITING_FOR_COMMAND


def test_failed_initial_return_recovers_command_intake() -> None:
    """Special function: A successful guarded retry transitions ERROR back to waiting."""
    harness = _RecoveryHarness([True])
    harness.recover()
    assert harness.states == [
        SchedulerState.RECOVERING_TO_INSTRUCTION_POINT,
        SchedulerState.WAITING_FOR_COMMAND,
    ]
    assert harness.cycle.try_accept_command("command_after_recovery") is True


def test_failed_recovery_returns_to_error_then_retries() -> None:
    """Special function: A failed retry remains closed until a later retry succeeds."""
    harness = _RecoveryHarness([False, True])
    harness.recover()
    assert harness.cycle.snapshot().state == SchedulerState.ERROR
    assert harness.cycle.try_accept_command("blocked") is False
    harness.error_recovery_next_attempt_time = 0.0
    harness.recover()
    assert harness.cycle.snapshot().state == SchedulerState.WAITING_FOR_COMMAND


def test_post_command_recovery_commits_cycle_and_clears_ownership() -> None:
    """Special function: Recovery after a failed final return completes command bookkeeping."""
    harness = _RecoveryHarness([True])
    harness.cycle = GpsrCycleController(max_commands=3)
    harness.cycle.start_initial_return()
    harness.cycle.finish_initial_return(True)
    harness.cycle.try_accept_command("command_1")
    harness.cycle.finish_planning("command_1", True)
    harness.cycle.finish_execution("command_1")
    harness.cycle.finish_return("command_1", False)
    harness.recover()
    snapshot = harness.cycle.snapshot()
    assert snapshot.state == SchedulerState.WAITING_FOR_COMMAND
    assert snapshot.active_command_id == ""
    assert snapshot.completed_command_count == 1


def test_exhausted_recovery_stays_in_error() -> None:
    """Special function: Bounded automatic recovery stops issuing motion after its limit."""
    harness = _RecoveryHarness([False], max_attempts=1)
    harness.recover()
    harness.error_recovery_next_attempt_time = 0.0
    harness.recover()
    assert harness.cycle.snapshot().state == SchedulerState.ERROR
    assert harness.return_outcomes == []
    assert any(level == "error" for level, _message in harness.logger.messages)


def test_reset_service_rearms_exhausted_error_recovery() -> None:
    """Special function: Operator reset clears retry guards without bypassing recovery."""
    harness = _RecoveryHarness([], max_attempts=3)
    harness.error_recovery_attempts = 3
    harness.error_recovery_next_attempt_time = 999.0
    harness.error_recovery_exhaustion_logged = True
    response = GpsrTaskSchedulerNode.reset_state_callback(
        harness,
        None,
        _TriggerResponse(),
    )
    assert response.success is True
    assert harness.error_recovery_attempts == 0
    assert harness.error_recovery_next_attempt_time == 0.0
    assert harness.error_recovery_exhaustion_logged is False
    assert harness.cycle.snapshot().state == SchedulerState.ERROR


def test_reset_service_rejects_non_error_state() -> None:
    """Special function: Operator reset cannot bypass normal scheduler transitions."""
    harness = _RecoveryHarness([], max_attempts=3)
    harness.cycle = GpsrCycleController(max_commands=3)
    harness.cycle.start_initial_return()
    harness.cycle.finish_initial_return(True)
    response = GpsrTaskSchedulerNode.reset_state_callback(
        harness,
        None,
        _TriggerResponse(),
    )
    assert response.success is False
    assert "WAITING_FOR_COMMAND" in response.message


def test_normal_command_returns_before_waiting_again() -> None:
    """Special function: Verify WAIT, PLAN, EXECUTE, RETURN, WAIT lifecycle."""
    harness = _CycleHarness(["NAVIGATE", "COUNT_OBJECTS", "SAY"])
    result = harness.process("command_1")
    assert result["success"] is True
    assert harness.states == [
        SchedulerState.PLANNING,
        SchedulerState.EXECUTING,
        SchedulerState.RETURNING_TO_INSTRUCTION_POINT,
        SchedulerState.WAITING_FOR_COMMAND,
    ]


def test_multi_skill_command_returns_only_after_final_skill() -> None:
    """Special function: Keep NAVIGATE, COUNT_OBJECTS, and SAY in one task plan."""
    harness = _CycleHarness(["NAVIGATE", "COUNT_OBJECTS", "SAY"])
    harness.process("command_1")
    assert harness.submissions == [
        ("command_1", ["NAVIGATE", "COUNT_OBJECTS", "SAY"]),
        ("command_1_return", ["RETURN_TO_INSTRUCTION_POINT"]),
    ]


def test_second_command_is_rejected_during_execution() -> None:
    """Special function: Prevent a concurrent transcript from starting another plan."""
    harness = _CycleHarness(["NAVIGATE", "COUNT_OBJECTS"])
    harness.process("command_1")
    assert harness.second_command_accepted is False
    assert len(harness.submissions) == 2


def test_three_consecutive_commands_finish_competition() -> None:
    """Special function: Count completed cycles without hard-coding command identities."""
    harness = _CycleHarness(["SAY"], max_commands=3)
    for command_number in range(1, 4):
        assert harness.process(f"command_{command_number}")["accepted"] is True
        harness.submissions.clear()
    snapshot = harness.cycle.snapshot()
    assert snapshot.completed_command_count == 3
    assert snapshot.state == SchedulerState.FINISHED
    assert harness.process("command_4")["accepted"] is False


def test_terminal_task_failure_returns_and_releases_scheduler() -> None:
    """Special function: Return after cleaned executor failure without deadlocking intake."""
    harness = _CycleHarness(["COUNT_OBJECTS"], task_success=False)
    result = harness.process("command_1")
    assert result["success"] is False
    assert harness.submissions[-1][1] == ["RETURN_TO_INSTRUCTION_POINT"]
    assert harness.cycle.snapshot().state == SchedulerState.WAITING_FOR_COMMAND


def test_stale_result_cannot_mutate_new_command_state() -> None:
    """Special function: Reject an old return result after a new command becomes active."""
    cycle = GpsrCycleController(max_commands=0)
    cycle.start_initial_return()
    cycle.finish_initial_return(True)
    cycle.try_accept_command("command_1")
    cycle.finish_planning("command_1", True)
    cycle.finish_execution("command_1")
    cycle.finish_return("command_1", True)
    cycle.try_accept_command("command_2")
    assert cycle.finish_return("command_1", True) is False
    snapshot = cycle.snapshot()
    assert snapshot.active_command_id == "command_2"
    assert snapshot.state == SchedulerState.PLANNING
