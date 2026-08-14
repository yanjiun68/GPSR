"""Section: Thread-safe GPSR single-command competition-cycle state model."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from enum import Enum


class SchedulerState(str, Enum):
    """Special function: Define externally visible GPSR scheduler lifecycle states."""

    INITIALIZING = "INITIALIZING"
    GO_TO_INSTRUCTION_POINT = "GO_TO_INSTRUCTION_POINT"
    WAITING_FOR_COMMAND = "WAITING_FOR_COMMAND"
    PLANNING = "PLANNING"
    EXECUTING = "EXECUTING"
    RETURNING_TO_INSTRUCTION_POINT = "RETURNING_TO_INSTRUCTION_POINT"
    RECOVERING_TO_INSTRUCTION_POINT = "RECOVERING_TO_INSTRUCTION_POINT"
    FINISHED = "FINISHED"
    ERROR = "ERROR"


@dataclass(frozen=True)
class CycleSnapshot:
    """Special function: Expose an immutable scheduler-cycle status snapshot."""

    state: SchedulerState
    active_command_id: str
    completed_command_count: int
    max_commands: int


class GpsrCycleController:
    """Special function: Serialize one competition command from planning through return."""

    def __init__(self, max_commands: int = 3) -> None:
        """Special function: Initialize the competition cycle before first navigation."""
        self._lock = threading.Lock()
        self._state = SchedulerState.INITIALIZING
        self._active_command_id = ""
        self._completed_command_count = 0
        self._max_commands = max(0, int(max_commands))

    def snapshot(self) -> CycleSnapshot:
        """Special function: Read cycle state without exposing mutable internals."""
        with self._lock:
            return CycleSnapshot(
                self._state,
                self._active_command_id,
                self._completed_command_count,
                self._max_commands,
            )

    def start_initial_return(self) -> bool:
        """Special function: Begin initial navigation to the configured Instruction Point."""
        with self._lock:
            if self._state != SchedulerState.INITIALIZING:
                return False
            self._state = SchedulerState.GO_TO_INSTRUCTION_POINT
            return True

    def finish_initial_return(self, success: bool) -> bool:
        """Special function: Enable command intake only after initial navigation terminates."""
        with self._lock:
            if self._state != SchedulerState.GO_TO_INSTRUCTION_POINT:
                return False
            self._state = (
                SchedulerState.WAITING_FOR_COMMAND if success else SchedulerState.ERROR
            )
            return True

    def try_accept_command(self, command_id: str) -> bool:
        """Special function: Atomically admit one new command only while waiting."""
        normalized_id = str(command_id).strip()
        with self._lock:
            if not normalized_id or self._state != SchedulerState.WAITING_FOR_COMMAND:
                return False
            if self._max_commands and self._completed_command_count >= self._max_commands:
                self._state = SchedulerState.FINISHED
                return False
            self._active_command_id = normalized_id
            self._state = SchedulerState.PLANNING
            return True

    def finish_planning(self, command_id: str, valid: bool) -> bool:
        """Special function: Enter execution for a valid plan or release an invalid command."""
        with self._lock:
            if not self._matches(command_id, SchedulerState.PLANNING):
                return False
            if valid:
                self._state = SchedulerState.EXECUTING
            else:
                self._active_command_id = ""
                self._state = SchedulerState.WAITING_FOR_COMMAND
            return True

    def finish_execution(self, command_id: str) -> bool:
        """Special function: Route every terminal task-plan outcome through return navigation."""
        with self._lock:
            if not self._matches(command_id, SchedulerState.EXECUTING):
                return False
            self._state = SchedulerState.RETURNING_TO_INSTRUCTION_POINT
            return True

    def finish_return(self, command_id: str, success: bool) -> bool:
        """Special function: Complete only the active command and reject stale result callbacks."""
        with self._lock:
            if not self._matches(command_id, SchedulerState.RETURNING_TO_INSTRUCTION_POINT):
                return False
            if not success:
                self._state = SchedulerState.ERROR
                return True
            self._completed_command_count += 1
            self._active_command_id = ""
            self._state = (
                SchedulerState.FINISHED
                if self._max_commands and self._completed_command_count >= self._max_commands
                else SchedulerState.WAITING_FOR_COMMAND
            )
            return True

    def start_error_recovery(self) -> bool:
        """Special function: Admit one recovery attempt only from the terminal error state."""
        with self._lock:
            if self._state != SchedulerState.ERROR:
                return False
            self._state = SchedulerState.RECOVERING_TO_INSTRUCTION_POINT
            return True

    def finish_error_recovery(self, success: bool) -> bool:
        """Special function: Return to command intake only after recovery navigation succeeds."""
        with self._lock:
            if self._state != SchedulerState.RECOVERING_TO_INSTRUCTION_POINT:
                return False
            if not success:
                self._state = SchedulerState.ERROR
                return True

            # Section: A post-command recovery completes the same command-count commitment.
            if self._active_command_id:
                self._completed_command_count += 1
            self._active_command_id = ""
            self._state = (
                SchedulerState.FINISHED
                if self._max_commands
                and self._completed_command_count >= self._max_commands
                else SchedulerState.WAITING_FOR_COMMAND
            )
            return True

    def _matches(self, command_id: str, expected_state: SchedulerState) -> bool:
        """Special function: Match both lifecycle state and active command identity."""
        return (
            self._state == expected_state
            and self._active_command_id == str(command_id).strip()
        )
