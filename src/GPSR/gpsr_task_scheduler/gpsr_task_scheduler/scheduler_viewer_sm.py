"""Section: YASMIN visualization mirror for the GPSR scheduler cycle."""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Any

from yasmin import State, StateMachine
from yasmin_viewer import YasminViewerPub

from gpsr_task_scheduler.cycle import SchedulerState


# Section: Terminal outcome used only when the scheduler node shuts down.
SHUTDOWN_OUTCOME = "shutdown"

# Section: Valid scheduler-cycle edges displayed by YASMIN Viewer.
SCHEDULER_TRANSITIONS: dict[str, tuple[str, ...]] = {
    SchedulerState.INITIALIZING.value: (
        SchedulerState.GO_TO_INSTRUCTION_POINT.value,
    ),
    SchedulerState.GO_TO_INSTRUCTION_POINT.value: (
        SchedulerState.WAITING_FOR_COMMAND.value,
        SchedulerState.ERROR.value,
    ),
    SchedulerState.WAITING_FOR_COMMAND.value: (
        SchedulerState.PLANNING.value,
        SchedulerState.FINISHED.value,
    ),
    SchedulerState.PLANNING.value: (
        SchedulerState.EXECUTING.value,
        SchedulerState.WAITING_FOR_COMMAND.value,
    ),
    SchedulerState.EXECUTING.value: (
        SchedulerState.RETURNING_TO_INSTRUCTION_POINT.value,
    ),
    SchedulerState.RETURNING_TO_INSTRUCTION_POINT.value: (
        SchedulerState.WAITING_FOR_COMMAND.value,
        SchedulerState.FINISHED.value,
        SchedulerState.ERROR.value,
    ),
    SchedulerState.ERROR.value: (
        SchedulerState.RECOVERING_TO_INSTRUCTION_POINT.value,
        SchedulerState.WAITING_FOR_COMMAND.value,
    ),
    SchedulerState.RECOVERING_TO_INSTRUCTION_POINT.value: (
        SchedulerState.WAITING_FOR_COMMAND.value,
        SchedulerState.FINISHED.value,
        SchedulerState.ERROR.value,
    ),
    SchedulerState.FINISHED.value: (),
}


class SchedulerCycleState(State):
    """Special function: Hold one visible scheduler state until the cycle changes."""

    def __init__(self, state_name: str, supervisor: "SchedulerViewerSM") -> None:
        """Special function: Configure transitions from this visible scheduler state."""
        # Section: Expose only authoritative scheduler transitions in the viewer diagram.
        outcomes = set(SCHEDULER_TRANSITIONS[state_name])
        # Section: Let node shutdown terminate the otherwise persistent viewer machine.
        outcomes.add(SHUTDOWN_OUTCOME)
        super().__init__(outcomes)
        self.state_name = state_name
        self.supervisor = supervisor

    def execute(self, _blackboard: Any) -> str:
        """Special function: Wait until the authoritative scheduler leaves this state."""
        return self.supervisor.wait_for_state_change(self.state_name)


class SchedulerViewerSM:
    """Special function: Mirror scheduler-cycle transitions into a live YASMIN FSM."""

    def __init__(self, node: Any, fsm_name: str = "GPSR_SCHEDULER_CYCLE") -> None:
        """Special function: Build, publish, and run the scheduler visualization FSM."""
        # Section: Synchronize ROS callbacks with the blocking YASMIN state thread.
        self.condition = threading.Condition()
        self.pending_states: deque[str] = deque()
        self.last_requested_state = SchedulerState.INITIALIZING.value
        self.running = True

        # Section: Give YASMIN Viewer a complete topology including the explicit ERROR state.
        self.state_machine = StateMachine([SHUTDOWN_OUTCOME], handle_sigint=False)
        for scheduler_state in SchedulerState:
            state_name = scheduler_state.value
            transitions = {
                candidate: candidate
                for candidate in SCHEDULER_TRANSITIONS[state_name]
            }
            transitions[SHUTDOWN_OUTCOME] = SHUTDOWN_OUTCOME
            self.state_machine.add_state(
                state_name,
                SchedulerCycleState(state_name, self),
                transitions,
            )
        self.state_machine.set_start_state(SchedulerState.INITIALIZING.value)
        self.state_machine.validate()

        # Section: Reuse the scheduler ROS node for /fsm_viewer publication.
        self.viewer = YasminViewerPub(
            self.state_machine,
            fsm_name,
            node=node,
        )

        # Section: Keep the visualization FSM responsive without blocking ROS callbacks.
        self.thread = threading.Thread(
            target=self._run,
            name="gpsr_scheduler_viewer_sm",
            daemon=True,
        )
        self.thread.start()

    def _run(self) -> None:
        """Special function: Execute the persistent visualization state machine."""
        self.state_machine()

    def update(self, state: SchedulerState | str) -> None:
        """Special function: Wake YASMIN when the authoritative scheduler state changes."""
        # Section: Normalize enum and text callers to the viewer's canonical state labels.
        state_name = state.value if isinstance(state, SchedulerState) else str(state)
        if state_name not in {candidate.value for candidate in SchedulerState}:
            return
        with self.condition:
            # Section: Preserve rapid intermediate transitions for accurate live debugging.
            if state_name == self.last_requested_state:
                return
            self.pending_states.append(state_name)
            self.last_requested_state = state_name
            self.condition.notify_all()

    def wait_for_state_change(self, current_state: str) -> str:
        """Special function: Block one YASMIN state until a new scheduler state is available."""
        with self.condition:
            while self.running:
                while self.pending_states:
                    next_state = self.pending_states.popleft()
                    if next_state != current_state:
                        return next_state
                self.condition.wait(timeout=0.25)
            return SHUTDOWN_OUTCOME

    def wait_until_visible(self, state: SchedulerState | str, timeout_sec: float = 1.0) -> bool:
        """Special function: Support deterministic tests of asynchronous viewer transitions."""
        # Section: Poll only the YASMIN-owned current-state field during the bounded test wait.
        expected = state.value if isinstance(state, SchedulerState) else str(state)
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while time.monotonic() < deadline:
            if self.state_machine.get_current_state() == expected:
                return True
            time.sleep(0.01)
        return self.state_machine.get_current_state() == expected

    def shutdown(self) -> None:
        """Special function: Stop viewer publication and release the YASMIN worker thread."""
        # Section: Release whichever visible state is waiting for the next transition.
        with self.condition:
            self.running = False
            self.condition.notify_all()
        self.thread.join(timeout=1.0)
        self.viewer.shutdown()
