"""Section: GPSR scheduler YASMIN visualization regression tests."""

from __future__ import annotations

from gpsr_task_scheduler.cycle import SchedulerState
from gpsr_task_scheduler.scheduler_viewer_sm import SchedulerViewerSM


class _Timer:
    """Special function: Provide the timer interface required by YasminViewerPub."""

    def cancel(self) -> None:
        """Special function: Accept viewer shutdown without starting a ROS timer."""
        return None


class _Node:
    """Special function: Provide isolated publisher and timer factories for unit tests."""

    def create_publisher(self, *_args, **_kwargs):
        """Special function: Return an inert publisher placeholder."""
        return object()

    def create_timer(self, *_args, **_kwargs) -> _Timer:
        """Special function: Return an inert viewer timer placeholder."""
        return _Timer()

    def destroy_timer(self, _timer) -> None:
        """Special function: Accept timer cleanup from YasminViewerPub."""
        return None

    def destroy_publisher(self, _publisher) -> None:
        """Special function: Accept publisher cleanup from YasminViewerPub."""
        return None


def test_error_state_is_visible_and_can_reset_to_waiting() -> None:
    """Special function: Show ERROR and its reset destination in the YASMIN FSM."""
    # Section: Use one isolated node double so this topology test needs no ROS graph.
    node = _Node()
    viewer = SchedulerViewerSM(node)
    try:
        # Section: Verify ERROR exists as a native child state with a waiting transition.
        states = viewer.state_machine.get_states()
        assert SchedulerState.ERROR.value in states
        # Section: Expose every command-start return and deferred-report transition.
        for state in (
            SchedulerState.CAPTURING_RETURN_POSE,
            SchedulerState.RETURNING_TO_COMMAND_START,
            SchedulerState.RECOVERING_TO_COMMAND_START,
            SchedulerState.REPORTING_RESULT,
        ):
            assert state.value in states
        error_transitions = states[SchedulerState.ERROR.value]["transitions"]
        assert (
            error_transitions[SchedulerState.WAITING_FOR_COMMAND.value]
            == SchedulerState.WAITING_FOR_COMMAND.value
        )

        # Section: Drive the asynchronous mirror through startup failure and operator reset.
        viewer.update(SchedulerState.GO_TO_INSTRUCTION_POINT)
        assert viewer.wait_until_visible(SchedulerState.GO_TO_INSTRUCTION_POINT)
        viewer.update(SchedulerState.ERROR)
        assert viewer.wait_until_visible(SchedulerState.ERROR)
        viewer.update(SchedulerState.WAITING_FOR_COMMAND)
        assert viewer.wait_until_visible(SchedulerState.WAITING_FOR_COMMAND)
    finally:
        # Section: Release the worker and its inert publication resources.
        viewer.shutdown()
