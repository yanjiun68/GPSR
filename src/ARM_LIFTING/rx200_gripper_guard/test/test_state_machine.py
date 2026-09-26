"""Transition tests for the guarded-close internal phase model."""

import pytest

from rx200_gripper_guard import constants as phase
from rx200_gripper_guard.guard_state_machine import GuardStateMachine


def test_contact_confirmation_and_hold_path() -> None:
    """Exercise IDLE through persistent contact and gentle hold."""
    machine = GuardStateMachine()
    assert machine.transition(phase.CLOSING) == phase.CLOSING
    assert machine.transition(phase.CONTACT_CANDIDATE) == phase.CONTACT_CANDIDATE
    assert machine.transition(phase.CLOSING) == phase.CLOSING
    assert machine.transition(phase.CONTACT_CANDIDATE) == phase.CONTACT_CANDIDATE
    assert machine.transition(phase.CONTACT_CONFIRMED) == phase.CONTACT_CONFIRMED
    assert machine.transition(phase.HOLDING) == phase.HOLDING
    assert machine.transition(phase.STOPPING) == phase.STOPPING
    assert machine.transition(phase.IDLE) == phase.IDLE


@pytest.mark.parametrize("terminal", [phase.TIMEOUT, phase.FAULT, phase.CANCELLED])
def test_error_terminal_paths_require_stopping(terminal: str) -> None:
    """Require every active error phase to pass through STOPPING before IDLE."""
    machine = GuardStateMachine()
    machine.transition(phase.CLOSING)
    machine.transition(terminal)
    with pytest.raises(ValueError):
        machine.transition(phase.IDLE)
    assert machine.transition(phase.STOPPING) == phase.STOPPING
    assert machine.transition(phase.IDLE) == phase.IDLE


def test_empty_mechanical_close_has_a_distinct_terminal_state() -> None:
    """Keep an empty end-stop outcome separate from hardware FAULT."""
    machine = GuardStateMachine()
    machine.transition(phase.CLOSING)
    assert machine.transition(phase.EMPTY_CLOSED) == phase.EMPTY_CLOSED
    assert machine.transition(phase.STOPPING) == phase.STOPPING
    assert machine.transition(phase.IDLE) == phase.IDLE
