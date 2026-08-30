"""ROS-independent guarded-close phase transition model."""

from rx200_gripper_guard import constants as phase


class GuardStateMachine:
    """Enforce the allowed internal guarded-close phase transitions."""

    _ALLOWED = {
        phase.IDLE: {phase.CLOSING},
        phase.CLOSING: {
            phase.CONTACT_CANDIDATE,
            phase.TIMEOUT,
            phase.FAULT,
            phase.CANCELLED,
        },
        phase.CONTACT_CANDIDATE: {
            phase.CLOSING,
            phase.CONTACT_CONFIRMED,
            phase.TIMEOUT,
            phase.FAULT,
            phase.CANCELLED,
        },
        phase.CONTACT_CONFIRMED: {phase.HOLDING, phase.STOPPING},
        phase.HOLDING: {phase.STOPPING},
        phase.STOPPING: {phase.IDLE},
        phase.TIMEOUT: {phase.STOPPING},
        phase.FAULT: {phase.STOPPING},
        phase.CANCELLED: {phase.STOPPING},
    }

    def __init__(self) -> None:
        """Initialize every guarded-close operation in IDLE."""
        self.state = phase.IDLE

    def transition(self, next_state: str) -> str:
        """Apply one valid transition or reject a programming error."""
        if next_state not in self._ALLOWED[self.state]:
            raise ValueError(f"invalid guard transition: {self.state} -> {next_state}")
        self.state = next_state
        return self.state

    def reset(self) -> None:
        """Return a completed operation to the required initial state."""
        if self.state != phase.IDLE:
            if self.state != phase.STOPPING:
                self.transition(phase.STOPPING)
            self.transition(phase.IDLE)
