"""Section: Reusable YASMIN skill state-machine base."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from yasmin import Blackboard, State, StateMachine

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import ActionStepData, make_skill_result, normalize_action_step
from robot_yasmin_tasks.gpsr.outcomes import FAILED, INVALID_INPUT, SKILL_OUTCOMES, SUCCESS


ValidationFn = Callable[[Blackboard, ActionStepData], tuple[bool, str]]
ClientFn = Callable[[Blackboard, ActionStepData], Any]


def require_any_field(*names: str) -> ValidationFn:
    """Special function: Build a validator that requires one field or parameter."""
    def _validate(_blackboard: Blackboard, action: ActionStepData) -> tuple[bool, str]:
        """Special function: Check configured names against action fields and parameters."""
        for name in names:
            if getattr(action, name, "") or action.parameters.get(name, ""):
                return True, ""
        return False, f"Missing one of required fields: {', '.join(names)}"
    return _validate


class ValidateSkillInputState(State):
    """Special function: Validate one GPSR skill before execution."""

    def __init__(self, validator: ValidationFn | None) -> None:
        """Special function: Store skill-specific validation callback."""
        super().__init__([SUCCESS, INVALID_INPUT])
        self.validator = validator

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Normalize current action and reject invalid skill input."""
        action = normalize_action_step(bb_get(blackboard, bb.CURRENT_ACTION, {}))
        blackboard[bb.CURRENT_ACTION] = action
        if action.timeout_sec < 0.0:
            return self._invalid(blackboard, action, "Timeout must not be negative.")
        if self.validator is not None:
            valid, message = self.validator(blackboard, action)
            if not valid:
                return self._invalid(blackboard, action, message)
        return SUCCESS

    def _invalid(self, blackboard: Blackboard, action: ActionStepData, message: str) -> str:
        """Special function: Store a fatal invalid-input result."""
        blackboard[bb.FAILURE_REASON] = message
        blackboard[bb.LAST_SKILL_RESULT] = make_skill_result(action.action_type, INVALID_INPUT, message, fatal=True).to_dict()
        return INVALID_INPUT


class ExecuteSkillClientState(State):
    """Special function: Execute one deterministic client wrapper."""

    def __init__(self, client_call: ClientFn) -> None:
        """Special function: Store the client callback."""
        super().__init__(SKILL_OUTCOMES)
        self.client_call = client_call

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Run client callback and store the deterministic result."""
        action = normalize_action_step(bb_get(blackboard, bb.CURRENT_ACTION, {}))
        try:
            result = self.client_call(blackboard, action)
        except Exception as exc:  # noqa: BLE001
            result = make_skill_result(action.action_type, FAILED, f"Skill client error: {exc}", recoverable=True)
        result_dict = result.to_dict() if hasattr(result, "to_dict") else dict(result)
        blackboard[bb.LAST_SKILL_RESULT] = result_dict
        blackboard[bb.DISPATCH_OUTCOME] = result_dict.get("outcome", FAILED)
        return str(result_dict.get("outcome", FAILED))


class SimpleSkillSM(StateMachine):
    """Special function: Minimal validate-then-execute skill sub-state-machine."""

    def __init__(self, name: str, client_call: ClientFn, validator: ValidationFn | None = None) -> None:
        """Special function: Build the YASMIN subgraph for one robot skill."""
        super().__init__(outcomes=SKILL_OUTCOMES)
        self.set_name(name)
        self.add_state("VALIDATE_INPUT", ValidateSkillInputState(validator), transitions={SUCCESS: "EXECUTE_CLIENT", INVALID_INPUT: INVALID_INPUT})
        self.add_state("EXECUTE_CLIENT", ExecuteSkillClientState(client_call), transitions={outcome: outcome for outcome in SKILL_OUTCOMES})
