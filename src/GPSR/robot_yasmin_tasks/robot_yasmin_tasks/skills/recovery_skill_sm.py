"""Section: Deterministic bounded recovery skill."""
from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import make_skill_result, normalize_action_step
from robot_yasmin_tasks.gpsr.outcomes import FATAL_FAILURE, SUCCESS
from robot_yasmin_tasks.skills.skill_base import SimpleSkillSM


class RecoveryPolicyClient:
    """Special function: Apply only the ActionStep's configured bounded retries."""
    def recover(self, blackboard, _action):
        """Special function: Update recovery attempts and decide retry."""
        current = normalize_action_step(bb_get(blackboard, bb.CURRENT_ACTION, {}))
        key = current.step_id or f"{bb_get(blackboard, bb.CURRENT_ACTION_INDEX, 0)}:{current.action_type}"
        attempts = dict(bb_get(blackboard, bb.RECOVERY_ATTEMPTS, {}))
        if int(attempts.get(key, 0)) >= int(current.max_retries):
            return make_skill_result(
                current.action_type,
                FATAL_FAILURE,
                f"Recovery exhausted after {current.max_retries} retries.",
                fatal=True,
            )
        attempts[key] = int(attempts.get(key, 0)) + 1
        blackboard[bb.RECOVERY_ATTEMPTS] = attempts
        blackboard[bb.RECOVERY_HINT] = "bounded_retry"
        return make_skill_result(current.action_type, SUCCESS, "Recovery applied.", {"attempt": attempts[key]})


class RecoverySkillSM(SimpleSkillSM):
    """Special function: Recovery sub-state-machine."""
    def __init__(self, client: RecoveryPolicyClient | None = None) -> None:
        """Special function: Build RECOVERY state machine."""
        super().__init__("RECOVERY_SKILL_SM", (client or RecoveryPolicyClient()).recover, None)
