"""Section: HANDOVER_OBJECT executable primitive skill."""
from yasmin import Blackboard
from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import ActionStepData
from robot_yasmin_tasks.ros_clients.manipulation_clients import ManipulationClients
from robot_yasmin_tasks.skills.skill_base import SimpleSkillSM


def validate_handover(blackboard: Blackboard, action: ActionStepData) -> tuple[bool, str]:
    """Special function: Validate handover safety preconditions."""
    if not bb_get(blackboard, bb.HELD_OBJECT, ""):
        return False, "Give requires held_object."
    if not action.target_person and not bb_get(blackboard, bb.TARGET_PERSON, ""):
        return False, "Give requires target person."
    return True, ""


class HandoverSkillSM(SimpleSkillSM):
    """Special function: Handover held object to person."""
    def __init__(self, client: ManipulationClients | None = None) -> None:
        """Special function: Build HANDOVER_OBJECT state machine."""
        super().__init__("HANDOVER_SKILL_SM", (client or ManipulationClients()).handover, validate_handover)
