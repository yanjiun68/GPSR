"""Section: PICK_OBJECT executable primitive skill."""
from yasmin import Blackboard
from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import ActionStepData
from robot_yasmin_tasks.ros_clients.manipulation_clients import ManipulationClients
from robot_yasmin_tasks.skills.skill_base import SimpleSkillSM


def validate_pick(blackboard: Blackboard, action: ActionStepData) -> tuple[bool, str]:
    """Special function: Validate pick safety preconditions."""
    if bb_get(blackboard, bb.HELD_OBJECT, ""):
        return False, "Robot is already holding an object."
    if not action.target_object and not bb_get(blackboard, bb.TARGET_OBJECT, ""):
        return False, "Pick requires object reference."
    if bb.TARGET_POSE not in blackboard and not action.parameters.get("object_pose"):
        return False, "Pick requires object pose."
    return True, ""


class PickObjectSkillSM(SimpleSkillSM):
    """Special function: Pick object and update held_object."""
    def __init__(self, client: ManipulationClients | None = None) -> None:
        """Special function: Build PICK_OBJECT state machine."""
        super().__init__("PICK_OBJECT_SKILL_SM", (client or ManipulationClients()).pick_object, validate_pick)
