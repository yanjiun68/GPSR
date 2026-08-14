"""Section: PLACE skill."""
from yasmin import Blackboard
from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import ActionStepData
from robot_yasmin_tasks.ros_clients.manipulation_clients import ManipulationClients
from robot_yasmin_tasks.skills.skill_base import SimpleSkillSM


def validate_place(blackboard: Blackboard, action: ActionStepData) -> tuple[bool, str]:
    """Special function: Validate place safety preconditions."""
    if not bb_get(blackboard, bb.HELD_OBJECT, ""):
        return False, "Place requires held_object."
    if bb.TARGET_POSE not in blackboard and not action.target_location and not action.parameters.get("target_pose"):
        return False, "Place requires target pose or location."
    return True, ""


class PlaceObjectSkillSM(SimpleSkillSM):
    """Special function: Place object and clear held_object."""
    def __init__(self, client: ManipulationClients | None = None) -> None:
        """Special function: Build PLACE state machine."""
        super().__init__("PLACE_OBJECT_SKILL_SM", (client or ManipulationClients()).place_object, validate_place)
