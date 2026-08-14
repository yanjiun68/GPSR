"""Section: DISPOSE_TRASH skill."""
from yasmin import Blackboard
from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import ActionStepData
from robot_yasmin_tasks.ros_clients.manipulation_clients import ManipulationClients
from robot_yasmin_tasks.skills.skill_base import SimpleSkillSM


def validate_dispose(blackboard: Blackboard, action: ActionStepData) -> tuple[bool, str]:
    """Special function: Validate trash disposal target."""
    return (bool(bb_get(blackboard, bb.HELD_OBJECT, "") or action.target_object), "Dispose requires held or target object.")


class DisposeTrashSkillSM(SimpleSkillSM):
    """Special function: Dispose trash deterministically."""
    def __init__(self, client: ManipulationClients | None = None) -> None:
        """Special function: Build DISPOSE_TRASH state machine."""
        super().__init__("DISPOSE_TRASH_SKILL_SM", (client or ManipulationClients()).dispose_trash, validate_dispose)
