"""Section: EXTRACT_PERSON_INFO skill."""
from yasmin import Blackboard
from robot_yasmin_tasks.gpsr.models import ActionStepData
from robot_yasmin_tasks.ros_clients.perception_clients import PerceptionClients
from robot_yasmin_tasks.skills.skill_base import SimpleSkillSM


def validate_info(_blackboard: Blackboard, action: ActionStepData) -> tuple[bool, str]:
    """Special function: Validate extractable person field."""
    return (str(action.parameters.get("field", "")).lower() in {"name", "pose", "gesture"}, "Invalid person info field.")


class ExtractPersonInfoSkillSM(SimpleSkillSM):
    """Special function: Extract person information."""
    def __init__(self, client: PerceptionClients | None = None) -> None:
        """Special function: Build EXTRACT_PERSON_INFO state machine."""
        super().__init__("EXTRACT_PERSON_INFO_SKILL_SM", (client or PerceptionClients()).extract_person_info, validate_info)
