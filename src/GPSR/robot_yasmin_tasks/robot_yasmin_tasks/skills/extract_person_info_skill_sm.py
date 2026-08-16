"""Section: EXTRACT_PERSON_INFO skill."""
from yasmin import Blackboard
from robot_yasmin_tasks.gpsr.models import ActionStepData
from robot_yasmin_tasks.ros_clients.perception_clients import PerceptionClients
from robot_yasmin_tasks.skills.skill_base import SimpleSkillSM


def validate_info(_blackboard: Blackboard, action: ActionStepData) -> tuple[bool, str]:
    """Special function: Validate extractable person field."""
    fields = list(action.parameters.get("fields", []))
    if not fields and action.parameters.get("field"):
        fields = [action.parameters.get("field")]
    supported = {"name", "age", "gender", "favorite_drink", "favourite_drink", "hometown", "shirt_color", "shirt_colour", "glasses", "hat", "face_mask", "pose", "gesture", "profile_id", "person_id", "track_id"}
    return (all(str(field).lower() in supported for field in fields), "Invalid person info field.")


class ExtractPersonInfoSkillSM(SimpleSkillSM):
    """Special function: Extract person information."""
    def __init__(self, client: PerceptionClients | None = None) -> None:
        """Special function: Build EXTRACT_PERSON_INFO state machine."""
        super().__init__("EXTRACT_PERSON_INFO_SKILL_SM", (client or PerceptionClients()).extract_person_info, validate_info)
