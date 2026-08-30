"""Section: GUIDE_PERSON executable primitive skill."""

# Section: State-machine composition and skill-specific execution behavior.
from yasmin import Blackboard
from robot_yasmin_tasks.gpsr.models import ActionStepData
from robot_yasmin_tasks.ros_clients.manipulation_clients import ManipulationClients
from robot_yasmin_tasks.skills.skill_base import SimpleSkillSM


def validate_guide(_blackboard: Blackboard, action: ActionStepData) -> tuple[bool, str]:
    """Special function: Validate guide person and destination."""
    return (bool(action.target_person and (action.target_location or action.parameters.get("destination"))), "Guide requires person and destination.")


class GuidePersonSkillSM(SimpleSkillSM):
    """Special function: Guide person to destination."""
    def __init__(self, client: ManipulationClients | None = None) -> None:
        """Special function: Build GUIDE_PERSON state machine."""
        super().__init__("GUIDE_PERSON_SKILL_SM", (client or ManipulationClients()).guide_person, validate_guide)
