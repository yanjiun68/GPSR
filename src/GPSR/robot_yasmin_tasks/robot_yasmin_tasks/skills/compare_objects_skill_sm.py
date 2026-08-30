"""Section: COMPARE_OBJECTS skill."""

# Section: State-machine composition and skill-specific execution behavior.
from yasmin import Blackboard
from robot_yasmin_tasks.gpsr.models import ActionStepData
from robot_yasmin_tasks.ros_clients.perception_clients import PerceptionClients
from robot_yasmin_tasks.skills.skill_base import SimpleSkillSM


def validate_compare(_blackboard: Blackboard, action: ActionStepData) -> tuple[bool, str]:
    """Special function: Validate comparison property."""
    return (str(action.parameters.get("property", "")).lower() in {"biggest", "largest", "smallest", "heaviest", "lightest", "thinnest"}, "Invalid comparison property.")


class CompareObjectsSkillSM(SimpleSkillSM):
    """Special function: Compare objects and store selected object."""
    def __init__(self, client: PerceptionClients | None = None) -> None:
        """Special function: Build COMPARE_OBJECTS state machine."""
        super().__init__("COMPARE_OBJECTS_SKILL_SM", (client or PerceptionClients()).compare_objects, validate_compare)
