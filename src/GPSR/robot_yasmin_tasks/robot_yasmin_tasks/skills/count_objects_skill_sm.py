"""Section: COUNT_OBJECTS skill."""
from robot_yasmin_tasks.ros_clients.perception_clients import PerceptionClients
from robot_yasmin_tasks.skills.skill_base import SimpleSkillSM


class CountObjectsSkillSM(SimpleSkillSM):
    """Special function: Count requested objects or summarize every detected object type."""
    def __init__(self, client: PerceptionClients | None = None) -> None:
        """Special function: Build COUNT_OBJECTS state machine."""
        super().__init__("COUNT_OBJECTS_SKILL_SM", (client or PerceptionClients()).count_objects, None)
