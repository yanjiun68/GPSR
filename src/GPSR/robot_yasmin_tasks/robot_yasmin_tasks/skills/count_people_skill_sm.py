"""Section: COUNT_PEOPLE skill."""
from robot_yasmin_tasks.ros_clients.perception_clients import PerceptionClients
from robot_yasmin_tasks.skills.skill_base import SimpleSkillSM


class CountPeopleSkillSM(SimpleSkillSM):
    """Special function: Count people and store count_result."""
    def __init__(self, client: PerceptionClients | None = None) -> None:
        """Special function: Build COUNT_PEOPLE state machine."""
        super().__init__("COUNT_PEOPLE_SKILL_SM", (client or PerceptionClients()).count_people, None)
