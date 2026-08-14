"""Section: FOLLOW skill."""
from robot_yasmin_tasks.ros_clients.manipulation_clients import ManipulationClients
from robot_yasmin_tasks.skills.skill_base import SimpleSkillSM, require_any_field


class FollowPersonSkillSM(SimpleSkillSM):
    """Special function: Follow a target person."""
    def __init__(self, client: ManipulationClients | None = None) -> None:
        """Special function: Build FOLLOW state machine."""
        super().__init__("FOLLOW_PERSON_SKILL_SM", (client or ManipulationClients()).follow_person, require_any_field("target_person"))
