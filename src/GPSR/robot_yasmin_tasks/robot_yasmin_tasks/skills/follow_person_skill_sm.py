"""Section: FOLLOW_PERSON executable primitive skill."""

# Section: State-machine composition and skill-specific execution behavior.
from typing import Any

from robot_yasmin_tasks.ros_clients.manipulation_clients import ManipulationClients
from robot_yasmin_tasks.skills.skill_base import SimpleSkillSM, require_any_field


class FollowPersonSkillSM(SimpleSkillSM):
    """Special function: Follow a target person."""
    def __init__(self, client: Any | None = None) -> None:
        """Special function: Build FOLLOW_PERSON state machine."""
        super().__init__("FOLLOW_PERSON_SKILL_SM", (client or ManipulationClients()).follow_person, require_any_field("target_person"))
