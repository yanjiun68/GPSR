"""Section: NAVIGATE skill."""
from robot_yasmin_tasks.ros_clients.nav2_client import Nav2Client
from robot_yasmin_tasks.skills.skill_base import SimpleSkillSM, require_any_field


class NavigationSkillSM(SimpleSkillSM):
    """Special function: Validate target and execute navigation."""
    def __init__(self, client: Nav2Client | None = None) -> None:
        """Special function: Build NAVIGATE state machine."""
        super().__init__("NAVIGATION_SKILL_SM", (client or Nav2Client()).navigate_to_pose, require_any_field("target_location", "target_pose"))
