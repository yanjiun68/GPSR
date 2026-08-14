"""Section: ASK skill."""
from robot_yasmin_tasks.ros_clients.dialogue_clients import DialogueClients
from robot_yasmin_tasks.skills.skill_base import SimpleSkillSM, require_any_field


class DialogueSkillSM(SimpleSkillSM):
    """Special function: Ask with TTS and listen with STT."""
    def __init__(self, client: DialogueClients | None = None) -> None:
        """Special function: Build ASK state machine."""
        super().__init__("DIALOGUE_SKILL_SM", (client or DialogueClients()).ask_and_listen, require_any_field("question", "text"))
