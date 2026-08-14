"""Section: SAY skill."""
from robot_yasmin_tasks.ros_clients.speech_clients import SpeechClients
from robot_yasmin_tasks.skills.skill_base import SimpleSkillSM, require_any_field


class SpeechSkillSM(SimpleSkillSM):
    """Special function: Speak text through local TTS service."""
    def __init__(self, client: SpeechClients | None = None) -> None:
        """Special function: Build SAY state machine."""
        super().__init__("SPEECH_SKILL_SM", (client or SpeechClients()).say, require_any_field("text", "text_template"))
