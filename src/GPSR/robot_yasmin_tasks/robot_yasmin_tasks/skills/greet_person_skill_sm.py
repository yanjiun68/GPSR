"""Section: GREET skill."""
from robot_yasmin_tasks.gpsr.models import ActionStepData
from robot_yasmin_tasks.ros_clients.speech_clients import SpeechClients
from robot_yasmin_tasks.skills.skill_base import SimpleSkillSM, require_any_field


class GreetPersonClient:
    """Special function: Build greeting text and call speech client."""
    def __init__(self, speech: SpeechClients | None = None) -> None:
        """Special function: Store speech client."""
        self.speech = speech or SpeechClients()
    def greet(self, blackboard, action: ActionStepData):
        """Special function: Speak deterministic greeting."""
        action.parameters["text"] = action.parameters.get("text", f"Hello {action.target_person or 'there'}.")
        return self.speech.say(blackboard, action)


class GreetPersonSkillSM(SimpleSkillSM):
    """Special function: Greet target person."""
    def __init__(self, client: GreetPersonClient | None = None) -> None:
        """Special function: Build GREET state machine."""
        super().__init__("GREET_PERSON_SKILL_SM", (client or GreetPersonClient()).greet, require_any_field("target_person", "description"))
