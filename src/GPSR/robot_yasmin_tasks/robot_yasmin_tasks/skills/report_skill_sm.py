"""Section: REPORT skill."""

# Section: State-machine composition and skill-specific execution behavior.
from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import ActionStepData
from robot_yasmin_tasks.ros_clients.speech_clients import SpeechClients
from robot_yasmin_tasks.skills.skill_base import SimpleSkillSM


class ReportClient:
    """Special function: Build report text and call speech client."""
    def __init__(self, speech: SpeechClients | None = None) -> None:
        """Special function: Store speech client."""
        self.speech = speech or SpeechClients()
    def report(self, blackboard, action: ActionStepData):
        """Special function: Summarize blackboard skill results."""
        results = list(bb_get(blackboard, bb.SKILL_RESULTS, []))
        text = action.parameters.get("text") or "; ".join(f"{r.get('action_type')} {r.get('outcome')}" for r in results) or "No results to report."
        blackboard[bb.REPORT_TEXT] = text
        action.parameters["text"] = text
        return self.speech.say(blackboard, action)


class ReportSkillSM(SimpleSkillSM):
    """Special function: Report task results through TTS."""
    def __init__(self, client: ReportClient | None = None) -> None:
        """Special function: Build REPORT state machine."""
        super().__init__("REPORT_SKILL_SM", (client or ReportClient()).report, None)
