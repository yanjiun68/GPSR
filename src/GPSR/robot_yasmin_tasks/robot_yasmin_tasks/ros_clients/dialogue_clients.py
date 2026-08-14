"""Section: Local dialogue client using TTS prompt plus STT topics."""

import threading
import time
from typing import Any

from std_msgs.msg import Float32, String

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import ActionStepData, make_skill_result
from robot_yasmin_tasks.gpsr.outcomes import CANCELED, NEED_CLARIFICATION, SUCCESS, TIMEOUT
from robot_yasmin_tasks.ros_clients.speech_clients import SpeechClients


class DialogueClients:
    """Special function: Ask with TTS and listen through local STT topics."""

    def __init__(self, node: Any | None = None, simulate: bool = True, speech_client: SpeechClients | None = None) -> None:
        """Special function: Create optional STT publisher/subscriber."""
        self.node = node
        self.simulate = simulate
        self.speech_client = speech_client or SpeechClients(node=node, simulate=simulate)
        self.latest = ""
        self.event = threading.Event()
        self.pub = node.create_publisher(Float32, "/voice/stt/start", 10) if node is not None and not simulate else None
        self.sub = node.create_subscription(String, "/voice/stt/transcript", self._cb, 10) if node is not None and not simulate else None

    def _cb(self, msg: String) -> None:
        """Special function: Store latest STT transcript."""
        self.latest = msg.data.strip()
        self.event.set()

    def ask_and_listen(self, blackboard: Any, action: ActionStepData) -> Any:
        """Special function: Speak question, trigger STT, and store answer."""
        if action.parameters.get("simulate_timeout", False):
            return make_skill_result(action.action_type, TIMEOUT, "Dialogue timed out.", recoverable=True)
        if self.simulate or self.pub is None:
            answer = action.parameters.get("simulated_answer", "")
            if not answer:
                return make_skill_result(action.action_type, NEED_CLARIFICATION, "No clear answer.", recoverable=True)
            blackboard[bb.DIALOGUE_ANSWER] = answer
            return make_skill_result(action.action_type, SUCCESS, "Answer received.", {"answer": answer})
        question = action.parameters.get("question") or action.parameters.get("text", "")
        if question:
            action.parameters["text"] = question
            speech_result = self.speech_client.say(blackboard, action)
            if speech_result.outcome != SUCCESS:
                return speech_result
        self.latest = ""
        self.event.clear()
        trigger = Float32()
        trigger.data = float(action.parameters.get("record_sec", 4.0))
        self.pub.publish(trigger)
        deadline = time.monotonic() + float(action.parameters.get("listen_timeout_sec", trigger.data + 8.0))
        while time.monotonic() < deadline:
            # Section: Dialogue must obey queue cancellation while waiting for an STT callback.
            cancel_requested = bb_get(blackboard, bb.CANCEL_REQUESTED, None)
            if callable(cancel_requested) and cancel_requested():
                return make_skill_result(action.action_type, CANCELED, "Dialogue canceled.", fatal=True)
            if self.event.wait(timeout=0.05):
                blackboard[bb.DIALOGUE_ANSWER] = self.latest
                return make_skill_result(action.action_type, SUCCESS, "Local STT answer received.", {"answer": self.latest})
        return make_skill_result(action.action_type, NEED_CLARIFICATION, "No STT transcript received.", recoverable=True)
