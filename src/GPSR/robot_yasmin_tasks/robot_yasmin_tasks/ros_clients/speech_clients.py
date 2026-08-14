"""Section: Local TTS service client wrapper."""

import time
from typing import Any

from robot_yasmin_tasks.gpsr.models import ActionStepData, make_skill_result
from robot_yasmin_tasks.gpsr.outcomes import FAILED, SUCCESS

try:
    from voice_interfaces.srv import SayText
except ImportError:  # pragma: no cover
    SayText = None


class SpeechClients:
    """Special function: Speak through /voice/tts/say or simulate in tests."""

    def __init__(self, node: Any | None = None, simulate: bool = True, service_name: str = "/voice/tts/say") -> None:
        """Special function: Create optional local TTS service client."""
        self.node = node
        self.simulate = simulate
        self.service_name = service_name
        self.client = node.create_client(SayText, service_name) if node is not None and not simulate and SayText is not None else None

    def say(self, _blackboard: Any, action: ActionStepData) -> Any:
        """Special function: Send text to local TTS service."""
        text = action.parameters.get("text") or action.parameters.get("text_template", "")
        if not text:
            return make_skill_result(action.action_type, FAILED, "Speech text missing.", recoverable=True)
        if self.simulate or self.client is None:
            return make_skill_result(action.action_type, SUCCESS, "Simulated speech completed.", {"text": text})
        if not self.client.wait_for_service(timeout_sec=3.0):
            return make_skill_result(action.action_type, FAILED, f"TTS service unavailable: {self.service_name}", recoverable=True)
        request = SayText.Request()
        request.text = text
        future = self.client.call_async(request)
        deadline = time.monotonic() + (action.timeout_sec or 8.0)
        while time.monotonic() < deadline:
            if future.done():
                response = future.result()
                if response is not None and response.success:
                    return make_skill_result(action.action_type, SUCCESS, response.message, {"text": text})
                return make_skill_result(action.action_type, FAILED, getattr(response, "message", "TTS failed."), recoverable=True)
            time.sleep(0.05)
        return make_skill_result(action.action_type, FAILED, "TTS response timed out.", recoverable=True)
