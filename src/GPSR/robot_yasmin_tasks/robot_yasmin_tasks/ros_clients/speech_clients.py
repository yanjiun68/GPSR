"""Section: Local TTS service client wrapper."""

import re
import time
from typing import Any

from robot_yasmin_tasks.gpsr.models import ActionStepData, make_skill_result
from robot_yasmin_tasks.gpsr.outcomes import FAILED, SUCCESS
from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get

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

    def say(self, blackboard: Any, action: ActionStepData) -> Any:
        """Special function: Send text to local TTS service."""
        text = action.parameters.get("text") or action.parameters.get("text_template", "")
        if not text:
            return make_skill_result(action.action_type, FAILED, "Speech text missing.", recoverable=True)
        # Section: Resolve only explicit dotted placeholders from one named result variable.
        substitution = str(action.parameters.get("substitution", "")).strip()
        if substitution:
            # Section: Deferred report queues carry explicit immutable cross-queue result data.
            explicit_data = action.parameters.get("substitution_data")
            variables = dict(bb_get(blackboard, bb.RESULT_VARIABLES, {}))
            source = (
                explicit_data
                if isinstance(explicit_data, dict)
                else variables.get(substitution, bb_get(blackboard, substitution, {}))
            )
            text, missing = self.resolve_template(str(text), source)
            if missing:
                return make_skill_result(
                    action.action_type,
                    FAILED,
                    f"Speech substitution fields are missing: {missing}.",
                    {"missing_fields": missing},
                    recoverable=True,
                )
        if self.simulate:  # Only explicitly requested simulation may bypass the voice driver.
            return make_skill_result(action.action_type, SUCCESS, "Simulated speech completed.", {"text": text})
        if self.client is None:  # A missing real driver must not report that a greeting was spoken.
            return make_skill_result(action.action_type, FAILED, f"TTS client unavailable: {self.service_name}", recoverable=True)  # Preserve failure feedback for queue recovery.
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

    @staticmethod
    def resolve_template(template: str, source: Any) -> tuple[str, list[str]]:
        """Special function: Resolve safe result.foo placeholders without evaluating expressions."""
        missing: list[str] = []

        def replacement(match: Any) -> str:
            """Special function: Resolve one dotted path against dictionary-only data."""
            path = str(match.group(1))
            parts = path.split(".")
            if parts and parts[0] == "result":
                parts = parts[1:]
            value = source
            for part in parts:
                if not isinstance(value, dict) or part not in value:
                    missing.append(path)
                    return match.group(0)
                value = value[part]
            if isinstance(value, (dict, list)):
                return str(value)
            return str(value)

        rendered = re.sub(r"\{([A-Za-z_][A-Za-z0-9_.]*)\}", replacement, str(template))
        return rendered, missing
