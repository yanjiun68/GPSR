"""Section: Trait-independent greeting and persistent voice-driver regression tests."""

from concurrent.futures import Future  # Model a completed ROS service response without a live robot.
from types import SimpleNamespace  # Provide only the node/service surface exercised by speech.
import pytest  # Cover absent and descriptive person inputs with the same contract.
from yasmin import Blackboard  # Exercise actual skill state-machine bookkeeping.
from robot_yasmin_tasks.gpsr import blackboard_keys as bb  # Inspect the result consumed by ActionQueue.
from robot_yasmin_tasks.gpsr.greetings import DEFAULT_GREETING  # Verify consistency across greeting entrypoints.
from robot_yasmin_tasks.gpsr.models import ActionStepData, normalize_action_step  # Exercise direct and legacy queue actions.
from robot_yasmin_tasks.gpsr.outcomes import FAILED, SUCCESS  # Compare actual completion outcomes.
from robot_yasmin_tasks.skills.greet_person_skill_sm import GreetPersonSkillSM  # Test the public greeting skill.


@pytest.mark.parametrize("person,description", [("", ""), ("", "person wearing a red shirt"), ("unknown_person_42", "")])  # Traits and identity are optional and never spoken implicitly.
@pytest.mark.parametrize("driver_success", [True, False])  # Completion must follow the driver's response.
def test_greeting_uses_voice_driver_without_traits(person, description, driver_success) -> None:  # Cover successful and failed speech through the real client wrapper.
    """Special function: Speak neutral text via SayText without identifying the recipient."""
    requests = []  # Capture driver requests for verification.
    def call_async(request):  # Implement the service boundary without simulating skill completion.
        """Special function: Return the configured voice driver's result."""
        requests.append(request)  # Retain the actual generated SayText request.
        future = Future()  # Supply the asynchronous response expected by SpeechClients.
        future.set_result(SimpleNamespace(success=driver_success, message="driver playback result"))  # Model playback completion or failure.
        return future  # Let the production speech client interpret the response.
    service = SimpleNamespace(wait_for_service=lambda timeout_sec: True, call_async=call_async)  # Expose the persistent TTS service.
    def create_client(service_type, service_name):  # Verify driver selection during skill construction.
        """Special function: Require the voice package's stable SayText endpoint."""
        assert service_type.__name__ == "SayText" and service_name == "/voice/tts/say"  # Reject a private or perception-based driver.
        return service  # Reuse one service handle for this greeting.
    node = SimpleNamespace(create_client=create_client)  # Inject the existing executor node surface.
    board = Blackboard()  # Create one execution context.
    board[bb.CURRENT_ACTION] = ActionStepData("SAY", target_person=person, parameters={"description": description})  # Provide no explicit speech or required traits.
    outcome = GreetPersonSkillSM(node=node)(board)  # Execute input validation and driver invocation.
    assert outcome == (SUCCESS if driver_success else FAILED)  # Do not advance ActionQueue on driver failure.
    assert len(requests) == 1 and requests[0].text == DEFAULT_GREETING  # Speak one neutral greeting with no inferred attributes.
    assert board[bb.LAST_SKILL_RESULT]["outcome"] == outcome  # Preserve structured completion feedback.
    assert "text" not in board[bb.CURRENT_ACTION].parameters  # Do not mutate the queued action to construct speech.


def test_greeting_without_real_voice_client_fails() -> None:  # Missing drivers must never look like completed greetings.
    """Special function: Default skill construction fails closed without a ROS speech client."""
    board = Blackboard()  # Create an anonymous greeting context.
    board[bb.CURRENT_ACTION] = {"action_type": "SAY"}  # Omit all person and speech fields.
    assert GreetPersonSkillSM()(board) == FAILED  # Default execution is real, not implicitly simulated.
    assert "TTS client unavailable" in board[bb.LAST_SKILL_RESULT]["message"]  # Report the driver problem to ActionQueue.


@pytest.mark.parametrize("text", [None, "", "   "])  # Old callers may supply an absent or blank greeting.
def test_legacy_greet_queue_uses_neutral_default(text) -> None:  # The active dispatcher normalizes GREET into SAY.
    """Special function: Preserve neutral greetings through the production queue alias."""
    action = normalize_action_step({"action_type": "GREET", "target_person": "person_in_red", "parameters": {"text": text}})  # Exercise the actual admission conversion.
    assert action.action_type == "SAY" and action.parameters["text"] == DEFAULT_GREETING  # Do not speak descriptions or require identifying traits.


def test_explicit_greeting_wording_is_preserved() -> None:  # Operators may supply a simple alternative greeting.
    """Special function: Keep an explicit greeting override through legacy normalization."""
    action = normalize_action_step({"action_type": "GREET", "parameters": {"text": "Hello! Welcome."}})  # Use caller-provided wording.
    assert action.parameters["text"] == "Hello! Welcome."  # Do not replace intentional greeting text.
