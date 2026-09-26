"""Section: Hardware-independent GPSR spoken-command window tests."""

from __future__ import annotations

import json
from dataclasses import dataclass

import pytest

import robot_yasmin_tasks.gpsr.voice_command_node as voice_command_module
from robot_yasmin_tasks.gpsr.voice_command_node import GpsrVoiceCommandNode


@dataclass
class _Parameter:
    """Special function: Carry one fake ROS parameter value."""

    value: object


class _LogicalClock:
    """Special function: Advance timeout tests without wall-clock sleeping."""

    def __init__(self) -> None:
        """Special function: Start deterministic time at zero seconds."""
        self.now = 0.0


class _Publisher:
    """Special function: Record logical STT triggers and accepted commands."""

    def __init__(self) -> None:
        """Special function: Initialize an empty publication history."""
        self.messages = []

    def publish(self, message) -> None:
        """Special function: Store one published ROS-style message."""
        self.messages.append(message)


class _Logger:
    """Special function: Record command-window runtime diagnostics."""

    def __init__(self) -> None:
        """Special function: Initialize an empty log history."""
        self.messages: list[str] = []

    def info(self, message: str) -> None:
        """Special function: Record one informational message."""
        self.messages.append(message)


class _Event:
    """Special function: Model transcript arrival against a logical clock."""

    def __init__(self, owner, clock: _LogicalClock) -> None:
        """Special function: Initialize a silent listening window."""
        self.owner = owner
        self.clock = clock
        self.trigger_at: float | None = None
        self.transcript = ""
        self.emitted = False

    def configure(self, trigger_at: float | None, transcript: str = "") -> None:
        """Special function: Configure one optional transcript arrival."""
        self.trigger_at = trigger_at
        self.transcript = transcript
        self.emitted = False

    def clear(self) -> None:
        """Special function: Match the threading Event API without removing its schedule."""
        return None

    def wait(self, timeout: float) -> bool:
        """Special function: Advance time and emit a transcript at its scheduled instant."""
        # Section: Deliver an arrival inside the current polling interval exactly once.
        if (
            self.trigger_at is not None
            and not self.emitted
            and self.trigger_at <= self.clock.now + timeout
        ):
            self.clock.now = self.trigger_at
            self.owner.latest = self.transcript
            self.emitted = True
            return True
        # Section: A silent interval consumes logical time without CPU-intensive waiting.
        self.clock.now += timeout
        return False


class _ListenHarness:
    """Special function: Exercise command listening without constructing a ROS node."""

    def __init__(self, clock: _LogicalClock) -> None:
        """Special function: Configure the production 12-second input window."""
        self.clock = clock
        self.latest = ""
        self.stt_pub = _Publisher()
        self.event = _Event(self, clock)
        self.values = {
            "record_sec": 6.0,
            "listen_timeout_sec": 12.0,
        }

    def get_parameter(self, name: str) -> _Parameter:
        """Special function: Return one fake voice-command parameter."""
        return _Parameter(self.values[name])

    def listen(self) -> str:
        """Special function: Invoke the production listening implementation."""
        return GpsrVoiceCommandNode._listen(self)


class _RunHarness:
    """Special function: Exercise timeout cleanup and command publication."""

    def __init__(self, transcripts: list[str]) -> None:
        """Special function: Configure one complete logical command cycle."""
        self.transcripts = list(transcripts)
        self.scheduler_state = "WAITING_FOR_COMMAND"
        self.busy = True
        self.awaiting_result = False
        self.command_pub = _Publisher()
        self.logger = _Logger()
        self.spoken: list[str] = []
        self.values = {
            "prompt_text": "Please give me a GPSR command after the beep.",
            "listen_timeout_sec": 12.0,
        }

    def get_parameter(self, name: str) -> _Parameter:
        """Special function: Return one fake runtime parameter."""
        return _Parameter(self.values[name])

    def get_logger(self) -> _Logger:
        """Special function: Return the recording logger."""
        return self.logger

    def _say(self, text: str) -> bool:
        """Special function: Record speech without invoking a TTS service."""
        self.spoken.append(text)
        return True

    def _listen(self) -> str:
        """Special function: Return the configured transcript or timeout result."""
        transcript = self.transcripts.pop(0)
        # Section: Let timeout-only tests end after proving the re-arm path once.
        if not transcript and not self.transcripts:
            self.scheduler_state = "FINISHED"
        return transcript

    def run(self) -> None:
        """Special function: Invoke the production command-cycle implementation."""
        GpsrVoiceCommandNode._run(self)

    def apply_result(self, result: dict) -> None:
        """Special function: Deliver one structured scheduler command verdict."""
        # Section: Build the same JSON String shape published by the scheduler.
        routed_result = dict(result)
        routed_result.setdefault("command_id", getattr(self, "active_command_id", ""))
        message = type("ResultMessage", (), {"data": json.dumps(routed_result)})()
        GpsrVoiceCommandNode._command_result_cb(self, message)


def test_command_received_at_five_seconds_ends_window_early(monkeypatch) -> None:
    """Special function: Continue immediately when a valid command arrives before timeout."""
    # Section: Replace monotonic wall time with deterministic logical time.
    clock = _LogicalClock()
    harness = _ListenHarness(clock)
    harness.event.configure(5.0, "go to the kitchen")
    monkeypatch.setattr(voice_command_module.time, "monotonic", lambda: clock.now)

    # Section: Verify the transcript ends the active window at five seconds.
    assert harness.listen() == "go to the kitchen"
    assert clock.now == pytest.approx(5.0)
    assert len(harness.stt_pub.messages) == 1


def test_silent_command_window_times_out_at_twelve_seconds(monkeypatch) -> None:
    """Special function: Enforce the exact configured command-input deadline."""
    # Section: Configure one silent logical listening cycle.
    clock = _LogicalClock()
    harness = _ListenHarness(clock)
    monkeypatch.setattr(voice_command_module.time, "monotonic", lambda: clock.now)

    # Section: Verify no transcript returns precisely at the twelve-second deadline.
    assert harness.listen() == ""
    assert clock.now == pytest.approx(12.0)
    assert len(harness.stt_pub.messages) == 1


def test_three_timeouts_reuse_one_listener_without_navigation(monkeypatch) -> None:
    """Special function: Re-arm repeated windows without adding subscribers or motion."""
    # Section: Reuse one harness as the persistent voice-command node would.
    clock = _LogicalClock()
    harness = _ListenHarness(clock)
    monkeypatch.setattr(voice_command_module.time, "monotonic", lambda: clock.now)

    # Section: Three silent windows consume 36 seconds and publish only STT triggers.
    assert [harness.listen() for _index in range(3)] == ["", "", ""]
    assert clock.now == pytest.approx(36.0)
    assert len(harness.stt_pub.messages) == 3
    assert not hasattr(harness, "navigation_client")


def test_command_after_first_timeout_is_received_in_second_window(monkeypatch) -> None:
    """Special function: Accept a command normally after one completed timeout."""
    # Section: Complete an initial silent twelve-second listening window.
    clock = _LogicalClock()
    harness = _ListenHarness(clock)
    monkeypatch.setattr(voice_command_module.time, "monotonic", lambda: clock.now)
    assert harness.listen() == ""

    # Section: Deliver the command six seconds into the re-armed second window.
    harness.event.configure(18.0, "find the bottle")
    assert harness.listen() == "find the bottle"
    assert clock.now == pytest.approx(18.0)
    assert len(harness.stt_pub.messages) == 2


def test_timeout_releases_busy_guard_for_timer_rearm() -> None:
    """Special function: Finish a timeout without publishing a command or navigation request."""
    # Section: Execute one complete silent command cycle.
    harness = _RunHarness([""])
    harness.run()

    # Section: Verify the existing timer may re-arm the same node on its next tick.
    assert harness.busy is False
    assert harness.command_pub.messages == []
    assert "No command received within 12.0 s; re-arming command input." in harness.logger.messages
    assert not hasattr(harness, "navigation_client")


def test_valid_command_waits_for_proven_queue_success_before_speaking() -> None:
    """Special function: Delay acknowledgment until the action queue proves success."""
    # Section: Execute one command cycle with a valid transcript.
    harness = _RunHarness(["go to the living room"])
    harness.run()

    # Section: Publishing a transcript alone must not speak command acceptance.
    assert harness.busy is True
    assert harness.awaiting_result is True
    assert [json.loads(message.data)["instruction"] for message in harness.command_pub.messages] == [
        "go to the living room"
    ]
    assert harness.spoken == ["Please give me a GPSR command after the beep."]

    # Section: A successful action-queue verdict permits the acknowledgment speech.
    harness.apply_result({
        "accepted": True,
        "success": True,
        "execution": {"success": True},
    })
    assert harness.spoken[-1] == "I received the command."
    assert harness.busy is False
    assert harness.awaiting_result is False


def test_invalid_command_speaks_rejection_instead_of_acceptance() -> None:
    """Special function: Report deterministic parser or planner rejection."""
    # Section: Submit the same short invalid transcript shown in the runtime failure.
    harness = _RunHarness(["You"])
    harness.run()
    harness.apply_result({
        "accepted": False,
        "success": False,
        "message": "No supported deterministic GPSR task was recognized.",
    })

    # Section: Reject once and never emit the success acknowledgment.
    assert harness.spoken[-1] == "The command is rejected."
    assert "I received the command." not in harness.spoken
    assert harness.busy is False


class _ClarificationHarness(_RunHarness):
    """Special function: Exercise correlated voice replies without hardware clients."""

    _run_clarification = GpsrVoiceCommandNode._run_clarification

    def __init__(self, transcript: str) -> None:
        """Special function: Configure one pending task and one STT service reply."""
        super().__init__(["Bring me the Coke."])
        self.reply_transcript = transcript
        self.clarification_response_pub = _Publisher()
        self.active_command_id = ""
        self.active_clarification_attempt = 0

    def _listen_clarification(self) -> str:
        """Special function: Return a request-scoped transcription result."""
        return self.reply_transcript


def test_voice_clarification_speaks_prompt_and_routes_reply(monkeypatch) -> None:
    """Special function: Use existing TTS and STT while retaining command ownership."""
    # Section: Run the short background worker synchronously for deterministic assertions.
    class _ImmediateThread:
        """Special function: Model a started ROS-safe worker in the test thread."""

        def __init__(self, target, args, daemon):
            """Special function: Retain the worker call and its arguments."""
            self.target = target
            self.args = args

        def start(self) -> None:
            """Special function: Execute the voice exchange immediately."""
            self.target(*self.args)

    monkeypatch.setattr(voice_command_module.threading, "Thread", _ImmediateThread)
    harness = _ClarificationHarness("The kitchen.")
    harness.run()
    command_id = json.loads(harness.command_pub.messages[0].data)["command_id"]
    harness.apply_result({
        "command_id": command_id, "accepted": False,
        "clarification": {
            "command_id": command_id, "attempt": 1,
            "prompt": "Where should I go or look?",
        },
    })
    assert harness.spoken[-1] == "Where should I go or look?"
    assert harness.awaiting_result is True
    assert json.loads(harness.clarification_response_pub.messages[-1].data) == {
        "attempt": 1, "command_id": command_id, "transcript": "The kitchen.",
    }
    # Section: A different command cannot close the pending voice dialogue.
    harness.apply_result({"command_id": "topic_stale", "accepted": False})
    assert harness.awaiting_result is True
    harness.apply_result({"command_id": command_id, "accepted": True, "success": True})
    assert harness.awaiting_result is False
    assert harness.busy is False


def test_idle_stt_transcript_is_ignored() -> None:
    """Special function: Keep old topic transcripts out of a future listening window."""
    harness = _ClarificationHarness("")
    harness.latest = ""
    harness.listening = False
    harness.event = type("Event", (), {"set": lambda self: setattr(self, "was_set", True)})()
    GpsrVoiceCommandNode._stt_cb(harness, type("Message", (), {"data": "old speech"})())
    assert harness.latest == ""
    assert not hasattr(harness.event, "was_set")


def test_production_listen_uses_request_scoped_stt_result() -> None:
    """Special function: A late topic transcript cannot complete a newer service request."""
    # Section: Build the smallest fake STT service that returns its own transcript.
    class _Service:
        """Special function: Model a completed request-scoped STT call."""

        def wait_for_service(self, timeout_sec: float) -> bool:
            """Special function: Report an available existing STT service."""
            return True

        def call_async(self, _request):
            """Special function: Return the transcript tied to this request."""
            return type("Future", (), {"result": lambda _self: type("Result", (), {"success": True, "transcript": "The kitchen."})()})()

    class _ScopedHarness(_ListenHarness):
        """Special function: Exercise the production service branch without ROS."""

        _transcribe_once = GpsrVoiceCommandNode._transcribe_once

        def __init__(self):
            """Special function: Keep a stale topic value separate from the service reply."""
            super().__init__(_LogicalClock())
            self.stt_service = _Service()
            self.latest = "old speech"

        def _wait(self, _future, _timeout: float) -> bool:
            """Special function: Complete the fake service response immediately."""
            return True

    harness = _ScopedHarness()
    assert harness.listen() == "The kitchen."
    assert harness.stt_pub.messages == []
    assert harness.latest == "old speech"


def test_scheduler_state_message_still_opens_voice_intake() -> None:
    """Special function: Preserve the existing scheduler readiness subscription."""
    harness = _RunHarness([])
    harness.scheduler_state = "INITIALIZING"
    GpsrVoiceCommandNode._scheduler_state_cb(
        harness, type("Message", (), {"data": json.dumps({"state": "WAITING_FOR_COMMAND"})})()
    )
    assert harness.scheduler_state == "WAITING_FOR_COMMAND"


def test_failed_action_queue_speaks_rejection() -> None:
    """Special function: Report a submitted queue that finishes unsuccessfully."""
    # Section: Deliver an executor failure after a syntactically valid transcript.
    harness = _RunHarness(["go to the living room"])
    harness.run()
    harness.apply_result({
        "accepted": True,
        "success": False,
        "execution": {"success": False, "outcome": "fatal_failure"},
    })

    # Section: Queue failure maps exclusively to rejection speech.
    assert harness.spoken[-1] == "The command is rejected."
    assert "I received the command." not in harness.spoken


def test_dry_run_result_remains_quiet_without_queue_proof() -> None:
    """Special function: Keep quiet when planning succeeds without queue execution."""
    # Section: A dry run has no execution object and therefore no queue verdict.
    harness = _RunHarness(["go to the living room"])
    harness.run()
    harness.apply_result({
        "accepted": True,
        "success": True,
        "plan": {"dry_run": True},
    })

    # Section: Only the initial prompt is spoken and intake is released cleanly.
    assert harness.spoken == ["Please give me a GPSR command after the beep."]
    assert harness.busy is False


def test_worker_rearms_three_timeouts_before_later_command() -> None:
    """Special function: Reuse one command worker across repeated timeout windows."""
    # Section: Model three silent windows followed by a valid fourth-window transcript.
    harness = _RunHarness(["", "", "", "find the bottle"])
    harness.run()

    # Section: Verify immediate logical re-arm and exactly one final command publication.
    timeout_log = "No command received within 12.0 s; re-arming command input."
    assert harness.logger.messages.count(timeout_log) == 3
    assert [json.loads(message.data)["instruction"] for message in harness.command_pub.messages] == [
        "find the bottle"
    ]
    assert harness.busy is True
    assert harness.awaiting_result is True
