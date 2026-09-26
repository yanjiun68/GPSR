"""Section: Hardware-free semantic and dialogue clarification acceptance tests."""

from __future__ import annotations

import json
import threading
from dataclasses import replace
from types import SimpleNamespace

import pytest
from std_msgs.msg import String

from gpsr_task_scheduler.clarification import PROMPTS, assess_command, merge_clarification
from gpsr_task_scheduler.config import default_config
from gpsr_task_scheduler.planner import plan_to_dict
from gpsr_task_scheduler.scheduler import schedule_command
from gpsr_task_scheduler.scheduler_node import GpsrTaskSchedulerNode
from robot_ai_interfaces.srv import ScheduleCommand


def _simulated_config():
    """Special function: Enable all registered primitives for semantic-only tests."""
    config = default_config()
    config.primitive_capabilities = {
        name: replace(capability, implementation="real", production_enabled=True)
        for name, capability in config.primitive_capabilities.items()
    }
    return config


# Section: Every required prompt is selected before a queue can be generated.
@pytest.mark.parametrize("command,status,prompt", [
    ("The Coke in the kitchen.", "MISSING_ACTION", PROMPTS["MISSING_ACTION"]),
    ("Bring it from the kitchen.", "MISSING_TARGET", PROMPTS["MISSING_TARGET"]),
    ("Bring me the Coke.", "MISSING_LOCATION", PROMPTS["MISSING_LOCATION"]),
    ("Can you help me?", "MULTIPLE_MISSING", PROMPTS["MULTIPLE_MISSING"]),
    ("Find her in the living room.", "MISSING_TARGET", PROMPTS["MISSING_TARGET"]),
    ("Go there.", "MISSING_LOCATION", PROMPTS["MISSING_LOCATION"]),
])
def test_incomplete_command_has_prompt_and_no_queue(command: str, status: str, prompt: str) -> None:
    """Special function: Reject missing semantic slots before ActionQueue construction."""
    config = _simulated_config()
    result = schedule_command("incomplete", command, config)
    assert not result.valid
    assert result.steps == []
    assert result.command_context.action_queue == []
    assert result.command_context.validation["status"] == status
    assert result.command_context.validation["clarification_prompt"] == prompt


# Section: A reply can fill one slot without repeating the original command.
@pytest.mark.parametrize("initial,reply,expected", [
    ("The Coke in the kitchen.", "Bring it to me.", "Bring coke from the kitchen to me"),
    ("Bring it from the kitchen.", "The Coke.", "Bring coke from the kitchen."),
    ("Bring me the Coke.", "The kitchen.", "Bring me the Coke from the kitchen"),
    ("Find her in the living room.", "John.", "Find john in the living room."),
    ("Go there.", "The kitchen.", "Go to the kitchen."),
    ("Bring me the Coke from there.", "The kitchen.", "Bring me the Coke from the kitchen."),
])
def test_partial_context_merge_compiles(initial: str, reply: str, expected: str) -> None:
    """Special function: Preserve already recognized action, target, and room."""
    config = _simulated_config()
    merged = merge_clarification(initial, reply, config)
    assert merged == expected
    result = schedule_command("merged", merged, config)
    assert result.valid, result.issues
    assert result.steps


def test_complete_restatement_replaces_partial_command() -> None:
    """Special function: Accept a fully stated task as the new command context."""
    config = _simulated_config()
    replacement = "Bring me the Coke from the kitchen."
    assert merge_clarification("Bring me the Coke.", replacement, config) == replacement
    result = schedule_command("replacement", replacement, config)
    assert result.valid
    assert [step.skill_name for step in result.steps] == [
        "FIND_OBJECT", "PICK_OBJECT", "NAVIGATE", "HANDOVER_OBJECT",
    ]


def test_irrelevant_location_is_not_requested() -> None:
    """Special function: Keep global FIND and speech free of a universal room rule."""
    config = _simulated_config()
    for command in ("Find the Coke.", "Find John.", "Greet.", "Answer question."):
        assert assess_command(command, config).complete


def test_unsupported_explicit_command_is_not_guessed() -> None:
    """Special function: Preserve existing unsupported-command parser diagnostics."""
    result = schedule_command("unsupported", "Meet John in the kitchen.", _simulated_config())
    assert not result.valid
    assert not any(issue.code == "CLARIFICATION_REQUIRED" for issue in result.issues)


def test_normalizer_cannot_invent_a_missing_action() -> None:
    """Special function: Reject an action added only by normalized wording."""
    result = schedule_command(
        "normalized", "The Coke in the kitchen.", _simulated_config(),
        normalized_instruction="Bring me the Coke from the kitchen.",
    )
    assert not result.valid
    assert result.steps == []
    assert result.command_context.validation["status"] == "MISSING_ACTION"


def test_wrong_target_kind_does_not_change_a_person_search() -> None:
    """Special function: An object reply cannot replace an unresolved human pronoun."""
    config = _simulated_config()
    original = "Find her in the living room."
    assert merge_clarification(original, "The Coke.", config) == original
    assert assess_command(original, config).status == "MISSING_TARGET"


class _DialogueHarness:
    """Special function: Exercise ROS callback routing without constructing a live node."""

    start_clarification = GpsrTaskSchedulerNode.start_clarification
    command_callback = GpsrTaskSchedulerNode.command_callback
    clarification_response_callback = GpsrTaskSchedulerNode.clarification_response_callback
    finish_clarification = GpsrTaskSchedulerNode.finish_clarification
    check_clarification_timeout = GpsrTaskSchedulerNode.check_clarification_timeout

    def __init__(self) -> None:
        """Special function: Initialize a dry-run command and one guarded session."""
        self.config = _simulated_config()
        self.values = {
            "dry_run": True,
            "max_clarification_attempts": 2,
            "clarification_timeout_sec": 40.0,
        }
        self.clarification_lock = threading.Lock()
        self.clarification_session = None
        self.command_intake_busy = False
        self.results = []
        self.states = []
        self.plans = []
        self.logger = SimpleNamespace(info=lambda _message: None, warn=lambda _message: None)

    def get_parameter(self, name: str):
        """Special function: Supply one deterministic ROS-style parameter."""
        return SimpleNamespace(value=self.values[name])

    def get_logger(self):
        """Special function: Supply a silent callback logger."""
        return self.logger

    def publish_cycle_state(self) -> None:
        """Special function: Record pending-dialogue observability."""
        self.states.append(self.clarification_session is not None)

    def publish_command_result(self, result: dict) -> None:
        """Special function: Record structured interim and terminal verdicts."""
        self.results.append(dict(result))

    def process_one_command(self, command_id: str, instruction: str, dry_run: bool, timeout_sec: float, **_kwargs) -> dict:
        """Special function: Run the real compiler while omitting hardware dispatch."""
        plan = plan_to_dict(schedule_command(command_id, instruction, self.config))
        self.plans.append(plan)
        return {
            "accepted": bool(plan["valid"]), "success": bool(plan["valid"]),
            "command_id": command_id, "message": "planned" if plan["valid"] else "incomplete",
            "plan": plan,
        }

    def reply(self, command_id: str, attempt: int, transcript: str) -> None:
        """Special function: Deliver one correlated JSON clarification answer."""
        self.clarification_response_callback(String(data=json.dumps({
            "command_id": command_id, "attempt": attempt, "transcript": transcript,
        })))


def test_topic_dialogue_reparses_only_after_correlated_reply() -> None:
    """Special function: Retain partial context and emit a complete plan after reply."""
    harness = _DialogueHarness()
    harness.command_callback(String(data="Bring me the Coke."))
    first = harness.results[-1]
    command_id = first["command_id"]
    assert first["clarification"]["prompt"] == PROMPTS["MISSING_LOCATION"]
    assert harness.plans[-1]["steps"] == []
    # Section: An unrelated or previous-attempt answer cannot advance the session.
    harness.reply("stale", 1, "The kitchen.")
    harness.reply(command_id, 0, "The kitchen.")
    assert len(harness.results) == 1
    harness.reply(command_id, 1, "The kitchen.")
    assert harness.results[-1]["accepted"] is True
    assert harness.plans[-1]["steps"]
    assert harness.clarification_session is None


def test_voice_envelope_preserves_command_id_and_raw_topic_compatibility() -> None:
    """Special function: Correlate voice verdicts without changing plain topic commands."""
    harness = _DialogueHarness()
    harness.command_callback(String(data=json.dumps({
        "type": "gpsr_voice_command", "command_id": "voice_test_1",
        "instruction": "Bring me the Coke.",
    })))
    assert harness.results[-1]["command_id"] == "voice_test_1"
    assert harness.clarification_session.command_id == "voice_test_1"
    harness.reply("voice_test_1", 1, "The kitchen.")
    assert harness.results[-1]["accepted"] is True
    # Section: A raw topic command still receives a scheduler-generated identity.
    harness.command_callback(String(data="Go to the kitchen."))
    assert harness.results[-1]["command_id"].startswith("topic_")
    assert harness.results[-1]["accepted"] is True


def test_service_returns_incomplete_plan_without_dispatch() -> None:
    """Special function: Apply the shared queue gate to a service intake request."""
    harness = _DialogueHarness()
    request = ScheduleCommand.Request()
    request.command_id = "service_clarify"
    request.instruction = "Bring me the Coke."
    request.dry_run = True
    response = GpsrTaskSchedulerNode.schedule_service_callback(
        harness, request, ScheduleCommand.Response(),
    )
    assert response.accepted is False
    plan = json.loads(response.plan_json)
    assert plan["steps"] == []
    assert plan["command_context"]["validation"]["status"] == "MISSING_LOCATION"


def test_timer_cannot_cancel_an_in_flight_reply() -> None:
    """Special function: Keep timer expiration from racing a reserved response callback."""
    harness = _DialogueHarness()
    harness.command_callback(String(data="Bring me the Coke."))
    session = harness.clarification_session
    harness.command_intake_busy = True
    harness.finish_clarification(session, "clarification_timeout", require_idle=True)
    assert harness.clarification_session is session
    assert len(harness.results) == 1


def test_empty_replies_exhaust_retry_budget_without_a_queue() -> None:
    """Special function: Count empty STT results and fail after two answers."""
    harness = _DialogueHarness()
    harness.command_callback(String(data="Can you help me?"))
    command_id = harness.results[-1]["command_id"]
    harness.reply(command_id, 1, "   ")
    assert harness.results[-1]["clarification"]["attempt"] == 2
    harness.reply(command_id, 2, "")
    assert harness.results[-1]["message"] == "clarification_retry_exhausted"
    assert harness.clarification_session is None
    assert all(not plan["steps"] for plan in harness.plans)


def test_silent_clarification_times_out_without_dispatch(monkeypatch) -> None:
    """Special function: Timer abandonment releases the pending command identity."""
    harness = _DialogueHarness()
    now = [100.0]
    monkeypatch.setattr("gpsr_task_scheduler.scheduler_node.time.monotonic", lambda: now[0])
    harness.command_callback(String(data="Bring me the Coke."))
    now[0] = 141.0
    harness.check_clarification_timeout()
    assert harness.results[-1]["message"] == "clarification_timeout"
    assert harness.clarification_session is None
    assert harness.plans[0]["steps"] == []
