"""Section: Camera-free branch tests for two-mode FIND_PERSON identity resolution."""

from __future__ import annotations

import time
from types import SimpleNamespace
from typing import Any

from yasmin import Blackboard

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.models import ActionStepData
from robot_yasmin_tasks.gpsr.outcomes import CANCELED, INVALID_INPUT, NOT_FOUND, SUCCESS, TIMEOUT
from robot_yasmin_tasks.ros_clients.find_person_clients import FindPersonClients
from robot_yasmin_tasks.skills.find_person_skill_sm import (
    FindPersonSkillSM,
    classify_binary_confirmation,
)


class FakeIdentityClient:
    """Special function: Provide deterministic identity, candidate, speech, and enrollment responses."""

    def __init__(
        self,
        profile: dict[str, Any] | None = None,
        registered: dict[str, Any] | None = None,
        candidates: list[dict[str, Any]] | None = None,
        enrollment: dict[str, Any] | None = None,
        candidate_delay_sec: float = 0.0,
    ) -> None:
        """Special function: Store scripted branch responses and call history."""
        # Section: Defaults select verbal mode with an exhausted candidate search.
        self.profile = profile or {"status": "missing", "reason": "PROFILE_NOT_FOUND"}
        self.registered = registered or {"status": "not_found", "reason": "PROFILE_NOT_OBSERVED"}
        self.candidates = list(candidates or [])
        self.enrollment = enrollment or {"status": "enrolled", "profile_id": "gpsr_alex"}
        self.candidate_delay_sec = candidate_delay_sec
        self.rejected_snapshots: list[set[str]] = []
        self.enrollment_calls = 0
        self.enrollment_candidates: list[dict[str, Any]] = []

    def lookup_profile(self, *_args: Any) -> dict[str, Any]:
        """Special function: Return the scripted profile-availability result."""
        return dict(self.profile)

    def find_registered(self, *_args: Any) -> dict[str, Any]:
        """Special function: Return the scripted registered-recognition result."""
        return dict(self.registered)

    def next_candidate(
        self,
        _blackboard: Blackboard,
        _action: ActionStepData,
        rejected_ids: set[str],
        _timeout_sec: float,
    ) -> dict[str, Any]:
        """Special function: Return the first scripted candidate not rejected by the task."""
        self.rejected_snapshots.append(set(rejected_ids))
        if self.candidate_delay_sec:
            time.sleep(self.candidate_delay_sec)
        while self.candidates:
            candidate = dict(self.candidates.pop(0))
            if str(candidate["candidate_id"]) not in rejected_ids:
                return {"status": "candidate", "candidate": candidate}
        return {"status": "exhausted", "reason": "NO_MORE_CANDIDATES"}

    def ask_confirmation(self, *_args: Any) -> dict[str, Any]:
        """Special function: Return unknown when a test omits candidate-local transcripts."""
        return {"status": "answer", "transcript": ""}

    def enroll_profile(
        self,
        _blackboard: Blackboard,
        _action: ActionStepData,
        _target_name: str,
        candidate: dict[str, Any],
        _timeout_sec: float,
    ) -> dict[str, Any]:
        """Special function: Count and return the scripted best-effort enrollment result."""
        self.enrollment_calls += 1
        self.enrollment_candidates.append(dict(candidate))
        return dict(self.enrollment)


def run_name_skill(
    client: FakeIdentityClient,
    parameters: dict[str, Any] | None = None,
    timeout_sec: float = 2.0,
    cancel: Any = None,
) -> tuple[str, Blackboard]:
    """Special function: Execute one structured name-selector goal with a fake identity adapter."""
    # Section: Every test enters through the public state machine rather than individual states.
    blackboard = Blackboard()
    blackboard[bb.CURRENT_ACTION] = ActionStepData(
        "FIND_PERSON",
        target_location="living_room",
        parameters={"selector": {"type": "name", "value": "Alex"}, **(parameters or {})},
        timeout_sec=timeout_sec,
    )
    if cancel is not None:
        blackboard[bb.CANCEL_REQUESTED] = cancel
    return FindPersonSkillSM(client=client)(blackboard), blackboard


def registered_person_ref() -> dict[str, Any]:
    """Special function: Build one normalized successful registered-profile result."""
    return {
        "person_id": "guest_alex",
        "target_name": "Alex",
        "profile_id": "guest_alex",
        "tracking_id": 7,
        "location": "living_room",
        "pose": None,
        "confidence": 0.91,
        "source": "registered_profile",
    }


def test_live_confirmation_asks_for_the_specific_name_and_returns_the_answer() -> None:
    """Special function: Verify verbal fallback uses the shared ask-and-listen interface."""
    class RecordingDialogue:
        """Special function: Capture the dialogue action without requiring audio hardware."""

        def __init__(self) -> None:
            """Special function: Initialize an empty recorded dialogue action."""
            self.action = None

        def ask_and_listen(self, _blackboard: Blackboard, action: ActionStepData) -> Any:
            """Special function: Record the question and return a confirmed spoken answer."""
            self.action = action
            return SimpleNamespace(
                outcome=SUCCESS,
                message="heard",
                result={"answer": "yes"},
            )

    # Section: Exercise the real client adapter while replacing only external TTS/STT transport.
    dialogue = RecordingDialogue()
    client = FindPersonClients(node=None, simulate=False, dialogue=dialogue)
    answer = client.ask_confirmation(
        Blackboard(),
        ActionStepData("FIND_PERSON"),
        "Alex",
        {"candidate_id": "track:4"},
        3.0,
    )

    assert dialogue.action.action_type == "ASK_AND_LISTEN"
    assert dialogue.action.parameters["question"] == "Are you Alex?"
    assert answer == {"status": "answer", "transcript": "yes"}


def test_live_enrollment_goal_carries_confirmed_session_and_track() -> None:
    """Special function: The real adapter copies the exact confirmed lock into its action goal."""
    # Section: Minimal action fakes exercise goal construction without a ROS graph.
    goal_handle = SimpleNamespace(
        accepted=True,
        get_result_async=lambda: object(),
        cancel_goal_async=lambda: None,
    )
    wrapped_result = SimpleNamespace(
        result=SimpleNamespace(
            success=True,
            profile=SimpleNamespace(profile_id="gpsr_alex"),
            person_uuid="person-uuid-a",
            message="captured",
        )
    )

    class RecordingEnrollmentClient:
        """Special function: Record the generated goal and provide inert future tokens."""

        def __init__(self) -> None:
            """Special function: Initialize one empty goal record."""
            self.goal = None

        def wait_for_server(self, timeout_sec: float) -> bool:
            """Special function: Report the fake action server as available."""
            return timeout_sec > 0.0

        def send_goal_async(self, goal: Any) -> object:
            """Special function: Retain the outgoing goal for exact-field assertions."""
            self.goal = goal
            return object()

    client = FindPersonClients(node=None, simulate=False)
    recording_client = RecordingEnrollmentClient()
    client._enrollment_client = recording_client
    responses = iter([goal_handle, wrapped_result])
    client._wait_future = lambda *_args: next(responses)

    result = client.enroll_profile(
        Blackboard(),
        ActionStepData("FIND_PERSON"),
        "Alex",
        {
            "tracking_session_id": "session-b",
            "track_id": 7,
            "person_uuid": "person-uuid-a",
        },
        1.0,
    )

    assert result["status"] == "enrolled"
    assert recording_client.goal.tracking_session_id == "session-b"
    assert recording_client.goal.track_id == 7
    assert recording_client.goal.person_uuid == "person-uuid-a"


def test_registered_profile_success() -> None:
    """Special function: A usable profile selects face matching and returns normalized identity."""
    client = FakeIdentityClient(
        profile={"status": "found", "profile_id": "guest_alex", "usable": True},
        registered={"status": "found", "person_ref": registered_person_ref()},
    )

    outcome, blackboard = run_name_skill(client)

    assert outcome == SUCCESS
    assert blackboard[bb.PERSON_REF]["profile_id"] == "guest_alex"
    assert blackboard[bb.IDENTIFICATION_MODE_USED] == "REGISTERED_PROFILE"
    assert blackboard[bb.PERSON_REF]["identification_source"] == "REGISTERED_PROFILE"
    assert blackboard[bb.PROFILE_REGISTERED] is True
    assert client.enrollment_calls == 0


def test_verbal_confirmation_success_and_enrollment() -> None:
    """Special function: Missing profile confirms the candidate and enrolls it for future tasks."""
    client = FakeIdentityClient(
        candidates=[{"candidate_id": "track:4", "tracking_id": 4, "transcript": "Yes."}]
    )

    outcome, blackboard = run_name_skill(client)

    assert outcome == SUCCESS
    assert blackboard[bb.PERSON_REF]["tracking_id"] == 4
    assert blackboard[bb.PERSON_REF]["profile_id"] == "gpsr_alex"
    assert blackboard[bb.PERSON_REF]["name"] == "Alex"
    assert blackboard[bb.PROFILE_REGISTERED] is True
    assert client.enrollment_calls == 1


def test_wrong_candidate_rejected_then_correct_candidate_confirmed() -> None:
    """Special function: A negative answer rejects one track before accepting another."""
    client = FakeIdentityClient(
        candidates=[
            {"candidate_id": "track:1", "tracking_id": 1, "transcript": "no"},
            {"candidate_id": "track:2", "tracking_id": 2, "transcript": "yes"},
        ]
    )

    outcome, blackboard = run_name_skill(client)

    assert outcome == SUCCESS
    assert blackboard[bb.PERSON_REF]["tracking_id"] == 2
    assert "track:1" in blackboard[bb.LAST_SKILL_RESULT]["result"]["candidate_registry"]["rejected"]
    assert "track:1" in client.rejected_snapshots[-1]


def test_session_track_keys_prevent_reset_collisions_and_lock_enrollment() -> None:
    """Special function: Reused track integers in different sessions remain independent people."""
    client = FakeIdentityClient(
        candidates=[
            {
                "candidate_id": "session-a:7",
                "tracking_session_id": "session-a",
                "track_id": 7,
                "tracking_id": 7,
                "transcript": "no",
            },
            {
                "candidate_id": "session-b:7",
                "tracking_session_id": "session-b",
                "track_id": 7,
                "tracking_id": 7,
                "transcript": "yes",
            },
        ]
    )

    outcome, blackboard = run_name_skill(client)

    assert outcome == SUCCESS
    assert blackboard[bb.PERSON_REF]["candidate_key"] == "session-b:7"
    assert blackboard[bb.PERSON_REF]["tracking_session_id"] == "session-b"
    assert client.enrollment_candidates[0]["candidate_id"] == "session-b:7"
    assert "session-a:7" in client.rejected_snapshots[-1]


def test_unclear_answer_retries_same_candidate_once() -> None:
    """Special function: Unknown speech receives exactly one retry before a yes decision."""
    client = FakeIdentityClient(
        candidates=[
            {
                "candidate_id": "track:3",
                "tracking_id": 3,
                "transcripts": ["perhaps", "that's me"],
            }
        ]
    )

    outcome, blackboard = run_name_skill(client)

    assert outcome == SUCCESS
    assert blackboard[bb.PERSON_REF]["tracking_id"] == 3


def test_negative_response_never_creates_false_positive() -> None:
    """Special function: Exhausting candidates after no returns NOT_FOUND without person_ref."""
    client = FakeIdentityClient(
        candidates=[{"candidate_id": "track:5", "tracking_id": 5, "transcript": "No, not me"}]
    )

    outcome, blackboard = run_name_skill(client)

    assert outcome == NOT_FOUND
    assert bb.PERSON_REF not in blackboard
    assert client.enrollment_calls == 0
    assert blackboard[bb.LAST_SKILL_RESULT]["result"]["reason"] == "TARGET_NOT_IDENTIFIED"


def test_registered_failure_falls_back_with_reason() -> None:
    """Special function: Registered search failure can enter verbal mode and records why."""
    client = FakeIdentityClient(
        profile={"status": "found", "profile_id": "guest_alex", "usable": True},
        registered={"status": "not_found", "reason": "PROFILE_NOT_OBSERVED"},
        candidates=[{"candidate_id": "track:6", "tracking_id": 6, "transcript": "yes"}],
    )

    outcome, blackboard = run_name_skill(client)

    assert outcome == SUCCESS
    result = blackboard[bb.LAST_SKILL_RESULT]["result"]
    assert result["identification_mode_used"] == "VERBAL_IDENTIFICATION"
    assert result["fallback_reason"] == "PROFILE_NOT_OBSERVED"


def test_legacy_disable_flag_cannot_bypass_required_verbal_confirmation() -> None:
    """Special function: A visual miss always starts conversation for a specifically named person."""
    client = FakeIdentityClient(
        profile={"status": "found", "profile_id": "guest_alex", "usable": True},
        registered={"status": "not_found", "reason": "PROFILE_NOT_OBSERVED"},
        candidates=[{"candidate_id": "track:9", "tracking_id": 9, "transcript": "yes"}],
    )

    outcome, blackboard = run_name_skill(client, {"allow_verbal_fallback": False})

    assert outcome == SUCCESS
    assert blackboard[bb.PERSON_REF]["tracking_id"] == 9
    result = blackboard[bb.LAST_SKILL_RESULT]["result"]
    assert result["identification_mode_used"] == "VERBAL_IDENTIFICATION"
    assert result["fallback_reason"] == "PROFILE_NOT_OBSERVED"


def test_enrollment_failure_keeps_confirmed_person_success() -> None:
    """Special function: A database failure is diagnostic and never erases verbal confirmation."""
    client = FakeIdentityClient(
        candidates=[{"candidate_id": "track:8", "tracking_id": 8, "transcript": "yes"}],
        enrollment={"status": "failed", "reason": "DATABASE_READ_ONLY"},
    )

    outcome, blackboard = run_name_skill(client)

    assert outcome == SUCCESS
    assert blackboard[bb.PERSON_REF]["tracking_id"] == 8
    assert blackboard[bb.PROFILE_REGISTERED] is False
    assert blackboard[bb.LAST_SKILL_RESULT]["result"]["enrollment"]["reason"] == "DATABASE_READ_ONLY"
    assert blackboard[bb.LAST_SKILL_RESULT]["result"]["warning"] == "PROFILE_ENROLLMENT_FAILED"


def test_non_name_selector_is_rejected_by_name_skill() -> None:
    """Special function: Clothing selection cannot enter name identity resolution."""
    identity = FakeIdentityClient()
    blackboard = Blackboard()
    blackboard[bb.CURRENT_ACTION] = ActionStepData(
        "FIND_PERSON",
        parameters={
            "selector": {"type": "clothing", "value": "red shirt"},
            "description": "person wearing a red shirt",
        },
    )

    outcome = FindPersonSkillSM(identity)(blackboard)

    assert outcome == INVALID_INPUT
    assert identity.enrollment_calls == 0


def test_mixed_name_and_gesture_selector_is_rejected() -> None:
    """Special function: Explicit gesture plus target name cannot enter either hidden route."""
    identity = FakeIdentityClient()
    blackboard = Blackboard()
    blackboard[bb.CURRENT_ACTION] = ActionStepData(
        "FIND_PERSON",
        target_person="waving person",
        parameters={"selector": {"type": "gesture", "value": "waving"}},
    )

    outcome = FindPersonSkillSM(identity)(blackboard)

    assert outcome == INVALID_INPUT
    assert identity.enrollment_calls == 0


def test_pose_selector_is_rejected_by_name_skill() -> None:
    """Special function: Pose selection must use FIND_PERSON_BY_ATTRIBUTE instead."""
    identity = FakeIdentityClient()
    blackboard = Blackboard()
    blackboard[bb.CURRENT_ACTION] = ActionStepData(
        "FIND_PERSON",
        parameters={"selector": {"type": "pose", "value": "standing"}},
    )

    outcome = FindPersonSkillSM(identity)(blackboard)

    assert outcome == INVALID_INPUT
    assert identity.enrollment_calls == 0


def test_cancellation_stops_before_side_effects() -> None:
    """Special function: Preexisting cancellation exits before lookup, speech, or enrollment."""
    client = FakeIdentityClient(
        candidates=[{"candidate_id": "track:9", "tracking_id": 9, "transcript": "yes"}]
    )

    outcome, blackboard = run_name_skill(client, cancel=lambda: True)

    assert outcome == CANCELED
    assert bb.PERSON_REF not in blackboard
    assert client.enrollment_calls == 0
    assert "_find_person_candidate_registry" not in blackboard


def test_timeout_during_candidate_search_has_no_stale_success() -> None:
    """Special function: Deadline expiry returns TIMEOUT and never commits partial identity."""
    client = FakeIdentityClient(candidate_delay_sec=0.08)

    outcome, blackboard = run_name_skill(client, timeout_sec=0.01)

    assert outcome == TIMEOUT
    assert bb.PERSON_REF not in blackboard
    assert client.enrollment_calls == 0
    assert "_find_person_deadline" not in blackboard


def test_binary_classifier_is_deterministic_and_negative_first() -> None:
    """Special function: Confirmation parsing handles punctuation and ambiguous language safely."""
    assert classify_binary_confirmation("YES!") == "yes"
    assert classify_binary_confirmation("No, not me.") == "no"
    assert classify_binary_confirmation("maybe yes maybe no") == "unknown"
    assert classify_binary_confirmation("yes no") == "unknown"
    assert classify_binary_confirmation("") == "unknown"
