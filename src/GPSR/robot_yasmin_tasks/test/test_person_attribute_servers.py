"""Section: Pure matching and serialization tests for lifecycle person servers."""

import threading
import time
from types import SimpleNamespace

from rclpy.action import GoalResponse
from robot_ai_interfaces.msg import (
    GuestAppearance,
    PersonProfile,
    TrackedPersonAppearance,
    TrackedPersonAppearanceArray,
)
from yasmin import Blackboard

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.models import ActionStepData
from robot_yasmin_tasks.gpsr.outcomes import CANCELED, FAILED, NOT_FOUND
from robot_yasmin_tasks.ros_clients.person_attribute_action_clients import (
    FindPersonByAttributeActionClient,
)
from robot_yasmin_tasks.ros_servers.person_attribute_skill_servers import (
    ExtractGuestInfoSkillServer,
    FindPersonByAttributeSkillServer,
    WaitForTrackedMatchState,
    match_score,
    merge_person_profiles,
    normalize_selector,
)


def tracked_person() -> TrackedPersonAppearance:
    """Special function: Build one deterministic positive tracked observation."""
    person = TrackedPersonAppearance()
    person.track_id = 4
    person.pose = "standing"
    person.pose_confidence = 0.9
    person.gesture = "waving"
    person.gesture_confidence = 0.8
    person.appearance.shirt_color = "red"
    person.appearance.shirt_color_confidence = 0.7
    person.appearance.glasses = GuestAppearance.YES
    person.appearance.glasses_confidence = 0.85
    return person


def test_combined_positive_predicates_use_weakest_confidence() -> None:
    """Special function: Every conjunct must match and determines the stable score."""
    predicates, error = normalize_selector(
        '{"all":[{"type":"shirt_color","value":"red"},{"type":"glasses","value":"yes"}]}'
    )
    assert not error
    assert match_score(tracked_person(), predicates) == 0.7


def test_generic_waving_predicate_accepts_either_arm() -> None:
    """Special function: The lifecycle matcher treats an unsided wave as either-arm evidence."""
    # Section: Exercise each side-specific perception label against the generic waving selector.
    for gesture in ("left_arm_waving", "right_arm_waving", "both_arms_waving"):
        person = tracked_person()
        person.gesture = gesture
        assert match_score(person, [{"type": "gesture", "value": "waving"}]) == 0.8


def test_right_raised_arm_predicate_rejects_left_only_evidence() -> None:
    """Special function: A right-arm request remains side-specific in the lifecycle matcher."""
    person = tracked_person()
    person.gesture = "raising_left_arm"
    assert match_score(person, [{"type": "gesture", "value": "right_hand_raised"}]) < 0.0


def test_negative_wearable_selector_is_rejected() -> None:
    """Special function: Missing detection cannot be treated as negative evidence."""
    predicates, error = normalize_selector('{"type":"glasses","value":"no"}')
    assert predicates == []
    assert "Negative glasses" in error


def test_unconfigured_positive_wearable_model_is_rejected() -> None:
    """Special function: Keep hat matching disabled until its model is validated."""
    predicates, error = normalize_selector(
        '{"type":"hat","value":"yes"}',
        {"glasses"},
    )
    assert predicates == []
    assert "positive-class model" in error


def test_profile_merge_preserves_known_fields_and_accepts_registered_data() -> None:
    """Special function: Merge profile lookup fields without UNKNOWN data loss."""
    existing = PersonProfile()
    existing.name = "Alex"
    existing.hometown = "Kuala Lumpur"
    incoming = PersonProfile()
    incoming.name = "UNKNOWN"
    incoming.favorite_drink = "tea"
    merged = merge_person_profiles(existing, incoming)
    assert merged.name == "Alex"
    assert merged.hometown == "Kuala Lumpur"
    assert merged.favorite_drink == "tea"


class _EmptyFrameServer:
    """Special function: Supply one fresh empty observation frame to the wait state."""

    def __init__(self) -> None:
        """Special function: Initialize the minimum server contract used by the state."""
        self.default_timeout_sec = 0.1
        self.confirmation_min_frames = 1
        self.skill_active = True
        self.observation_condition = threading.Condition()
        self.observation_sequence = 0
        self.returned_frame = False

    def frames_after(self, _cursor: int, deadline: float):
        """Special function: Return one empty frame, then wait until the local deadline."""
        if not self.returned_frame:
            self.returned_frame = True
            return [(1, TrackedPersonAppearanceArray())]
        time.sleep(min(0.01, max(0.0, deadline - time.monotonic())))
        return []


class _SilentFrameServer(_EmptyFrameServer):
    """Special function: Model an active server receiving no perception frames."""

    def frames_after(self, _cursor: int, deadline: float):
        """Special function: Wait boundedly and return no tracked-person observations."""
        time.sleep(min(0.01, max(0.0, deadline - time.monotonic())))
        return []


class _GoalHandle:
    """Special function: Provide an intentionally unhashable action goal test double."""

    __hash__ = None

    def __init__(self) -> None:
        """Special function: Initialize a bounded attribute request and feedback cache."""
        self.is_cancel_requested = False
        self.request = SimpleNamespace(timeout_sec=0.01, confirmation_min_frames=1)
        self.feedback = []

    def publish_feedback(self, feedback) -> None:
        """Special function: Record state feedback without ROS transport."""
        self.feedback.append(feedback)


def test_fresh_empty_tracked_frame_is_local_not_found() -> None:
    """Special function: An empty room advances the route instead of reporting timeout."""
    blackboard = Blackboard()
    blackboard["goal_handle"] = _GoalHandle()
    blackboard["predicates"] = [("pose", "standing")]

    outcome = WaitForTrackedMatchState(_EmptyFrameServer()).execute(blackboard)

    assert outcome == NOT_FOUND
    assert "no people were visible" in blackboard["message"]


def test_silent_tracked_stream_is_infrastructure_failure() -> None:
    """Special function: Missing frames cannot be reported as a person-not-found result."""
    blackboard = Blackboard()
    blackboard["goal_handle"] = _GoalHandle()
    blackboard["predicates"] = [("pose", "standing")]

    outcome = WaitForTrackedMatchState(_SilentFrameServer()).execute(blackboard)

    assert outcome == FAILED
    assert "No fresh tracked-person observations" in blackboard["message"]


def _assert_single_atomic_admission(server_type) -> None:
    """Special function: Prove concurrent callbacks reserve exactly one server goal."""
    server = object.__new__(server_type)
    server.skill_active = True
    server.goal_lock = threading.Lock()
    server.goal_admitted = False
    server.active_goal_handle = None
    barrier = threading.Barrier(12)
    responses = []
    response_lock = threading.Lock()

    def admit() -> None:
        """Special function: Race one admission callback against all peer threads."""
        barrier.wait()
        response = server.goal_callback(SimpleNamespace())
        with response_lock:
            responses.append(response)

    # Section: All callbacks begin together so a check-then-set race would admit duplicates.
    workers = [threading.Thread(target=admit) for _ in range(12)]
    for worker in workers:
        worker.start()
    for worker in workers:
        worker.join(timeout=1.0)
    assert responses.count(GoalResponse.ACCEPT) == 1
    assert responses.count(GoalResponse.REJECT) == 11


def test_find_attribute_server_admission_is_atomic() -> None:
    """Special function: Attribute search admits exactly one concurrent mission."""
    _assert_single_atomic_admission(FindPersonByAttributeSkillServer)


def test_extract_server_admission_is_atomic() -> None:
    """Special function: Guest extraction admits exactly one concurrent mission."""
    _assert_single_atomic_admission(ExtractGuestInfoSkillServer)


class _ImmediateFuture:
    """Special function: Provide one already-completed action future."""

    def __init__(self, value) -> None:
        """Special function: Store the immediate result value."""
        self.value = value

    def add_done_callback(self, callback) -> None:
        """Special function: Notify the waiter immediately."""
        callback(self)

    def result(self):
        """Special function: Return the stored result value."""
        return self.value


class _PendingResultFuture:
    """Special function: Complete only when the fake goal receives cancellation."""

    def __init__(self) -> None:
        """Special function: Initialize callback and result storage."""
        self.callbacks = []
        self.value = None

    def add_done_callback(self, callback) -> None:
        """Special function: Retain callbacks until cancellation completes the result."""
        if self.value is not None:
            callback(self)
            return
        self.callbacks.append(callback)

    def result(self):
        """Special function: Return the cancellation terminal wrapper."""
        return self.value

    def finish(self) -> None:
        """Special function: Publish a terminal action result to every waiter."""
        self.value = SimpleNamespace(result=SimpleNamespace(), status=5)
        for callback in list(self.callbacks):
            callback(self)


class _CancelableGoal:
    """Special function: Model a server goal that terminates when canceled."""

    def __init__(self) -> None:
        """Special function: Initialize accepted state and pending result transport."""
        self.accepted = True
        self.result_future = _PendingResultFuture()
        self.cancel_calls = 0

    def get_result_async(self):
        """Special function: Return the pending terminal-result future."""
        return self.result_future

    def cancel_goal_async(self):
        """Special function: Complete the result before acknowledging cancellation."""
        self.cancel_calls += 1
        self.result_future.finish()
        return _ImmediateFuture(SimpleNamespace())


class _CancelableActionTransport:
    """Special function: Accept one goal whose result remains pending until cancellation."""

    def __init__(self) -> None:
        """Special function: Store the single fake goal handle."""
        self.goal = _CancelableGoal()

    def wait_for_server(self, timeout_sec: float) -> bool:
        """Special function: Report availability for a positive discovery budget."""
        return timeout_sec > 0.0

    def send_goal_async(self, _goal):
        """Special function: Return the accepted goal immediately."""
        return _ImmediateFuture(self.goal)


def test_attribute_client_drains_canceled_goal_before_returning() -> None:
    """Special function: Queue cancellation waits for server terminal state before cleanup."""
    transport = _CancelableActionTransport()
    client = object.__new__(FindPersonByAttributeActionClient)
    client.action_name = "/gpsr/find_person_by_attribute"
    client.client = transport
    blackboard = Blackboard()
    started = time.monotonic()
    blackboard[bb.CANCEL_REQUESTED] = lambda: time.monotonic() - started >= 0.02
    action = ActionStepData(
        "FIND_PERSON_BY_ATTRIBUTE",
        target_location="living_room",
        parameters={"selector": {"type": "pose", "value": "standing"}},
        timeout_sec=1.0,
    )

    result = client.find_person(blackboard, action)

    assert result.outcome == CANCELED
    assert result.result["cancellation_confirmed"] is True
    assert transport.goal.cancel_calls == 1
