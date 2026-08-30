"""Section: Attribute-only FIND_PERSON state-machine and perception tests."""

from __future__ import annotations

import threading
import time

from std_msgs.msg import String
from yasmin import Blackboard

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.models import ActionStepData, make_skill_result
from robot_yasmin_tasks.gpsr.outcomes import INVALID_INPUT, NOT_FOUND, SUCCESS, TIMEOUT
from robot_yasmin_tasks.ros_clients.find_person_attribute_clients import FindPersonAttributeClients
from robot_yasmin_tasks.skills.find_person_by_attribute_sm import FindPersonByAttributeSM


class _Node:
    """Special function: Provide an inert subscription factory for client tests."""

    def create_subscription(self, *_args, **_kwargs):
        """Special function: Return a non-null subscription marker."""
        return object()


class _SuccessClient:
    """Special function: Return one deterministic attribute match to the state machine."""

    def __init__(self) -> None:
        """Special function: Initialize the call counter."""
        self.calls = 0

    def find_person(self, blackboard: Blackboard, action: ActionStepData):
        """Special function: Store a result proving the attribute route executed."""
        self.calls += 1
        blackboard[bb.PERSON_REF] = {"person_id": "7", "source": "POSE_ATTRIBUTE"}
        return make_skill_result(action.action_type, SUCCESS, "matched")


class _RecordingNavigation:
    """Special function: Record every YAML waypoint without requiring a Nav2 server."""

    def __init__(self) -> None:
        """Special function: Initialize ordered named-goal history."""
        self.targets: list[str] = []

    def navigate_to_pose(self, _blackboard: Blackboard, action: ActionStepData):
        """Special function: Confirm simulated arrival at the requested waypoint."""
        self.targets.append(action.target_location)
        return make_skill_result(action.action_type, SUCCESS, "arrived")


class _ScanningNavigation(_RecordingNavigation):
    """Special function: Record controlled relative rotations as well as waypoint goals."""

    def __init__(self) -> None:
        """Special function: Initialize navigation and scan call histories."""
        super().__init__()
        self.scan_angles: list[float] = []

    def spin_by_angle(
        self,
        _blackboard: Blackboard,
        angle_rad: float,
        _timeout_sec: float,
    ):
        """Special function: Confirm a simulated local search rotation."""
        self.scan_angles.append(angle_rad)
        return make_skill_result("NAVIGATE", SUCCESS, "rotated")


class _EmptyKitchenThenSuccessClient:
    """Special function: Model an empty first room and a match at the next waypoint."""

    def __init__(self) -> None:
        """Special function: Initialize ordered local-observation history."""
        self.locations: list[str] = []

    def find_person(self, _blackboard: Blackboard, action: ActionStepData):
        """Special function: Return a local miss in kitchen and success elsewhere."""
        self.locations.append(action.target_location)
        outcome = NOT_FOUND if action.target_location == "kitchen" else SUCCESS
        return make_skill_result(action.action_type, outcome, "local observation")


class _SlowNavigation(_RecordingNavigation):
    """Special function: Consume the complete mission budget during one navigation leg."""

    def navigate_to_pose(self, blackboard: Blackboard, action: ActionStepData):
        """Special function: Delay a nominal success so the outer deadline must win."""
        time.sleep(0.12)
        return super().navigate_to_pose(blackboard, action)


class _LateSuccessClient:
    """Special function: Return a nominal match only after the admitted deadline."""

    def find_person(self, _blackboard: Blackboard, action: ActionStepData):
        """Special function: Delay success so the outer deadline guard must reject it."""
        time.sleep(0.12)
        return make_skill_result(action.action_type, SUCCESS, "late match")


class _InfrastructureTimeoutClient:
    """Special function: Model a missing perception stream at a reachable waypoint."""

    def find_person(self, _blackboard: Blackboard, action: ActionStepData):
        """Special function: Return infrastructure timeout without a semantic empty frame."""
        return make_skill_result(action.action_type, TIMEOUT, "stream silent", recoverable=True)


class _ThirdWaypointSuccessClient:
    """Special function: Return NOT_FOUND twice, then a deterministic match."""

    def __init__(self) -> None:
        """Special function: Initialize observed search locations."""
        self.locations: list[str] = []

    def find_person(self, _blackboard: Blackboard, action: ActionStepData):
        """Special function: Prove that local perception advances through YAML order."""
        self.locations.append(action.target_location)
        outcome = SUCCESS if len(self.locations) == 3 else NOT_FOUND
        return make_skill_result(action.action_type, outcome, "search result")


def _attribute_action(selector_type: str, value: str, timeout_sec: float = 1.0) -> ActionStepData:
    """Special function: Build one normalized attribute action for tests."""
    return ActionStepData(
        "FIND_PERSON_BY_ATTRIBUTE",
        target_location="living_room",
        parameters={"selector": {"type": selector_type, "value": value}},
        timeout_sec=timeout_sec,
    )


def test_attribute_machine_executes_only_attribute_selector() -> None:
    """Special function: A pose selector reaches only the attribute client."""
    client = _SuccessClient()
    blackboard = Blackboard()
    blackboard[bb.CURRENT_ACTION] = _attribute_action("pose", "standing")

    assert FindPersonByAttributeSM(client)(blackboard) == SUCCESS
    assert client.calls == 1


def test_attribute_machine_rejects_name_selector() -> None:
    """Special function: A name cannot cross into the attribute state machine."""
    client = _SuccessClient()
    blackboard = Blackboard()
    blackboard[bb.CURRENT_ACTION] = ActionStepData(
        "FIND_PERSON_BY_ATTRIBUTE",
        target_person="john",
        parameters={"selector": {"type": "name", "value": "john"}},
    )

    assert FindPersonByAttributeSM(client)(blackboard) == INVALID_INPUT
    assert client.calls == 0


def test_global_attribute_search_loops_yaml_waypoints_until_success() -> None:
    """Special function: Location-free FIND_PERSON_BY_ATTRIBUTE owns its navigation loop."""
    client = _ThirdWaypointSuccessClient()
    navigation = _RecordingNavigation()
    blackboard = Blackboard()
    blackboard[bb.CURRENT_ACTION] = ActionStepData(
        "FIND_PERSON_BY_ATTRIBUTE",
        parameters={
            "selector": {"type": "pose", "value": "standing"},
            "search_scope": "GLOBAL",
        },
        timeout_sec=300.0,
    )

    assert FindPersonByAttributeSM(client, navigation)(blackboard) == SUCCESS
    assert navigation.targets == ["kitchen", "living_room", "dining_room"]
    assert client.locations == navigation.targets


def test_strict_attribute_search_uses_only_requested_waypoint() -> None:
    """Special function: Explicit locations never expand into the global YAML route."""
    client = _SuccessClient()
    navigation = _RecordingNavigation()
    blackboard = Blackboard()
    blackboard[bb.CURRENT_ACTION] = _attribute_action("pose", "standing")

    assert FindPersonByAttributeSM(client, navigation)(blackboard) == SUCCESS
    assert navigation.targets == ["living_room"]


def test_empty_room_scans_then_advances_to_next_waypoint() -> None:
    """Special function: A local empty scene must not terminate the global route."""
    client = _EmptyKitchenThenSuccessClient()
    navigation = _ScanningNavigation()
    blackboard = Blackboard()
    blackboard[bb.CURRENT_ACTION] = ActionStepData(
        "FIND_PERSON_BY_ATTRIBUTE",
        parameters={
            "selector": {"type": "pose", "value": "standing"},
            "search_scope": "GLOBAL",
        },
        timeout_sec=30.0,
    )

    assert FindPersonByAttributeSM(client, navigation)(blackboard) == SUCCESS
    assert navigation.targets == ["kitchen", "living_room"]
    assert client.locations == ["kitchen", "kitchen", "kitchen", "living_room"]
    assert len(navigation.scan_angles) == 2


def test_global_person_search_enforces_one_mission_deadline() -> None:
    """Special function: Navigation success after the total deadline returns TIMEOUT."""
    blackboard = Blackboard()
    blackboard[bb.CURRENT_ACTION] = ActionStepData(
        "FIND_PERSON_BY_ATTRIBUTE",
        parameters={
            "selector": {"type": "pose", "value": "standing"},
            "search_scope": "GLOBAL",
        },
        timeout_sec=0.01,
    )

    outcome = FindPersonByAttributeSM(_SuccessClient(), _SlowNavigation())(blackboard)

    assert outcome == TIMEOUT
    assert "mission-wide time budget" in blackboard[bb.LAST_SKILL_RESULT]["message"]


def test_local_success_after_absolute_deadline_is_rejected() -> None:
    """Special function: A late detector result cannot become mission success."""
    blackboard = Blackboard()
    blackboard[bb.CURRENT_ACTION] = ActionStepData(
        "FIND_PERSON_BY_ATTRIBUTE",
        target_location="living_room",
        parameters={"selector": {"type": "pose", "value": "standing"}},
        timeout_sec=0.1,
    )

    outcome = FindPersonByAttributeSM(_LateSuccessClient(), _RecordingNavigation())(blackboard)

    assert outcome == TIMEOUT
    assert blackboard[bb.LAST_SKILL_RESULT]["outcome"] == TIMEOUT


def test_perception_timeout_does_not_rotate_or_become_not_found() -> None:
    """Special function: Stream failure terminates instead of scanning empty viewpoints."""
    navigation = _ScanningNavigation()
    blackboard = Blackboard()
    blackboard[bb.CURRENT_ACTION] = ActionStepData(
        "FIND_PERSON_BY_ATTRIBUTE",
        target_location="living_room",
        parameters={"selector": {"type": "pose", "value": "standing"}},
        timeout_sec=5.0,
    )

    outcome = FindPersonByAttributeSM(_InfrastructureTimeoutClient(), navigation)(blackboard)

    assert outcome == TIMEOUT
    assert navigation.scan_angles == []


def test_pose_summary_client_requires_three_real_matching_frames() -> None:
    """Special function: Temporal confirmation returns a real pose-backed person reference."""
    client = FindPersonAttributeClients(node=_Node())
    blackboard = Blackboard()
    action = _attribute_action("gesture", "right_hand_raised")
    result_holder = {}

    # Section: Run the bounded wait concurrently with simulated ROS subscription callbacks.
    worker = threading.Thread(
        target=lambda: result_holder.setdefault("result", client.find_person(blackboard, action)),
        daemon=True,
    )
    worker.start()
    time.sleep(0.03)
    payload = (
        '{"people":[{"person_id":7,"status":"DETECTED",'
        '"pose_label":"standing","gesture":"raising_right_arm",'
        '"gesture_confidence":0.91,"raised_arm_state":"RIGHT_ARM_RAISED"}]}'
    )
    for _ in range(3):
        client._pose_summary_cb(String(data=payload))
        time.sleep(0.01)
    worker.join(timeout=1.0)

    result = result_holder["result"].to_dict()
    assert result["outcome"] == SUCCESS
    assert blackboard[bb.PERSON_REF]["selector_value"] == "right_hand_raised"
    assert blackboard[bb.PERSON_REF]["source"] == "POSE_ATTRIBUTE"


def test_pose_summary_matcher_distinguishes_left_and_right_arm() -> None:
    """Special function: Right-arm selection does not accept left-arm evidence."""
    client = FindPersonAttributeClients(node=None)
    left_person = {
        "status": "DETECTED",
        "gesture": "raising_left_arm",
        "raised_arm_state": "LEFT_ARM_RAISED",
    }
    assert not client._matches(left_person, {"type": "gesture", "value": "right_hand_raised"})
    assert client._matches(left_person, {"type": "gesture", "value": "left_hand_raised"})
