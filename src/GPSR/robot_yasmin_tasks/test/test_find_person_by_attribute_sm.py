"""Section: Attribute-only FIND_PERSON state-machine and perception tests."""

from __future__ import annotations

import threading
import time

from std_msgs.msg import String
from yasmin import Blackboard

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.models import ActionStepData, make_skill_result
from robot_yasmin_tasks.gpsr.outcomes import INVALID_INPUT, SUCCESS
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
