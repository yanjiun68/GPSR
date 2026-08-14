"""Section: Skill blackboard contract tests."""

from yasmin import Blackboard

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.outcomes import INVALID_INPUT, SUCCESS
from robot_yasmin_tasks.skills.count_people_skill_sm import CountPeopleSkillSM
from robot_yasmin_tasks.skills.handover_skill_sm import HandoverSkillSM
from robot_yasmin_tasks.skills.pick_object_skill_sm import PickObjectSkillSM
from robot_yasmin_tasks.skills.place_object_skill_sm import PlaceObjectSkillSM


def test_pick_requires_pose() -> None:
    """Special function: PICK rejects missing pose."""
    blackboard = Blackboard()
    blackboard[bb.CURRENT_ACTION] = {"action_type": "PICK", "target_object": "cup"}
    assert PickObjectSkillSM()(blackboard) == INVALID_INPUT


def test_place_requires_held_object() -> None:
    """Special function: PLACE rejects missing held object."""
    blackboard = Blackboard()
    blackboard[bb.CURRENT_ACTION] = {"action_type": "PLACE", "target_location": "table"}
    assert PlaceObjectSkillSM()(blackboard) == INVALID_INPUT


def test_give_requires_held_object() -> None:
    """Special function: GIVE rejects missing held object."""
    blackboard = Blackboard()
    blackboard[bb.CURRENT_ACTION] = {"action_type": "GIVE", "target_person": "operator"}
    assert HandoverSkillSM()(blackboard) == INVALID_INPUT


def test_count_people_accepts_current_view_without_location() -> None:
    """Special function: COUNT_PEOPLE accepts current-view counts without a target location."""
    blackboard = Blackboard()
    blackboard[bb.CURRENT_ACTION] = {"action_type": "COUNT_PEOPLE", "parameters": {"simulated_count": 2}}
    assert CountPeopleSkillSM()(blackboard) == SUCCESS
    assert blackboard[bb.COUNT_RESULT] == 2
    assert blackboard[bb.COUNT_PEOPLE_RESULT_JSON] == "{}"
    assert blackboard[bb.DETECTED_PEOPLE] == []
