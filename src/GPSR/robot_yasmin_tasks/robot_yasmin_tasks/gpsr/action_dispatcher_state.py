"""Section: Dispatch symbolic GPSR action types to skill state machines."""

from collections.abc import Callable
from typing import Any

from yasmin import Blackboard, State

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.action_types import *
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import make_skill_result, normalize_action_step
from robot_yasmin_tasks.gpsr.outcomes import INVALID_INPUT, SKILL_OUTCOMES
from robot_yasmin_tasks.skills.count_objects_skill_sm import CountObjectsSkillSM
from robot_yasmin_tasks.skills.count_people_skill_sm import CountPeopleSkillSM
from robot_yasmin_tasks.skills.dialogue_skill_sm import DialogueSkillSM
from robot_yasmin_tasks.skills.extract_person_info_skill_sm import ExtractPersonInfoSkillSM
from robot_yasmin_tasks.skills.find_object_skill_sm import FindObjectSkillSM
from robot_yasmin_tasks.skills.find_person_by_attribute_sm import FindPersonByAttributeSM
from robot_yasmin_tasks.skills.find_person_skill_sm import FindPersonSkillSM
from robot_yasmin_tasks.skills.follow_person_skill_sm import FollowPersonSkillSM
from robot_yasmin_tasks.skills.guide_person_skill_sm import GuidePersonSkillSM
from robot_yasmin_tasks.skills.handover_skill_sm import HandoverSkillSM
from robot_yasmin_tasks.skills.navigation_skill_sm import NavigationSkillSM
from robot_yasmin_tasks.skills.pick_object_skill_sm import PickObjectSkillSM
from robot_yasmin_tasks.skills.place_object_skill_sm import PlaceObjectSkillSM
from robot_yasmin_tasks.skills.speech_skill_sm import SpeechSkillSM


# Section: Required GPSR dispatcher mapping.
ACTION_TYPE_TO_SKILL: dict[str, Callable[[], Any]] = {
    NAVIGATE: NavigationSkillSM, FIND_OBJECT: FindObjectSkillSM, FIND_PERSON: FindPersonSkillSM,
    FIND_PERSON_BY_ATTRIBUTE: FindPersonByAttributeSM,
    COUNT_OBJECTS: CountObjectsSkillSM, COUNT_PEOPLE: CountPeopleSkillSM,
    EXTRACT_PERSON_INFO: ExtractPersonInfoSkillSM,
    PICK_OBJECT: PickObjectSkillSM, PLACE_OBJECT: PlaceObjectSkillSM,
    HANDOVER_OBJECT: HandoverSkillSM, FOLLOW_PERSON: FollowPersonSkillSM,
    GUIDE_PERSON: GuidePersonSkillSM, SAY: SpeechSkillSM,
    ASK_AND_LISTEN: DialogueSkillSM,
}


class ActionDispatcherState(State):
    """Special function: Execute the skill mapped to CURRENT_ACTION."""

    def __init__(self, factories: dict[str, Callable[[], Any]] | None = None) -> None:
        """Special function: Store default or test-injected factories."""
        super().__init__(SKILL_OUTCOMES)
        self.factories = factories or ACTION_TYPE_TO_SKILL

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Dispatch one symbolic action without low-level robot control."""
        action = normalize_action_step(bb_get(blackboard, bb.CURRENT_ACTION, {}))
        factory = self.factories.get(action.action_type)
        if factory is None:
            message = f"Unknown GPSR action type: {action.action_type}"
            blackboard[bb.LAST_SKILL_RESULT] = make_skill_result(action.action_type, INVALID_INPUT, message, fatal=True).to_dict()
            blackboard[bb.DISPATCH_OUTCOME] = INVALID_INPUT
            return INVALID_INPUT
        blackboard[bb.CURRENT_ACTION] = action
        outcome = factory()(blackboard)
        blackboard[bb.DISPATCH_OUTCOME] = outcome
        return outcome
