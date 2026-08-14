"""Section: Manipulation and human-motion client wrappers."""

from typing import Any

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import ActionStepData, make_skill_result
from robot_yasmin_tasks.gpsr.outcomes import FAILED, PERSON_LOST, SUCCESS


class ManipulationClients:
    """Special function: Wrap pick/place/handover/follow/guide behavior."""

    def __init__(self, node: Any | None = None, simulate: bool = True) -> None:
        """Special function: Store optional ROS node and simulation mode."""
        self.node = node
        self.simulate = simulate

    def pick_object(self, blackboard: Any, action: ActionStepData) -> Any:
        """Special function: Mark target object as held."""
        obj = action.target_object or bb_get(blackboard, bb.TARGET_OBJECT, "")
        if action.parameters.get("simulate_failure", False):
            return make_skill_result(action.action_type, FAILED, "Pick failed.", recoverable=True)
        blackboard[bb.HELD_OBJECT] = obj
        return make_skill_result(action.action_type, SUCCESS, "Object picked.", {"held_object": obj})

    def place_object(self, blackboard: Any, action: ActionStepData) -> Any:
        """Special function: Place held object and clear held state."""
        held = bb_get(blackboard, bb.HELD_OBJECT, "")
        blackboard[bb.HELD_OBJECT] = ""
        return make_skill_result(action.action_type, SUCCESS, "Object placed.", {"object": held})

    def dispose_trash(self, blackboard: Any, action: ActionStepData) -> Any:
        """Special function: Dispose held or target object."""
        held = bb_get(blackboard, bb.HELD_OBJECT, "") or action.target_object
        blackboard[bb.HELD_OBJECT] = ""
        return make_skill_result(action.action_type, SUCCESS, "Trash disposed.", {"object": held})

    def handover(self, blackboard: Any, action: ActionStepData) -> Any:
        """Special function: Hand held object to recipient."""
        held = bb_get(blackboard, bb.HELD_OBJECT, "")
        blackboard[bb.HELD_OBJECT] = ""
        return make_skill_result(action.action_type, SUCCESS, "Object handed over.", {"object": held, "recipient": action.target_person})

    def follow_person(self, _blackboard: Any, action: ActionStepData) -> Any:
        """Special function: Follow target person."""
        if action.parameters.get("simulate_person_lost", False):
            return make_skill_result(action.action_type, PERSON_LOST, "Person lost.", recoverable=True)
        return make_skill_result(action.action_type, SUCCESS, "Follow completed.")

    def guide_person(self, _blackboard: Any, action: ActionStepData) -> Any:
        """Special function: Guide target person to destination."""
        return make_skill_result(action.action_type, SUCCESS, "Guide completed.")
