"""Section: Attribute-only FIND_PERSON YASMIN state machine."""

from __future__ import annotations

from typing import Any

from yasmin import Blackboard, State, StateMachine

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import ActionStepData, make_skill_result, normalize_action_step
from robot_yasmin_tasks.gpsr.outcomes import CANCELED, FAILED, INVALID_INPUT, SKILL_OUTCOMES
from robot_yasmin_tasks.ros_clients.find_person_attribute_clients import FindPersonAttributeClients
from robot_yasmin_tasks.ros_clients.nav2_client import Nav2Client
from robot_yasmin_tasks.skills.person_waypoint_search import PersonWaypointSearchSM


# Section: Positive selectors implemented by the lifecycle tracked-appearance server.
SUPPORTED_ATTRIBUTE_SELECTORS = {"pose", "gesture", "any_person", "shirt_color", "glasses", "hat", "face_mask"}


class ValidateAttributeState(State):
    """Special function: Admit only non-name selectors supported by pose perception."""

    def __init__(self) -> None:
        """Special function: Declare validation outcomes for the attribute route."""
        super().__init__(["valid", INVALID_INPUT, CANCELED])

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Reject names, mixed selectors, and unsupported attributes."""
        action = normalize_action_step(bb_get(blackboard, bb.CURRENT_ACTION, {}))
        blackboard[bb.CURRENT_ACTION] = action
        cancel_requested = bb_get(blackboard, bb.CANCEL_REQUESTED, None)
        if callable(cancel_requested) and cancel_requested():
            return self._reject(blackboard, action, CANCELED, "Attribute person search canceled before execution.")
        selector = action.parameters.get("selector", {})
        selector = dict(selector) if isinstance(selector, dict) else {}
        predicates = selector.get("all") if isinstance(selector.get("all"), list) else [selector]
        if action.target_person or any(str(item.get("type", "")).strip().casefold() == "name" for item in predicates if isinstance(item, dict)):
            return self._reject(blackboard, action, INVALID_INPUT, "FIND_PERSON_BY_ATTRIBUTE cannot accept a name selector.")
        if not predicates:
            return self._reject(blackboard, action, INVALID_INPUT, "At least one attribute selector is required.")
        for predicate in predicates:
            if not isinstance(predicate, dict):
                return self._reject(blackboard, action, INVALID_INPUT, "Every attribute selector must be an object.")
            selector_type = str(predicate.get("type", "")).strip().casefold()
            selector_value = str(predicate.get("value", "")).strip().casefold()
            if selector_type not in SUPPORTED_ATTRIBUTE_SELECTORS:
                return self._reject(blackboard, action, INVALID_INPUT, f"Unsupported person attribute selector: {selector_type or 'empty'}.")
            if selector_type != "any_person" and not selector_value:
                return self._reject(blackboard, action, INVALID_INPUT, "Attribute selector requires a non-empty value.")
            if selector_type in {"glasses", "hat", "face_mask"} and selector_value not in {"yes", "true", "wearing", "1"}:
                return self._reject(blackboard, action, INVALID_INPUT, f"Negative {selector_type} matching is not supported.")
        if action.timeout_sec < 0.0:
            return self._reject(blackboard, action, INVALID_INPUT, "Timeout must not be negative.")
        return "valid"

    def _reject(
        self,
        blackboard: Blackboard,
        action: ActionStepData,
        outcome: str,
        message: str,
    ) -> str:
        """Special function: Store one validation failure using the common skill contract."""
        result = make_skill_result(action.action_type, outcome, message, fatal=True)
        blackboard[bb.LAST_SKILL_RESULT] = result.to_dict()
        blackboard[bb.DISPATCH_OUTCOME] = outcome
        blackboard[bb.FAILURE_REASON] = message
        return outcome


class WaitForAttributeMatchState(State):
    """Special function: Execute the real pose-summary search adapter."""

    def __init__(self, client: FindPersonAttributeClients) -> None:
        """Special function: Store the persistent attribute perception client."""
        super().__init__(SKILL_OUTCOMES)
        self.client = client

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Wait for a stable match and publish its normalized result."""
        action = normalize_action_step(bb_get(blackboard, bb.CURRENT_ACTION, {}))
        try:
            result = self.client.find_person(blackboard, action)
        except Exception as exc:  # noqa: BLE001
            result = make_skill_result(action.action_type, FAILED, f"Attribute person client error: {exc}", recoverable=True)
        result_dict = result.to_dict() if hasattr(result, "to_dict") else dict(result)
        outcome = str(result_dict.get("outcome", FAILED))
        blackboard[bb.LAST_SKILL_RESULT] = result_dict
        blackboard[bb.DISPATCH_OUTCOME] = outcome
        if outcome != "success":
            blackboard[bb.FAILURE_REASON] = str(result_dict.get("message", "Attribute person search failed."))
        return outcome


class _FindPersonByAttributeAtLocationSM(StateMachine):
    """Special function: Search one already selected waypoint for an attribute match."""

    def __init__(self, client: FindPersonAttributeClients | None = None) -> None:
        """Special function: Build the attribute-only validation and perception graph."""
        super().__init__(outcomes=SKILL_OUTCOMES)
        self.set_name("FIND_PERSON_BY_ATTRIBUTE_AT_LOCATION_SM")
        active_client = client or FindPersonAttributeClients()
        self.add_state(
            "VALIDATE_ATTRIBUTE",
            ValidateAttributeState(),
            transitions={"valid": "WAIT_FOR_ATTRIBUTE_MATCH", INVALID_INPUT: INVALID_INPUT, CANCELED: CANCELED},
        )
        self.add_state(
            "WAIT_FOR_ATTRIBUTE_MATCH",
            WaitForAttributeMatchState(active_client),
            transitions={outcome: outcome for outcome in SKILL_OUTCOMES},
        )


class FindPersonByAttributeSM(PersonWaypointSearchSM):
    """Special function: Loop through YAML waypoints until an attribute match is found."""

    def __init__(
        self,
        client: FindPersonAttributeClients | None = None,
        navigation: Nav2Client | None = None,
        search_config_file: str | None = None,
    ) -> None:
        """Special function: Wrap local attribute perception in the shared person-search route."""
        super().__init__(
            "FIND_PERSON_BY_ATTRIBUTE_SM",
            _FindPersonByAttributeAtLocationSM(client),
            navigation,
            search_config_file,
        )
