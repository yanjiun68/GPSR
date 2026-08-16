"""Section: Name-only FIND_PERSON identity-resolution state machine."""

from __future__ import annotations

import re
import time
from typing import Any

from yasmin import Blackboard, State, StateMachine

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import ActionStepData, make_skill_result, normalize_action_step
from robot_yasmin_tasks.gpsr.outcomes import (
    CANCELED,
    FAILED,
    INVALID_INPUT,
    NOT_FOUND,
    SKILL_OUTCOMES,
    SUCCESS,
    TIMEOUT,
)
from robot_yasmin_tasks.ros_clients.find_person_clients import FindPersonClients


# Section: Private task-local keys prevent partially resolved identities leaking to consumers.
TARGET_NAME = "_find_person_target_name"
PROFILE = "_find_person_profile"
PERSON_REF_PENDING = "_find_person_ref_pending"
DEADLINE = "_find_person_deadline"
FALLBACK_REASON = "_find_person_fallback_reason"
CANDIDATE_REGISTRY = "_find_person_candidate_registry"
MODE = "_find_person_mode"
CONFIRMED_CANDIDATE = "_find_person_confirmed_candidate"
SIMULATED_CANDIDATE_INDEX = "_find_person_simulated_candidate_index"


def classify_binary_confirmation(transcript: str) -> str:
    """Special function: Classify deterministic yes/no language without an LLM."""
    # Section: Punctuation removal and whitespace collapse make ASR variants comparable.
    normalized = " ".join(re.sub(r"[^\w\s']", " ", str(transcript).casefold()).split())
    if not normalized:
        return "unknown"
    # Section: Explicit negative phrases take priority over embedded affirmative words.
    no_phrases = {
        "no",
        "nope",
        "nah",
        "negative",
        "not me",
        "i am not",
        "i'm not",
        "wrong person",
    }
    yes_phrases = {
        "yes",
        "yeah",
        "yep",
        "correct",
        "affirmative",
        "that is me",
        "that's me",
        "i am",
        "i'm",
    }
    words = set(normalized.split())
    negative = (
        normalized in no_phrases or
        "not me" in normalized or
        "wrong person" in normalized or
        bool(words & {"no", "nope", "nah", "negative"})
    )
    affirmative = (
        normalized in yes_phrases or
        "that is me" in normalized or
        "that's me" in normalized or
        bool(words & {"yes", "yeah", "yep", "correct", "affirmative"})
    )
    # Section: Contradictory transcripts remain unknown instead of risking false identity.
    if negative and affirmative:
        return "unknown"
    if negative:
        return "no"
    if affirmative:
        return "yes"
    return "unknown"


def _is_canceled(blackboard: Blackboard) -> bool:
    """Special function: Read the queue-owned cancellation callback."""
    callback = bb_get(blackboard, bb.CANCEL_REQUESTED, None)
    return bool(callback()) if callable(callback) else False


def _remaining(blackboard: Blackboard) -> float:
    """Special function: Return remaining skill time from one shared monotonic deadline."""
    return max(0.0, float(bb_get(blackboard, DEADLINE, time.monotonic())) - time.monotonic())


def _verbal_person_ref(
    action: ActionStepData,
    target_name: str,
    candidate: dict[str, Any],
) -> dict[str, Any]:
    """Special function: Build a normalized reference for a verbally confirmed candidate."""
    # Section: Candidate tracking remains optional because the face pipeline also supports largest-face mode.
    tracking_id = candidate.get("tracking_id")
    observation = dict(candidate.get("observation", {}))
    person_id = f"track_{tracking_id}" if tracking_id is not None else target_name
    return {
        "person_id": str(person_id),
        "name": target_name,
        "target_name": target_name,
        "profile_id": None,
        "tracking_id": tracking_id,
        "location": str(action.target_location or action.parameters.get("location", "")),
        "pose": observation.get("pose"),
        "confidence": 1.0,
        "source": "VERBAL_IDENTIFICATION",
        "identification_source": "VERBAL_IDENTIFICATION",
    }


def _clear_task_state(blackboard: Blackboard) -> None:
    """Special function: Remove temporary FIND_PERSON state after every terminal outcome."""
    # Section: Stable outputs remain available while task-local search evidence is released.
    for key in (
        TARGET_NAME,
        PROFILE,
        PERSON_REF_PENDING,
        DEADLINE,
        FALLBACK_REASON,
        CANDIDATE_REGISTRY,
        MODE,
        CONFIRMED_CANDIDATE,
        SIMULATED_CANDIDATE_INDEX,
    ):
        if key in blackboard:
            del blackboard[key]


def _store_result(
    blackboard: Blackboard,
    action: ActionStepData,
    outcome: str,
    message: str,
    result: dict[str, Any] | None = None,
    recoverable: bool = False,
    fatal: bool = False,
) -> str:
    """Special function: Store one complete GPSR skill result and return its outcome."""
    payload = make_skill_result(
        action.action_type,
        outcome,
        message,
        result or {},
        recoverable=recoverable,
        fatal=fatal,
    ).to_dict()
    blackboard[bb.LAST_SKILL_RESULT] = payload
    blackboard[bb.DISPATCH_OUTCOME] = outcome
    if outcome != SUCCESS:
        blackboard[bb.FAILURE_REASON] = message
    _clear_task_state(blackboard)
    return outcome


class ValidateAndRouteState(State):
    """Special function: Validate that FIND_PERSON carries only a name selector."""

    def __init__(self) -> None:
        """Special function: Declare the initial routing outcomes."""
        super().__init__(["name", INVALID_INPUT, CANCELED])

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Normalize FIND_PERSON goal fields and initialize task-local state."""
        # Section: Cancellation is checked before any perception or speech side effect.
        action = normalize_action_step(bb_get(blackboard, bb.CURRENT_ACTION, {}))
        blackboard[bb.CURRENT_ACTION] = action
        if _is_canceled(blackboard):
            _store_result(blackboard, action, CANCELED, "FIND_PERSON canceled before execution.", fatal=True)
            return CANCELED
        if action.timeout_sec < 0.0:
            _store_result(blackboard, action, INVALID_INPUT, "Timeout must not be negative.", fatal=True)
            return INVALID_INPUT
        # Section: A structured selector is authoritative; target_person remains a compatible name selector.
        raw_selector = action.parameters.get("selector", {})
        selector = dict(raw_selector) if isinstance(raw_selector, dict) else {}
        selector_type = str(
            selector.get("type") or action.parameters.get("selector_type", "")
        ).strip().casefold()
        selector_value = str(
            selector.get("value") or action.parameters.get("selector_value", "")
        ).strip()
        if selector_type == "name":
            target_name = str(selector_value or action.target_person).strip()
        elif selector_type:
            _store_result(
                blackboard,
                action,
                INVALID_INPUT,
                f"FIND_PERSON accepts only a name selector, not '{selector_type}'.",
                fatal=True,
            )
            return INVALID_INPUT
        else:
            target_name = str(action.target_person).strip()
        if selector_type == "name" and not target_name:
            _store_result(blackboard, action, INVALID_INPUT, "Name selector requires a non-empty value.", fatal=True)
            return INVALID_INPUT
        # Section: Empty or description-only calls cannot enter identity resolution.
        if not target_name:
            _store_result(
                blackboard,
                action,
                INVALID_INPUT,
                "FIND_PERSON requires a non-empty name selector.",
                fatal=True,
            )
            return INVALID_INPUT
        timeout_sec = float(action.timeout_sec or action.parameters.get("timeout_sec", 30.0) or 30.0)
        blackboard[DEADLINE] = time.monotonic() + max(0.05, timeout_sec)
        blackboard[CANDIDATE_REGISTRY] = {
            "detected": [],
            "rejected": [],
            "confirmed": None,
        }
        blackboard[FALLBACK_REASON] = ""
        blackboard[TARGET_NAME] = target_name
        return "name"


class CheckProfileState(State):
    """Special function: Decide registered-profile or verbal-identification mode."""

    def __init__(self, client: FindPersonClients) -> None:
        """Special function: Store the identity registry adapter."""
        super().__init__(["registered", "verbal", CANCELED, TIMEOUT])
        self.client = client

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Perform bounded profile lookup and record the selected mode."""
        action = normalize_action_step(bb_get(blackboard, bb.CURRENT_ACTION, {}))
        if _is_canceled(blackboard):
            _store_result(blackboard, action, CANCELED, "FIND_PERSON canceled during profile lookup.", fatal=True)
            return CANCELED
        remaining = _remaining(blackboard)
        if remaining <= 0.0:
            _store_result(blackboard, action, TIMEOUT, "FIND_PERSON timed out before profile lookup.", recoverable=True)
            return TIMEOUT
        response = self.client.lookup_profile(
            blackboard,
            action,
            str(blackboard[TARGET_NAME]),
            min(remaining, float(action.parameters.get("profile_lookup_timeout_sec", 2.0))),
        )
        status = str(response.get("status", "failed"))
        if status == "canceled":
            _store_result(blackboard, action, CANCELED, "FIND_PERSON canceled during profile lookup.", fatal=True)
            return CANCELED
        if status == "timeout" and _remaining(blackboard) <= 0.0:
            _store_result(blackboard, action, TIMEOUT, "FIND_PERSON profile lookup timed out.", recoverable=True)
            return TIMEOUT
        if status == "found" and bool(response.get("usable", True)):
            blackboard[PROFILE] = dict(response)
            blackboard[MODE] = "REGISTERED_PROFILE"
            return "registered"
        # Section: Missing, unusable, or unavailable registry data selects the safe verbal path.
        blackboard[FALLBACK_REASON] = str(response.get("reason", "PROFILE_NOT_AVAILABLE"))
        blackboard[MODE] = "VERBAL_IDENTIFICATION"
        return "verbal"


class RegisteredProfileState(State):
    """Special function: Resolve the target using temporal registered-face matching."""

    def __init__(self, client: FindPersonClients) -> None:
        """Special function: Store the face recognition adapter."""
        super().__init__([SUCCESS, "fallback", NOT_FOUND, FAILED, CANCELED, TIMEOUT])
        self.client = client

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Search for the exact profile and optionally fall back to verbal confirmation."""
        action = normalize_action_step(bb_get(blackboard, bb.CURRENT_ACTION, {}))
        remaining = _remaining(blackboard)
        if _is_canceled(blackboard):
            _store_result(blackboard, action, CANCELED, "FIND_PERSON registered search canceled.", fatal=True)
            return CANCELED
        if remaining <= 0.0:
            _store_result(blackboard, action, TIMEOUT, "FIND_PERSON registered search timed out.", recoverable=True)
            return TIMEOUT
        allow_fallback = bool(
            action.parameters.get(
                "allow_verbal_fallback",
                action.parameters.get("fallback_to_verbal", True),
            )
        )
        # Section: Reserve bounded time for fallback only when the caller explicitly permits it.
        search_timeout = remaining
        if allow_fallback:
            configured = float(action.parameters.get("profile_search_timeout_sec", min(10.0, remaining * 0.5)))
            search_timeout = min(remaining, max(0.05, configured))
        response = self.client.find_registered(
            blackboard,
            action,
            str(blackboard[TARGET_NAME]),
            dict(blackboard[PROFILE]),
            float(action.parameters.get("identity_threshold", 0.45)),
            search_timeout,
        )
        status = str(response.get("status", "failed"))
        if status == "found":
            blackboard[PERSON_REF_PENDING] = dict(response["person_ref"])
            return SUCCESS
        if status == "canceled":
            _store_result(blackboard, action, CANCELED, "FIND_PERSON registered search canceled.", fatal=True)
            return CANCELED
        # Section: Fallback reason is always surfaced in the final structured result.
        reason = str(response.get("reason", "REGISTERED_PROFILE_NOT_FOUND"))
        if allow_fallback and _remaining(blackboard) > 0.0:
            blackboard[FALLBACK_REASON] = reason
            blackboard[MODE] = "VERBAL_IDENTIFICATION"
            return "fallback"
        outcome = TIMEOUT if status == "timeout" else (FAILED if status == "failed" else NOT_FOUND)
        _store_result(
            blackboard,
            action,
            outcome,
            "PROFILE_NOT_FOUND_IN_SEARCH_SCOPE: " + reason,
            {
                "identification_mode_used": "REGISTERED_PROFILE",
                "reason": "PROFILE_NOT_FOUND_IN_SEARCH_SCOPE",
                "adapter_reason": reason,
            },
            recoverable=True,
        )
        return outcome


class VerbalIdentificationState(State):
    """Special function: Confirm named identity candidate-by-candidate using deterministic speech."""

    def __init__(self, client: FindPersonClients) -> None:
        """Special function: Store candidate search and dialogue adapters."""
        super().__init__([SUCCESS, NOT_FOUND, CANCELED, TIMEOUT])
        self.client = client

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Ask each candidate once, retry unknown once, and stop at the first yes."""
        action = normalize_action_step(bb_get(blackboard, bb.CURRENT_ACTION, {}))
        target_name = str(blackboard[TARGET_NAME])
        registry = dict(blackboard[CANDIDATE_REGISTRY])
        rejected = set(str(item) for item in registry.get("rejected", []))
        while _remaining(blackboard) > 0.0:
            if _is_canceled(blackboard):
                _store_result(blackboard, action, CANCELED, "FIND_PERSON verbal identification canceled.", fatal=True)
                return CANCELED
            # Section: Candidate discovery consumes only fresh upstream tracker observations.
            candidate_response = self.client.next_candidate(
                blackboard,
                action,
                rejected,
                _remaining(blackboard),
            )
            candidate_status = str(candidate_response.get("status", "exhausted"))
            if candidate_status == "canceled":
                _store_result(blackboard, action, CANCELED, "FIND_PERSON candidate search canceled.", fatal=True)
                return CANCELED
            if candidate_status != "candidate":
                outcome = TIMEOUT if _remaining(blackboard) <= 0.0 else NOT_FOUND
                _store_result(
                    blackboard,
                    action,
                    outcome,
                    "TARGET_NOT_IDENTIFIED: " + str(
                        candidate_response.get(
                            "reason", "No candidate confirmed the requested name."
                        )
                    ),
                    {
                        "identification_mode_used": "VERBAL_IDENTIFICATION",
                        "reason": "TARGET_NOT_IDENTIFIED",
                        "fallback_reason": str(bb_get(blackboard, FALLBACK_REASON, "")),
                        "candidate_registry": registry,
                    },
                    recoverable=True,
                )
                return outcome
            candidate = dict(candidate_response["candidate"])
            candidate_id = str(candidate.get("candidate_id", "untracked:current_face"))
            registry.setdefault("detected", []).append(candidate_id)
            blackboard[CANDIDATE_REGISTRY] = registry
            decision = "unknown"
            # Section: Unknown answers use a bounded per-candidate clarification retry count.
            confirmation_retries = max(
                0, int(action.parameters.get("max_confirmation_retries", 1))
            )
            for attempt in range(confirmation_retries + 1):
                answer = self._ask_once(
                    blackboard,
                    action,
                    target_name,
                    candidate,
                    attempt,
                )
                if answer.get("status") == "canceled":
                    _store_result(blackboard, action, CANCELED, "FIND_PERSON dialogue canceled.", fatal=True)
                    return CANCELED
                decision = classify_binary_confirmation(str(answer.get("transcript", "")))
                if decision != "unknown":
                    break
            if decision == "yes":
                registry["confirmed"] = candidate_id
                blackboard[CANDIDATE_REGISTRY] = registry
                blackboard[PERSON_REF_PENDING] = _verbal_person_ref(
                    action, target_name, candidate
                )
                blackboard[CONFIRMED_CANDIDATE] = candidate
                return SUCCESS
            # Section: No or repeated unknown rejects this candidate for the remainder of the task.
            rejected.add(candidate_id)
            registry.setdefault("rejected", []).append(candidate_id)
            blackboard[CANDIDATE_REGISTRY] = registry
        _store_result(blackboard, action, TIMEOUT, "FIND_PERSON verbal identification timed out.", recoverable=True)
        return TIMEOUT

    def _ask_once(
        self,
        blackboard: Blackboard,
        action: ActionStepData,
        target_name: str,
        candidate: dict[str, Any],
        attempt: int,
    ) -> dict[str, Any]:
        """Special function: Obtain one answer while supporting deterministic candidate test data."""
        # Section: Candidate transcripts model speech results without bypassing the confirmation parser.
        transcripts = candidate.get("transcripts")
        if isinstance(transcripts, list):
            transcript = str(transcripts[min(attempt, len(transcripts) - 1)]) if transcripts else ""
            return {"status": "answer", "transcript": transcript}
        if "transcript" in candidate:
            return {"status": "answer", "transcript": str(candidate.get("transcript", ""))}
        return self.client.ask_confirmation(
            blackboard,
            action,
            target_name,
            candidate,
            min(_remaining(blackboard), float(action.parameters.get("confirmation_timeout_sec", 12.0))),
        )


class EnrollAndStoreState(State):
    """Special function: Best-effort enroll verbal identities and publish the stable blackboard contract."""

    def __init__(self, client: FindPersonClients) -> None:
        """Special function: Store the enrollment adapter."""
        super().__init__([SUCCESS, CANCELED])
        self.client = client

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Finalize person_ref and keep enrollment failure non-fatal."""
        action = normalize_action_step(bb_get(blackboard, bb.CURRENT_ACTION, {}))
        if _is_canceled(blackboard):
            _store_result(blackboard, action, CANCELED, "FIND_PERSON canceled before result commit.", fatal=True)
            return CANCELED
        person_ref = dict(bb_get(blackboard, PERSON_REF_PENDING, {}))
        mode = str(bb_get(blackboard, MODE, ""))
        # Section: Both identity adapters expose one mode-independent downstream contract.
        person_ref.setdefault("name", str(bb_get(blackboard, TARGET_NAME, "")))
        person_ref.setdefault("target_name", str(bb_get(blackboard, TARGET_NAME, "")))
        person_ref.setdefault("source", mode)
        person_ref.setdefault("identification_source", mode)
        enrollment = {"status": "not_requested"}
        if mode == "VERBAL_IDENTIFICATION" and bool(
            action.parameters.get("enroll_after_confirmation", True)
        ):
            # Section: Enrollment gets only the time left and cannot invalidate a confirmed identity.
            enrollment = self.client.enroll_profile(
                blackboard,
                action,
                str(blackboard[TARGET_NAME]),
                dict(bb_get(blackboard, CONFIRMED_CANDIDATE, {})),
                max(0.05, _remaining(blackboard)),
            )
            if enrollment.get("status") == "enrolled" and enrollment.get("profile_id"):
                person_ref["profile_id"] = str(enrollment["profile_id"])
            # Section: Cancellation remains terminal even though ordinary enrollment failures are non-fatal.
            if enrollment.get("status") == "canceled" or _is_canceled(blackboard):
                return _store_result(
                    blackboard,
                    action,
                    CANCELED,
                    "FIND_PERSON canceled during profile enrollment.",
                    fatal=True,
                )
        # Section: Commit all compatible and normalized result keys only after identity succeeds.
        blackboard[bb.PERSON_REF] = person_ref
        blackboard[bb.TARGET_PERSON] = str(person_ref.get("person_id", ""))
        blackboard[bb.PROFILE_REGISTERED] = (
            mode == "REGISTERED_PROFILE" or
            enrollment.get("status") == "enrolled"
        )
        blackboard[bb.IDENTIFICATION_MODE_USED] = mode
        memory = dict(bb_get(blackboard, bb.ENTITY_MEMORY, {}))
        memory[str(person_ref.get("person_id", blackboard[TARGET_NAME]))] = {
            "type": "person",
            **person_ref,
        }
        blackboard[bb.ENTITY_MEMORY] = memory
        result = {
            "person_ref": person_ref,
            "identity_confirmed": True,
            "identification_mode_used": mode,
            "profile_registered": bool(blackboard[bb.PROFILE_REGISTERED]),
            "enrollment": enrollment,
            "fallback_reason": str(bb_get(blackboard, FALLBACK_REASON, "")),
            "candidate_registry": dict(bb_get(blackboard, CANDIDATE_REGISTRY, {})),
        }
        if enrollment.get("status") == "failed":
            result["warning"] = "PROFILE_ENROLLMENT_FAILED"
        return _store_result(blackboard, action, SUCCESS, "Target person found.", result)


class FindPersonSkillSM(StateMachine):
    """Special function: Resolve a named person by registered or verbal identity."""

    def __init__(
        self,
        client: FindPersonClients | None = None,
    ) -> None:
        """Special function: Build the explicit two-mode YASMIN state graph."""
        super().__init__(outcomes=SKILL_OUTCOMES)
        self.set_name("FIND_PERSON_SKILL_SM")
        identity_client = client or FindPersonClients()
        # Section: Validation admits only names before identity mode selection.
        self.add_state(
            "VALIDATE_AND_ROUTE",
            ValidateAndRouteState(),
            transitions={
                "name": "CHECK_PROFILE",
                INVALID_INPUT: INVALID_INPUT,
                CANCELED: CANCELED,
            },
        )
        # Section: Profile availability is the sole deterministic identity-mode decision.
        self.add_state(
            "CHECK_PROFILE",
            CheckProfileState(identity_client),
            transitions={
                "registered": "REGISTERED_PROFILE",
                "verbal": "VERBAL_IDENTIFICATION",
                CANCELED: CANCELED,
                TIMEOUT: TIMEOUT,
            },
        )
        # Section: Registered lookup can fall back without silently changing success semantics.
        self.add_state(
            "REGISTERED_PROFILE",
            RegisteredProfileState(identity_client),
            transitions={
                SUCCESS: "ENROLL_AND_STORE",
                "fallback": "VERBAL_IDENTIFICATION",
                NOT_FOUND: NOT_FOUND,
                FAILED: FAILED,
                CANCELED: CANCELED,
                TIMEOUT: TIMEOUT,
            },
        )
        # Section: Verbal mode owns candidate rejection and bounded confirmation retries.
        self.add_state(
            "VERBAL_IDENTIFICATION",
            VerbalIdentificationState(identity_client),
            transitions={
                SUCCESS: "ENROLL_AND_STORE",
                NOT_FOUND: NOT_FOUND,
                CANCELED: CANCELED,
                TIMEOUT: TIMEOUT,
            },
        )
        # Section: One final state atomically writes downstream identity and memory keys.
        self.add_state(
            "ENROLL_AND_STORE",
            EnrollAndStoreState(identity_client),
            transitions={SUCCESS: SUCCESS, CANCELED: CANCELED},
        )
