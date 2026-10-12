"""Section: Shared deterministic GPSR outcomes."""

# Section: Common outcomes required by GPSR skills.
SUCCESS = "success"
FAILED = "failed"
CANCELED = "canceled"
TIMEOUT = "timeout"
INVALID_INPUT = "invalid_input"
NOT_FOUND = "not_found"
NEED_CLARIFICATION = "need_clarification"
RECOVERABLE_FAILURE = "recoverable_failure"
FATAL_FAILURE = "fatal_failure"
PERSON_LOST = "person_lost"
SKILL_UNAVAILABLE = "skill_unavailable"
SKIPPED_DEPENDENCY_FAILED = "skipped_dependency_failed"
PARTIAL_SUCCESS = "partial_success"

# Section: Outcome groups used by executor routing.
SKILL_OUTCOMES = [SUCCESS, FAILED, CANCELED, TIMEOUT, INVALID_INPUT, NOT_FOUND, NEED_CLARIFICATION, RECOVERABLE_FAILURE, FATAL_FAILURE, PERSON_LOST, SKILL_UNAVAILABLE, SKIPPED_DEPENDENCY_FAILED]
RECOVERABLE_OUTCOMES = {FAILED, TIMEOUT, NOT_FOUND, NEED_CLARIFICATION, RECOVERABLE_FAILURE, PERSON_LOST}
FATAL_OUTCOMES = {CANCELED, INVALID_INPUT, FATAL_FAILURE, SKILL_UNAVAILABLE}


def is_recoverable(outcome: str) -> bool:
    """Special function: Return true when recovery may retry the action."""
    return outcome in RECOVERABLE_OUTCOMES


def is_fatal(outcome: str) -> bool:
    """Special function: Return true when queue execution must abort."""
    return outcome in FATAL_OUTCOMES
