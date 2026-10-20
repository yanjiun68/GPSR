"""Section: Shared Python compatibility failure-payload regression tests."""

from robot_yasmin_tasks.gpsr.models import make_skill_result


def test_invalid_input_receives_stable_failure_metadata():
    """Special function: Legacy validators gain structured details automatically."""
    result = make_skill_result("FIND_OBJECT", "invalid_input", "missing target")
    assert result.result["failure_code"] == "INVALID_INPUT"
    assert result.result["failure_phase"] == "validation"
    assert result.fatal is False


def test_skill_unavailable_preserves_specific_caller_metadata():
    """Special function: Explicit lifecycle details are never overwritten."""
    result = make_skill_result(
        "FIND_OBJECT",
        "skill_unavailable",
        "prepare failed",
        {
            "failure_code": "LIFECYCLE_SKILL_UNAVAILABLE",
            "failure_phase": "lifecycle_prepare",
        },
        fatal=True,
    )
    assert result.result["failure_code"] == "LIFECYCLE_SKILL_UNAVAILABLE"
    assert result.result["failure_phase"] == "lifecycle_prepare"
    assert result.fatal is True
