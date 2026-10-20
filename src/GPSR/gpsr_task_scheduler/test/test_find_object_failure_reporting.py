"""Section: Deterministic compound FIND_OBJECT failure-report regression tests."""

from gpsr_task_scheduler.scheduler_node import GpsrTaskSchedulerNode


def _compound_plan():
    """Special function: Build a minimal find-then-pick plan for report tests."""
    return {
        "task_request": {
            "task_type": "acquire_object",
            "target": {"object_name": "coke"},
            "task_location": "kitchen",
        },
        "steps": [
            {
                "skill_name": "FIND_OBJECT",
                "arguments": {
                    "object_name": "coke",
                    "target_location": "kitchen",
                    "search_scope": "STRICT_LOCATION",
                },
            },
            {"skill_name": "PICK_OBJECT", "arguments": {"object_name": "coke"}},
        ],
    }


def test_compound_not_found_report_includes_two_pass_diagnostics():
    """Special function: Confirm a compound miss is reported after two passes."""
    result = {
        "results": [
            {
                "action_type": "FIND_OBJECT",
                "outcome": "not_found",
                "result": {
                    "search_passes_completed": 2,
                    "not_found_retries_used": 1,
                },
            }
        ]
    }
    assert GpsrTaskSchedulerNode.requires_not_found_report(_compound_plan(), result)
    assert (
        GpsrTaskSchedulerNode.failure_report_text(_compound_plan(), result)
        == "I could not find the Coke in the kitchen after searching twice."
    )


def test_non_not_found_failures_do_not_generate_object_miss_report():
    """Special function: Keep timeout and infrastructure failures on generic paths."""
    for outcome in ("timeout", "canceled", "recoverable_failure", "failed"):
        result = {
            "results": [
                {"action_type": "FIND_OBJECT", "outcome": outcome, "result": {}}
            ]
        }
        assert not GpsrTaskSchedulerNode.requires_not_found_report(_compound_plan(), result)


def test_incomplete_search_does_not_generate_false_object_miss_report():
    """Special function: Keep navigation and scan failures on infrastructure paths."""
    result = {
        "results": [
            {
                "action_type": "FIND_OBJECT",
                "outcome": "not_found",
                "result": {
                    "search_passes_completed": 1,
                    "navigation_failures": [{"waypoint": "kitchen"}],
                    "search_passes": [{"pass_index": 1, "complete_search": False}],
                },
            }
        ]
    }
    assert not GpsrTaskSchedulerNode.requires_not_found_report(_compound_plan(), result)


def test_category_global_report_uses_indefinite_article_without_location():
    """Special function: Render category misses without inventing a room."""
    planned = {
        "task_request": {"task_type": "deliver_object", "target": {"category": "snack"}},
        "steps": [{"skill_name": "FIND_OBJECT", "arguments": {"category": "snack", "search_scope": "GLOBAL"}}],
    }
    result = {
        "results": [
            {
                "action_type": "FIND_OBJECT",
                "outcome": "not_found",
                "result": {"search_passes_completed": 2, "not_found_retries_used": 1},
            }
        ]
    }
    assert GpsrTaskSchedulerNode.failure_report_text(planned, result) == (
        "I could not find a snack after searching twice."
    )


def test_object_attribute_mismatch_has_specific_report():
    """Special function: Do not describe a visible wrong-color object as absent."""
    result = {
        "results": [
            {
                "action_type": "FIND_OBJECT",
                "outcome": "not_found",
                "result": {
                    "failure_code": "OBJECT_ATTRIBUTE_MISMATCH",
                    "search_passes_completed": 1,
                },
            }
        ]
    }
    assert GpsrTaskSchedulerNode.requires_automatic_failure_report(
        _compound_plan(), result
    )
    assert GpsrTaskSchedulerNode.failure_report_text(_compound_plan(), result) == (
        "I found the Coke in the kitchen, but its attributes did not match the request."
    )


def test_structured_execution_failures_have_deterministic_reports():
    """Special function: Cover every newly supported non-object failure family."""
    expected = {
        "PERSON_ATTRIBUTE_MISMATCH":
            "I found people, but none matched all requested attributes.",
        "PERCEPTION_UNAVAILABLE":
            "I could not complete the task because perception was unavailable.",
        "INVALID_INPUT":
            "I could not execute the task because the command input was invalid.",
        "LIFECYCLE_SKILL_UNAVAILABLE":
            "I could not complete the task because a required skill could not be started.",
        "SKILL_UNAVAILABLE":
            "I could not complete the task because a required skill was unavailable.",
        "MANIPULATION_FAILED":
            "I could not complete the requested manipulation.",
        "MANIPULATION_GOAL_REJECTED":
            "The manipulation system did not accept the requested action.",
        "MANIPULATION_TIMEOUT":
            "The requested manipulation did not finish in time.",
    }
    planned = {"task_request": {}, "steps": []}
    for failure_code, speech in expected.items():
        result = {
            "results": [
                {
                    "action_type": "TEST",
                    "outcome": "failed",
                    "result": {"failure_code": failure_code},
                }
            ]
        }
        assert GpsrTaskSchedulerNode.requires_automatic_failure_report(
            planned, result
        )
        assert GpsrTaskSchedulerNode.failure_report_text(planned, result) == speech
