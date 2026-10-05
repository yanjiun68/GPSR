"""Section: Conservative fuzzy object suggestions and clarification tests."""

from gpsr_task_scheduler.clarification import assess_command, merge_clarification
from gpsr_task_scheduler.config import SchedulerConfig
from gpsr_task_scheduler.scheduler import schedule_command


def _config() -> SchedulerConfig:
    """Special function: Build a small authoritative RoboCup vocabulary for unit tests."""
    # Section: Keep the fixture independent from the installed primitive registry.
    return SchedulerConfig(
        known_locations={"kitchen", "bedroom"},
        known_objects={"apple", "coke", "white_shirt"},
        known_categories={"fruit"},
        object_aliases={
            "apple": "apple",
            "coke": "coke",
            "coca_cola": "coke",
            "white_shirt": "white_shirt",
        },
        known_people={"operator"},
        verb_aliases={
            "FIND_OBJECT": ["find"],
            "COUNT_OBJECTS": ["count"],
            "PICK_OBJECT": ["pick up"],
            "DELIVER_OBJECT": ["bring", "deliver"],
        },
        supported_object_features={"object_color": ["red", "blue"]},
    )


def test_close_object_name_gets_explicit_confirmation_prompt():
    """Special function: Suggest apple for a close misspelling without completing the task."""
    # Section: A fuzzy candidate must leave the command incomplete until confirmation.
    result = assess_command("find aplle in the kitchen", _config())

    assert result.status == "MISSING_TARGET"
    assert result.suggestion is not None
    assert result.suggestion.source == "aplle"
    assert result.suggestion.suggested_target == "apple"
    assert 'Did you mean "apple"?' in result.prompt


def test_scheduler_validation_exposes_machine_readable_suggestion():
    """Special function: Publish the suggestion through the scheduler validation context."""
    # Section: The dry planning facade must expose the same suggestion used by voice dialogue.
    result = schedule_command("fuzzy_test", "find aplle in the kitchen", _config())

    assert not result.valid
    assert result.command_context is not None
    assert result.command_context.validation["suggestion"] == {
        "input": "aplle",
        "suggested_target": "apple",
        "score": 0.8,
    }


def test_yes_confirmation_replaces_the_misspelled_target():
    """Special function: Confirming yes repairs the original command exactly once."""
    # Section: The affirmative reply must not append a duplicate object name.
    suggestion = {
        "input": "aplle",
        "suggested_target": "apple",
        "score": 0.8,
    }

    merged = merge_clarification(
        "find aplle in the kitchen",
        "yes",
        _config(),
        suggestion,
    )

    assert merged == "find apple in the kitchen"


def test_user_can_replace_suggestion_with_another_known_object():
    """Special function: A negative correction can provide a different configured object."""
    # Section: The user-provided object must replace the typo without trusting the suggestion.
    suggestion = {
        "input": "aplle",
        "suggested_target": "apple",
        "score": 0.8,
    }

    merged = merge_clarification(
        "find aplle in the kitchen",
        "no, coke",
        _config(),
        suggestion,
    )

    assert merged == "find coke in the kitchen"


def test_unrelated_word_does_not_receive_a_weak_suggestion():
    """Special function: Low-confidence candidates remain ordinary clarification requests."""
    # Section: Safety requires no automatic guess for a distant vocabulary term.
    result = assess_command("find qwerty in the kitchen", _config())

    assert result.status == "MISSING_TARGET"
    assert result.suggestion is None
    assert result.prompt == "Which object or person do you mean?"


def test_exact_alias_does_not_trigger_fuzzy_confirmation():
    """Special function: Existing aliases continue directly through completeness checking."""
    # Section: Exact configured aliases must preserve the original deterministic behavior.
    result = assess_command("find coca cola in the kitchen", _config())

    assert result.complete
    assert result.suggestion is None
