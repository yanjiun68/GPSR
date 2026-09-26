"""Section: Fixed-information questions compile to their dedicated YASMIN skill."""

import pytest

from gpsr_task_scheduler.config import default_config
from gpsr_task_scheduler.scheduler import schedule_command


@pytest.mark.parametrize(
    "instruction",
    [
        "tell today's date",
        "what time is it",
        "what is your team name",
        "who are you",
        "tell me about yourself",
        "today's date",
    ],
)
def test_package_owned_question_triggers_fixed_information_skill(
    instruction: str,
) -> None:
    """Special function: Preserve the question and emit one provider-routed SAY action."""
    result = schedule_command("fixed_information", instruction, default_config())
    # Section: Every supported question must bypass physical-task clarification.
    assert result.valid, [issue.message for issue in result.issues]
    assert len(result.steps) == 1
    assert result.steps[0].skill_name == "SAY"
    assert result.steps[0].arguments["question"] == instruction
    assert result.steps[0].arguments["fixed_information"] is True


def test_incidental_runtime_word_does_not_trigger_information_skill() -> None:
    """Special function: Keep ordinary physical tasks out of the information route."""
    result = schedule_command(
        "navigation_tomorrow",
        "go to the kitchen tomorrow",
        default_config(),
    )
    # Section: A date word without an information-request form remains navigation.
    assert result.valid, [issue.message for issue in result.issues]
    assert [step.skill_name for step in result.steps] == ["NAVIGATE"]


def test_named_person_receives_fixed_information_after_meeting() -> None:
    """Special function: Classify a recipient-directed fact as fixed information, not SAY."""
    result = schedule_command(
        "meet_then_date",
        "meet Axel in the bedroom, tell him today's date",
        default_config(),
    )
    # Section: The prior meet satisfies recipient discovery and the provider speaks the answer.
    assert result.valid, [issue.message for issue in result.issues]
    assert [step.skill_name for step in result.steps] == [
        "FIND_PERSON",
        "SAY",
    ]
    assert result.steps[1].arguments["question"] == "today's date"
    assert result.steps[1].arguments["fixed_information"] is True
    assert result.steps[1].arguments["target_person"] == "axel"
    assert result.steps[1].dependencies == [result.steps[0].step_id]


def test_fixed_information_preserves_following_person_action() -> None:
    """Special function: Keep a later pronoun-resolved action after direct fact delivery."""
    result = schedule_command(
        "meet_date_follow",
        "meet Axel in the bedroom, tell him today's date, then follow him along",
        default_config(),
    )
    # Section: The information action remains ordered between discovery and following.
    assert result.valid, [issue.message for issue in result.issues]
    assert [step.skill_name for step in result.steps] == [
        "FIND_PERSON",
        "SAY",
        "FOLLOW_PERSON",
    ]
