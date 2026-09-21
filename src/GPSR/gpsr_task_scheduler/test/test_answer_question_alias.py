"""Section: Answer-question wording uses the existing ask-and-listen primitive."""

import pytest

from gpsr_task_scheduler.config import default_config
from gpsr_task_scheduler.scheduler import schedule_command
from gpsr_task_scheduler.semantic_compiler import ANSWER_QUESTION_PROMPT


@pytest.mark.parametrize(
    ("instruction", "expected_actions"),
    [
        ("answer question", ["ASK_AND_LISTEN"]),
        ("go to the kitchen and answer question", ["NAVIGATE", "ASK_AND_LISTEN"]),
        ("go to the kitchen then answer question", ["NAVIGATE", "ASK_AND_LISTEN"]),
    ],
)
def test_answer_question_creates_listening_step(
    instruction: str,
    expected_actions: list[str],
) -> None:
    """Special function: Keep the requested listening step in the ActionQueue."""
    result = schedule_command("answer_question", instruction, default_config())
    # Section: The alias must compile in both single- and multi-clause commands.
    assert result.valid, [issue.message for issue in result.issues]
    assert result.command_context is not None
    queue = result.command_context.action_queue
    assert [step["action_type"] for step in queue] == expected_actions
    # Section: ASK_AND_LISTEN needs a prompt to speak before it starts STT.
    assert queue[-1]["arguments"]["question"].casefold() == ANSWER_QUESTION_PROMPT.casefold()
    if len(queue) > 1:
        assert queue[-1]["dependencies"] == [queue[-2]["step_id"]]
