"""Section: Fixed-information request-envelope parsing regressions."""

from fixed_information_dialogue.fixed_information_node import (
    FixedInformationDialogueNode,
)


def test_parse_request_accepts_correlated_object_envelope() -> None:
    """Special function: Preserve caller identity, question text, and object context."""
    request_id, text, context = FixedInformationDialogueNode._parse_request(
        '{"request_id":"request-7","text":"what time is it","context":{"turn":2}}'
    )
    # Section: Correlation fields must survive transport decoding unchanged.
    assert request_id == "request-7"
    assert text == "what time is it"
    assert context == {"turn": 2}


def test_parse_request_rejects_non_object_json_without_raising() -> None:
    """Special function: Convert a JSON array into an empty, safely unanswerable request."""
    request_id, text, context = FixedInformationDialogueNode._parse_request(
        '["what time is it"]'
    )
    # Section: The generated identifier still permits a structured NO_MATCH response.
    assert request_id
    assert text == ""
    assert context == {}
