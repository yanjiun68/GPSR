"""Section: Unit tests for answer-text-only TTS forwarding helpers."""

from fixed_information_dialogue.answer_text_tts_bridge import normalize_answer_text


def test_normalize_answer_text_trims_topic_payload() -> None:
    """Special function: Preserve answer content while removing transport whitespace."""
    # Section: Confirm the bridge sends the answer itself without surrounding whitespace.
    assert normalize_answer_text("  My team is Putrabot UPM.\n") == "My team is Putrabot UPM."


def test_normalize_answer_text_rejects_whitespace_only_payload() -> None:
    """Special function: Convert a non-speakable payload into an empty value."""
    # Section: Confirm subscriber validation can ignore empty answer messages.
    assert normalize_answer_text(" \n\t ") == ""
