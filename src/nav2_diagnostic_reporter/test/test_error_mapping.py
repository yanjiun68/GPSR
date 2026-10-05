"""Unit tests for stable diagnostic error and user-message contracts."""

# Section: System under test imports.
from nav2_diagnostic_reporter.error_codes import ErrorCode
from nav2_diagnostic_reporter.message_catalog import USER_MESSAGES, user_message_for


# Section: Catalogue completeness tests.
def test_every_declared_error_code_has_a_user_message():
    """Ensure no reported error can fall through to an empty user-facing response."""

    assert set(USER_MESSAGES) == set(ErrorCode)
    assert all(user_message_for(code) for code in ErrorCode)


def test_blocked_goal_message_does_not_expose_technical_detail():
    """Ensure the user-facing layer remains safe for speech and task presentation."""

    message = user_message_for(ErrorCode.GOAL_OCCUPIED).lower()
    assert 'costmap' not in message
    assert 'tf' not in message
