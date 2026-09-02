from gpsr_sys_ui.topic_types import (
    TWIST_STAMPED_TYPE,
    TWIST_TYPE,
    resolve_velocity_type,
)


def test_twist_resolves():
    assert resolve_velocity_type([TWIST_TYPE]) == TWIST_TYPE


def test_twist_stamped_resolves():
    assert resolve_velocity_type([TWIST_STAMPED_TYPE]) == TWIST_STAMPED_TYPE


def test_absent_or_unsupported_does_not_resolve():
    assert resolve_velocity_type([]) is None
    assert resolve_velocity_type(['custom_msgs/msg/Velocity']) is None


def test_multiple_types_are_rejected_as_ambiguous():
    assert resolve_velocity_type([TWIST_TYPE, TWIST_STAMPED_TYPE]) is None
