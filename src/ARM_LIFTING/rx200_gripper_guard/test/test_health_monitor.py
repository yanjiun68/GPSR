"""Hardware-free safety contract tests for gripper health snapshots."""

import pytest

from rx200_gripper_guard.health_monitor import HealthSnapshot, is_hardware_fault


@pytest.mark.parametrize("status", [1, 4, 8, 16, 32, 36, 63])
def test_every_nonzero_hardware_error_combination_is_a_fault(status: int) -> None:
    """Do not special-case overload when any DYNAMIXEL fault bit must stop motion."""
    assert is_hardware_fault(status)


def test_zero_or_unavailable_hardware_error_is_not_a_reported_fault() -> None:
    """Distinguish an explicit clear value from unavailable diagnostics."""
    assert not is_hardware_fault(0)
    assert not is_hardware_fault(None)


def test_critical_status_freshness_and_pwm_mode_contract() -> None:
    """Require recent status and DYNAMIXEL PWM Operating_Mode value 16."""
    snapshot = HealthSnapshot(
        register_service_available=True,
        hardware_error_status=0,
        hardware_error_timestamp=10.0,
        operating_mode=16,
    )
    assert snapshot.critical_status_fresh(now=10.4, timeout_sec=0.5)
    assert not snapshot.critical_status_fresh(now=10.6, timeout_sec=0.5)
    assert snapshot.operating_mode_matches(16)
    assert not snapshot.operating_mode_matches(3)
