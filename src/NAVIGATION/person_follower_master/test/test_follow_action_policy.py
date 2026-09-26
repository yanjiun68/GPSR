"""Section: PersonFollow completion geometry policy tests."""

import math
from types import SimpleNamespace

from pfm_utils.follow_action_policy import master_has_moved, master_is_facing


def make_master(
    distance: float = 1.2,
    bearing: float = 0.0,
    speed: float = 0.0,
    state: str = "FOLLOWING",
    locked: bool = True,
) -> SimpleNamespace:
    """Special function: Build one MasterTarget-compatible observation."""
    return SimpleNamespace(
        distance=float(distance),
        bearing=float(bearing),
        vx=float(speed),
        vy=0.0,
        state=str(state),
        is_locked=bool(locked),
    )


def test_facing_requires_locked_close_centered_stationary_master() -> None:
    """Special function: Completion gesture rejects distant, turning, or moving targets."""
    assert master_is_facing(make_master(), 1.2, 0.25, math.radians(10.0), 0.12)
    assert not master_is_facing(make_master(distance=1.6), 1.2, 0.25, math.radians(10.0), 0.12)
    assert not master_is_facing(make_master(bearing=math.radians(20.0)), 1.2, 0.25, math.radians(10.0), 0.12)
    assert not master_is_facing(make_master(speed=0.3), 1.2, 0.25, math.radians(10.0), 0.12)
    assert not master_is_facing(make_master(state="LOST", locked=False), 1.2, 0.25, math.radians(10.0), 0.12)


def test_motion_arming_prevents_initial_enrollment_from_completing_goal() -> None:
    """Special function: A stationary enrollment pose cannot immediately finish FOLLOW_PERSON."""
    assert not master_has_moved(make_master(), 1.2, 0.30, math.radians(15.0), 0.18)
    assert master_has_moved(make_master(distance=1.7), 1.2, 0.30, math.radians(15.0), 0.18)
    assert master_has_moved(make_master(speed=0.25), 1.2, 0.30, math.radians(15.0), 0.18)
