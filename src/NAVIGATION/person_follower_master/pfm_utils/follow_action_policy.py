"""Section: Pure completion policy used by the GPSR person-follow action adapter."""

from __future__ import annotations

import math
from typing import Any


# Section: The pure policy intentionally depends only on MasterTarget-compatible fields.
STATE_FOLLOWING = "FOLLOWING"


def master_is_facing(
    master: Any | None,
    follow_distance_m: float,
    distance_margin_m: float,
    bearing_tolerance_rad: float,
    max_speed_mps: float,
) -> bool:
    """Special function: Infer a stop-and-face gesture from stable centered target motion."""
    if master is None or not bool(master.is_locked) or str(master.state) != STATE_FOLLOWING:
        return False
    speed_mps = math.hypot(float(master.vx), float(master.vy))
    return (
        float(master.distance) <= float(follow_distance_m) + float(distance_margin_m)
        and abs(float(master.bearing)) <= float(bearing_tolerance_rad)
        and speed_mps <= float(max_speed_mps)
    )


def master_has_moved(
    master: Any | None,
    follow_distance_m: float,
    distance_delta_m: float,
    bearing_delta_rad: float,
    speed_threshold_mps: float,
) -> bool:
    """Special function: Arm completion only after evidence that following actually began."""
    if master is None or not bool(master.is_locked) or str(master.state) != STATE_FOLLOWING:
        return False
    return (
        float(master.distance) >= float(follow_distance_m) + float(distance_delta_m)
        or abs(float(master.bearing)) >= float(bearing_delta_rad)
        or math.hypot(float(master.vx), float(master.vy)) >= float(speed_threshold_mps)
    )
