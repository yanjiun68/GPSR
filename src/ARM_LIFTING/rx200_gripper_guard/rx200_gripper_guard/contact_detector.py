"""ROS-independent persistent contact detector for an RX200 gripper."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from statistics import median
from typing import Iterable


@dataclass(frozen=True)
class ContactObservation:
    """Expose independently testable stall, contact, and empty-close evidence."""

    motion_stalled: bool
    contact_candidate: bool
    contact_confirmed: bool
    empty_close_detected: bool
    load_supported_contact: bool
    stable_samples: int
    empty_stable_samples: int
    position_span: float
    load_delta: float


class ContactDetector:
    """Detect persistent object contact without treating every stall as a grasp."""

    def __init__(
        self,
        required_samples: int,
        velocity_threshold: float,
        position_span_threshold: float,
        load_delta_threshold: float,
        require_load_confirmation: bool,
        empty_closed_aperture: float,
        aperture_margin: float,
        history_length: int,
    ) -> None:
        """Configure calibration-dependent thresholds without importing ROS."""
        # Section: Reject nonsensical history sizes while allowing neutral zero thresholds.
        self.required_samples = max(1, int(required_samples))
        self.velocity_threshold = max(0.0, float(velocity_threshold))
        self.position_span_threshold = max(0.0, float(position_span_threshold))
        self.load_delta_threshold = max(0.0, float(load_delta_threshold))
        self.require_load_confirmation = bool(require_load_confirmation)
        self.empty_closed_aperture = float(empty_closed_aperture)
        self.aperture_margin = max(0.0, float(aperture_margin))
        self.history_length = max(self.required_samples, int(history_length), 1)
        self.position_history: deque[float] = deque(maxlen=self.history_length)
        self.velocity_history: deque[float] = deque(maxlen=self.history_length)
        self.aperture_history: deque[float] = deque(maxlen=self.history_length)
        self.load_history: deque[float] = deque(maxlen=self.history_length)
        self.baseline_load = 0.0
        self.stable_samples = 0
        self.empty_stable_samples = 0

    def reset(self, baseline_samples: Iterable[float] = ()) -> None:
        """Start a new close operation using a median initial load baseline."""
        # Section: Ignore unavailable values while retaining zero as a valid load sample.
        valid_samples = [float(value) for value in baseline_samples if value is not None]
        self.baseline_load = median(valid_samples) if valid_samples else 0.0
        self.position_history.clear()
        self.velocity_history.clear()
        self.aperture_history.clear()
        self.load_history.clear()
        self.stable_samples = 0
        self.empty_stable_samples = 0

    def update(
        self,
        motor_position: float,
        motor_velocity: float,
        finger_aperture: float,
        load_value: float,
        closing_active: bool,
        timestamp: float,
    ) -> ContactObservation:
        """Evaluate one feedback sample and update persistent evidence counters."""
        del timestamp  # Section: Accepted for deterministic callers and future rate validation.
        self.position_history.append(float(motor_position))
        self.velocity_history.append(float(motor_velocity))
        self.aperture_history.append(float(finger_aperture))
        self.load_history.append(float(load_value))

        # Section: A full history prevents startup samples from masquerading as a stable stall.
        history_ready = len(self.position_history) >= self.history_length
        position_span = (
            max(self.position_history) - min(self.position_history)
            if self.position_history
            else float("inf")
        )
        velocity_low = all(
            abs(value) <= self.velocity_threshold for value in self.velocity_history
        )
        motion_stalled = (
            bool(closing_active)
            and history_ready
            and velocity_low
            and position_span <= self.position_span_threshold
        )

        # Section: Aperture separates an object-blocked close from mechanical empty closure.
        aperture_boundary = self.empty_closed_aperture + self.aperture_margin
        object_aperture = history_ready and all(
            value > aperture_boundary for value in self.aperture_history
        )
        empty_aperture = history_ready and all(
            value <= aperture_boundary for value in self.aperture_history
        )
        load_delta = abs(float(load_value) - self.baseline_load)
        load_confirmed = history_ready and all(
            abs(value - self.baseline_load) >= self.load_delta_threshold
            for value in self.load_history
        )
        probable_contact = motion_stalled and object_aperture
        candidate = probable_contact and (
            load_confirmed or not self.require_load_confirmation
        )

        # Section: Every required condition must remain true for consecutive samples.
        if candidate:
            self.stable_samples = (
                min(self.required_samples, len(self.position_history))
                if self.stable_samples == 0
                else self.stable_samples + 1
            )
        else:
            self.stable_samples = 0
        empty_candidate = motion_stalled and empty_aperture
        if empty_candidate:
            self.empty_stable_samples = (
                min(self.required_samples, len(self.position_history))
                if self.empty_stable_samples == 0
                else self.empty_stable_samples + 1
            )
        else:
            self.empty_stable_samples = 0
        contact_confirmed = self.stable_samples >= self.required_samples
        empty_close_detected = self.empty_stable_samples >= self.required_samples

        return ContactObservation(
            motion_stalled=motion_stalled,
            contact_candidate=candidate,
            contact_confirmed=contact_confirmed,
            empty_close_detected=empty_close_detected,
            load_supported_contact=probable_contact and load_confirmed,
            stable_samples=self.stable_samples,
            empty_stable_samples=self.empty_stable_samples,
            position_span=position_span,
            load_delta=load_delta,
        )
