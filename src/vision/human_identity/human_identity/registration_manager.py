"""Section: Thread-safe registration target ownership."""

from __future__ import annotations

from dataclasses import dataclass
from threading import RLock
from uuid import uuid4


@dataclass(frozen=True, slots=True)
class RegistrationContext:
    """Section: Immutable snapshot of the currently locked registration target."""

    person_id: int
    track_id: int
    started_nanoseconds: int
    registration_token: str


class RegistrationManager:
    """Section: Allow exactly one active person registration at a time."""

    def __init__(self) -> None:
        """Special function: Initialize the lock and empty active context."""
        self._lock = RLock()
        self._active: RegistrationContext | None = None

    def start(
        self,
        person_id: int,
        track_id: int,
        started_nanoseconds: int,
    ) -> RegistrationContext:
        """Special function: Lock a positive person and track pair for registration."""
        if person_id <= 0:
            raise ValueError("person_id must be greater than zero")
        if track_id <= 0:
            raise ValueError("track_id must be greater than zero")
        with self._lock:
            if self._active is not None:
                raise RuntimeError("A person registration is already active")
            self._active = RegistrationContext(
                person_id=person_id,
                track_id=track_id,
                started_nanoseconds=started_nanoseconds,
                registration_token=uuid4().hex,
            )
            return self._active

    def get_active(self) -> RegistrationContext | None:
        """Special function: Return the immutable active registration snapshot."""
        with self._lock:
            return self._active

    def resolve_active_target(self) -> tuple[int, int]:
        """Special function: Resolve profile updates against the locked registration target."""
        with self._lock:
            if self._active is None:
                raise RuntimeError("No active registration target")
            return self._active.person_id, self._active.track_id

    def finish(self, registration_token: str) -> bool:
        """Special function: Release the registration only when its owner supplies the token."""
        with self._lock:
            if self._active is None or self._active.registration_token != registration_token:
                return False
            self._active = None
            return True


def resolve_profile_target(
    is_nearest: bool,
    requested_person_id: int,
    registration: RegistrationManager,
) -> tuple[int, int]:
    """Special function: Apply service target semantics without recomputing the nearest track."""
    if is_nearest:
        return registration.resolve_active_target()
    if requested_person_id <= 0:
        raise ValueError("person_id must be greater than zero")
    return requested_person_id, 0


def rank_bounding_boxes(observations: list[dict]) -> list[dict]:
    """Special function: Rank visible people by descending bounding-box area."""
    ranked = sorted(observations, key=lambda item: float(item["area"]), reverse=True)
    for rank, observation in enumerate(ranked, start=1):
        observation["rank"] = rank
    return ranked
