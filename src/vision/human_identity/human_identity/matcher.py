"""Section: Deterministic multi-feature identity matching."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np

from .feature_types import PersonFeatures


@dataclass(slots=True)
class MatchResult:
    """Section: Match decision and component scores published by the ROS node."""

    status: str = "UNKNOWN"
    person_id: int = 0
    final_score: float = 0.0
    second_score: float = 0.0
    body_score: float = 0.0
    upper_score: float = 0.0
    lower_score: float = 0.0
    object_score: float = 0.0


class IdentityMatcher:
    """Section: Match observations against stored person prototypes."""

    def __init__(self, threshold: float = 0.72, ambiguity_margin: float = 0.08) -> None:
        """Special function: Store bounded recognition and ambiguity thresholds."""
        self.threshold = float(np.clip(threshold, 0.0, 1.0))
        self.ambiguity_margin = float(np.clip(ambiguity_margin, 0.0, 1.0))

    def match(
        self,
        features: PersonFeatures,
        prototypes: Iterable[tuple[int, PersonFeatures]],
    ) -> MatchResult:
        """Special function: Choose a person only when score and separation tests both pass."""
        best_by_person: dict[int, tuple[float, tuple[float, float, float, float]]] = {}
        for person_id, prototype in prototypes:
            components = self._component_scores(features, prototype)
            score = self._weighted_score(features, prototype, components)
            previous = best_by_person.get(person_id)
            if previous is None or score > previous[0]:
                best_by_person[person_id] = (score, components)
        if not best_by_person:
            return MatchResult()
        ordered = sorted(best_by_person.items(), key=lambda item: item[1][0], reverse=True)
        best_person_id, (best_score, components) = ordered[0]
        second_score = ordered[1][1][0] if len(ordered) > 1 else 0.0
        if best_score < self.threshold:
            status = "UNKNOWN"
            resolved_person_id = 0
        elif best_score - second_score < self.ambiguity_margin:
            status = "AMBIGUOUS"
            resolved_person_id = 0
        else:
            status = "RECOGNIZED"
            resolved_person_id = best_person_id
        return MatchResult(
            status=status,
            person_id=resolved_person_id,
            final_score=float(best_score),
            second_score=float(second_score),
            body_score=float(components[0]),
            upper_score=float(components[1]),
            lower_score=float(components[2]),
            object_score=float(components[3]),
        )

    @staticmethod
    def _similarity(left: np.ndarray, right: np.ndarray) -> float:
        """Special function: Convert cosine similarity into a bounded zero-to-one score."""
        left = np.asarray(left, dtype=np.float32).reshape(-1)
        right = np.asarray(right, dtype=np.float32).reshape(-1)
        if left.size == 0 or left.shape != right.shape:
            return 0.0
        denominator = float(np.linalg.norm(left) * np.linalg.norm(right))
        if denominator <= 1e-8:
            return 0.0
        cosine = float(np.dot(left, right) / denominator)
        return float(np.clip((cosine + 1.0) * 0.5, 0.0, 1.0))

    def _component_scores(
        self,
        left: PersonFeatures,
        right: PersonFeatures,
    ) -> tuple[float, float, float, float]:
        """Special function: Score each visual identity component independently."""
        return (
            self._similarity(left.body_vector, right.body_vector),
            self._similarity(left.upper_histogram, right.upper_histogram),
            self._similarity(left.lower_histogram, right.lower_histogram),
            self._similarity(left.object_vector, right.object_vector),
        )

    @staticmethod
    def _weighted_score(
        left: PersonFeatures,
        right: PersonFeatures,
        components: tuple[float, float, float, float],
    ) -> float:
        """Special function: Weight each component by the weaker observation quality."""
        base_weights = (0.45, 0.25, 0.20, 0.10)
        left_qualities = (
            left.body_quality,
            left.upper_quality,
            left.lower_quality,
            left.object_quality,
        )
        right_qualities = (
            right.body_quality,
            right.upper_quality,
            right.lower_quality,
            right.object_quality,
        )
        weights = [
            base * max(0.0, min(float(left_quality), float(right_quality), 1.0))
            for base, left_quality, right_quality in zip(
                base_weights,
                left_qualities,
                right_qualities,
            )
        ]
        denominator = sum(weights)
        if denominator <= 1e-8:
            return 0.0
        weighted_total = sum(score * weight for score, weight in zip(components, weights))
        return float(weighted_total / denominator)
