"""Section: Typed feature records shared by extraction, storage, and matching."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable

import numpy as np


# Section: ROS-compatible three-state visual attribute values.
ATTRIBUTE_UNKNOWN = -1
ATTRIBUTE_FALSE = 0
ATTRIBUTE_TRUE = 1


def _empty_vector() -> np.ndarray:
    """Special function: Create an independent empty float feature vector."""
    return np.empty(0, dtype=np.float32)


@dataclass(slots=True)
class VisualAttributes:
    """Section: Human-readable visual attributes and their evidence quality."""

    wearing_glasses: int = ATTRIBUTE_UNKNOWN
    glasses_confidence: float = 0.0
    shirt_colour: str = "unknown"
    shirt_colour_confidence: float = 0.0
    wearing_face_mask: int = ATTRIBUTE_UNKNOWN
    face_mask_confidence: float = 0.0
    hair_colour: str = "unknown"
    hair_colour_confidence: float = 0.0
    overall_quality: float = 0.0


@dataclass(slots=True)
class PersonFeatures:
    """Section: One tracked person's normalized visual identity features."""

    track_id: int = 0
    body_vector: np.ndarray = field(default_factory=_empty_vector)
    upper_histogram: np.ndarray = field(default_factory=_empty_vector)
    lower_histogram: np.ndarray = field(default_factory=_empty_vector)
    object_vector: np.ndarray = field(default_factory=_empty_vector)
    body_quality: float = 0.0
    upper_quality: float = 0.0
    lower_quality: float = 0.0
    object_quality: float = 0.0
    visual_attributes: VisualAttributes = field(default_factory=VisualAttributes)


def normalized(vector: np.ndarray | Iterable[float]) -> np.ndarray:
    """Special function: Return a finite unit vector or an empty vector for invalid input."""
    array = np.asarray(vector, dtype=np.float32).reshape(-1)
    if array.size == 0 or not np.all(np.isfinite(array)):
        return _empty_vector()
    norm = float(np.linalg.norm(array))
    if norm <= 1e-8:
        return _empty_vector()
    return array / norm
