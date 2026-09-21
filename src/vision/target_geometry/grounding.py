"""Strict machine-readable Qwen grounding validation for target-object perception."""

from __future__ import annotations

# Section: JSON validation and immutable records isolate model output from executable code.
from dataclasses import dataclass
import json
import math
import re
from typing import Any


@dataclass(frozen=True)
class GroundedTarget:
    """Store one validated semantic target in source-image pixel coordinates."""

    label: str
    confidence: float
    bbox_xyxy: tuple[int, int, int, int]
    point_uv: tuple[float, float]


class GroundingError(ValueError):
    """Identify invalid or semantically mismatched grounding."""


# Section: Spatial and descriptive words do not by themselves define the requested object class.
_SEMANTIC_STOPWORDS = {
    "a", "an", "the", "on", "at", "in", "to", "of", "object", "item",
    "left", "right", "middle", "center", "front", "back", "near", "far",
    "red", "green", "blue", "yellow", "black", "white", "orange", "purple",
    "small", "large", "big", "tall", "short",
}


def _semantic_tokens(text: str) -> set[str]:
    """Extract conservative object-class tokens from a semantic label."""
    # Section: Simple alphanumeric tokens avoid executing or interpreting arbitrary model syntax.
    return {
        token
        for token in re.findall(r"[a-z0-9]+", str(text).casefold())
        if token not in _SEMANTIC_STOPWORDS
    }


def _semantic_match(requested_target: str, returned_label: str) -> bool:
    """Require object-class overlap between the request and Qwen selection."""
    # Section: Exact normalized containment supports labels such as coke can versus coke.
    request_normalized = " ".join(str(requested_target).casefold().split())
    label_normalized = " ".join(str(returned_label).casefold().split())
    if request_normalized in label_normalized or label_normalized in request_normalized:
        return True
    request_tokens = _semantic_tokens(request_normalized)
    label_tokens = _semantic_tokens(label_normalized)
    return bool(request_tokens and label_tokens and request_tokens.intersection(label_tokens))


def _as_finite_number(value: Any, field_name: str) -> float:
    """Convert a model scalar while rejecting invalid values."""
    # Section: JSON booleans are Python integers but are never valid image coordinates.
    if isinstance(value, bool):
        raise GroundingError(f"{field_name} must be numeric")
    try:
        converted = float(value)
    except (TypeError, ValueError) as exc:
        raise GroundingError(f"{field_name} must be numeric") from exc
    if not math.isfinite(converted):
        raise GroundingError(f"{field_name} must be finite")
    return converted


def _select_instance(
    instances: list[dict[str, Any]],
    requested_target: str,
    minimum_confidence_margin: float,
) -> dict[str, Any]:
    """Resolve explicit left/right language or choose the strongest candidate."""
    # Section: Candidates without a usable box cannot participate in spatial disambiguation.
    usable = [item for item in instances if isinstance(item, dict)]
    if not usable:
        raise GroundingError("Qwen returned no usable target instances")

    def center_x(item: dict[str, Any]) -> float:
        """Return a sortable horizontal center for one candidate."""
        bbox = item.get("bbox_2d")
        if not isinstance(bbox, list) or len(bbox) != 4:
            return math.inf
        return 0.5 * (
            _as_finite_number(bbox[0], "bbox_2d[0]")
            + _as_finite_number(bbox[2], "bbox_2d[2]")
        )

    # Section: Explicit spatial qualifiers select deterministically among model-returned instances.
    normalized_request = str(requested_target).casefold()
    if "left" in normalized_request:
        return min(usable, key=center_x)
    if "right" in normalized_request:
        return max(
            usable,
            key=lambda item: -math.inf if math.isinf(center_x(item)) else center_x(item),
        )
    # Section: A non-spatial request must not silently guess between equally likely instances.
    ranked = sorted(
        usable,
        key=lambda item: _as_finite_number(item.get("confidence", 0.0), "confidence"),
        reverse=True,
    )
    if len(ranked) > 1:
        best_confidence = _as_finite_number(ranked[0].get("confidence"), "confidence")
        next_confidence = _as_finite_number(ranked[1].get("confidence"), "confidence")
        if best_confidence - next_confidence < float(minimum_confidence_margin):
            raise GroundingError("requested target is ambiguous between multiple instances")
    return ranked[0]


def parse_qwen_grounding(
    raw_response: str,
    *,
    requested_target: str,
    image_width: int,
    image_height: int,
    minimum_confidence: float,
    minimum_bbox_fraction: float,
    maximum_bbox_fraction: float,
    minimum_instance_confidence_margin: float = 0.05,
) -> GroundedTarget:
    """Validate Qwen grounding and convert it to source pixels."""
    # Section: Strict JSON parsing keeps Markdown or generated code out of perception.
    text = str(raw_response or "").strip()
    if not text:
        raise GroundingError("Qwen returned an empty grounding response")
    if text.startswith("```"):
        raise GroundingError("Qwen returned Markdown instead of JSON")
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as exc:
        raise GroundingError(f"Qwen returned invalid JSON: {exc.msg}") from exc
    if not isinstance(payload, dict):
        raise GroundingError("Qwen grounding response must be an object")

    # Section: Support the existing schema and the requested found/instances schema.
    if "selected_entity" in payload:
        selected = payload.get("selected_entity")
        if selected is None:
            raise GroundingError("requested target is not visible")
        if not isinstance(selected, dict):
            raise GroundingError("selected_entity must be an object or null")
    else:
        found = payload.get("found")
        if found is not True:
            raise GroundingError("requested target is not visible")
        if isinstance(payload.get("instances"), list):
            selected = _select_instance(
                payload["instances"],
                requested_target,
                minimum_instance_confidence_margin,
            )
        else:
            selected = payload

    # Section: The semantic label must still describe the requested physical object class.
    label = str(selected.get("label") or selected.get("target_id") or "").strip()
    if not label:
        raise GroundingError("grounded target label is empty")
    if not _semantic_match(requested_target, label):
        raise GroundingError(
            f"grounded label '{label}' does not match requested target '{requested_target}'"
        )
    confidence = _as_finite_number(selected.get("confidence"), "confidence")
    if confidence < float(minimum_confidence) or confidence > 1.0:
        raise GroundingError(
            f"grounding confidence is outside the accepted range: {confidence:.3f}"
        )

    # Section: Coordinate dimensions are explicit; absent dimensions mean native source pixels.
    coordinate_width = int(
        selected.get("coordinate_width", payload.get("coordinate_width", image_width))
    )
    coordinate_height = int(
        selected.get("coordinate_height", payload.get("coordinate_height", image_height))
    )
    if coordinate_width <= 0 or coordinate_height <= 0 or image_width <= 0 or image_height <= 0:
        raise GroundingError("grounding and image dimensions must be positive")
    bbox = selected.get("bbox_2d")
    if not isinstance(bbox, list) or len(bbox) != 4:
        raise GroundingError("bbox_2d must contain four numeric coordinates")
    coordinates = [
        _as_finite_number(value, f"bbox_2d[{index}]")
        for index, value in enumerate(bbox)
    ]
    if coordinates[2] <= coordinates[0] or coordinates[3] <= coordinates[1]:
        raise GroundingError("bbox_2d must be strictly ordered")
    if (
        coordinates[0] < 0.0
        or coordinates[1] < 0.0
        or coordinates[2] > coordinate_width
        or coordinates[3] > coordinate_height
    ):
        raise GroundingError("bbox_2d lies outside the declared coordinate dimensions")

    # Section: Proportional conversion uses floor/ceil so the grounded object is not truncated.
    left = int(math.floor(coordinates[0] * image_width / coordinate_width))
    top = int(math.floor(coordinates[1] * image_height / coordinate_height))
    right = int(math.ceil(coordinates[2] * image_width / coordinate_width))
    bottom = int(math.ceil(coordinates[3] * image_height / coordinate_height))
    left = max(0, min(image_width - 1, left))
    top = max(0, min(image_height - 1, top))
    right = max(left + 1, min(image_width, right))
    bottom = max(top + 1, min(image_height, bottom))
    area_fraction = float((right - left) * (bottom - top)) / float(image_width * image_height)
    if area_fraction < float(minimum_bbox_fraction):
        raise GroundingError(f"grounding box is too small: {area_fraction:.6f}")
    if area_fraction > float(maximum_bbox_fraction):
        raise GroundingError(f"grounding box is too large: {area_fraction:.3f}")

    # Section: A missing point falls back only to the validated 2D box center.
    point = selected.get("point_2d")
    if point is None:
        point_u = 0.5 * float(left + right)
        point_v = 0.5 * float(top + bottom)
    else:
        if not isinstance(point, list) or len(point) != 2:
            raise GroundingError("point_2d must contain two numeric coordinates")
        point_x = _as_finite_number(point[0], "point_2d[0]")
        point_y = _as_finite_number(point[1], "point_2d[1]")
        if not (0.0 <= point_x <= coordinate_width and 0.0 <= point_y <= coordinate_height):
            raise GroundingError("point_2d lies outside the declared coordinate dimensions")
        point_u = point_x * image_width / coordinate_width
        point_v = point_y * image_height / coordinate_height
    return GroundedTarget(label, confidence, (left, top, right, bottom), (point_u, point_v))
