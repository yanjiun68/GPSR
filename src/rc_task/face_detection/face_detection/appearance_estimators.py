"""Section: Reusable ROI, appearance-inference, and temporal-voting estimators."""

from __future__ import annotations

from collections import deque
from typing import Any

import cv2
import numpy as np


def get_upper_body_roi(frame: Any, face_bbox: list[float], person_bbox: list[float] | None = None) -> tuple[Any, Any]:
    """Special function: Derive a torso ROI from a face while respecting a person box."""
    height, width = frame.shape[:2]
    x1, y1, x2, y2 = [int(value) for value in face_bbox]
    face_width = x2 - x1
    face_height = y2 - y1
    shirt_x1 = max(0, x1 - int(0.4 * face_width))
    shirt_x2 = min(width, x2 + int(0.4 * face_width))
    shirt_y1 = min(height, y2 + int(0.15 * face_height))
    shirt_y2 = min(height, y2 + int(1.8 * face_height))
    # Section: The canonical person box prevents pixels from a neighbor entering the ROI.
    if person_bbox is not None and len(person_bbox) == 4:
        person_x1, person_y1, person_x2, person_y2 = [int(value) for value in person_bbox]
        shirt_x1 = max(shirt_x1, person_x1)
        shirt_x2 = min(shirt_x2, person_x2)
        shirt_y1 = max(shirt_y1, person_y1)
        shirt_y2 = min(shirt_y2, person_y2)
    if shirt_x2 <= shirt_x1 or shirt_y2 <= shirt_y1:
        return None, None
    return frame[shirt_y1:shirt_y2, shirt_x1:shirt_x2], (
        shirt_x1, shirt_y1, shirt_x2, shirt_y2,
    )


def get_person_roi(frame: Any, face_bbox: list[float]) -> tuple[Any, Any]:
    """Special function: Crop a conservative face-and-torso region around one face."""
    height, width = frame.shape[:2]
    x1, y1, x2, y2 = [int(value) for value in face_bbox]
    face_width = max(1, x2 - x1)
    face_height = max(1, y2 - y1)
    person_x1 = max(0, x1 - int(0.7 * face_width))
    person_x2 = min(width, x2 + int(0.7 * face_width))
    person_y1 = max(0, y1 - int(0.4 * face_height))
    person_y2 = min(height, y2 + int(2.2 * face_height))
    if person_x2 <= person_x1 or person_y2 <= person_y1:
        return None, None
    return frame[person_y1:person_y2, person_x1:person_x2], (
        person_x1, person_y1, person_x2, person_y2,
    )


def crop_person_roi(frame: Any, person_bbox: list[float]) -> tuple[Any, Any]:
    """Special function: Crop a tracker-provided full-frame xyxy person box."""
    height, width = frame.shape[:2]
    if person_bbox is None or len(person_bbox) != 4:
        return None, None
    x1, y1, x2, y2 = [int(value) for value in person_bbox]
    left = max(0, min(x1, width))
    top = max(0, min(y1, height))
    right = max(0, min(x2, width))
    bottom = max(0, min(y2, height))
    if right <= left or bottom <= top:
        return None, None
    return frame[top:bottom, left:right], (left, top, right, bottom)


def detect_dominant_shirt_color(roi_bgr: Any) -> tuple[str, float]:
    """Special function: Estimate a robust named shirt color from an HSV torso ROI."""
    if roi_bgr is None or roi_bgr.size == 0:
        return "unknown", 0.0
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    # Section: Disjoint red bands and neutral ranges cover the supported GPSR vocabulary.
    color_ranges = {
        "red": [((0, 70, 50), (10, 255, 255)), ((170, 70, 50), (180, 255, 255))],
        "orange": [((11, 70, 50), (20, 255, 255))],
        "yellow": [((21, 70, 50), (35, 255, 255))],
        "green": [((36, 50, 40), (85, 255, 255))],
        "blue": [((86, 50, 40), (130, 255, 255))],
        "purple": [((131, 50, 40), (160, 255, 255))],
        "white": [((0, 0, 180), (180, 50, 255))],
        "black": [((0, 0, 0), (180, 255, 50))],
        "gray": [((0, 0, 51), (180, 50, 179))],
    }
    best_color = "unknown"
    best_count = 0
    for color_name, ranges in color_ranges.items():
        total_count = 0
        for lower, upper in ranges:
            mask = cv2.inRange(
                hsv,
                np.array(lower, dtype=np.uint8),
                np.array(upper, dtype=np.uint8),
            )
            total_count += cv2.countNonZero(mask)
        if total_count > best_count:
            best_count = total_count
            best_color = color_name
    if best_count < 100:
        return "unknown", 0.0
    total_pixels = max(roi_bgr.shape[0] * roi_bgr.shape[1], 1)
    return best_color, min(1.0, float(best_count) / float(total_pixels))


def detect_attributes_with_yolo(
    frame: Any,
    yolo_models: list[Any],
    conf_threshold: float = 0.4,
    device: str = "cpu",
) -> dict[str, float]:
    """Special function: Aggregate positive YOLO classes from one selected person ROI."""
    detected_scores: dict[str, float] = {}
    for yolo_model in yolo_models:
        results = yolo_model.predict(frame, verbose=False, device=device)
        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            for box in boxes:
                confidence = float(box.conf[0])
                if confidence < conf_threshold:
                    continue
                class_id = int(box.cls[0])
                label = str(result.names[class_id]).strip().lower().replace(" ", "_").replace("-", "_")
                detected_scores[label] = max(detected_scores.get(label, 0.0), confidence)
    return detected_scores


def best_attribute_confidence(detected_scores: dict[str, float], aliases: tuple[str, ...]) -> float:
    """Special function: Select the strongest positive detection matching known aliases."""
    best_confidence = 0.0
    for label, confidence in detected_scores.items():
        if any(alias == label or alias in label for alias in aliases):
            best_confidence = max(best_confidence, confidence)
    return best_confidence


class YoloAttributeEstimator:
    """Special function: Reuse configured YOLO models for positive attribute evidence."""

    def __init__(self, models: list[Any], confidence: float, device: str) -> None:
        """Special function: Retain model references and immutable inference settings."""
        self.models = list(models)
        self.confidence = float(confidence)
        self.device = str(device)

    def estimate(self, person_roi: Any) -> dict[str, float]:
        """Special function: Infer normalized positive-class confidence values."""
        if person_roi is None or not self.models:
            return {}
        return detect_attributes_with_yolo(
            person_roi,
            self.models,
            conf_threshold=self.confidence,
            device=self.device,
        )


class HsvShirtColorEstimator:
    """Special function: Reuse the shared HSV shirt-color algorithm."""

    def estimate(self, torso_image: Any) -> tuple[str, float]:
        """Special function: Return the strongest supported color and confidence."""
        return detect_dominant_shirt_color(torso_image)


class TemporalCategoricalVoter:
    """Special function: Stabilize one categorical estimate without cross-person state."""

    def __init__(self, history_size: int, minimum_votes: int, minimum_confidence: float = 0.0) -> None:
        """Special function: Configure the bounded confidence-weighted history."""
        self.minimum_votes = max(1, int(minimum_votes))
        self.minimum_confidence = max(0.0, float(minimum_confidence))
        self.history: deque[tuple[str, float]] = deque(maxlen=max(1, int(history_size)))

    def update(self, value: str, confidence: float) -> tuple[str, float]:
        """Special function: Add meaningful evidence and return a confirmed vote."""
        normalized = str(value or "").strip().casefold()
        if normalized and normalized != "unknown" and float(confidence) >= self.minimum_confidence:
            self.history.append((normalized, float(confidence)))
        scores: dict[str, float] = {}
        counts: dict[str, int] = {}
        for observed_value, observed_confidence in self.history:
            scores[observed_value] = scores.get(observed_value, 0.0) + observed_confidence
            counts[observed_value] = counts.get(observed_value, 0) + 1
        if not scores:
            return "unknown", 0.0
        selected = max(scores, key=lambda key: (scores[key], counts[key], key))
        if counts[selected] < self.minimum_votes:
            return "unknown", 0.0
        return selected, scores[selected] / float(counts[selected])

    def clear(self) -> None:
        """Special function: Remove all temporal evidence during lifecycle cleanup."""
        self.history.clear()
