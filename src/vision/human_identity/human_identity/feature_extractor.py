"""Section: Pose-guided visual feature and human-attribute extraction."""

from __future__ import annotations

from collections import Counter
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np

from .feature_types import (
    ATTRIBUTE_FALSE,
    ATTRIBUTE_TRUE,
    ATTRIBUTE_UNKNOWN,
    PersonFeatures,
    VisualAttributes,
    normalized,
)


# Section: MediaPipe imports remain optional so unit tests and degraded operation stay available.
try:
    import mediapipe as mp
    from mediapipe.tasks import python
    from mediapipe.tasks.python import vision as mp_vision
except ImportError:
    mp = None
    python = None
    mp_vision = None


# Section: Landmark groups used to build pose-guided image regions.
HEAD_LANDMARKS = tuple(range(0, 11))
TORSO_LANDMARKS = (11, 12, 23, 24)
LOWER_BODY_LANDMARKS = (23, 24, 25, 26, 27, 28)


class FeatureExtractor:
    """Section: Extract identity vectors and interpretable appearance attributes."""

    def __init__(
        self,
        attribute_model: Any = None,
        device: str = "cpu",
        pose_model_path: str | None = None,
    ) -> None:
        """Special function: Initialize attribute and MediaPipe Full pose models."""
        self.attribute_model = attribute_model
        self.device = device
        self.pose_landmarker = self._load_pose_landmarker(pose_model_path)

    @staticmethod
    def _load_pose_landmarker(pose_model_path: str | None) -> Any | None:
        """Special function: Load the Full pose model in image mode when available."""
        model_available = pose_model_path and Path(pose_model_path).is_file()
        if python is None or mp_vision is None or not model_available:
            return None
        base_options = python.BaseOptions(model_asset_path=pose_model_path)
        options = mp_vision.PoseLandmarkerOptions(
            base_options=base_options,
            running_mode=mp_vision.RunningMode.IMAGE,
            num_poses=1,
            min_pose_detection_confidence=0.4,
            min_pose_presence_confidence=0.4,
            min_tracking_confidence=0.4,
            output_segmentation_masks=False,
        )
        return mp_vision.PoseLandmarker.create_from_options(options)

    def extract(
        self,
        frame: np.ndarray,
        person_box: tuple[int, int, int, int],
        track_id: int,
    ) -> PersonFeatures:
        """Special function: Extract a person's pose, clothing, and attributes."""
        x1, y1, x2, y2 = self._clip_box(person_box, frame.shape[1], frame.shape[0])
        person_crop = frame[y1:y2, x1:x2]
        if person_crop.size == 0:
            return PersonFeatures(track_id=track_id)
        landmarks = self._detect_pose(person_crop)
        body_vector, body_quality = self._body_feature(landmarks)
        head_box, torso_box, lower_box = self._guided_regions(
            landmarks,
            person_crop.shape[1],
            person_crop.shape[0],
        )
        head_crop = self._crop(person_crop, head_box)
        torso_crop = self._crop(person_crop, torso_box)
        lower_crop = self._crop(person_crop, lower_box)
        upper_histogram, upper_quality = self._colour_histogram(torso_crop)
        lower_histogram, lower_quality = self._colour_histogram(lower_crop)
        shirt_colour, shirt_confidence = self._dominant_colour(torso_crop)
        hair_crop = head_crop[: max(1, head_crop.shape[0] // 3)] if head_crop.size else head_crop
        hair_colour, hair_confidence = self._dominant_colour(hair_crop)
        attribute_states = self._attribute_states(head_crop)
        glasses_state, glasses_confidence, mask_state, mask_confidence = attribute_states
        object_vector = np.asarray(
            [
                float(glasses_state == ATTRIBUTE_FALSE),
                float(glasses_state == ATTRIBUTE_TRUE),
                float(mask_state == ATTRIBUTE_FALSE),
                float(mask_state == ATTRIBUTE_TRUE),
            ],
            dtype=np.float32,
        )
        known_attributes = sum(
            state != ATTRIBUTE_UNKNOWN
            for state in (glasses_state, mask_state)
        )
        object_quality = known_attributes / 2.0
        visual_quality = float(np.mean([shirt_confidence, hair_confidence, object_quality]))
        visual = VisualAttributes(
            wearing_glasses=glasses_state,
            glasses_confidence=glasses_confidence,
            shirt_colour=shirt_colour,
            shirt_colour_confidence=shirt_confidence,
            wearing_face_mask=mask_state,
            face_mask_confidence=mask_confidence,
            hair_colour=hair_colour,
            hair_colour_confidence=hair_confidence,
            overall_quality=visual_quality,
        )
        return PersonFeatures(
            track_id=track_id,
            body_vector=body_vector,
            upper_histogram=upper_histogram,
            lower_histogram=lower_histogram,
            object_vector=(
                normalized(object_vector)
                if known_attributes
                else np.empty(0, dtype=np.float32)
            ),
            body_quality=body_quality,
            upper_quality=upper_quality,
            lower_quality=lower_quality,
            object_quality=object_quality,
            visual_attributes=visual,
        )

    def aggregate(self, samples: Iterable[PersonFeatures]) -> PersonFeatures:
        """Special function: Average compatible registration samples into one stable prototype."""
        collected = list(samples)
        if not collected:
            raise ValueError("At least one feature sample is required")
        return PersonFeatures(
            track_id=collected[-1].track_id,
            body_vector=self._average_vectors(item.body_vector for item in collected),
            upper_histogram=self._average_vectors(item.upper_histogram for item in collected),
            lower_histogram=self._average_vectors(item.lower_histogram for item in collected),
            object_vector=self._average_vectors(item.object_vector for item in collected),
            body_quality=float(np.mean([item.body_quality for item in collected])),
            upper_quality=float(np.mean([item.upper_quality for item in collected])),
            lower_quality=float(np.mean([item.lower_quality for item in collected])),
            object_quality=float(np.mean([item.object_quality for item in collected])),
            visual_attributes=self._aggregate_attributes(
                item.visual_attributes for item in collected
            ),
        )

    def _detect_pose(self, person_crop: np.ndarray) -> list[Any]:
        """Special function: Run MediaPipe on the person crop and return one landmark list."""
        if self.pose_landmarker is None or mp is None:
            return []
        rgb_crop = cv2.cvtColor(person_crop, cv2.COLOR_BGR2RGB)
        media_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=np.ascontiguousarray(rgb_crop),
        )
        result = self.pose_landmarker.detect(media_image)
        return list(result.pose_landmarks[0]) if result.pose_landmarks else []

    @staticmethod
    def _body_feature(landmarks: list[Any]) -> tuple[np.ndarray, float]:
        """Special function: Normalize visible skeleton geometry around the hip midpoint."""
        if len(landmarks) < 29:
            return np.empty(0, dtype=np.float32), 0.0
        coordinates = np.asarray(
            [[item.x, item.y, item.z] for item in landmarks],
            dtype=np.float32,
        )
        confidence = np.asarray(
            [
                min(
                    float(getattr(item, "visibility", 0.0)),
                    float(getattr(item, "presence", 1.0)),
                )
                for item in landmarks
            ],
            dtype=np.float32,
        )
        center = (coordinates[23] + coordinates[24]) * 0.5
        shoulder_width = float(
            np.linalg.norm(coordinates[11, :2] - coordinates[12, :2])
        )
        shoulder_center = ((coordinates[11] + coordinates[12]) * 0.5)[:2]
        torso_height = float(np.linalg.norm(shoulder_center - center[:2]))
        scale = max(shoulder_width, torso_height, 1e-4)
        vector = ((coordinates - center) / scale).reshape(-1)
        vector = np.concatenate([vector, confidence])
        quality = float(np.clip(np.mean(confidence[TORSO_LANDMARKS]), 0.0, 1.0))
        return normalized(vector), quality

    @staticmethod
    def _guided_regions(
        landmarks: list[Any],
        width: int,
        height: int,
    ) -> tuple[tuple[int, int, int, int], ...]:
        """Special function: Derive head, torso, and lower-body crops from pose landmarks."""
        if len(landmarks) < 29:
            return (
                (int(width * 0.25), 0, int(width * 0.75), int(height * 0.28)),
                (
                    int(width * 0.15),
                    int(height * 0.18),
                    int(width * 0.85),
                    int(height * 0.58),
                ),
                (
                    int(width * 0.15),
                    int(height * 0.50),
                    int(width * 0.85),
                    int(height * 0.96),
                ),
            )
        head = FeatureExtractor._landmark_box(
            landmarks, HEAD_LANDMARKS, width, height, 0.25
        )
        torso = FeatureExtractor._landmark_box(
            landmarks, TORSO_LANDMARKS, width, height, 0.15
        )
        lower = FeatureExtractor._landmark_box(
            landmarks, LOWER_BODY_LANDMARKS, width, height, 0.10
        )
        return head, torso, lower

    @staticmethod
    def _landmark_box(
        landmarks: list[Any],
        indices: tuple[int, ...],
        width: int,
        height: int,
        padding: float,
    ) -> tuple[int, int, int, int]:
        """Special function: Convert a landmark group into a padded pixel rectangle."""
        points = np.asarray(
            [
                [landmarks[index].x * width, landmarks[index].y * height]
                for index in indices
            ]
        )
        minimum = points.min(axis=0)
        maximum = points.max(axis=0)
        minimum_extent = np.asarray([width * 0.08, height * 0.08])
        extent = np.maximum(maximum - minimum, minimum_extent)
        minimum -= extent * padding
        maximum += extent * padding
        pixel_box = (*minimum.astype(int), *maximum.astype(int))
        return FeatureExtractor._clip_box(pixel_box, width, height)

    @staticmethod
    def _clip_box(
        box: tuple[int, int, int, int],
        width: int,
        height: int,
    ) -> tuple[int, int, int, int]:
        """Special function: Clip a rectangle to valid image coordinates."""
        x1, y1, x2, y2 = box
        return (
            max(0, min(x1, width)),
            max(0, min(y1, height)),
            max(0, min(x2, width)),
            max(0, min(y2, height)),
        )

    @staticmethod
    def _crop(image: np.ndarray, box: tuple[int, int, int, int]) -> np.ndarray:
        """Special function: Slice a clipped region from an image."""
        x1, y1, x2, y2 = box
        return image[y1:y2, x1:x2]

    @staticmethod
    def _colour_histogram(crop: np.ndarray) -> tuple[np.ndarray, float]:
        """Special function: Build a normalized HSV clothing histogram and area quality."""
        if crop.size == 0:
            return np.empty(0, dtype=np.float32), 0.0
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        histogram = cv2.calcHist([hsv], [0, 1], None, [16, 4], [0, 180, 0, 256]).reshape(-1)
        area_quality = np.clip(crop.shape[0] * crop.shape[1] / 20000.0, 0.0, 1.0)
        return normalized(histogram), float(area_quality)

    @staticmethod
    def _dominant_colour(crop: np.ndarray) -> tuple[str, float]:
        """Special function: Estimate a named dominant colour from robust HSV pixel categories."""
        if crop.size == 0:
            return "unknown", 0.0
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV).reshape(-1, 3)
        if hsv.shape[0] > 4000:
            hsv = hsv[:: max(1, hsv.shape[0] // 4000)]
        labels = [FeatureExtractor._name_hsv(int(h), int(s), int(v)) for h, s, v in hsv]
        counts = Counter(labels)
        label, count = counts.most_common(1)[0]
        return label, float(count / max(1, len(labels)))

    @staticmethod
    def _name_hsv(hue: int, saturation: int, value: int) -> str:
        """Special function: Map one OpenCV HSV pixel into a compact colour vocabulary."""
        if value < 45:
            return "black"
        if saturation < 35 and value > 200:
            return "white"
        if saturation < 45:
            return "grey"
        if hue < 7 or hue >= 170:
            return "red"
        if hue < 18:
            return "orange"
        if hue < 35:
            return "yellow"
        if hue < 85:
            return "green"
        if hue < 130:
            return "blue"
        if hue < 155:
            return "purple"
        return "pink"

    def _attribute_states(self, head_crop: np.ndarray) -> tuple[int, float, int, float]:
        """Special function: Keep attributes unknown without explicit model evidence."""
        if self.attribute_model is None or head_crop.size == 0:
            return ATTRIBUTE_UNKNOWN, 0.0, ATTRIBUTE_UNKNOWN, 0.0
        results = self.attribute_model.predict(head_crop, verbose=False, device=self.device)
        labels = self._result_labels(results)
        glasses = self._resolve_binary_attribute(
            labels,
            ("glasses", "eyeglasses", "sunglasses"),
            ("no_glasses", "without_glasses"),
        )
        mask = self._resolve_binary_attribute(
            labels,
            ("face_mask", "mask"),
            ("no_mask", "without_mask"),
        )
        return glasses[0], glasses[1], mask[0], mask[1]

    @staticmethod
    def _result_labels(results: Any) -> list[tuple[str, float]]:
        """Special function: Normalize Ultralytics classification or detection output labels."""
        if not results:
            return []
        result = results[0]
        names = result.names
        if getattr(result, "probs", None) is not None:
            index = int(result.probs.top1)
            label = str(names[index]).lower().replace(" ", "_")
            return [(label, float(result.probs.top1conf))]
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return []
        return [
            (str(names[int(class_id)]).lower().replace(" ", "_"), float(confidence))
            for class_id, confidence in zip(
                boxes.cls.cpu().tolist(),
                boxes.conf.cpu().tolist(),
            )
        ]

    @staticmethod
    def _resolve_binary_attribute(
        labels: list[tuple[str, float]],
        positive_names: tuple[str, ...],
        negative_names: tuple[str, ...],
    ) -> tuple[int, float]:
        """Special function: Resolve only explicit positive or negative model classes."""
        candidates: list[tuple[int, float]] = []
        for label, confidence in labels:
            if any(name in label for name in negative_names):
                candidates.append((ATTRIBUTE_FALSE, confidence))
            elif any(name in label for name in positive_names):
                candidates.append((ATTRIBUTE_TRUE, confidence))
        if not candidates:
            return ATTRIBUTE_UNKNOWN, 0.0
        return max(candidates, key=lambda item: item[1])

    @staticmethod
    def _average_vectors(vectors: Iterable[np.ndarray]) -> np.ndarray:
        """Special function: Average only vectors sharing the most common nonempty shape."""
        available = [
            np.asarray(item, dtype=np.float32).reshape(-1)
            for item in vectors
            if np.asarray(item).size
        ]
        if not available:
            return np.empty(0, dtype=np.float32)
        common_size = Counter(item.size for item in available).most_common(1)[0][0]
        compatible = [item for item in available if item.size == common_size]
        return normalized(np.mean(compatible, axis=0))

    @staticmethod
    def _aggregate_attributes(attributes: Iterable[VisualAttributes]) -> VisualAttributes:
        """Special function: Aggregate states by confidence and colours by weighted voting."""
        values = list(attributes)
        glasses = FeatureExtractor._best_state(values, "wearing_glasses", "glasses_confidence")
        mask = FeatureExtractor._best_state(values, "wearing_face_mask", "face_mask_confidence")
        shirt = FeatureExtractor._best_label(values, "shirt_colour", "shirt_colour_confidence")
        hair = FeatureExtractor._best_label(values, "hair_colour", "hair_colour_confidence")
        return VisualAttributes(
            wearing_glasses=glasses[0], glasses_confidence=glasses[1],
            shirt_colour=shirt[0], shirt_colour_confidence=shirt[1],
            wearing_face_mask=mask[0], face_mask_confidence=mask[1],
            hair_colour=hair[0], hair_colour_confidence=hair[1],
            overall_quality=float(np.mean([item.overall_quality for item in values])),
        )

    @staticmethod
    def _best_state(
        values: list[VisualAttributes],
        state_name: str,
        confidence_name: str,
    ) -> tuple[int, float]:
        """Special function: Select the highest-confidence known binary attribute sample."""
        candidates = [
            (getattr(item, state_name), getattr(item, confidence_name))
            for item in values
            if getattr(item, state_name) != ATTRIBUTE_UNKNOWN
        ]
        if not candidates:
            return ATTRIBUTE_UNKNOWN, 0.0
        return max(candidates, key=lambda item: item[1])

    @staticmethod
    def _best_label(
        values: list[VisualAttributes],
        label_name: str,
        confidence_name: str,
    ) -> tuple[str, float]:
        """Special function: Select the label with the largest accumulated confidence."""
        totals: dict[str, float] = {}
        for item in values:
            label = str(getattr(item, label_name))
            if label != "unknown":
                totals[label] = totals.get(label, 0.0) + float(getattr(item, confidence_name))
        if not totals:
            return "unknown", 0.0
        label = max(totals, key=totals.get)
        confidences = [
            float(getattr(item, confidence_name))
            for item in values
            if getattr(item, label_name) == label
        ]
        return label, float(np.mean(confidences))
