"""Section: GPSR object detector CUDA behavior tests."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest


def load_gpsr_object_detector_module():
    """Special function: Load the GPSR object detector script as a testable Python module."""
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "gpsr_object_detection_node.py"
    spec = importlib.util.spec_from_file_location("gpsr_object_detection_node", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class InvalidCudaModel:
    """Special function: Fake YOLO model that rejects CUDA inference."""

    def predict(self, **_kwargs):
        """Special function: Raise the same class of error produced by invalid CUDA selection."""
        raise ValueError("Invalid CUDA 'device=0' requested")


class OpenVocabularyModel:
    """Special function: Fake YOLOE model that records configured text prompts."""

    def __init__(self) -> None:
        """Special function: Start with the numeric names exposed by an unconfigured YOLOE model."""
        self.names = {0: "0", 1: "1"}
        self.configured_classes = []

    def set_classes(self, classes) -> None:
        """Special function: Mirror YOLOE class-name replacement for a deterministic unit test."""
        self.configured_classes = list(classes)
        self.names = dict(enumerate(classes))


class WarmupModel:
    """Special function: Record the synthetic frame used to initialize inference."""

    def __init__(self) -> None:
        """Special function: Initialize an empty prediction call history."""
        self.calls = []

    def predict(self, **kwargs):
        """Special function: Capture one warm-up inference request."""
        self.calls.append(kwargs)
        return []


class RecordingLogger:
    """Special function: Capture informational detector messages without constructing a ROS node."""

    def __init__(self) -> None:
        """Special function: Initialize an empty message list."""
        self.messages = []

    def info(self, message: str) -> None:
        """Special function: Store one informational message."""
        self.messages.append(message)


def test_resolve_yolo_device_requires_cuda_availability() -> None:
    """Special function: CUDA device selection fails instead of falling back to CPU."""
    module = load_gpsr_object_detector_module()
    detector = object.__new__(module.GpsrObjectDetectionNode)
    detector.cuda_is_available = lambda: False
    with pytest.raises(RuntimeError, match="CPU fallback is disabled"):
        module.GpsrObjectDetectionNode.resolve_yolo_device(detector, "cuda")


def test_invalid_cuda_predict_does_not_retry_on_cpu() -> None:
    """Special function: YOLO invalid-CUDA errors are propagated without CPU retry."""
    module = load_gpsr_object_detector_module()
    detector = object.__new__(module.GpsrObjectDetectionNode)
    detector.model = InvalidCudaModel()
    detector.yolo_device = "cuda"
    detector.image_size = 640
    detector.confidence_threshold = 0.35
    detector.requires_cuda_device = lambda _device: True
    with pytest.raises(RuntimeError, match="CPU fallback is disabled"):
        module.GpsrObjectDetectionNode.detect_objects(detector, object())


def test_yoloe_receives_named_object_prompts_before_inference() -> None:
    """Special function: Replace numeric YOLOE labels so named GPSR matching and boxes work."""
    module = load_gpsr_object_detector_module()
    detector = object.__new__(module.GpsrObjectDetectionNode)
    detector.model = OpenVocabularyModel()
    detector.include_classes = set()
    detector.exclude_classes = {"person"}
    detector.yolo_class_prompts = ["person", "bottle", "cup"]
    logger = RecordingLogger()
    detector.get_logger = lambda: logger

    detector.configure_open_vocabulary_classes()

    assert detector.model.configured_classes == ["bottle", "cup"]
    assert detector.model.names == {0: "bottle", 1: "cup"}
    assert logger.messages == ["Configured open-vocabulary YOLO with 2 object prompts."]


def test_model_warmup_initializes_backend_without_camera_evidence() -> None:
    """Special function: Configuration warms CUDA without publishing a detection frame."""
    module = load_gpsr_object_detector_module()
    detector = object.__new__(module.GpsrObjectDetectionNode)
    detector.model = WarmupModel()
    detector.image_size = 320
    detector.confidence_threshold = 0.35
    detector.yolo_device = "cuda"
    logger = RecordingLogger()
    detector.get_logger = lambda: logger

    detector.warm_up_model()

    assert len(detector.model.calls) == 1
    assert detector.model.calls[0]["source"].shape == (320, 320, 3)
    assert detector.model.calls[0]["device"] == "cuda"
    assert logger.messages == ["YOLO inference backend warm-up completed."]
