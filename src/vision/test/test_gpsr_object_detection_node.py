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
