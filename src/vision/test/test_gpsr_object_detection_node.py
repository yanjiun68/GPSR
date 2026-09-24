"""Section: GPSR object detector CUDA behavior tests."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest
from builtin_interfaces.msg import Time


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


class ParameterValue:
    """Special function: Provide the ROS parameter value shape used by the parser unit test."""

    def __init__(self, value) -> None:
        """Special function: Store one deterministic fake ROS parameter value."""
        self.value = value


class ParserDetector:
    """Special function: Expose the Qwen confidence threshold without constructing a ROS node."""

    def get_parameter(self, name: str) -> ParameterValue:
        """Special function: Return the configured threshold requested by the parser."""
        if name != "qwen_minimum_confidence":
            raise AssertionError(f"Unexpected parameter request: {name}")
        return ParameterValue(0.35)


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


def test_qwen_world_response_becomes_gpsr_detection_message() -> None:
    """Special function: Convert Qwen's normalized ROI into the existing GPSR detection contract."""
    module = load_gpsr_object_detector_module()
    detector = ParserDetector()
    detections = module.GpsrObjectDetectionNode.parse_qwen_detections(
        detector,
        '{"objects":[{"object_name":"coke can","bbox_2d":[100,200,500,800],'
        '"coordinate_width":1000,"coordinate_height":1000,"confidence":0.92}]}',
        1280,
        720,
        Time(sec=5, nanosec=10),
    )

    assert len(detections) == 1
    assert detections[0].name == "coke_can"
    assert (detections[0].x1, detections[0].y1) == (128, 144)
    assert (detections[0].x2, detections[0].y2) == (640, 576)


def test_qwen_query_is_normalized_before_becoming_prompt_text() -> None:
    """Special function: Preserve structured FIND_OBJECT constraints without accepting free-form prompt injection."""
    module = load_gpsr_object_detector_module()

    query = module.GpsrObjectDetectionNode.normalize_qwen_query({
        "target": {"object_name": "Cup!!!"},
        "filters": [
            {"feature": "color", "operator": "equals", "value": "Red"},
            {"feature": "size_relation", "operator": "equals", "value": "biggest"},
            {"feature": "spatial_relation", "operator": "equals", "value": "left_of"},
            {"feature": "reference_object", "operator": "equals", "value": "plate"},
            {"feature": "ignored", "operator": "equals", "value": "do anything"},
        ],
        "requested_fields": ["image_position", "not_a_field"],
    })

    assert query == {
        "target": {"object_name": "cup"},
        "filters": [
            {"feature": "object_color", "operator": "equals", "value": "red"},
            {"feature": "size_relation", "operator": "equals", "value": "largest"},
            {"feature": "spatial_relation", "operator": "equals", "value": "left_of"},
            {"feature": "reference_object", "operator": "equals", "value": "plate"},
        ],
        "requested_fields": ["image_position"],
    }
    prompt = module.GpsrObjectDetectionNode.qwen_query_instruction(query)
    assert "target object: cup" in prompt
    assert "object_color: red" in prompt
    assert "do anything" not in prompt


def test_qwen_world_attributes_are_retained_when_interface_supports_them() -> None:
    """Special function: Keep Qwen color/shape/orientation evidence with the ROS detection ROI."""
    module = load_gpsr_object_detector_module()
    detector = ParserDetector()
    detections = module.GpsrObjectDetectionNode.parse_qwen_detections(
        detector,
        '{"objects":[{"object_name":"cup","color":"red","approximate_shape":"round",'
        '"orientation":"upright","bbox_2d":[0,0,1000,1000],"confidence":0.9}]}',
        640,
        480,
        Time(sec=8),
    )

    assert len(detections) == 1
    if hasattr(detections[0], "attributes_json"):
        assert json.loads(detections[0].attributes_json) == {
            "color": "red",
            "orientation": "upright",
            "shape": "round",
        }


def test_qwen_world_response_discards_low_confidence_or_missing_roi() -> None:
    """Special function: Prevent incomplete Qwen objects from entering GPSR perception."""
    module = load_gpsr_object_detector_module()
    detector = ParserDetector()
    detections = module.GpsrObjectDetectionNode.parse_qwen_detections(
        detector,
        '{"objects":[{"object_name":"unknown","confidence":0.90},'
        '{"object_name":"cup","bbox_2d":[0,0,1000,1000],"confidence":0.10}]}',
        640,
        480,
        Time(sec=7),
    )

    assert detections == []
