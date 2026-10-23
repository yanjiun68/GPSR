"""Regression checks for ER-Flow ROI validation and lightweight OpenCV tracking."""

# Section: Import the installed ROS interface first, then load the executable source as a test module.
import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest


def load_er_flow_module():
    """Special function: Load helpers from the installed executable without starting a ROS node."""
    # Section: Resolve the script relative to this test so source and installed test runs use identical code.
    source_path = Path(__file__).resolve().parents[1] / "scripts" / "astra_cam_qw.py"
    spec = importlib.util.spec_from_file_location("astra_cam_qw_under_test", source_path)
    module = importlib.util.module_from_spec(spec)
    # Section: Dataclass annotation resolution requires the temporary module to be registered before execution.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_found_alias_becomes_valid_ok_status():
    """Special function: Preserve compatibility while enforcing the canonical public status vocabulary."""
    # Section: A valid normalized box must scale deterministically into source-image pixels.
    module = load_er_flow_module()
    result = module.normalize_qwen_roi(
        {
            "status": "found",
            "bbox_2d": [100, 200, 300, 500],
            "coordinate_width": 1000,
            "coordinate_height": 1000,
            "confidence": 0.90,
        },
        module.TargetRequest("cup"),
        200,
        100,
        0.35,
    )
    assert result["status"] == "ok"
    assert result["bbox_pixels"] == [20, 20, 60, 50]


def test_er_flow_found_bbox_uses_source_image_pixels():
    """Special function: Parse the canonical UnifoLM-ER-Flow found/bbox contract."""
    module = load_er_flow_module()
    result = module.normalize_qwen_roi(
        {
            "found": True,
            "bbox": [20, 10, 80, 60],
            "confidence": 0.90,
        },
        module.TargetRequest("cup"),
        100,
        80,
        0.35,
    )
    assert result["status"] == "ok"
    assert result["bbox_2d"] == [20.0, 10.0, 80.0, 60.0]
    assert result["coordinate_width"] == 100
    assert result["coordinate_height"] == 80
    assert result["bbox_pixels"] == [20, 10, 80, 60]


def test_er_flow_not_found_is_valid_without_bbox():
    """Special function: Accept ER-Flow's explicit negative grounding result."""
    module = load_er_flow_module()
    result = module.normalize_qwen_roi(
        {"found": False, "bbox": None, "confidence": 0.0},
        module.TargetRequest("cup"),
        100,
        80,
        0.35,
    )
    assert result["status"] == "not_found"
    assert result["bbox_pixels"] is None


def test_point_anchor_can_seed_a_small_compatibility_roi():
    """Special function: Reduce model-box dependence while preserving the ROS ROI interface."""
    module = load_er_flow_module()
    result = module.normalize_qwen_roi(
        {
            "found": True,
            "point_2d": [50, 40],
            "confidence": 0.90,
        },
        module.TargetRequest("cup"),
        100,
        80,
        0.35,
        20,
        16,
    )
    assert result["status"] == "ok"
    assert result["roi_source"] == "point"
    assert result["bbox_pixels"] == [40, 32, 60, 48]


def test_malformed_negative_response_is_rejected():
    """Malformed negative output is an error, not evidence that a target is absent."""
    module = load_er_flow_module()
    with pytest.raises(ValueError, match="invalid JSON"):
        module.extract_json_object('{"found":false,"bbox":[[518,6,534,100],"confidence":0.90}')


def test_model_contract_rejects_contradictory_and_out_of_bounds_outputs():
    module = load_er_flow_module()
    validate = lambda data: module.validate_model_output(data, 640, 480)
    validate({"found": False, "bbox": None, "point_2d": None, "confidence": 0.0})
    validate({"found": True, "bbox": None, "point_2d": [40, 30], "confidence": 0.9})
    with pytest.raises(ValueError, match="missing target"):
        validate({"found": False, "bbox": [1, 2, 3, 4], "point_2d": None, "confidence": 0.9})
    with pytest.raises(ValueError, match="within"):
        validate({"found": True, "bbox": [402, 20, 466, 887], "point_2d": None, "confidence": 0.9})
    with pytest.raises(ValueError, match="point_2d"):
        validate({"found": True, "bbox": [1, 2, 30, 40], "point_2d": [1, 480], "confidence": 0.9})


def test_invalid_model_output_retries_once_and_fails_visibly():
    module = load_er_flow_module()
    class Logger:
        def __init__(self):
            self.messages = []
        def warning(self, message):
            self.messages.append(message)
    logger = Logger()
    prompts = []
    def generate(correction):
        prompts.append(correction)
        return '{"found":false,"bbox":[[1,2,3,4],"confidence":0.9}'
    with pytest.raises(ValueError, match="after two attempts"):
        module.generate_validated_output(generate, lambda data: module.validate_model_output(data, 640, 480), logger)
    assert len(prompts) == len(logger.messages) == 2
    assert prompts[0] == "" and "previous response was invalid" in prompts[1]


def test_invalid_model_output_can_recover_on_retry():
    module = load_er_flow_module()
    responses = iter(['{}', '{"found":false,"bbox":null,"point_2d":null,"confidence":0.0}'])
    class Logger:
        def warning(self, message):
            pass
    result = module.generate_validated_output(
        lambda correction: next(responses),
        lambda data: module.validate_model_output(data, 640, 480), Logger(),
    )
    assert result["found"] is False


def test_prompt_includes_source_image_bounds():
    module = load_er_flow_module()
    prompt = module.AstraCamQwenNode._build_roi_prompt(module.TargetRequest("key"), 640, 480)
    assert "640 pixels wide and 480 pixels high" in prompt
    assert "right <= 640" in prompt and "bottom <= 480" in prompt


def test_invalid_confidence_is_rejected():
    """Special function: Prevent NaN or out-of-range confidence from reaching downstream consumers."""
    # Section: Non-finite confidence must fail closed instead of becoming a valid tracked ROI.
    module = load_er_flow_module()
    with pytest.raises(ValueError, match="confidence"):
        module.normalize_qwen_roi(
            {"status": "ok", "bbox_2d": [1, 1, 10, 10], "confidence": float("nan")},
            module.TargetRequest("cup"),
            100,
            100,
            0.35,
        )


def test_opencv_tracker_follows_translation():
    """Special function: Verify that 20 Hz updates can move the ROI without another model request."""
    # Section: A deterministic textured patch avoids false matches in an otherwise empty image.
    module = load_er_flow_module()
    first = np.zeros((120, 160, 3), dtype=np.uint8)
    pattern = np.arange(30 * 30, dtype=np.uint8).reshape(30, 30)
    first[30:60, 40:70, :] = pattern[:, :, None]
    second = np.zeros_like(first)
    second[36:66, 49:79, :] = pattern[:, :, None]
    tracker = module.OpenCvTemplateTracker(search_margin=0.75, minimum_score=0.50)
    tracker.initialize(first, (40, 30, 70, 60))
    tracked = tracker.update(second)
    assert tracked is not None
    assert tracked[0] == (49, 36, 79, 66)
    assert tracked[1] >= 0.99
