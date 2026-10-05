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
