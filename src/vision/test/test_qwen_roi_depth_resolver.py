# flake8: noqa: E501
"""Section: Qwen ROI depth resolver geometry and safety regression tests."""

from __future__ import annotations

# Section: Load the installed-style script directly while retaining its pure helper functions.
import importlib.util
import math
import sys
from pathlib import Path

# Section: Synthetic depth and segmentation arrays make sensor behavior deterministic.
import numpy as np
import pytest


def load_resolver_module():
    """Special function: Load the resolver script without starting a ROS node."""
    # Section: Match the import pattern used by the other vision executable tests.
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "qwen_roi_depth_resolver.py"
    spec = importlib.util.spec_from_file_location("qwen_roi_depth_resolver", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_normalized_qwen_roi_scales_into_depth_pixels() -> None:
    """Special function: A 0..1000 model box maps proportionally into a 640x480 depth image."""
    # Section: Verify floor/ceil behavior and the explicit Qwen coordinate-space contract.
    module = load_resolver_module()
    assert module.scale_and_clip_roi((250, 250, 750, 750), (1000, 1000), (640, 480)) == (
        160, 120, 480, 360,
    )


def test_segmentation_mask_excludes_background_depth() -> None:
    """Special function: Mask-supported sampling ignores closer background inside the bounding box."""
    # Section: Create a large ROI whose central segmented instance is exactly two metres away.
    module = load_resolver_module()
    depth = np.full((100, 100), 4.0, dtype=np.float32)
    depth[40:60, 45:55] = 2.0
    mask = np.zeros((100, 100), dtype=bool)
    mask[40:60, 45:55] = True
    sample = module.sample_roi_depth(
        depth,
        (20, 20, 80, 80),
        mask,
        0.2,
        8.0,
        0.9,
        20,
        0.15,
        0.08,
    )
    assert sample.depth_m == 2.0
    assert sample.valid_count == 200
    assert sample.pixel_u == 49.5
    assert sample.pixel_v == 49.5


def test_depth_quality_rejects_sparse_sensor_support() -> None:
    """Special function: A Qwen ROI with almost no valid depth cannot become a navigation target."""
    # Section: Only one valid pixel is intentionally below both configured quality thresholds.
    module = load_resolver_module()
    depth = np.zeros((20, 20), dtype=np.float32)
    depth[10, 10] = 1.0
    try:
        module.sample_roi_depth(depth, (0, 0, 20, 20), None, 0.2, 8.0, 0.25, 10, 0.0, 0.08)
    except ValueError as exc:
        assert "Insufficient target depth support" in str(exc)
        return
    raise AssertionError("Sparse depth must be rejected.")


def test_depth_wait_policy_preserves_legacy_one_shot_goals() -> None:
    """Special function: Keep existing callers on the one-sample resolver path unless they explicitly opt in."""
    # Section: Zero-valued persistent settings are valid only because the opt-in flag is false.
    module = load_resolver_module()
    assert module.validate_depth_wait_policy(False, 0.0, 0.0, 0.0, 0) is None


def test_depth_wait_policy_rejects_unsafe_persistent_inputs() -> None:
    """Special function: Require bounded, positive settings before a goal can wait for a depth threshold."""
    # Section: Each invalid policy field must reject the action goal instead of creating an unbounded wait.
    module = load_resolver_module()
    invalid_policies = (
        (0.0, 0.01, 120.0, 3),
        (0.05, -0.01, 120.0, 3),
        (0.05, 0.01, 0.0, 3),
        (0.05, 0.01, 120.0, 0),
    )
    for target_depth_m, tolerance_m, timeout_sec, confirmations in invalid_policies:
        with pytest.raises(ValueError):
            module.validate_depth_wait_policy(True, target_depth_m, tolerance_m, timeout_sec, confirmations)


def test_depth_wait_progress_requires_three_inclusive_target_samples() -> None:
    """Special function: Complete only after three consecutive samples inside the selected 4-to-6 cm band."""
    # Section: Both tolerance boundaries are inclusive and 5 cm remains the intended nominal success depth.
    module = load_resolver_module()
    policy = module.DepthWaitPolicy(0.05, 0.01, 120.0, 3)
    first = module.advance_depth_wait_progress(0.04, policy, 0, 10.0, 130.0)
    second = module.advance_depth_wait_progress(0.05, policy, first.consecutive_samples, 11.0, 130.0)
    third = module.advance_depth_wait_progress(0.06, policy, second.consecutive_samples, 12.0, 130.0)
    assert (first.phase, first.consecutive_samples, first.terminal_outcome) == (
        "WAITING_FOR_CONFIRMATION", 1, None,
    )
    assert (second.phase, second.consecutive_samples, second.terminal_outcome) == (
        "WAITING_FOR_CONFIRMATION", 2, None,
    )
    assert (third.phase, third.consecutive_samples, third.terminal_outcome) == (
        "TARGET_DEPTH_REACHED", 3, "SUCCESS",
    )


def test_depth_wait_progress_resets_on_distant_or_invalid_samples() -> None:
    """Special function: Prevent non-consecutive or invalid frames from satisfying the target-depth confirmation count."""
    # Section: A too-distant sample and an absent sample both reset the in-band confirmation sequence.
    module = load_resolver_module()
    policy = module.DepthWaitPolicy(0.05, 0.01, 120.0, 3)
    distant = module.advance_depth_wait_progress(0.07, policy, 2, 10.0, 130.0)
    invalid = module.advance_depth_wait_progress(None, policy, 2, 11.0, 130.0)
    assert (distant.phase, distant.consecutive_samples, distant.terminal_outcome) == (
        "WAITING_FOR_TARGET_DEPTH", 0, None,
    )
    assert (invalid.phase, invalid.consecutive_samples, invalid.terminal_outcome) == (
        "WAITING_FOR_VALID_DEPTH", 0, None,
    )


def test_depth_wait_progress_stops_on_overshoot_or_timeout() -> None:
    """Special function: Report distinct terminal outcomes for a too-close ROI and an expired wait deadline."""
    # Section: Timeout takes precedence at the exact deadline so an action cannot succeed after its bound.
    module = load_resolver_module()
    policy = module.DepthWaitPolicy(0.05, 0.01, 120.0, 3)
    overshot = module.advance_depth_wait_progress(0.039, policy, 0, 10.0, 130.0)
    timed_out = module.advance_depth_wait_progress(0.05, policy, 0, 130.0, 130.0)
    assert (overshot.phase, overshot.terminal_outcome) == ("OVERSHOT", "OVERSHOT")
    assert (timed_out.phase, timed_out.terminal_outcome) == ("TIMEOUT", "TIMEOUT")


def test_sample_roi_depth_accepts_five_centimetres_with_near_field_floor() -> None:
    """Special function: Permit a valid 5 cm ROI sample only when the configured depth floor supports it."""
    # Section: The default 20 cm safety floor remains unchanged; the launch override enables this experiment.
    module = load_resolver_module()
    depth = np.full((20, 20), 0.05, dtype=np.float32)
    sample = module.sample_roi_depth(depth, (0, 0, 20, 20), None, 0.01, 1.0, 0.25, 10, 0.0, 0.01)
    assert sample.depth_m == pytest.approx(0.05)


def test_pixel_projection_uses_camera_intrinsics() -> None:
    """Special function: Calibrated pixel projection returns a metric optical-frame point."""
    # Section: A pixel one focal length right of centre projects to X equal to Z.
    module = load_resolver_module()
    point = module.project_pixel_to_camera(
        600.0,
        240.0,
        2.0,
        [300.0, 0.0, 300.0, 0.0, 300.0, 240.0, 0.0, 0.0, 1.0],
    )
    assert point == (2.0, 0.0, 2.0)


def test_camera_intrinsics_scale_with_aligned_depth_resolution() -> None:
    """Special function: Half-resolution aligned depth retains the same optical projection geometry."""
    # Section: Every axis-specific focal length and principal point scales with its pixel dimension.
    module = load_resolver_module()
    scaled = module.scale_camera_matrix(
        [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0],
        (640, 480),
        (320, 240),
    )
    assert scaled == [300.0, 0.0, 160.0, 0.0, 300.0, 120.0, 0.0, 0.0, 1.0]


def test_standoff_pose_faces_target() -> None:
    """Special function: The approach pose stays before the target and points toward it."""
    # Section: A target three metres ahead with one-metre stand-off yields an x=2 goal.
    module = load_resolver_module()
    approach_x, approach_y, yaw = module.compute_standoff_geometry((0.0, 0.0), (3.0, 0.0), 1.0)
    assert approach_x == 2.0
    assert approach_y == 0.0
    assert yaw == 0.0
    quaternion = module.yaw_to_quaternion(math.pi)
    assert math.isclose(quaternion[2], 1.0)
    assert math.isclose(quaternion[3], 0.0, abs_tol=1.0e-9)
