#!/usr/bin/env python3

# Section: Import the installed-style ROS script directly for pure helper testing.
import importlib.util
import math
from pathlib import Path
import sys

import numpy as np


MODULE_PATH = Path(__file__).parents[1] / 'src' / 'lidar_loc.py'
MODULE_SPEC = importlib.util.spec_from_file_location('lidar_loc_under_test', MODULE_PATH)
LIDAR_LOC = importlib.util.module_from_spec(MODULE_SPEC)
sys.modules[MODULE_SPEC.name] = LIDAR_LOC
MODULE_SPEC.loader.exec_module(LIDAR_LOC)


# Section: Verify odometry propagation always starts from immutable session anchors.
def test_fixed_anchor_prediction_does_not_accumulate_refinements():
    seed_map = LIDAR_LOC.Pose2D(x=1.0, y=2.0, yaw=0.0)
    seed_odom = LIDAR_LOC.Pose2D(x=5.0, y=7.0, yaw=0.0)
    current_odom = LIDAR_LOC.Pose2D(x=5.4, y=6.8, yaw=0.1)

    first_prediction = LIDAR_LOC.predict_pose_from_anchors(
        seed_map,
        seed_odom,
        current_odom,
    )
    second_prediction = LIDAR_LOC.predict_pose_from_anchors(
        seed_map,
        seed_odom,
        current_odom,
    )

    assert math.isclose(first_prediction.x, 1.4, abs_tol=1e-9)
    assert math.isclose(first_prediction.y, 1.8, abs_tol=1e-9)
    assert math.isclose(first_prediction.yaw, 0.1, abs_tol=1e-9)
    assert first_prediction == second_prediction


# Section: Verify translation, wrapped yaw, and rejection boundaries independently.
def test_correction_limits_reject_large_translation_and_accept_wrapped_yaw():
    reference = LIDAR_LOC.Pose2D(x=0.0, y=0.0, yaw=3.10)
    wrapped_candidate = LIDAR_LOC.Pose2D(x=0.10, y=0.00, yaw=-3.10)
    large_candidate = LIDAR_LOC.Pose2D(x=0.30, y=0.00, yaw=-3.10)

    wrapped_ok, wrapped_distance, wrapped_yaw = LIDAR_LOC.correction_is_within_limits(
        reference,
        wrapped_candidate,
        max_distance=0.15,
        max_yaw=0.15,
    )
    large_ok, _, _ = LIDAR_LOC.correction_is_within_limits(
        reference,
        large_candidate,
        max_distance=0.15,
        max_yaw=0.15,
    )

    assert wrapped_ok
    assert math.isclose(wrapped_distance, 0.10, abs_tol=1e-9)
    assert wrapped_yaw < 0.10
    assert not large_ok


# Section: Verify static-map clearance rejects wall-adjacent and off-map poses.
def test_pose_clearance_rejects_wall_overlap_and_map_edges():
    unsafe_map = np.zeros((20, 20), dtype=bool)
    unsafe_map[10, 10] = True

    near_wall = LIDAR_LOC.Pose2D(x=1.05, y=1.05, yaw=0.0)
    safe_pose = LIDAR_LOC.Pose2D(x=0.40, y=0.40, yaw=0.0)
    map_edge = LIDAR_LOC.Pose2D(x=0.05, y=0.05, yaw=0.0)

    assert not LIDAR_LOC.pose_has_clearance(
        unsafe_map,
        resolution=0.10,
        origin_x=0.0,
        origin_y=0.0,
        pose=near_wall,
        minimum_clearance=0.25,
    )
    assert LIDAR_LOC.pose_has_clearance(
        unsafe_map,
        resolution=0.10,
        origin_x=0.0,
        origin_y=0.0,
        pose=safe_pose,
        minimum_clearance=0.25,
    )
    assert not LIDAR_LOC.pose_has_clearance(
        unsafe_map,
        resolution=0.10,
        origin_x=0.0,
        origin_y=0.0,
        pose=map_edge,
        minimum_clearance=0.25,
    )
