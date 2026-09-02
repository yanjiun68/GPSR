"""Section: Shared appearance estimator and per-track temporal policy tests."""

import numpy as np

from face_detection.appearance_estimators import (
    HsvShirtColorEstimator,
    TemporalCategoricalVoter,
    crop_person_roi,
)
from face_detection.tracked_appearance_lifecycle import associate_unique_skeletons


def test_person_roi_is_clipped_to_image_bounds() -> None:
    """Special function: Verify canonical boxes cannot address pixels outside the image."""
    image = np.zeros((20, 30, 3), dtype=np.uint8)
    roi, bounds = crop_person_roi(image, [-10, -5, 40, 25])
    assert bounds == (0, 0, 30, 20)
    assert roi.shape == (20, 30, 3)


def test_hsv_estimator_recognizes_saturated_blue() -> None:
    """Special function: Verify the shared HSV estimator supports GPSR shirt colors."""
    blue_bgr = np.zeros((20, 20, 3), dtype=np.uint8)
    blue_bgr[:, :] = [255, 0, 0]
    color, confidence = HsvShirtColorEstimator().estimate(blue_bgr)
    assert color == "blue"
    assert confidence == 1.0


def test_temporal_voters_do_not_share_person_history() -> None:
    """Special function: Verify separate tracks require their own confirmation evidence."""
    first_track = TemporalCategoricalVoter(history_size=5, minimum_votes=3)
    second_track = TemporalCategoricalVoter(history_size=5, minimum_votes=3)
    assert first_track.update("red", 0.8)[0] == "unknown"
    assert first_track.update("red", 0.8)[0] == "unknown"
    assert first_track.update("red", 0.8)[0] == "red"
    assert second_track.update("blue", 0.9)[0] == "unknown"


def test_temporal_voter_clear_removes_confirmation() -> None:
    """Special function: Verify lifecycle cleanup releases accumulated evidence."""
    voter = TemporalCategoricalVoter(history_size=3, minimum_votes=1)
    assert voter.update("green", 0.7)[0] == "green"
    voter.clear()
    assert voter.update("unknown", 0.0)[0] == "unknown"


def test_skeleton_assignment_is_one_to_one_and_thresholded() -> None:
    """Special function: Overlap ranking never assigns one skeleton to two people."""
    tracks = [[0, 0, 100, 200], [60, 0, 160, 200]]
    skeletons = [[20, 0, 120, 200], [100, 0, 160, 200]]
    assignments = associate_unique_skeletons(tracks, skeletons, 0.10)
    assert len(assignments) == 2
    assert len(set(assignments.values())) == 2
    assert associate_unique_skeletons(
        [[0, 0, 10, 10]], [[20, 20, 30, 30]], 0.1
    ) == {}


def test_skeleton_assignment_single_pair() -> None:
    """Special function: One overlapping person and skeleton produce one exact pairing."""
    assert associate_unique_skeletons(
        [[0, 0, 100, 200]], [[10, 10, 90, 190]], 0.1
    ) == {0: 0}


def test_skeleton_assignment_overlapping_tracks_cannot_share_one_pose() -> None:
    """Special function: One central skeleton is consumed by only its best track match."""
    assignments = associate_unique_skeletons(
        [[0, 0, 100, 200], [40, 0, 140, 200]],
        [[35, 0, 105, 200]],
        0.1,
    )
    assert len(assignments) == 1
    assert len(set(assignments.values())) == 1
