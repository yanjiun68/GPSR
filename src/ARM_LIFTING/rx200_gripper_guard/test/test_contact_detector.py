"""Deterministic tests for ROS-independent gripper contact detection."""

from rx200_gripper_guard.contact_detector import ContactDetector


def make_detector(require_load: bool = False) -> ContactDetector:
    """Create a calibrated test detector with three-sample persistence."""
    detector = ContactDetector(
        required_samples=3,
        velocity_threshold=0.01,
        position_span_threshold=0.002,
        load_delta_threshold=2.0,
        require_load_confirmation=require_load,
        empty_closed_aperture=0.005,
        aperture_margin=0.002,
        history_length=3,
    )
    detector.reset([1.0, 1.0, 1.0])
    return detector


def update(detector, position, velocity, aperture, load=1.0, closing=True, stamp=0.0):
    """Apply one concise synthetic feedback sample."""
    return detector.update(position, velocity, aperture, load, closing, stamp)


def test_free_motion_does_not_report_contact() -> None:
    """Reject changing position and non-zero velocity."""
    detector = make_detector()
    observations = [
        update(detector, 0.00, -0.2, 0.020, stamp=0.0),
        update(detector, -0.01, -0.2, 0.019, stamp=0.1),
        update(detector, -0.02, -0.2, 0.018, stamp=0.2),
    ]
    assert not any(item.contact_confirmed for item in observations)


def test_brief_pause_does_not_report_contact() -> None:
    """Reject a single low-velocity sample among moving samples."""
    detector = make_detector()
    update(detector, 0.00, -0.2, 0.020)
    update(detector, -0.01, 0.0, 0.020)
    observation = update(detector, -0.02, -0.2, 0.020)
    assert not observation.contact_candidate
    assert not observation.contact_confirmed


def test_persistent_object_contact_is_confirmed() -> None:
    """Confirm a persistent non-zero noisy stall while an object holds fingers open."""
    detector = make_detector()
    observation = None
    positions = [0.1000, 0.1005, 0.1002, 0.1004]
    velocities = [0.008, 0.006, -0.005, 0.004]
    for index, (position, velocity) in enumerate(zip(positions, velocities)):
        observation = update(
            detector, position, velocity, 0.020, stamp=index * 0.05
        )
    assert observation is not None
    assert observation.contact_candidate
    assert observation.contact_confirmed
    assert not observation.empty_close_detected


def test_empty_mechanical_closure_is_not_object_contact() -> None:
    """Classify a persistent stall near calibrated empty closure separately."""
    detector = make_detector()
    observation = None
    for index in range(5):
        observation = update(detector, 0.100, 0.0, 0.005, stamp=index * 0.1)
    assert observation is not None
    assert observation.empty_close_detected
    assert not observation.contact_confirmed


def test_required_load_confirmation_rejects_unsupported_stall() -> None:
    """Reject motion stall when configured load delta never appears."""
    detector = make_detector(require_load=True)
    observation = None
    for index in range(5):
        observation = update(detector, 0.100, 0.0, 0.020, load=1.1, stamp=index * 0.1)
    assert observation is not None
    assert observation.motion_stalled
    assert not observation.contact_candidate
    assert not observation.contact_confirmed


def test_possible_contact_resets_when_motion_recovers() -> None:
    """Clear persistent evidence when position and velocity resume progressing."""
    detector = make_detector()
    update(detector, 0.100, 0.006, 0.020)
    possible = update(detector, 0.1005, 0.005, 0.020)
    assert possible.contact_candidate
    assert possible.stable_samples == 1

    recovered = update(detector, 0.090, -0.2, 0.019)
    assert not recovered.contact_candidate
    assert not recovered.contact_confirmed
    assert recovered.stable_samples == 0


def test_confirmation_uses_three_consecutive_low_motion_intervals() -> None:
    """Expose the configured 100-150 ms persistence contract at 20 Hz."""
    detector = make_detector()
    update(detector, 0.1000, 0.004, 0.020, stamp=0.00)
    first = update(detector, 0.1005, 0.004, 0.020, stamp=0.05)
    second = update(detector, 0.1002, -0.004, 0.020, stamp=0.10)
    third = update(detector, 0.1004, 0.003, 0.020, stamp=0.15)
    assert first.stable_samples == 1
    assert second.stable_samples == 2
    assert not second.contact_confirmed
    assert third.contact_confirmed
