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
    """Confirm a persistent stall while an object holds the fingers open."""
    detector = make_detector()
    observation = None
    for index in range(5):
        observation = update(detector, 0.100, 0.0, 0.020, stamp=index * 0.1)
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
