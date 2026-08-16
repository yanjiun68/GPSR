"""Section: Pure matching and serialization tests for lifecycle person servers."""

from receptionist_interfaces.msg import GuestAppearance, PersonProfile, TrackedPersonAppearance

from robot_yasmin_tasks.ros_servers.person_attribute_skill_servers import (
    match_score,
    merge_person_profiles,
    normalize_selector,
)


def tracked_person() -> TrackedPersonAppearance:
    """Special function: Build one deterministic positive tracked observation."""
    person = TrackedPersonAppearance()
    person.track_id = 4
    person.pose = "standing"
    person.pose_confidence = 0.9
    person.gesture = "waving"
    person.gesture_confidence = 0.8
    person.appearance.shirt_color = "red"
    person.appearance.shirt_color_confidence = 0.7
    person.appearance.glasses = GuestAppearance.YES
    person.appearance.glasses_confidence = 0.85
    return person


def test_combined_positive_predicates_use_weakest_confidence() -> None:
    """Special function: Every conjunct must match and determines the stable score."""
    predicates, error = normalize_selector(
        '{"all":[{"type":"shirt_color","value":"red"},{"type":"glasses","value":"yes"}]}'
    )
    assert not error
    assert match_score(tracked_person(), predicates) == 0.7


def test_negative_wearable_selector_is_rejected() -> None:
    """Special function: Missing detection cannot be treated as negative evidence."""
    predicates, error = normalize_selector('{"type":"glasses","value":"no"}')
    assert predicates == []
    assert "Negative glasses" in error


def test_unconfigured_positive_wearable_model_is_rejected() -> None:
    """Special function: Keep hat matching disabled until its model is validated."""
    predicates, error = normalize_selector(
        '{"type":"hat","value":"yes"}',
        {"glasses"},
    )
    assert predicates == []
    assert "positive-class model" in error


def test_profile_merge_preserves_known_fields_and_accepts_registered_data() -> None:
    """Special function: Merge profile lookup fields without UNKNOWN data loss."""
    existing = PersonProfile()
    existing.name = "Alex"
    existing.hometown = "Kuala Lumpur"
    incoming = PersonProfile()
    incoming.name = "UNKNOWN"
    incoming.favorite_drink = "tea"
    merged = merge_person_profiles(existing, incoming)
    assert merged.name == "Alex"
    assert merged.hometown == "Kuala Lumpur"
    assert merged.favorite_drink == "tea"
