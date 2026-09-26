"""Section: Compact GPSR entity-history conversion and matching tests."""

from robot_yasmin_tasks.gpsr.action_types import (
    FIND_OBJECT,
    FIND_PERSON,
    FIND_PERSON_BY_ATTRIBUTE,
)
from robot_yasmin_tasks.gpsr.models import ActionStepData
from robot_yasmin_tasks.ros_clients.find_history_client import (
    build_found_record,
    find_target_identity,
)


def test_object_find_result_builds_compact_map_record() -> None:
    """Special function: Object success retains identity and both useful map observations."""
    action = ActionStepData(FIND_OBJECT, target_object="red apple")
    pose = {
        "frame_id": "map",
        "position": {"x": 1.0, "y": 2.0, "z": 0.4},
        "orientation": {"w": 1.0},
    }
    record = build_found_record(
        action,
        {
            "object_id": "apple_1",
            "object_pose": pose,
            "observation_pose": pose,
            "found_room": "kitchen",
            "found_viewpoint": "kitchen",
            "localized": True,
        },
    )
    assert record is not None
    assert record.target_key == "name:red_apple"
    assert record.entity_pose.header.frame_id == "map"
    assert record.semantic_location == "kitchen"


def test_named_person_target_has_repeatable_lookup_key() -> None:
    """Special function: Spoken name variants normalize to the same memory key."""
    first = ActionStepData(FIND_PERSON, target_person="Alex Smith")
    second = ActionStepData(FIND_PERSON, target_person="  alex-smith ")
    assert find_target_identity(first) == find_target_identity(second)


def test_attribute_selector_key_is_fixed_size_and_order_independent() -> None:
    """Special function: Structured person matching uses a bounded canonical digest."""
    first = ActionStepData(
        FIND_PERSON_BY_ATTRIBUTE,
        parameters={"selector": {"type": "shirt_color", "value": "red"}},
    )
    second = ActionStepData(
        FIND_PERSON_BY_ATTRIBUTE,
        parameters={"selector": {"value": "red", "type": "shirt_color"}},
    )
    assert find_target_identity(first) == find_target_identity(second)
    assert len(find_target_identity(first)[1]) == len("selector:") + 32
