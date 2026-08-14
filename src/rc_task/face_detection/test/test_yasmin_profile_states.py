"""Unit tests for the face profile YASMIN Blackboard integration."""

import json

import pytest
from receptionist_interfaces.action import CaptureNamedProfile
from receptionist_interfaces.msg import GuestAppearance, PersonProfile
from std_msgs.msg import String

from face_detection.yasmin_profile_states import (
    CaptureInputError,
    CaptureInterfaceUnavailableError,
    build_capture_goal,
    initialize_profile_blackboard,
    update_blackboard_from_appearance_message,
    update_blackboard_from_capture_result,
    update_blackboard_from_identity_message,
)


# Section: Lightweight dictionaries exercise pure Blackboard mapping behavior without a ROS graph.
def test_blackboard_defaults_are_complete_and_non_destructive():
    """Special function: Verify all contract defaults while preserving an existing name."""
    blackboard = {'name': 'Alice'}
    initialize_profile_blackboard(blackboard)
    assert blackboard['name'] == 'Alice'
    assert blackboard['profile_id'] == ''
    assert blackboard['age'] == ''
    assert blackboard['glasses'] == 0
    assert blackboard['mask'] == 0
    assert blackboard['recognized_match_score'] == -1.0
    assert blackboard['capture_success'] is False
    assert blackboard['captured_profile'] is None


# Special function: Wrap a JSON dictionary in the existing std_msgs/String transport.
def _identity_message(payload):
    """Return one serialized recognized-identity topic message."""
    message = String()
    message.data = json.dumps(payload)
    return message


# Section: Stable identity parsing must tolerate all transient recognition states.
def test_valid_known_identity_updates_recognized_keys():
    """Special function: Verify a known stable payload populates recognition Blackboard keys."""
    blackboard = {}
    updated = update_blackboard_from_identity_message(
        blackboard,
        _identity_message(
            {
                'profile_id': 'guest_1',
                'name': 'Alice',
                'role': 'guest',
                'state': 'known',
                'is_known': True,
                'is_stable': True,
                'match_score': 0.75,
            }
        ),
    )
    assert updated
    assert blackboard['recognized_profile_id'] == 'guest_1'
    assert blackboard['recognized_name'] == 'Alice'
    assert blackboard['recognized_role'] == 'guest'
    assert blackboard['recognized_match_score'] == pytest.approx(0.75)


def test_malformed_and_unknown_identity_preserve_valid_recognition():
    """Special function: Verify transient or malformed messages never erase a valid identity."""
    blackboard = {
        'recognized_profile_id': 'guest_1',
        'recognized_name': 'Alice',
        'recognized_role': 'guest',
        'recognized_match_score': 0.75,
    }
    malformed = String()
    malformed.data = '{not-json'
    assert not update_blackboard_from_identity_message(blackboard, malformed)
    assert not update_blackboard_from_identity_message(
        blackboard,
        _identity_message(
            {
                'profile_id': '',
                'state': 'unknown',
                'is_known': False,
                'is_stable': True,
            }
        ),
    )
    assert blackboard['recognized_profile_id'] == 'guest_1'
    assert blackboard['recognized_name'] == 'Alice'


# Section: Appearance mapping preserves three-state semantics rather than boolean coercion.
@pytest.mark.parametrize('semantic_value', [0, 1, 2])
def test_appearance_mapping_preserves_uint8_semantics(semantic_value):
    """Special function: Verify valid glasses and mask values remain unchanged."""
    message = GuestAppearance()
    message.shirt_color = 'blue'
    message.glasses = semantic_value
    message.face_mask = semantic_value
    message.shirt_color_confidence = 0.7
    message.glasses_confidence = 0.8
    message.face_mask_confidence = 0.9
    blackboard = {}
    assert update_blackboard_from_appearance_message(blackboard, message)
    assert blackboard['shirt_color'] == 'blue'
    assert blackboard['glasses'] == semantic_value
    assert blackboard['mask'] == semantic_value
    assert blackboard['shirt_color_confidence'] == pytest.approx(0.7)
    assert blackboard['glasses_confidence'] == pytest.approx(0.8)
    assert blackboard['mask_confidence'] == pytest.approx(0.9)


def test_appearance_mapping_rejects_invalid_semantics():
    """Special function: Verify unsupported uint8 values map to UNKNOWN without becoming bools."""
    message = GuestAppearance()
    message.glasses = 7
    message.face_mask = 255
    blackboard = {}
    update_blackboard_from_appearance_message(blackboard, message)
    assert blackboard['glasses'] == 0
    assert blackboard['mask'] == 0
    assert type(blackboard['glasses']) is int
    assert type(blackboard['mask']) is int


# Section: Future action double verifies metadata before the external interface is patched.
class FutureCaptureGoal:
    """Special test double: Represent the requested future CaptureNamedProfile goal fields."""

    # Special function: Initialize every expected current and future goal field.
    def __init__(self):
        """Create empty goal fields matching the proposed ROS action definition."""
        self.profile_id = ''
        self.role = ''
        self.name = ''
        self.favorite_drink = ''
        self.age = 0


def test_capture_goal_populates_current_and_future_fields():
    """Special function: Verify identity, role default, drink, and string-to-int age conversion."""
    goal = build_capture_goal(
        {
            'profile_id': 'guest_1',
            'role': '',
            'name': 'Alice',
            'fav_drink': 'orange juice',
            'age': '25',
        },
        goal_factory=FutureCaptureGoal,
    )
    assert goal.profile_id == 'guest_1'
    assert goal.role == 'guest'
    assert goal.name == 'Alice'
    assert goal.favorite_drink == 'orange juice'
    assert goal.age == 25


def test_capture_goal_preserves_current_interface_without_metadata():
    """Special function: Verify existing capture works before action interface regeneration."""
    goal = build_capture_goal(
        {'profile_id': 'guest_1', 'role': 'host', 'name': 'Alice'},
        goal_factory=CaptureNamedProfile.Goal,
    )
    assert goal.profile_id == 'guest_1'
    assert goal.role == 'host'
    assert goal.name == 'Alice'


def test_capture_goal_rejects_invalid_age():
    """Special function: Verify malformed Blackboard age fails explicitly."""
    with pytest.raises(CaptureInputError, match='age must be an integer string'):
        build_capture_goal(
            {'profile_id': 'guest_1', 'role': 'guest', 'name': 'Alice', 'age': 'twenty'},
            goal_factory=FutureCaptureGoal,
        )


def test_capture_goal_reports_current_interface_metadata_blocker():
    """Special function: Verify the current action never silently drops requested metadata."""
    with pytest.raises(CaptureInterfaceUnavailableError, match='USER_ACTION_REQUIRED'):
        build_capture_goal(
            {
                'profile_id': 'guest_1',
                'role': 'guest',
                'name': 'Alice',
                'fav_drink': 'orange juice',
                'age': '25',
            },
            goal_factory=CaptureNamedProfile.Goal,
        )


# Section: Capture result mapping joins returned profile metadata and appearance into Blackboard.
class CaptureResponse:
    """Special test double: Represent the action result consumed by the YASMIN result handler."""

    # Special function: Construct a successful result around a supplied PersonProfile.
    def __init__(self, success, message, profile):
        """Store result fields matching CaptureNamedProfile.Result."""
        self.success = success
        self.message = message
        self.profile = profile


def test_capture_result_synchronizes_profile_and_appearance():
    """Special function: Verify successful action results populate all capture contract keys."""
    profile = PersonProfile()
    profile.profile_id = 'guest_1'
    profile.role = 'guest'
    profile.name = 'Alice'
    profile.favorite_drink = 'orange juice'
    profile.age = 25
    profile.appearance.shirt_color = 'blue'
    profile.appearance.glasses = GuestAppearance.YES
    profile.appearance.face_mask = GuestAppearance.NO
    blackboard = {}
    response = CaptureResponse(True, 'captured', profile)
    assert update_blackboard_from_capture_result(blackboard, response)
    assert blackboard['capture_success'] is True
    assert blackboard['captured_profile'] is profile
    assert blackboard['fav_drink'] == 'orange juice'
    assert blackboard['age'] == '25'
    assert blackboard['shirt_color'] == 'blue'
    assert blackboard['glasses'] == 2
    assert blackboard['mask'] == 1


def test_capture_failure_preserves_person_metadata():
    """Special function: Verify action failure updates status without erasing existing metadata."""
    blackboard = {'profile_id': 'guest_1', 'fav_drink': 'orange juice'}
    response = CaptureResponse(False, 'timeout', PersonProfile())
    assert not update_blackboard_from_capture_result(blackboard, response)
    assert blackboard['capture_success'] is False
    assert blackboard['capture_message'] == 'timeout'
    assert blackboard['profile_id'] == 'guest_1'
    assert blackboard['fav_drink'] == 'orange juice'
