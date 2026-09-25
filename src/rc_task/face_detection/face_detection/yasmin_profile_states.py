"""Reusable YASMIN states for face identity, appearance, and profile capture."""

import json

import yasmin
from robot_ai_interfaces.action import CaptureNamedProfile
from robot_ai_interfaces.msg import GuestAppearance
from std_msgs.msg import String
from yasmin_ros import ActionState, MonitorState
from yasmin_ros.basic_outcomes import ABORT, SUCCEED


# Section: Public outcomes let a state machine distinguish invalid input from interface work.
USER_ACTION_REQUIRED = 'user_action_required'
INVALID_INPUT = 'invalid_input'
VALID_SEMANTIC_VALUES = (0, 1, 2)


# Section: Blackboard defaults form the shared identity and appearance contract.
BLACKBOARD_DEFAULTS = {
    'profile_id': '',
    'role': '',
    'name': '',
    'fav_drink': '',
    'age': '',
    'shirt_color': '',
    'glasses': 0,
    'mask': 0,
    'shirt_color_confidence': 0.0,
    'glasses_confidence': 0.0,
    'mask_confidence': 0.0,
    'recognized_profile_id': '',
    'recognized_name': '',
    'recognized_role': '',
    'recognized_match_score': -1.0,
    'capture_success': False,
    'capture_message': '',
    'captured_profile': None,
}


class CaptureInputError(ValueError):
    """Special exception: Report invalid Blackboard capture input explicitly."""


class CaptureInterfaceUnavailableError(RuntimeError):
    """Special exception: Report metadata blocked by the current generated ROS action."""


# Special function: Read a Blackboard or mapping key without assuming dict.get semantics.
def blackboard_get(blackboard, key, default):
    """Return one Blackboard value or its default when the key is absent."""
    # Section: Native YASMIN Blackboard exposes contains(), while tests may use dictionaries.
    if hasattr(blackboard, 'contains'):
        return blackboard[key] if blackboard.contains(key) else default
    try:
        return blackboard[key]
    except (KeyError, RuntimeError, TypeError):
        return default


# Special function: Populate only missing Blackboard keys with the public contract defaults.
def initialize_profile_blackboard(blackboard):
    """Initialize identity, dialogue metadata, appearance, recognition, and capture keys."""
    # Section: Existing values are preserved so initialization is safe during state-machine reuse.
    for key, default in BLACKBOARD_DEFAULTS.items():
        if hasattr(blackboard, 'contains'):
            exists = blackboard.contains(key)
        else:
            exists = key in blackboard
        if not exists:
            blackboard[key] = default
    return blackboard


# Special function: Normalize GuestAppearance semantic values without converting them to booleans.
def normalize_semantic_value(value):
    """Return UNKNOWN=0 for values outside the required uint8 semantics 0, 1, and 2."""
    # Section: Conversion failures and unsupported numeric values both map to UNKNOWN.
    try:
        normalized = int(value)
    except (TypeError, ValueError):
        return 0
    return normalized if normalized in VALID_SEMANTIC_VALUES else 0


# Special function: Parse a stable recognized-identity JSON message into Blackboard keys.
def update_blackboard_from_identity_message(blackboard, message):
    """Update recognized keys only for a known, stable, non-empty identity payload."""
    # Section: Malformed and non-object JSON is ignored without changing valid prior identity.
    try:
        payload = json.loads(message.data)
    except (AttributeError, TypeError, json.JSONDecodeError):
        return False
    if not isinstance(payload, dict):
        return False
    profile_id = str(payload.get('profile_id', '')).strip()
    valid_identity = bool(
        payload.get('is_known') is True
        and payload.get('is_stable') is True
        and payload.get('state') == 'known'
        and profile_id
    )
    if not valid_identity:
        return False
    # Section: A valid stable identity atomically replaces the recognized Blackboard fields.
    blackboard['recognized_profile_id'] = profile_id
    blackboard['recognized_name'] = str(payload.get('name', '')).strip()
    blackboard['recognized_role'] = str(payload.get('role', '')).strip()
    try:
        blackboard['recognized_match_score'] = float(payload.get('match_score', -1.0))
    except (TypeError, ValueError):
        blackboard['recognized_match_score'] = -1.0
    return True


# Special function: Mirror one live GuestAppearance observation into Blackboard values.
def update_blackboard_from_appearance_message(blackboard, message):
    """Copy appearance values and confidences while preserving uint8 semantics."""
    # Section: Live appearance remains independent from persisted identity metadata.
    blackboard['shirt_color'] = str(message.shirt_color)
    blackboard['glasses'] = normalize_semantic_value(message.glasses)
    blackboard['mask'] = normalize_semantic_value(message.face_mask)
    blackboard['shirt_color_confidence'] = float(message.shirt_color_confidence)
    blackboard['glasses_confidence'] = float(message.glasses_confidence)
    blackboard['mask_confidence'] = float(message.face_mask_confidence)
    return True


# Special function: Validate and convert the Blackboard age string at the ROS boundary.
def parse_blackboard_age(value):
    """Return a non-negative int32 age or raise CaptureInputError for malformed input."""
    # Section: Empty age is optional and maps to the unspecified ROS value zero.
    age_text = str(value).strip()
    if not age_text:
        return 0
    try:
        age = int(age_text)
    except (TypeError, ValueError) as exc:
        raise CaptureInputError(f"age must be an integer string, received '{age_text}'") from exc
    if age < 0 or age > 2_147_483_647:
        raise CaptureInputError('age must fit a non-negative int32 value')
    return age


# Special function: Construct a backward-compatible capture action goal from Blackboard data.
def build_capture_goal(blackboard, goal_factory=CaptureNamedProfile.Goal):
    """Build and validate CaptureNamedProfile.Goal without claiming unavailable metadata fields."""
    # Section: Validate the required identity fields and constrained role before creating the goal.
    profile_id = str(blackboard_get(blackboard, 'profile_id', '')).strip()
    if not profile_id:
        raise CaptureInputError('profile_id is required for profile capture')
    role = str(blackboard_get(blackboard, 'role', '')).strip().lower() or 'guest'
    if role not in ('guest', 'host'):
        raise CaptureInputError("role must be 'guest' or 'host'")
    name = str(blackboard_get(blackboard, 'name', '')).strip()
    favorite_drink = str(blackboard_get(blackboard, 'fav_drink', '')).strip()
    age_text = str(blackboard_get(blackboard, 'age', '')).strip()
    age = parse_blackboard_age(age_text)

    # Section: Populate the current action fields that are always available.
    goal = goal_factory()
    goal.profile_id = profile_id
    goal.role = role
    goal.name = name
    # Section: Optional canonical and transient identity locks are passed when available.
    if hasattr(goal, 'person_uuid'):
        goal.person_uuid = str(
            blackboard_get(blackboard, 'person_uuid', '')
        ).strip()
    if hasattr(goal, 'tracking_session_id'):
        goal.tracking_session_id = str(
            blackboard_get(blackboard, 'tracking_session_id', '')
        ).strip()
    if hasattr(goal, 'track_id'):
        goal.track_id = int(blackboard_get(blackboard, 'track_id', 0) or 0)

    # Section: Refuse requested metadata when the generated action has not been extended yet.
    missing_fields = []
    if favorite_drink:
        if hasattr(goal, 'favorite_drink'):
            goal.favorite_drink = favorite_drink
        else:
            missing_fields.append('favorite_drink')
    elif hasattr(goal, 'favorite_drink'):
        goal.favorite_drink = ''
    if age_text:
        if hasattr(goal, 'age'):
            goal.age = age
        else:
            missing_fields.append('age')
    elif hasattr(goal, 'age'):
        goal.age = 0
    if missing_fields:
        missing = ', '.join(missing_fields)
        raise CaptureInterfaceUnavailableError(
            'USER_ACTION_REQUIRED: generated CaptureNamedProfile.Goal lacks '
            f'{missing}; extend robot_ai_interfaces/action/CaptureNamedProfile.action '
            'and rebuild the interface.'
        )
    return goal


# Special function: Synchronize a successful capture result into the Blackboard contract.
def update_blackboard_from_capture_result(blackboard, response):
    """Store capture result state and copy the returned PersonProfile on success."""
    # Section: Failure updates status only and deliberately preserves existing person metadata.
    blackboard['capture_success'] = bool(response.success)
    blackboard['capture_message'] = str(response.message)
    if not response.success:
        return False
    profile = response.profile
    blackboard['captured_profile'] = profile
    blackboard['profile_id'] = str(profile.profile_id)
    blackboard['role'] = str(profile.role)
    blackboard['name'] = str(profile.name)
    blackboard['fav_drink'] = str(profile.favorite_drink)
    blackboard['age'] = str(profile.age)
    blackboard['shirt_color'] = str(profile.appearance.shirt_color)
    blackboard['glasses'] = normalize_semantic_value(profile.appearance.glasses)
    blackboard['mask'] = normalize_semantic_value(profile.appearance.face_mask)
    return True


class RecognizedIdentityMonitorState(MonitorState):
    """Special YASMIN state: Monitor stable identities without clearing prior valid recognition."""

    # Special function: Create a monitor for the dedicated identity JSON topic.
    def __init__(
        self,
        topic_name='/appearance/recognized_human_identity',
        node=None,
        timeout=None,
    ):
        """Configure the reusable identity topic monitor."""
        # Section: Every consumed message completes the monitor even when safely ignored.
        super().__init__(
            String,
            topic_name,
            {SUCCEED},
            self._monitor_identity,
            node=node,
            timeout=timeout,
        )

    # Special function: Apply one identity message to the shared Blackboard.
    @staticmethod
    def _monitor_identity(blackboard, message):
        """Update stable recognition fields and return the standard success outcome."""
        update_blackboard_from_identity_message(blackboard, message)
        return SUCCEED


class AppearanceMonitorState(MonitorState):
    """Special YASMIN state: Mirror live appearance semantics into the Blackboard."""

    # Special function: Create a monitor for the existing GuestAppearance topic.
    def __init__(self, topic_name='/appearance/current_estimate', node=None, timeout=None):
        """Configure the reusable appearance topic monitor."""
        # Section: The state consumes existing perception output and starts no detector.
        super().__init__(
            GuestAppearance,
            topic_name,
            {SUCCEED},
            self._monitor_appearance,
            node=node,
            timeout=timeout,
        )

    # Special function: Apply one appearance message to the shared Blackboard.
    @staticmethod
    def _monitor_appearance(blackboard, message):
        """Copy semantic and confidence values and return the standard success outcome."""
        update_blackboard_from_appearance_message(blackboard, message)
        return SUCCEED


class CaptureNamedProfileState(ActionState):
    """Special YASMIN state: Capture or re-enroll one explicitly requested profile."""

    # Special function: Configure the existing capture action client and compatibility outcomes.
    def __init__(
        self,
        action_name='/appearance/capture_named_profile',
        node=None,
        wait_timeout=None,
        response_timeout=None,
    ):
        """Create a reusable capture action state without automatic recognition enrollment."""
        self._pending_goal = None
        # Section: Custom outcomes distinguish invalid input from interface regeneration.
        super().__init__(
            CaptureNamedProfile,
            action_name,
            self._create_goal,
            outcomes={USER_ACTION_REQUIRED, INVALID_INPUT},
            result_handler=self._handle_result,
            node=node,
            wait_timeout=wait_timeout,
            response_timeout=response_timeout,
        )

    # Special function: Return the goal prevalidated by execute().
    def _create_goal(self, blackboard):
        """Return a pending validated capture goal or build one directly."""
        # Section: Direct construction remains available for framework-driven execution paths.
        goal = self._pending_goal or build_capture_goal(blackboard)
        self._pending_goal = None
        return goal

    # Special function: Validate locally before contacting the action server.
    def execute(self, blackboard):
        """Return explicit outcomes for invalid input and unavailable metadata fields."""
        # Section: Metadata compatibility failures never send a partial capture goal.
        try:
            self._pending_goal = build_capture_goal(blackboard)
        except CaptureInterfaceUnavailableError as exc:
            blackboard['capture_success'] = False
            blackboard['capture_message'] = str(exc)
            yasmin.YASMIN_LOG_WARN(str(exc))
            return USER_ACTION_REQUIRED
        except CaptureInputError as exc:
            blackboard['capture_success'] = False
            blackboard['capture_message'] = str(exc)
            yasmin.YASMIN_LOG_ERROR(str(exc))
            return INVALID_INPUT
        return super().execute(blackboard)

    # Special function: Synchronize the capture result and select the YASMIN outcome.
    @staticmethod
    def _handle_result(blackboard, response):
        """Return SUCCEED for a successful profile and ABORT for an action-level failure."""
        # Section: Capture is the only enrollment path; recognition never invokes this action.
        return SUCCEED if update_blackboard_from_capture_result(blackboard, response) else ABORT
