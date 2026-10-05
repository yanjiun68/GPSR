"""Central user-message catalogue for navigation diagnostics."""

# Section: Error-code import for the catalogue keys.
from .error_codes import ErrorCode


# Section: Non-technical user-facing message mapping.
USER_MESSAGES = {
    ErrorCode.NONE: 'Navigation completed.',
    ErrorCode.INVALID_GOAL: 'I cannot navigate to that location.',
    ErrorCode.NAV_SERVER_UNAVAILABLE: 'Navigation is not ready yet.',
    ErrorCode.GOAL_REJECTED: 'I cannot accept that destination.',
    ErrorCode.NAVIGATION_CANCELED: 'Navigation has been canceled.',
    ErrorCode.NAVIGATION_TIMEOUT: 'I could not reach that destination in time.',
    ErrorCode.NAVIGATION_ABORTED: 'I could not reach that destination.',
    ErrorCode.TF_ERROR: 'I cannot determine that location right now.',
    ErrorCode.ROBOT_POSE_UNAVAILABLE: 'I cannot determine my position right now.',
    ErrorCode.COSTMAP_UNAVAILABLE: 'I am still preparing to navigate.',
    ErrorCode.START_OUTSIDE_MAP: 'I cannot determine my position on the map.',
    ErrorCode.START_OCCUPIED: 'I am blocked where I am standing.',
    ErrorCode.GOAL_OUTSIDE_MAP: 'That destination is outside my known area.',
    ErrorCode.GOAL_OCCUPIED: 'That destination is blocked.',
    ErrorCode.PLANNER_UNAVAILABLE: 'I am still preparing to navigate.',
    ErrorCode.PLANNER_GOAL_REJECTED: 'I cannot plan a route to that destination.',
    ErrorCode.PLANNING_TIMEOUT: 'I need more time to plan a route.',
    ErrorCode.NO_VALID_PATH: 'I cannot find a route to that destination.',
    ErrorCode.PLANNING_FAILED: 'I cannot plan a route to that destination.',
    ErrorCode.EXECUTION_FAILED: 'I could not complete the route to that destination.',
    ErrorCode.UNKNOWN_NAVIGATION_FAILURE: 'I could not navigate to that destination.',
}


# Section: Safe public lookup function.
def user_message_for(code: ErrorCode) -> str:
    """Return the approved non-technical message for one error code."""

    return USER_MESSAGES.get(code, USER_MESSAGES[ErrorCode.UNKNOWN_NAVIGATION_FAILURE])
