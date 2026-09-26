"""Hardware-independent regressions for Nav2 lifecycle STARTUP ownership."""
# flake8: noqa: E501

import importlib.util
from pathlib import Path

from lifecycle_msgs.msg import State


# Section: Load the installed-program style source file without changing package layout.
MODULE_PATH = Path(__file__).parents[1] / 'src' / 'nav2_autostart_gate.py'
SPEC = importlib.util.spec_from_file_location('nav2_autostart_gate', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_nav2_manager_owns_only_navigation_stack_nodes():
    """Special function: Keep GPSR lifecycle resources outside Nav2 ownership."""
    # Section: Match the lifecycle_manager_navigation node_names contract exactly.
    assert MODULE.NAV2_MANAGED_NODE_NAMES == (
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    )

    # Section: No GPSR-prefixed skill resource may enter the Nav2 state gate.
    assert not any(
        'gpsr' in node_name
        for node_name in MODULE.NAV2_MANAGED_NODE_NAMES
    )


def test_unconfigured_stack_allows_exactly_one_startup_request():
    """Special function: Cover initial STARTUP and repeated timer-equivalent attempts."""
    # Section: An entirely unconfigured stack is the sole startable state.
    states = [State.PRIMARY_STATE_UNCONFIGURED] * 7
    assert MODULE.classify_nav2_state(False, states) == MODULE.NAV2_STARTABLE

    # Section: Twenty repeated attempts still reserve only one in-flight request.
    guard = MODULE.StartupRequestGuard()
    requests = sum(guard.begin() for _ in range(20))
    assert requests == 1
    assert guard.startup_request_in_flight is True


def test_request_in_flight_suppresses_duplicate_startup():
    """Special function: Cover the explicit asynchronous duplicate-request guard."""
    # Section: A second begin call cannot overlap the first service request.
    guard = MODULE.StartupRequestGuard()
    assert guard.begin() is True
    assert guard.begin() is False


def test_manager_active_marks_ready_without_startup():
    """Special function: Cover authoritative manager ACTIVE classification."""
    # Section: Manager ACTIVE wins even when no per-node snapshot is needed.
    assert MODULE.classify_nav2_state(True, []) == MODULE.NAV2_READY
    guard = MODULE.StartupRequestGuard()
    guard.mark_ready()
    assert guard.nav2_started is True
    assert guard.begin() is False


def test_successful_startup_is_idempotent():
    """Special function: Cover cached readiness after a successful response."""
    # Section: Completion releases in-flight state and permanently suppresses repeats.
    guard = MODULE.StartupRequestGuard()
    assert guard.begin() is True
    guard.complete(True)
    assert guard.startup_request_in_flight is False
    assert guard.nav2_started is True
    assert sum(guard.begin() for _ in range(20)) == 0


def test_failed_startup_requires_re_evaluation_and_suppresses_retry():
    """Special function: Cover safe failure handling without blind retries."""
    # Section: Failure clears in-flight ownership but locks out another STARTUP attempt.
    guard = MODULE.StartupRequestGuard()
    assert guard.begin() is True
    guard.complete(False)
    assert guard.startup_request_in_flight is False
    assert guard.startup_failed is True
    assert guard.begin() is False

    # Section: The lifecycle snapshot can still be re-evaluated independently.
    states = [State.PRIMARY_STATE_UNCONFIGURED] * 7
    assert MODULE.classify_nav2_state(False, states) == MODULE.NAV2_STARTABLE
    assert guard.begin() is False


def test_mixed_and_unknown_states_refuse_blind_startup():
    """Special function: Cover mixed, inactive, and unknown lifecycle safety."""
    # Section: Active plus inactive nodes cannot safely receive manager STARTUP.
    mixed_states = [
        State.PRIMARY_STATE_ACTIVE,
        State.PRIMARY_STATE_ACTIVE,
        State.PRIMARY_STATE_INACTIVE,
    ]
    assert MODULE.classify_nav2_state(False, mixed_states) == MODULE.NAV2_UNSAFE

    # Section: Unknown and uniformly inactive snapshots are also unsafe.
    assert MODULE.classify_nav2_state(
        False,
        [State.PRIMARY_STATE_UNKNOWN],
    ) == MODULE.NAV2_UNSAFE
    assert MODULE.classify_nav2_state(
        False,
        [State.PRIMARY_STATE_INACTIVE] * 7,
    ) == MODULE.NAV2_UNSAFE
