"""Section: YAML-driven waypoint loop shared by person-finding skills."""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import time
import yaml
from ament_index_python.packages import get_package_share_directory
from yasmin import Blackboard, State, StateMachine

from robot_yasmin_tasks.gpsr import blackboard_keys as bb
from robot_yasmin_tasks.gpsr.blackboard_utils import bb_get
from robot_yasmin_tasks.gpsr.models import ActionStepData, make_skill_result, normalize_action_step
from robot_yasmin_tasks.gpsr.outcomes import CANCELED, FAILED, INVALID_INPUT, NOT_FOUND, SKILL_OUTCOMES, SUCCESS, TIMEOUT
from robot_yasmin_tasks.ros_clients.nav2_client import Nav2Client


# Section: Private keys keep one compound person search isolated from the next queue step.
ORIGINAL_ACTION = "_person_search_original_action"
WAYPOINTS = "_person_search_waypoints"
WAYPOINT_INDEX = "_person_search_waypoint_index"
VISITED_WAYPOINTS = "_person_search_visited_waypoints"
NAVIGATION_FAILURES = "_person_search_navigation_failures"
MISSION_DEADLINE = "_person_search_mission_deadline"
SCAN_INDEX = "_person_search_scan_index"
SCAN_FAILURES = "_person_search_scan_failures"
SCAN_HEADING_DEG = "_person_search_scan_heading_deg"

# Section: Internal transitions describe loop control without extending public skill outcomes.
READY = "ready"
ARRIVED = "arrived"
SKIP = "skip"
MORE = "more"
EXHAUSTED = "exhausted"
NO_REACHABLE_WAYPOINT = "no_reachable_waypoint"
ROTATED = "rotated"
SCAN_EXHAUSTED = "scan_exhausted"


def default_person_search_config_path() -> str:
    """Special function: Resolve the installed policy and retain a source-tree test fallback."""
    try:
        installed_path = (
            Path(get_package_share_directory("robot_yasmin_tasks"))
            / "config"
            / "find_person_search.yaml"
        )
        if installed_path.is_file():
            return str(installed_path)
    except Exception:  # noqa: BLE001 - package discovery is optional in source tests.
        pass
    return str(Path(__file__).parents[2] / "config" / "find_person_search.yaml")


class PersonSearchPolicy:
    """Special function: Validate and expose the configured person-search waypoint policy."""

    def __init__(self, config_file: str | None = None) -> None:
        """Special function: Load immutable waypoint and timeout values from YAML."""
        config_path = Path(str(config_file or default_person_search_config_path()))
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        search = dict(raw.get("person_search", {}))
        navigation = dict(raw.get("navigation", {}))
        configured = search.get("global_waypoints", [])
        if not isinstance(configured, list):
            raise ValueError("person_search.global_waypoints must be a YAML list.")
        # Section: Empty names and duplicates are removed without changing configured order.
        self.global_waypoints = list(
            dict.fromkeys(str(item).strip() for item in configured if str(item).strip())
        )
        if not self.global_waypoints:
            raise ValueError("person_search.global_waypoints must contain at least one waypoint.")
        self.observation_timeout_sec = max(
            0.1,
            float(search.get("observation_timeout_sec", 20.0)),
        )
        self.navigation_timeout_sec = max(
            0.1,
            float(navigation.get("waypoint_timeout_sec", 60.0)),
        )
        self.mission_timeout_sec = max(
            0.1,
            float(search.get("mission_timeout_sec", 300.0)),
        )
        self.scan_timeout_sec = max(
            0.1,
            float(search.get("scan_timeout_sec", 15.0)),
        )
        # Section: Each configured value is an absolute offset from the arrival heading.
        scan = dict(search.get("scan", {}))
        raw_tiers = (
            scan.get("fast_angles_deg", [0.0, 60.0, -60.0]),
            scan.get("extended_angles_deg", [120.0, -120.0]),
            scan.get("exhaustive_angles_deg", [180.0]),
        )
        if not all(isinstance(values, list) for values in raw_tiers):
            raise ValueError("person_search.scan angle tiers must be YAML lists.")
        tiers = tuple(list(values) for values in raw_tiers)
        configured_headings = [float(value) for values in tiers for value in values]
        if not configured_headings or configured_headings[0] != 0.0:
            raise ValueError("person_search.scan.fast_angles_deg must begin with 0.")
        if not all(math.isfinite(value) for value in configured_headings):
            raise ValueError("person_search.scan headings must contain finite numbers.")
        self.observation_headings_deg = configured_headings
        # Section: Arrival already observes zero degrees, so only later headings need Spin goals.
        self.scan_headings_deg = configured_headings[1:]

    def waypoints_for(self, action: ActionStepData) -> list[str]:
        """Special function: Select one strict waypoint or a copy of the global route."""
        requested = str(
            action.target_location
            or action.parameters.get("target_location", "")
            or action.parameters.get("location", "")
        ).strip()
        scope = str(
            action.parameters.get(
                "search_scope",
                "STRICT_LOCATION" if requested else "GLOBAL",
            )
        ).strip().upper()
        if scope == "STRICT_LOCATION":
            if not requested:
                raise ValueError("STRICT_LOCATION person search requires target_location.")
            return [requested]
        if scope == "GLOBAL":
            if requested:
                raise ValueError("GLOBAL person search cannot include target_location.")
            route = list(self.global_waypoints)
            # Section: A matching saved location is verified first, then exhaustive search continues.
            preferred = str(action.parameters.get("preferred_location", "")).strip()
            if preferred in route:
                route.remove(preferred)
                route.insert(0, preferred)
            return route
        raise ValueError(f"Unsupported person search_scope: {scope or 'empty'}.")


def normalized_heading_delta(desired_deg: float, current_deg: float) -> float:
    """Special function: Convert absolute observation headings into shortest relative turns."""
    raw_delta = float(desired_deg) - float(current_deg)
    normalized = (raw_delta + 180.0) % 360.0 - 180.0
    # Section: Resolve the exactly-opposite tie in the configured heading direction.
    if normalized == -180.0 and raw_delta > 0.0:
        return 180.0
    return normalized


def scan_spin_deltas(headings_deg: list[float]) -> list[float]:
    """Special function: Expose deterministic relative spin arithmetic for camera-free tests."""
    current = 0.0
    deltas: list[float] = []
    for desired in headings_deg:
        delta = normalized_heading_delta(desired, current)
        deltas.append(delta)
        current = float(desired)
    return deltas


def _clear_loop_state(blackboard: Blackboard) -> None:
    """Special function: Remove every task-local loop key after a terminal outcome."""
    for key in (
        ORIGINAL_ACTION,
        WAYPOINTS,
        WAYPOINT_INDEX,
        VISITED_WAYPOINTS,
        NAVIGATION_FAILURES,
        MISSION_DEADLINE,
        SCAN_INDEX,
        SCAN_FAILURES,
        SCAN_HEADING_DEG,
    ):
        if key in blackboard:
            del blackboard[key]


def _remaining_mission_sec(blackboard: Blackboard) -> float:
    """Special function: Return the non-negative budget left for the compound search."""
    deadline = float(bb_get(blackboard, MISSION_DEADLINE, 0.0))
    return max(0.0, deadline - time.monotonic())


def remaining_person_search_mission_sec(blackboard: Blackboard) -> float:
    """Special function: Expose the immutable outer deadline to local search states."""
    return _remaining_mission_sec(blackboard)


class PreparePersonSearchState(State):
    """Special function: Freeze the incoming action and build its YAML-backed route."""

    def __init__(self, policy: PersonSearchPolicy) -> None:
        """Special function: Store the already validated search policy."""
        super().__init__([READY, INVALID_INPUT, CANCELED])
        self.policy = policy

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Initialize route ownership without moving the robot."""
        action = normalize_action_step(bb_get(blackboard, bb.CURRENT_ACTION, {}))
        cancel_requested = bb_get(blackboard, bb.CANCEL_REQUESTED, None)
        if callable(cancel_requested) and cancel_requested():
            blackboard[bb.LAST_SKILL_RESULT] = make_skill_result(
                action.action_type,
                CANCELED,
                "Person search canceled before navigation.",
                fatal=True,
            ).to_dict()
            return CANCELED
        try:
            waypoints = self.policy.waypoints_for(action)
        except ValueError as exc:
            blackboard[bb.LAST_SKILL_RESULT] = make_skill_result(
                action.action_type,
                INVALID_INPUT,
                str(exc),
                fatal=True,
            ).to_dict()
            return INVALID_INPUT
        # Section: Copy parameters so per-waypoint mutation never changes the admitted command.
        blackboard[ORIGINAL_ACTION] = replace(action, parameters=dict(action.parameters))
        blackboard[WAYPOINTS] = waypoints
        blackboard[WAYPOINT_INDEX] = 0
        blackboard[VISITED_WAYPOINTS] = []
        blackboard[NAVIGATION_FAILURES] = []
        blackboard[SCAN_FAILURES] = []
        blackboard[SCAN_INDEX] = 0
        blackboard[SCAN_HEADING_DEG] = 0.0
        # Section: One deadline bounds every navigation, observation, and scan in the route.
        mission_timeout = float(action.timeout_sec or self.policy.mission_timeout_sec)
        blackboard[MISSION_DEADLINE] = time.monotonic() + max(0.1, mission_timeout)
        return READY


class NavigatePersonWaypointState(State):
    """Special function: Navigate to the current policy waypoint before local perception."""

    def __init__(self, navigation: Nav2Client, policy: PersonSearchPolicy) -> None:
        """Special function: Store the shared Nav2 adapter and navigation timeout policy."""
        super().__init__([ARRIVED, SKIP, CANCELED, TIMEOUT])
        self.navigation = navigation
        self.policy = policy

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Run one named navigation and prepare a location-scoped search action."""
        original = bb_get(blackboard, ORIGINAL_ACTION)
        waypoints = list(bb_get(blackboard, WAYPOINTS, []))
        index = int(bb_get(blackboard, WAYPOINT_INDEX, 0))
        waypoint = str(waypoints[index])
        remaining = _remaining_mission_sec(blackboard)
        if remaining <= 0.0:
            return TIMEOUT
        navigation_action = ActionStepData(
            "NAVIGATE",
            task_id=original.task_id,
            task_family=original.task_family,
            target_location=waypoint,
            timeout_sec=min(self.policy.navigation_timeout_sec, remaining),
            step_id=f"{original.step_id}_navigate_{index}",
        )
        result = self.navigation.navigate_to_pose(blackboard, navigation_action)
        result_dict = result.to_dict() if hasattr(result, "to_dict") else dict(result)
        outcome = str(result_dict.get("outcome", FAILED))
        if outcome == SUCCESS:
            remaining_after_navigation = _remaining_mission_sec(blackboard)
            if remaining_after_navigation <= 0.0:
                return TIMEOUT
            parameters = dict(original.parameters)
            parameters["location"] = waypoint
            parameters["active_search_waypoint"] = waypoint
            # Section: The inner perception pass is always local, even for a global outer mission.
            scoped_action = replace(
                original,
                target_location=waypoint,
                parameters=parameters,
                timeout_sec=min(
                    self.policy.observation_timeout_sec,
                    remaining_after_navigation,
                ),
            )
            blackboard[bb.CURRENT_ACTION] = scoped_action
            visited = list(bb_get(blackboard, VISITED_WAYPOINTS, []))
            visited.append(waypoint)
            blackboard[VISITED_WAYPOINTS] = visited
            blackboard[SCAN_INDEX] = 0
            blackboard[SCAN_HEADING_DEG] = 0.0
            return ARRIVED
        if outcome == CANCELED:
            blackboard[bb.LAST_SKILL_RESULT] = make_skill_result(
                original.action_type,
                outcome,
                f"Person search navigation {outcome} at {waypoint}.",
                {"waypoint": waypoint},
                fatal=True,
            ).to_dict()
            return outcome
        # Section: A per-waypoint Nav2 timeout skips that waypoint unless the mission expired.
        if outcome == TIMEOUT and _remaining_mission_sec(blackboard) <= 0.0:
            return TIMEOUT
        failures = list(bb_get(blackboard, NAVIGATION_FAILURES, []))
        failures.append(waypoint)
        blackboard[NAVIGATION_FAILURES] = failures
        return SKIP


class AdvancePersonWaypointState(State):
    """Special function: Advance after local NOT_FOUND or a waypoint navigation failure."""

    def __init__(self) -> None:
        """Special function: Declare loop continuation and exhaustion outcomes."""
        super().__init__([MORE, EXHAUSTED, NO_REACHABLE_WAYPOINT, TIMEOUT])

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Select the next YAML waypoint without recapturing the command."""
        if _remaining_mission_sec(blackboard) <= 0.0:
            return TIMEOUT
        next_index = int(bb_get(blackboard, WAYPOINT_INDEX, 0)) + 1
        blackboard[WAYPOINT_INDEX] = next_index
        if next_index < len(list(bb_get(blackboard, WAYPOINTS, []))):
            return MORE
        if list(bb_get(blackboard, VISITED_WAYPOINTS, [])):
            return EXHAUSTED
        return NO_REACHABLE_WAYPOINT


class RotatePersonSearchViewState(State):
    """Special function: Rotate through configured local viewpoints before leaving a waypoint."""

    def __init__(self, navigation: Nav2Client, policy: PersonSearchPolicy) -> None:
        """Special function: Store the Nav2 adapter and validated scan policy."""
        super().__init__([ROTATED, SCAN_EXHAUSTED, CANCELED, TIMEOUT])
        self.navigation = navigation
        self.policy = policy

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Run one relative Spin goal or finish the local scan sequence."""
        remaining = _remaining_mission_sec(blackboard)
        if remaining <= 0.0:
            return TIMEOUT
        scan_index = int(bb_get(blackboard, SCAN_INDEX, 0))
        if scan_index >= len(self.policy.scan_headings_deg):
            return SCAN_EXHAUSTED
        desired_heading_deg = float(self.policy.scan_headings_deg[scan_index])
        current_heading_deg = float(bb_get(blackboard, SCAN_HEADING_DEG, 0.0))
        angle_rad = math.radians(
            normalized_heading_delta(desired_heading_deg, current_heading_deg)
        )
        blackboard[SCAN_INDEX] = scan_index + 1
        spin = getattr(self.navigation, "spin_by_angle", None)
        if not callable(spin):
            # Section: Custom legacy navigation adapters skip scanning without crashing the skill.
            return SCAN_EXHAUSTED
        result = spin(
            blackboard,
            angle_rad,
            min(self.policy.scan_timeout_sec, remaining),
        )
        result_dict = result.to_dict() if hasattr(result, "to_dict") else dict(result)
        outcome = str(result_dict.get("outcome", FAILED))
        if outcome == SUCCESS:
            blackboard[SCAN_HEADING_DEG] = desired_heading_deg
            return ROTATED
        if outcome == CANCELED:
            return CANCELED
        if outcome == TIMEOUT and _remaining_mission_sec(blackboard) <= 0.0:
            return TIMEOUT
        # Section: A failed local rotation is recorded, then the route continues safely.
        failures = list(bb_get(blackboard, SCAN_FAILURES, []))
        failures.append(
            {
                "waypoint": str(
                    list(bb_get(blackboard, WAYPOINTS, []))[
                        int(bb_get(blackboard, WAYPOINT_INDEX, 0))
                    ]
                ),
                "angle_rad": angle_rad,
                "outcome": outcome,
            }
        )
        blackboard[SCAN_FAILURES] = failures
        return SCAN_EXHAUSTED


class PrepareLocalObservationState(State):
    """Special function: Refresh the local detector budget before every camera viewpoint."""

    def __init__(self, policy: PersonSearchPolicy) -> None:
        """Special function: Store the per-observation timeout ceiling."""
        super().__init__([READY, TIMEOUT])
        self.policy = policy

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Prevent repeated scans from outliving the mission deadline."""
        remaining = _remaining_mission_sec(blackboard)
        if remaining <= 0.0:
            return TIMEOUT
        scoped_action = normalize_action_step(bb_get(blackboard, bb.CURRENT_ACTION, {}))
        blackboard[bb.CURRENT_ACTION] = replace(
            scoped_action,
            timeout_sec=min(self.policy.observation_timeout_sec, remaining),
        )
        return READY


class GuardPersonSearchSuccessState(State):
    """Special function: Reject success received after cancellation or the mission deadline."""

    def __init__(self) -> None:
        """Special function: Declare admissible success and terminal timing outcomes."""
        super().__init__([SUCCESS, CANCELED, TIMEOUT])

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Admit a local match only while the original mission is live."""
        cancel_requested = bb_get(blackboard, bb.CANCEL_REQUESTED, None)
        if callable(cancel_requested) and cancel_requested():
            return CANCELED
        if _remaining_mission_sec(blackboard) <= 0.0:
            return TIMEOUT
        return SUCCESS


class FinalizePersonSearchState(State):
    """Special function: Restore the immutable action and publish one compound result."""

    def __init__(self, outcome: str, synthesize_result: bool = False) -> None:
        """Special function: Configure a terminal outcome and optional aggregate result."""
        super().__init__([outcome])
        self.outcome = outcome
        self.synthesize_result = synthesize_result

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Preserve successful perception data while attaching route history."""
        original = bb_get(blackboard, ORIGINAL_ACTION, bb_get(blackboard, bb.CURRENT_ACTION, {}))
        visited = list(bb_get(blackboard, VISITED_WAYPOINTS, []))
        failures = list(bb_get(blackboard, NAVIGATION_FAILURES, []))
        scan_failures = list(bb_get(blackboard, SCAN_FAILURES, []))
        if self.synthesize_result:
            if self.outcome == NOT_FOUND:
                message = "Person was not found after all configured waypoints were searched."
            elif self.outcome == TIMEOUT:
                message = "Person search exhausted its mission-wide time budget."
            else:
                message = "No configured person-search waypoint could be reached."
            # Section: Preserve the last local detector diagnostics while adding route evidence.
            prior = dict(bb_get(blackboard, bb.LAST_SKILL_RESULT, {}))
            aggregate_result = dict(prior.get("result", {}))
            aggregate_result.update(
                {
                    "searched_waypoints": visited,
                    "navigation_failures": failures,
                    "scan_failures": scan_failures,
                }
            )
            blackboard[bb.LAST_SKILL_RESULT] = make_skill_result(
                original.action_type,
                self.outcome,
                message,
                aggregate_result,
                recoverable=self.outcome in {FAILED, NOT_FOUND, TIMEOUT},
                fatal=self.outcome in {CANCELED, INVALID_INPUT},
            ).to_dict()
        else:
            prior = dict(bb_get(blackboard, bb.LAST_SKILL_RESULT, {}))
            result_payload = dict(prior.get("result", {}))
            result_payload.setdefault("searched_waypoints", visited)
            result_payload.setdefault("navigation_failures", failures)
            result_payload.setdefault("scan_failures", scan_failures)
            prior["result"] = result_payload
            # Section: A deadline/cancel guard must replace a preceding local success outcome.
            prior["action_type"] = original.action_type
            prior["outcome"] = self.outcome
            prior["recoverable"] = self.outcome in {FAILED, NOT_FOUND, TIMEOUT}
            prior["fatal"] = self.outcome in {CANCELED, INVALID_INPUT}
            if self.outcome == CANCELED and not str(prior.get("message", "")).lower().startswith("canceled"):
                prior["message"] = "Person search canceled before result commit."
            blackboard[bb.LAST_SKILL_RESULT] = prior
        blackboard[bb.CURRENT_ACTION] = original
        blackboard[bb.DISPATCH_OUTCOME] = self.outcome
        if self.outcome != SUCCESS:
            blackboard[bb.FAILURE_REASON] = str(
                dict(bb_get(blackboard, bb.LAST_SKILL_RESULT, {})).get(
                    "message",
                    "Person search failed.",
                )
            )
        _clear_loop_state(blackboard)
        return self.outcome


class PersonWaypointSearchSM(StateMachine):
    """Special function: Wrap one local person detector in a YAML-driven navigation loop."""

    def __init__(
        self,
        name: str,
        local_search: StateMachine,
        navigation: Nav2Client | None = None,
        search_config_file: str | None = None,
    ) -> None:
        """Special function: Build navigate-search-advance transitions for a find skill."""
        super().__init__(outcomes=SKILL_OUTCOMES)
        self.set_name(name)
        policy = PersonSearchPolicy(search_config_file)
        navigator = navigation or Nav2Client()
        self.add_state(
            "PREPARE_SEARCH_ROUTE",
            PreparePersonSearchState(policy),
            transitions={READY: "NAVIGATE_TO_WAYPOINT", INVALID_INPUT: INVALID_INPUT, CANCELED: CANCELED},
        )
        self.add_state(
            "NAVIGATE_TO_WAYPOINT",
            NavigatePersonWaypointState(navigator, policy),
            transitions={ARRIVED: "PREPARE_LOCAL_OBSERVATION", SKIP: "ADVANCE_WAYPOINT", CANCELED: "FINALIZE_CANCELED", TIMEOUT: "FINALIZE_TIMEOUT"},
        )
        self.add_state(
            "PREPARE_LOCAL_OBSERVATION",
            PrepareLocalObservationState(policy),
            transitions={READY: "SEARCH_CURRENT_WAYPOINT", TIMEOUT: "FINALIZE_TIMEOUT"},
        )
        self.add_state(
            "SEARCH_CURRENT_WAYPOINT",
            local_search,
            transitions={
                SUCCESS: "GUARD_SUCCESS_DEADLINE",
                NOT_FOUND: "ROTATE_SEARCH_VIEW",
                FAILED: "FINALIZE_FAILED",
                INVALID_INPUT: "FINALIZE_INVALID_INPUT",
                CANCELED: "FINALIZE_CANCELED",
                TIMEOUT: "FINALIZE_TIMEOUT",
            },
        )
        self.add_state(
            "GUARD_SUCCESS_DEADLINE",
            GuardPersonSearchSuccessState(),
            transitions={
                SUCCESS: "FINALIZE_SUCCESS",
                CANCELED: "FINALIZE_CANCELED",
                TIMEOUT: "FINALIZE_TIMEOUT",
            },
        )
        self.add_state(
            "ROTATE_SEARCH_VIEW",
            RotatePersonSearchViewState(navigator, policy),
            transitions={
                ROTATED: "PREPARE_LOCAL_OBSERVATION",
                SCAN_EXHAUSTED: "ADVANCE_WAYPOINT",
                CANCELED: "FINALIZE_CANCELED",
                TIMEOUT: "FINALIZE_TIMEOUT",
            },
        )
        self.add_state(
            "ADVANCE_WAYPOINT",
            AdvancePersonWaypointState(),
            transitions={MORE: "NAVIGATE_TO_WAYPOINT", EXHAUSTED: "FINALIZE_NOT_FOUND", NO_REACHABLE_WAYPOINT: "FINALIZE_NO_REACHABLE", TIMEOUT: "FINALIZE_TIMEOUT"},
        )
        self.add_state("FINALIZE_SUCCESS", FinalizePersonSearchState(SUCCESS), transitions={SUCCESS: SUCCESS})
        self.add_state("FINALIZE_NOT_FOUND", FinalizePersonSearchState(NOT_FOUND, True), transitions={NOT_FOUND: NOT_FOUND})
        self.add_state("FINALIZE_NO_REACHABLE", FinalizePersonSearchState(FAILED, True), transitions={FAILED: FAILED})
        self.add_state("FINALIZE_FAILED", FinalizePersonSearchState(FAILED), transitions={FAILED: FAILED})
        self.add_state("FINALIZE_INVALID_INPUT", FinalizePersonSearchState(INVALID_INPUT), transitions={INVALID_INPUT: INVALID_INPUT})
        self.add_state("FINALIZE_CANCELED", FinalizePersonSearchState(CANCELED), transitions={CANCELED: CANCELED})
        self.add_state("FINALIZE_TIMEOUT", FinalizePersonSearchState(TIMEOUT, True), transitions={TIMEOUT: TIMEOUT})
