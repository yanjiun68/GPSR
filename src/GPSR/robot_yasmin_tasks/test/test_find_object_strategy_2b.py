"""Section: Strategy 2B search, verification, localization, and approach tests."""

from __future__ import annotations

import asyncio
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from yasmin import Blackboard

from putrabot_interfaces.msg import ObjectDetection, ObjectDetections
import robot_yasmin_tasks.ros_servers.find_object_skill_server as find_object_server_module
from robot_yasmin_tasks.ros_servers.find_object_skill_server import (
    CANCELED,
    CANDIDATE,
    FOUND_UNREACHABLE,
    READY,
    TIMEOUT,
    AdvanceSearchState,
    BuildSearchPlanState,
    CandidateStore,
    DetectionSample,
    FindObjectMissionSM,
    FindObjectSkillServer,
    MarkViewpointState,
    SearchContext,
    SearchPlanBuilder,
    SearchViewpoint,
    SelectViewpointState,
    SlidingWindowVerifier,
    StandoffPlanner,
    ResultCode,
    VerifyTargetState,
    ViewpointStatus,
    target_status_payload,
)


def sample_detection(name: str = "bottle", confidence: float = 0.9) -> DetectionSample:
    """Special function: Build one deterministic structured detection sample."""
    detection = ObjectDetection()
    detection.name = name
    detection.confidence = confidence
    detection.center_u = 2.0
    detection.center_v = 2.0
    return DetectionSample(detection, detection.stamp, time.monotonic())


def context_with_viewpoints() -> SearchContext:
    """Special function: Build a non-expired global search context."""
    now = time.monotonic()
    context = SearchContext("bottle", "", "GLOBAL", "", True, 1.0, now, now + 60.0)
    context.viewpoints = [
        SearchViewpoint("kitchen", "kitchen", 1),
        SearchViewpoint("bedroom", "bedroom", 3),
    ]
    context.current_viewpoint = context.viewpoints[0]
    return context


def search_config() -> dict:
    """Special function: Return a small semantic plan with a low-priority fallback."""
    return {
        "rooms": {
            "kitchen": {"viewpoints": ["kitchen_table", "kitchen_center"]},
            "bedroom": {"viewpoints": ["bedroom_center"]},
            "entrance": {"viewpoints": ["entrance_center"]},
        },
        "object_priors": {"bottle": ["kitchen"]},
        "category_priors": {},
    }


def test_required_strategy_state_names_are_present() -> None:
    """Special function: Mission graph exposes every required Strategy 2B decision state."""
    server = SimpleNamespace()
    states = set(FindObjectMissionSM(server).get_states())
    assert {
        "INITIALIZE",
        "BUILD_SEARCH_PLAN",
        "SELECT_VIEWPOINT",
        "NAVIGATE_AND_MONITOR",
        "FAST_SCAN",
        "EXTENDED_SCAN",
        "FULL_SCAN",
        "VERIFY_TARGET",
        "LOCALIZE_TARGET",
        "PLAN_STANDOFF",
        "APPROACH_TARGET",
        "MARK_VIEWPOINT",
        "ADVANCE_SEARCH",
        "SUCCESS",
        "NOT_FOUND",
    }.issubset(states)


def test_semantic_first_viewpoint_allows_early_success() -> None:
    """Special function: The target's prior room is searched before fallback rooms."""
    plan = SearchPlanBuilder.build(search_config(), "bottle", "", "GLOBAL", "")
    assert plan[0].name == "kitchen_table"
    assert plan[0].tier == 1


def test_unexpected_location_remains_in_exhaustive_fallback() -> None:
    """Special function: Semantic priors order but never restrict global coverage."""
    plan = SearchPlanBuilder.build(search_config(), "bottle", "", "GLOBAL", "")
    assert [item.room for item in plan] == [
        "kitchen",
        "kitchen",
        "bedroom",
        "entrance",
    ]
    assert any(item.room == "entrance" and item.tier >= 3 for item in plan)


def test_remembered_viewpoint_is_verified_before_normal_fallback() -> None:
    """Special function: A history match changes order without reducing search coverage."""
    baseline = SearchPlanBuilder.build(search_config(), "bottle", "", "GLOBAL", "")
    prioritized = SearchPlanBuilder.build(
        search_config(),
        "bottle",
        "",
        "GLOBAL",
        "",
        "entrance_center",
    )
    assert prioritized[0].name == "entrance_center"
    assert {item.name for item in prioritized} == {item.name for item in baseline}


def test_candidate_during_navigation_cancels_current_goal() -> None:
    """Special function: Moving perception interrupts rather than confirms navigation."""
    server = object.__new__(FindObjectSkillServer)
    nav_goal = SimpleNamespace(accepted=True)
    nav_goal.get_result_async = lambda: SimpleNamespace(done=lambda: False)
    server.named_nav_client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: True,
        send_goal_async=lambda _goal: object(),
    )
    server.search_config = {
        "navigation": {"server_timeout_sec": 1.0, "result_timeout_sec": 5.0},
        "verification": {"min_confidence": 0.35},
    }
    server.wait_future = lambda *_args, **_kwargs: nav_goal
    server.track_motion = lambda _goal: None
    canceled = []
    server.cancel_motion = lambda goal: canceled.append(goal)
    server.guard_outcome = lambda _goal, _context: None
    server.target_names = lambda _context: {"bottle"}
    server.publish_feedback = lambda *_args, **_kwargs: None
    server.candidates = SimpleNamespace(
        latest_match=lambda *_args, **_kwargs: sample_detection()
    )
    context = context_with_viewpoints()
    outcome = server.navigate_named_and_monitor(SimpleNamespace(), context)
    assert outcome == CANDIDATE
    assert canceled == [nav_goal]
    assert context.current_viewpoint.status == ViewpointStatus.INTERRUPTED


def test_false_moving_candidate_resumes_interrupted_goal() -> None:
    """Special function: Stationary rejection leaves the viewpoint unsearched."""
    server = SimpleNamespace()
    server.search_config = {
        "scan": {"settle_interval_sec": 0.0},
        "verification": {
            "min_confidence": 0.35,
            "window_frames": 5,
            "minimum_hits": 3,
            "confirmation_timeout_sec": 1.0,
            "last_seen_timeout_sec": 0.5,
        },
    }
    server.cancellable_wait = lambda *_args: None
    server.target_names = lambda _context: {"bottle"}
    server.guard_outcome = lambda *_args: None
    server.publish_feedback = lambda *_args, **_kwargs: None
    server.candidates = SimpleNamespace(
        collect_confirmation=lambda *_args, **_kwargs: (False, None, True)
    )
    context = context_with_viewpoints()
    context.candidate_source = "navigation"
    board = Blackboard()
    board["context"] = context
    board["goal_handle"] = SimpleNamespace()
    assert VerifyTargetState(server).execute(board) == "resume_navigation"
    assert context.interrupted_viewpoint is context.current_viewpoint
    assert context.current_viewpoint.status == ViewpointStatus.INTERRUPTED


def test_target_during_first_scan_skips_remaining_angles() -> None:
    """Special function: A first-heading candidate stops the configured scan sequence."""
    server = object.__new__(FindObjectSkillServer)
    server.spin_client = object()
    server.search_config = {
        "scan": {"settle_interval_sec": 0.0},
        "verification": {"min_confidence": 0.35, "last_seen_timeout_sec": 0.1},
    }
    server.guard_outcome = lambda *_args: None
    server.cancellable_wait = lambda *_args: None
    server.target_names = lambda _context: {"bottle"}
    server.publish_feedback = lambda *_args, **_kwargs: None
    spins = []
    server.execute_spin = lambda _goal, _context, angle: spins.append(angle) or READY
    server.candidates = SimpleNamespace(
        wait_for_candidate=lambda *_args, **_kwargs: sample_detection()
    )
    context = context_with_viewpoints()
    outcome = server.scan_and_monitor(
        SimpleNamespace(),
        context,
        "fast_scan",
        [0.0, 60.0, -60.0],
    )
    assert outcome == CANDIDATE
    assert context.scan_indices["fast_scan"] == 1
    assert spins == []


def test_navigation_failure_advances_to_next_viewpoint() -> None:
    """Special function: One failed named pose does not abort remaining coverage."""
    server = SimpleNamespace(
        guard_outcome=lambda *_args: None,
        publish_feedback=lambda *_args, **_kwargs: None,
    )
    context = context_with_viewpoints()
    board = Blackboard()
    board["context"] = context
    board["goal_handle"] = SimpleNamespace()
    assert MarkViewpointState(False).execute(board) == READY
    assert AdvanceSearchState().execute(board) == READY
    assert SelectViewpointState(server).execute(board) == READY
    assert context.current_viewpoint.name == "bedroom"


def test_strict_kitchen_plan_never_enters_other_rooms() -> None:
    """Special function: STRICT_LOCATION is an immutable room boundary."""
    plan = SearchPlanBuilder.build(
        search_config(),
        "bottle",
        "",
        "STRICT_LOCATION",
        "kitchen",
    )
    assert {item.room for item in plan} == {"kitchen"}
    assert [item.name for item in plan] == ["kitchen_table", "kitchen_center"]


def test_global_plan_covers_every_configured_room() -> None:
    """Special function: GLOBAL search includes semantic and exhaustive fallback rooms."""
    plan = SearchPlanBuilder.build(search_config(), "bottle", "", "GLOBAL", "")
    assert {item.room for item in plan} == {"kitchen", "bedroom", "entrance"}


def test_invalid_depth_returns_localization_failure_path() -> None:
    """Special function: A verified bbox with unusable depth is not NOT_FOUND."""
    detection = sample_detection().detection
    camera_info = CameraInfo()
    camera_info.k = [100.0, 0.0, 2.0, 0.0, 100.0, 2.0, 0.0, 0.0, 1.0]
    point, message = FindObjectSkillServer.project_detection(
        np.zeros((5, 5), dtype=np.uint16),
        "16UC1",
        Header(frame_id="camera"),
        detection,
        camera_info,
        {
            "depth_roi_width_px": 3,
            "depth_roi_height_px": 3,
            "min_valid_depth_ratio": 0.5,
            "min_range_m": 0.15,
            "max_range_m": 6.0,
        },
    )
    assert point is None
    assert "insufficient valid" in message


def test_blocked_standoff_uses_alternate_candidate() -> None:
    """Special function: A blocked primary angle leaves a plannable alternative."""
    server = object.__new__(FindObjectSkillServer)
    server.search_config = {
        "approach": {
            "candidate_angles_deg": [180.0, 0.0],
            "planner_timeout_sec": 1.0,
            "costmap_lethal_threshold": 252,
        }
    }
    server.fetch_costmap = lambda *_args: None
    server.guard_outcome = lambda *_args: None
    server.path_exists = lambda _goal, _context, pose: pose.pose.position.x > 0.0
    context = context_with_viewpoints()
    context.object_pose.pose.orientation.w = 1.0
    assert server.plan_standoff_candidates(SimpleNamespace(), context) == READY
    assert len(context.standoff_candidates) == 1
    assert context.standoff_candidates[0].pose.position.x == 1.0


def test_no_standoff_candidate_returns_found_unreachable() -> None:
    """Special function: Verified target with no path is FOUND_UNREACHABLE."""
    server = object.__new__(FindObjectSkillServer)
    server.search_config = {
        "approach": {
            "candidate_angles_deg": [180.0, 0.0],
            "planner_timeout_sec": 1.0,
            "costmap_lethal_threshold": 252,
        }
    }
    server.fetch_costmap = lambda *_args: None
    server.guard_outcome = lambda *_args: None
    server.path_exists = lambda *_args: False
    context = context_with_viewpoints()
    assert server.plan_standoff_candidates(SimpleNamespace(), context) == FOUND_UNREACHABLE


def test_mission_timeout_guard() -> None:
    """Special function: Expired mission deadline maps to TIMEOUT."""
    server = object.__new__(FindObjectSkillServer)
    server.skill_active = True
    server.deactivate_event = threading.Event()
    context = context_with_viewpoints()
    context.deadline = time.monotonic() - 0.1
    goal = SimpleNamespace(is_cancel_requested=False)
    assert server.guard_outcome(goal, context) == TIMEOUT


def test_mission_cancel_guard() -> None:
    """Special function: Goal cancellation preempts search and motion."""
    server = object.__new__(FindObjectSkillServer)
    server.skill_active = True
    server.deactivate_event = threading.Event()
    goal = SimpleNamespace(is_cancel_requested=True)
    assert server.guard_outcome(goal, context_with_viewpoints()) == CANCELED


def test_sliding_window_rejects_lifetime_counter_pattern() -> None:
    """Special function: Only the latest complete window determines confirmation."""
    assert SlidingWindowVerifier.confirmed([True, True, False, True, True], 5, 3)
    assert not SlidingWindowVerifier.confirmed([True, False, False, False, True], 5, 3)


def test_standoff_candidates_face_target() -> None:
    """Special function: Generated candidate orientation points toward the object."""
    target = PoseStamped()
    target.header.frame_id = "map"
    candidates = StandoffPlanner.generate(target, 1.0, [0.0])
    candidate = candidates[0]
    assert candidate.pose.position.x == 1.0
    assert abs(candidate.pose.orientation.z) == 1.0


def test_target_status_reports_verified_terminal_detection() -> None:
    """Special function: Target status retains verified object identity after success."""
    context = context_with_viewpoints()
    context.confirmed_target = sample_detection("bottle", 0.91)
    context.terminal_code = ResultCode.SUCCESS
    payload = target_status_payload("SUCCESS", context, terminal=True)
    assert payload["target_name"] == "bottle"
    assert payload["detected"] is True
    assert payload["verified"] is True
    assert payload["found"] is True
    assert abs(payload["confidence"] - 0.91) < 1e-6
    assert payload["terminal"] is True


def test_target_status_clears_unverified_detection_after_false_candidate() -> None:
    """Special function: Historical rejected candidates do not remain currently detected."""
    context = context_with_viewpoints()
    context.candidate_history.append(sample_detection("bottle", 0.80))
    payload = target_status_payload("NAVIGATE_AND_MONITOR", context)
    assert payload["detected"] is False
    assert payload["verified"] is False
    assert payload["found"] is False


def test_perception_readiness_requires_all_live_inputs() -> None:
    """Special function: ACTIVE alone cannot admit search without detector, depth, and intrinsics."""
    server = object.__new__(FindObjectSkillServer)
    # Section: Preserve the production default that requires 3D localization inputs.
    server.enable_depth = True
    server.detector_ready_event = threading.Event()
    server.candidates = CandidateStore()
    server.sensor_lock = threading.Lock()
    server.latest_depth = None
    server.latest_camera_info = None
    server.skill_active = True
    server.deactivate_event = threading.Event()
    goal_handle = SimpleNamespace(is_cancel_requested=False)
    context = context_with_viewpoints()

    ready, message = server.wait_for_perception_ready(0.02, goal_handle, context)
    assert not ready
    assert "detector_ready" in message
    assert "aligned_depth" in message
    assert "camera_info" in message

    server.detector_ready_event.set()
    server.candidates.update(ObjectDetections())
    server.latest_depth = Image()
    server.latest_camera_info = CameraInfo()

    ready, message = server.wait_for_perception_ready(0.02, goal_handle, context)
    assert ready
    assert message == "Object perception inputs are ready."


def test_rgb_only_readiness_does_not_require_depth_or_camera_info() -> None:
    """Special function: Admit live RGB detection when depth localization is disabled."""
    # Section: Build the minimum active server state used by the readiness barrier.
    server = object.__new__(FindObjectSkillServer)
    server.enable_depth = False
    server.detector_ready_event = threading.Event()
    server.detector_ready_event.set()
    server.candidates = CandidateStore()
    server.candidates.update(ObjectDetections())
    server.sensor_lock = threading.Lock()
    server.latest_depth = None
    server.latest_camera_info = None
    server.skill_active = True
    server.deactivate_event = threading.Event()
    goal_handle = SimpleNamespace(is_cancel_requested=False)
    context = context_with_viewpoints()

    # Section: A completed detector frame is sufficient in intentional RGB-only mode.
    ready, message = server.wait_for_perception_ready(0.02, goal_handle, context)
    assert ready
    assert message == "Object perception inputs are ready."


def test_readiness_wait_preserves_goal_timeout_outcome() -> None:
    """Special function: A deadline reached during readiness remains TIMEOUT."""
    # Section: The first guard admits planning; the second reports interruption after the wait.
    guard_outcomes = iter((None, TIMEOUT))
    server = SimpleNamespace(
        search_config={
            "rooms": {"kitchen": {"viewpoints": ["kitchen"]}},
            "verification": {"perception_startup_timeout_sec": 0.1},
        },
        guard_outcome=lambda *_args: next(guard_outcomes),
        wait_for_perception_ready=lambda *_args: (False, "interrupted by timeout"),
    )
    board = Blackboard()
    board["context"] = context_with_viewpoints()
    board["goal_handle"] = SimpleNamespace(is_cancel_requested=False)
    assert BuildSearchPlanState(server).execute(board) == TIMEOUT


class UnhashableServerGoalHandle:
    """Special function: Reproduce rclpy ServerGoalHandle hash behavior in unit tests."""

    __hash__ = None

    def __init__(self) -> None:
        """Special function: Track whether exception handling terminates the goal."""
        self.aborted = False

    def __eq__(self, other: object) -> bool:
        """Special function: Define equality so Python deliberately disables hashing."""
        return self is other

    def abort(self) -> None:
        """Special function: Record the expected exception terminal transition."""
        self.aborted = True


class UnhashableMotionGoalHandle:
    """Special function: Reproduce rclpy ClientGoalHandle hash and UUID behavior."""

    __hash__ = None

    def __init__(self, identifier: int) -> None:
        """Special function: Build one deterministic Nav2 goal UUID and cancel marker."""
        self.goal_id = SimpleNamespace(uuid=[identifier] * 16)
        self.cancel_requested = False

    def __eq__(self, other: object) -> bool:
        """Special function: Compare goal UUIDs like the real rclpy client handle."""
        return (
            isinstance(other, UnhashableMotionGoalHandle)
            and self.goal_id.uuid == other.goal_id.uuid
        )

    def cancel_goal_async(self) -> None:
        """Special function: Record lifecycle cancellation without a ROS action server."""
        self.cancel_requested = True


def test_execute_callback_accepts_unhashable_server_goal_and_releases_slot(
    monkeypatch,
) -> None:
    """Special function: A real-style server handle cannot crash mission registration."""

    class RaisingMission:
        """Special function: Force the callback through its exception and finally paths."""

        def __init__(self, _server: object) -> None:
            """Special function: Accept the production mission constructor contract."""
            return None

        def __call__(self, _blackboard: Blackboard) -> None:
            """Special function: Simulate a failure after parent-goal registration."""
            raise RuntimeError("planned regression failure")

    # Section: Replace only mission execution while preserving the production callback.
    monkeypatch.setattr(find_object_server_module, "FindObjectMissionSM", RaisingMission)
    server = object.__new__(FindObjectSkillServer)
    server.goal_lock = threading.Lock()
    server.active_goal_handle = None
    server.goal_admitted = True
    server.default_stand_off = 0.8
    server.cancel_all_motion = lambda: None
    server.publish_target_status = lambda *_args, **_kwargs: None
    server.get_logger = lambda: SimpleNamespace(error=lambda _message: None)
    goal_handle = UnhashableServerGoalHandle()

    result = asyncio.run(server.execute_callback(goal_handle))

    assert result.result_code == int(ResultCode.PERCEPTION_FAILED)
    assert goal_handle.aborted
    assert server.active_goal_handle is None
    assert server.goal_admitted is False


def test_unhashable_nav2_goals_are_tracked_and_canceled_by_uuid() -> None:
    """Special function: UUID dictionaries support unhashable Nav2 client handles."""
    server = object.__new__(FindObjectSkillServer)
    server.motion_lock = threading.Lock()
    server.active_motion_goals = {}
    first_goal = UnhashableMotionGoalHandle(1)
    second_goal = UnhashableMotionGoalHandle(2)

    server.track_motion(first_goal)
    server.track_motion(second_goal)
    assert len(server.active_motion_goals) == 2

    server.cancel_all_motion()
    assert first_goal.cancel_requested
    assert second_goal.cancel_requested
    assert server.active_motion_goals == {}


def test_admitted_goal_blocks_lifecycle_cleanup_before_execute_starts() -> None:
    """Special function: The accepted-before-execute window retains mission ownership."""
    server = object.__new__(FindObjectSkillServer)
    server.goal_lock = threading.Lock()
    server.active_goal_handle = None
    server.goal_admitted = True
    assert server.has_active_goal()


def test_lifecycle_cleanup_releases_find_object_resources() -> None:
    """Special function: Deactivate and cleanup restore a strict UNCONFIGURED baseline."""
    if not rclpy.ok():
        rclpy.init()
    node = FindObjectSkillServer()
    config_path = Path(__file__).parents[1] / "config" / "find_object_search.yaml"
    node.set_parameters([
        Parameter("search_config_file", value=str(config_path)),
    ])
    try:
        assert node.on_configure(None) == TransitionCallbackReturn.SUCCESS
        assert node.action_server is not None
        assert node.on_activate(None) == TransitionCallbackReturn.SUCCESS
        assert node.skill_active
        assert node.on_deactivate(None) == TransitionCallbackReturn.SUCCESS
        assert node.on_cleanup(None) == TransitionCallbackReturn.SUCCESS
        assert node.action_server is None
        assert node.named_nav_client is None
        assert node.detection_sub is None
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
