#!/usr/bin/env python3
"""Section: Lifecycle-managed semantic-first exhaustive FIND_OBJECT mission."""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from enum import IntEnum
from pathlib import Path
from typing import Any, Callable

import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration as DurationMsg
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin
from nav2_msgs.srv import GetCostmap
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State as LifecycleState, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from receptionist_interfaces.action import NavigateNamedPose
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from yasmin import Blackboard, State, StateMachine

from putrabot_interfaces.msg import ObjectDetection, ObjectDetections
from robot_ai_interfaces.action import FindObject


# Section: Internal state-machine outcomes.
READY = "ready"
ARRIVED = "arrived"
CANDIDATE = "candidate"
REJECTED = "rejected"
EXHAUSTED = "exhausted"
COMPLETE = "complete"
NAV_FAILED = "nav_failed"
PERCEPTION_FAILED = "perception_failed"
LOCALIZATION_FAILED = "localization_failed"
FOUND_UNREACHABLE = "found_unreachable"
TIMEOUT = "timeout"
CANCELED = "canceled"


class ResultCode(IntEnum):
    """Special function: Mirror the stable FindObject action result semantics."""

    SUCCESS = 0
    NOT_FOUND = 1
    FOUND_UNREACHABLE = 2
    LOCALIZATION_FAILED = 3
    NAVIGATION_FAILED = 4
    PERCEPTION_FAILED = 5
    TIMEOUT = 6
    CANCELED = 7


class ViewpointStatus:
    """Special function: Define searchable viewpoint progress labels."""

    UNVISITED = "UNVISITED"
    NAVIGATING = "NAVIGATING"
    INTERRUPTED = "INTERRUPTED"
    ARRIVED = "ARRIVED"
    SCANNING = "SCANNING"
    SEARCHED = "SEARCHED"
    NAV_FAILED = "NAV_FAILED"
    SKIPPED = "SKIPPED"


@dataclass(slots=True)
class DetectionSample:
    """Special function: Bind a detection to its sensor and receipt timestamps."""

    detection: ObjectDetection
    stamp: Any
    received_at: float


@dataclass(slots=True)
class SearchViewpoint:
    """Special function: Track one configured named-pose search location."""

    name: str
    room: str
    tier: int
    status: str = ViewpointStatus.UNVISITED


@dataclass(slots=True)
class SearchContext:
    """Special function: Hold all Strategy 2B mission decisions outside Nav2."""

    target: str
    category: str
    scope: str
    requested_location: str
    approach_target: bool
    stand_off_distance: float
    start_time: float
    deadline: float
    current_tier: int = 1
    viewpoints: list[SearchViewpoint] = field(default_factory=list)
    visited_viewpoints: list[str] = field(default_factory=list)
    navigation_failures: list[str] = field(default_factory=list)
    current_viewpoint: SearchViewpoint | None = None
    interrupted_viewpoint: SearchViewpoint | None = None
    candidate_history: list[DetectionSample] = field(default_factory=list)
    confirmed_target: DetectionSample | None = None
    candidate_source: str = ""
    scan_indices: dict[str, int] = field(default_factory=dict)
    scan_heading_deg: float = 0.0
    object_pose: PoseStamped = field(default_factory=PoseStamped)
    observation_pose: PoseStamped = field(default_factory=PoseStamped)
    standoff_candidates: list[PoseStamped] = field(default_factory=list)
    approached: bool = False
    terminal_code: ResultCode = ResultCode.NOT_FOUND
    terminal_message: str = ""


def clean_symbol(value: Any) -> str:
    """Special function: Normalize spoken/configured labels for deterministic lookup."""
    return str(value or "").strip().lower().replace("-", "_").replace(" ", "_")


def target_status_payload(
    phase: str,
    context: SearchContext | None = None,
    confidence: float = 0.0,
    terminal: bool = False,
) -> dict[str, Any]:
    """Special function: Build one stable JSON contract for target-search observers."""
    # Section: Keep an explicit empty status valid before the first mission is admitted.
    if context is None:
        return {
            "phase": str(phase),
            "target_name": "",
            "category": "",
            "location": "",
            "detected": False,
            "verified": False,
            "found": False,
            "confidence": float(confidence),
            "object_id": "",
            "found_room": "",
            "found_viewpoint": "",
            "result_code": -1,
            "terminal": bool(terminal),
        }

    # Section: Derive live detection and terminal semantics from authoritative mission state.
    confirmed = context.confirmed_target
    live_detection = confirmed is not None or float(confidence) > 0.0
    effective_confidence = (
        float(confidence)
        if float(confidence) > 0.0
        else float(confirmed.detection.confidence) if confirmed is not None else 0.0
    )
    found = context.terminal_code in {
        ResultCode.SUCCESS,
        ResultCode.FOUND_UNREACHABLE,
        ResultCode.LOCALIZATION_FAILED,
    } if terminal else False
    viewpoint = context.current_viewpoint or context.interrupted_viewpoint
    object_id = ""
    if confirmed is not None:
        detection = confirmed.detection
        object_id = (
            f"{clean_symbol(detection.name)}_"
            f"{int(detection.center_u)}_{int(detection.center_v)}"
        )
    return {
        "phase": str(phase),
        "target_name": str(context.target),
        "category": str(context.category),
        "location": str(context.requested_location),
        "detected": live_detection,
        "verified": confirmed is not None,
        "found": bool(found),
        "confidence": effective_confidence,
        "object_id": object_id,
        "found_room": viewpoint.room if viewpoint is not None and found else "",
        "found_viewpoint": viewpoint.name if viewpoint is not None and found else "",
        "result_code": int(context.terminal_code) if terminal else -1,
        "terminal": bool(terminal),
    }


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Special function: Convert a planar yaw angle to a ROS quaternion."""
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw / 2.0)
    quaternion.w = math.cos(yaw / 2.0)
    return quaternion


def duration_msg(seconds: float) -> DurationMsg:
    """Special function: Convert floating seconds into a ROS duration message."""
    bounded = max(0.0, float(seconds))
    message = DurationMsg()
    message.sec = int(bounded)
    message.nanosec = int((bounded - message.sec) * 1e9)
    return message


def normalize_delta_degrees(value: float) -> float:
    """Special function: Select the shortest commanded scan rotation."""
    return (float(value) + 180.0) % 360.0 - 180.0


class SearchPlanBuilder:
    """Special function: Build semantic-priority plans with exhaustive fallback."""

    @staticmethod
    def build(config: dict[str, Any], target: str, category: str, scope: str, location: str) -> list[SearchViewpoint]:
        """Special function: Return ordered unique named poses without inventing coordinates."""
        rooms = dict(config.get("rooms", {}))
        normalized_scope = str(scope or "GLOBAL").upper()
        if normalized_scope == "STRICT_LOCATION":
            room_data = dict(rooms.get(location, {}))
            return [
                SearchViewpoint(str(name), location, 1)
                for name in room_data.get("viewpoints", [])
                if str(name).strip()
            ]

        # Section: Object priors override category priors but never remove fallback rooms.
        object_priors = dict(config.get("object_priors", {}))
        category_priors = dict(config.get("category_priors", {}))
        likely_rooms = list(object_priors.get(target, []))
        if not likely_rooms:
            likely_rooms = list(category_priors.get(category, []))
        likely_rooms = [room for room in likely_rooms if room in rooms]
        remaining_rooms = [room for room in rooms if room not in likely_rooms]
        plan: list[SearchViewpoint] = []

        # Section: Tier 1 takes the best configured viewpoint in each likely room.
        for room in likely_rooms:
            viewpoints = list(dict(rooms[room]).get("viewpoints", []))
            if viewpoints:
                plan.append(SearchViewpoint(str(viewpoints[0]), room, 1))

        # Section: Tier 2 exhausts every remaining viewpoint in likely rooms.
        for room in likely_rooms:
            viewpoints = list(dict(rooms[room]).get("viewpoints", []))
            plan.extend(SearchViewpoint(str(name), room, 2) for name in viewpoints[1:])

        # Section: Tier 3 samples secondary rooms before Tier 4 exhaustive remainder.
        for room in remaining_rooms:
            viewpoints = list(dict(rooms[room]).get("viewpoints", []))
            if viewpoints:
                plan.append(SearchViewpoint(str(viewpoints[0]), room, 3))
        for room in remaining_rooms:
            viewpoints = list(dict(rooms[room]).get("viewpoints", []))
            plan.extend(SearchViewpoint(str(name), room, 4) for name in viewpoints[1:])

        # Section: Deduplicate aliases while preserving semantic order.
        unique: list[SearchViewpoint] = []
        seen: set[str] = set()
        for viewpoint in plan:
            if viewpoint.name and viewpoint.name not in seen:
                unique.append(viewpoint)
                seen.add(viewpoint.name)
        return unique


class SlidingWindowVerifier:
    """Special function: Evaluate stationary confirmation frames without lifetime counters."""

    @staticmethod
    def confirmed(hits: list[bool], window_frames: int, minimum_hits: int) -> bool:
        """Special function: Confirm only the bounded latest frame window."""
        bounded = list(hits[-max(1, int(window_frames)):])
        return len(bounded) >= max(1, int(window_frames)) and sum(bounded) >= int(minimum_hits)


class StandoffPlanner:
    """Special function: Generate map-frame approach candidates that face the target."""

    @staticmethod
    def generate(target_pose: PoseStamped, distance: float, angles_deg: list[float]) -> list[PoseStamped]:
        """Special function: Place candidates around a target at the requested distance."""
        candidates: list[PoseStamped] = []
        target_x = float(target_pose.pose.position.x)
        target_y = float(target_pose.pose.position.y)
        for angle_deg in angles_deg:
            radial = math.radians(float(angle_deg))
            pose = PoseStamped()
            pose.header = target_pose.header
            pose.pose.position.x = target_x + float(distance) * math.cos(radial)
            pose.pose.position.y = target_y + float(distance) * math.sin(radial)
            pose.pose.position.z = 0.0
            facing_yaw = math.atan2(
                target_y - pose.pose.position.y,
                target_x - pose.pose.position.x,
            )
            pose.pose.orientation = yaw_to_quaternion(facing_yaw)
            candidates.append(pose)
        return candidates


class CandidateStore:
    """Special function: Cache complete detection frames for moving and stationary checks."""

    def __init__(self) -> None:
        """Special function: Initialize an empty synchronized detection stream."""
        self._condition = threading.Condition()
        self._sequence = 0
        self._received_at = 0.0
        self._detections: list[ObjectDetection] = []

    def update(self, message: ObjectDetections) -> None:
        """Special function: Store one detector frame, including empty frames."""
        with self._condition:
            self._sequence += 1
            self._received_at = time.monotonic()
            self._detections = list(message.detections)
            self._condition.notify_all()

    def wait_for_first_frame(self, timeout_sec: float, canceled: Callable[[], bool]) -> bool:
        """Special function: Verify that perception is producing frames before search."""
        deadline = time.monotonic() + max(0.0, timeout_sec)
        with self._condition:
            while self._sequence == 0 and time.monotonic() < deadline:
                if canceled():
                    return False
                self._condition.wait(timeout=min(0.1, max(0.0, deadline - time.monotonic())))
            return self._sequence > 0

    def has_frame(self) -> bool:
        """Special function: Report whether at least one complete detector frame arrived."""
        with self._condition:
            return self._sequence > 0

    def latest_match(self, names: set[str], min_confidence: float, since: float = 0.0) -> DetectionSample | None:
        """Special function: Return the best matching target in the latest fresh frame."""
        with self._condition:
            if self._received_at < since:
                return None
            detections = list(self._detections)
            received_at = self._received_at
        matches = [
            detection for detection in detections
            if float(detection.confidence) >= min_confidence
            and (not names or clean_symbol(detection.name) in names)
        ]
        if not matches:
            return None
        best = max(matches, key=lambda detection: float(detection.confidence))
        return DetectionSample(best, best.stamp, received_at)

    def wait_for_candidate(
        self,
        names: set[str],
        min_confidence: float,
        timeout_sec: float,
        canceled: Callable[[], bool],
        since: float,
    ) -> DetectionSample | None:
        """Special function: Wait briefly for one candidate frame after motion settles."""
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while time.monotonic() < deadline:
            if canceled():
                return None
            sample = self.latest_match(names, min_confidence, since)
            if sample is not None:
                return sample
            with self._condition:
                self._condition.wait(timeout=min(0.05, max(0.0, deadline - time.monotonic())))
        return None

    def collect_confirmation(
        self,
        names: set[str],
        min_confidence: float,
        window_frames: int,
        minimum_hits: int,
        timeout_sec: float,
        last_seen_timeout_sec: float,
        canceled: Callable[[], bool],
    ) -> tuple[bool, DetectionSample | None, bool]:
        """Special function: Collect a new sliding window and report stream availability."""
        deadline = time.monotonic() + max(0.0, timeout_sec)
        with self._condition:
            last_sequence = self._sequence
        hits: deque[bool] = deque(maxlen=max(1, int(window_frames)))
        best: DetectionSample | None = None
        saw_frame = False
        while time.monotonic() < deadline:
            if canceled():
                return False, best, saw_frame
            with self._condition:
                if self._sequence == last_sequence:
                    self._condition.wait(timeout=0.1)
                if self._sequence == last_sequence:
                    continue
                last_sequence = self._sequence
                received_at = self._received_at
                detections = list(self._detections)
            saw_frame = True
            matches = [
                detection for detection in detections
                if float(detection.confidence) >= min_confidence
                and (not names or clean_symbol(detection.name) in names)
            ]
            hit = bool(matches)
            hits.append(hit)
            if matches:
                detection = max(matches, key=lambda item: float(item.confidence))
                best = DetectionSample(detection, detection.stamp, received_at)
            if SlidingWindowVerifier.confirmed(list(hits), window_frames, minimum_hits):
                return True, best, True
            if len(hits) >= max(1, int(window_frames)) and not hit:
                newest_hit_age = (
                    time.monotonic() - best.received_at if best is not None else math.inf
                )
                if newest_hit_age > last_seen_timeout_sec:
                    return False, best, True
        return False, best, saw_frame


class InitialState(State):
    """Special function: Validate the goal and establish one bounded SearchContext."""

    def __init__(self, server: "FindObjectSkillServer") -> None:
        """Special function: Store the lifecycle server dependency."""
        super().__init__([READY, PERCEPTION_FAILED, CANCELED, TIMEOUT])
        self.server = server

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Normalize goal fields without allocating persistent resources."""
        goal_handle = blackboard["goal_handle"]
        goal = goal_handle.request
        target = clean_symbol(goal.object_name)
        category = clean_symbol(goal.category)
        scope = str(goal.search_scope or ("STRICT_LOCATION" if goal.location else "GLOBAL")).upper()
        location = clean_symbol(goal.location)
        if not target and not category:
            return PERCEPTION_FAILED
        if scope not in {"GLOBAL", "STRICT_LOCATION"}:
            return PERCEPTION_FAILED
        if scope == "STRICT_LOCATION" and not location:
            return PERCEPTION_FAILED
        timeout_sec = float(goal.timeout_sec) or float(self.server.default_timeout_sec)
        stand_off = float(goal.stand_off_distance) or float(self.server.default_stand_off)
        now = time.monotonic()
        blackboard["context"] = SearchContext(
            target=target,
            category=category,
            scope=scope,
            requested_location=location,
            approach_target=bool(goal.approach_target),
            stand_off_distance=stand_off,
            start_time=now,
            deadline=now + timeout_sec,
        )
        guard = self.server.guard_outcome(goal_handle, blackboard["context"])
        return guard or READY


class BuildSearchPlanState(State):
    """Special function: Build semantic-first order and verify perception availability."""

    def __init__(self, server: "FindObjectSkillServer") -> None:
        """Special function: Store the lifecycle server dependency."""
        super().__init__([READY, NAV_FAILED, PERCEPTION_FAILED, CANCELED, TIMEOUT])
        self.server = server

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Create the complete eligible viewpoint plan from YAML."""
        context = blackboard["context"]
        goal_handle = blackboard["goal_handle"]
        guard = self.server.guard_outcome(goal_handle, context)
        if guard:
            return guard
        context.viewpoints = SearchPlanBuilder.build(
            self.server.search_config,
            context.target,
            context.category,
            context.scope,
            context.requested_location,
        )
        if not context.viewpoints:
            context.terminal_message = (
                f"No recorded search viewpoints exist for {context.requested_location or 'global search'}."
            )
            return NAV_FAILED
        verification = dict(self.server.search_config.get("verification", {}))
        startup_timeout = float(verification.get("perception_startup_timeout_sec", 20.0))
        available, readiness_message = self.server.wait_for_perception_ready(
            startup_timeout,
            goal_handle,
            context,
        )
        if not available:
            context.terminal_message = readiness_message
            # Section: Preserve cancellation and mission-timeout semantics during startup waits.
            interrupted_outcome = self.server.guard_outcome(goal_handle, context)
            return interrupted_outcome or PERCEPTION_FAILED
        return READY


class SelectViewpointState(State):
    """Special function: Select an interrupted goal first, then the next unvisited pose."""

    def __init__(self, server: "FindObjectSkillServer") -> None:
        """Special function: Store the lifecycle server dependency."""
        super().__init__([READY, EXHAUSTED, NAV_FAILED, CANCELED, TIMEOUT])
        self.server = server

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Preserve strict boundaries while advancing search order."""
        context = blackboard["context"]
        goal_handle = blackboard["goal_handle"]
        guard = self.server.guard_outcome(goal_handle, context)
        if guard:
            return guard
        if (
            context.interrupted_viewpoint is not None
            and context.interrupted_viewpoint.status == ViewpointStatus.INTERRUPTED
        ):
            context.current_viewpoint = context.interrupted_viewpoint
        else:
            context.current_viewpoint = next(
                (item for item in context.viewpoints if item.status == ViewpointStatus.UNVISITED),
                None,
            )
        if context.current_viewpoint is None:
            searched = any(item.status == ViewpointStatus.SEARCHED for item in context.viewpoints)
            return EXHAUSTED if searched else NAV_FAILED
        context.current_tier = context.current_viewpoint.tier
        context.current_viewpoint.status = ViewpointStatus.NAVIGATING
        context.scan_indices.clear()
        context.scan_heading_deg = 0.0
        self.server.publish_feedback(goal_handle, context, "NAVIGATE_AND_MONITOR")
        return READY


class NavigateAndMonitorState(State):
    """Special function: Monitor perception continuously while NavigateNamedPose is active."""

    def __init__(self, server: "FindObjectSkillServer") -> None:
        """Special function: Store the lifecycle server dependency."""
        super().__init__([ARRIVED, CANDIDATE, NAV_FAILED, PERCEPTION_FAILED, CANCELED, TIMEOUT])
        self.server = server

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Cancel named navigation on a moving target candidate."""
        context = blackboard["context"]
        goal_handle = blackboard["goal_handle"]
        return self.server.navigate_named_and_monitor(goal_handle, context)


class ControlledScanState(State):
    """Special function: Execute configured desired headings through Nav2 Spin."""

    def __init__(self, server: "FindObjectSkillServer", phase: str, config_key: str) -> None:
        """Special function: Store the scan tier and YAML angle key."""
        super().__init__([CANDIDATE, EXHAUSTED, NAV_FAILED, CANCELED, TIMEOUT])
        self.server = server
        self.phase = phase
        self.config_key = config_key

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Stop after each heading and return immediately on a candidate."""
        context = blackboard["context"]
        goal_handle = blackboard["goal_handle"]
        angles = list(dict(self.server.search_config.get("scan", {})).get(self.config_key, []))
        return self.server.scan_and_monitor(goal_handle, context, self.phase, angles)


class VerifyTargetState(State):
    """Special function: Reject motion-only detections using stationary sliding-window evidence."""

    def __init__(self, server: "FindObjectSkillServer") -> None:
        """Special function: Store the lifecycle server dependency."""
        super().__init__([
            READY,
            "resume_navigation",
            "resume_fast_scan",
            "resume_extended_scan",
            "resume_full_scan",
            PERCEPTION_FAILED,
            CANCELED,
            TIMEOUT,
        ])
        self.server = server

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Settle, collect fresh frames, and route rejected candidates."""
        context = blackboard["context"]
        goal_handle = blackboard["goal_handle"]
        scan_config = dict(self.server.search_config.get("scan", {}))
        verify_config = dict(self.server.search_config.get("verification", {}))
        guard = self.server.cancellable_wait(
            float(scan_config.get("settle_interval_sec", 0.5)),
            goal_handle,
            context,
        )
        if guard:
            return guard
        verified, sample, saw_frame = self.server.candidates.collect_confirmation(
            self.server.target_names(context),
            float(verify_config.get("min_confidence", 0.35)),
            int(verify_config.get("window_frames", 5)),
            int(verify_config.get("minimum_hits", 3)),
            float(verify_config.get("confirmation_timeout_sec", 3.0)),
            float(verify_config.get("last_seen_timeout_sec", 1.0)),
            lambda: bool(self.server.guard_outcome(goal_handle, context)),
        )
        if self.server.guard_outcome(goal_handle, context):
            return self.server.guard_outcome(goal_handle, context) or CANCELED
        if not saw_frame:
            return PERCEPTION_FAILED
        if verified and sample is not None:
            context.confirmed_target = sample
            context.candidate_history.append(sample)
            self.server.publish_feedback(
                goal_handle,
                context,
                "VERIFY_TARGET",
                float(sample.detection.confidence),
            )
            return READY
        # Section: A false moving candidate resumes the interrupted named goal.
        if context.candidate_source == "navigation":
            if context.current_viewpoint is not None:
                context.current_viewpoint.status = ViewpointStatus.INTERRUPTED
                context.interrupted_viewpoint = context.current_viewpoint
            return "resume_navigation"
        return f"resume_{context.candidate_source.lower()}"


class LocalizeTargetState(State):
    """Special function: Convert confirmed bbox plus aligned depth into a map pose."""

    def __init__(self, server: "FindObjectSkillServer") -> None:
        """Special function: Store the lifecycle server dependency."""
        super().__init__([READY, LOCALIZATION_FAILED, CANCELED, TIMEOUT])
        self.server = server

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Preserve localization failures as distinct terminal results."""
        context = blackboard["context"]
        goal_handle = blackboard["goal_handle"]
        guard = self.server.guard_outcome(goal_handle, context)
        if guard:
            return guard
        # Section: RGB-only mode terminates successfully after verification without inventing a 3D pose.
        if not self.server.enable_depth:
            context.approach_target = False
            context.terminal_message = "Target found by RGB detection; depth localization and approach were disabled."
            return READY
        success, message = self.server.localize_confirmed_target(context)
        if not success:
            context.terminal_message = message
            return LOCALIZATION_FAILED
        return READY


class PlanStandoffState(State):
    """Special function: Filter stand-off poses through occupancy and Nav2 planning."""

    def __init__(self, server: "FindObjectSkillServer") -> None:
        """Special function: Store the lifecycle server dependency."""
        super().__init__([READY, FOUND_UNREACHABLE, CANCELED, TIMEOUT])
        self.server = server

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Retain every safe alternative candidate for approach recovery."""
        context = blackboard["context"]
        goal_handle = blackboard["goal_handle"]
        if not context.approach_target:
            return READY
        outcome = self.server.plan_standoff_candidates(goal_handle, context)
        return outcome


class ApproachTargetState(State):
    """Special function: Use NavigateToPose for the selected dynamic target approach."""

    def __init__(self, server: "FindObjectSkillServer") -> None:
        """Special function: Store the lifecycle server dependency."""
        super().__init__([READY, FOUND_UNREACHABLE, CANCELED, TIMEOUT])
        self.server = server

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Try safe alternatives without recreating Nav2 control logic."""
        context = blackboard["context"]
        goal_handle = blackboard["goal_handle"]
        if not context.approach_target:
            return READY
        return self.server.approach_candidates(goal_handle, context)


class MarkViewpointState(State):
    """Special function: Commit a fully scanned or navigation-failed viewpoint."""

    def __init__(self, searched: bool) -> None:
        """Special function: Select the committed viewpoint status."""
        super().__init__([READY])
        self.searched = searched

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Never mark an interrupted false candidate as searched."""
        context = blackboard["context"]
        viewpoint = context.current_viewpoint
        if viewpoint is not None:
            viewpoint.status = (
                ViewpointStatus.SEARCHED if self.searched else ViewpointStatus.NAV_FAILED
            )
            if viewpoint.name not in context.visited_viewpoints:
                context.visited_viewpoints.append(viewpoint.name)
            if not self.searched:
                context.navigation_failures.append(viewpoint.name)
        return READY


class AdvanceSearchState(State):
    """Special function: Clear per-viewpoint state before selecting the next pose."""

    def __init__(self) -> None:
        """Special function: Declare the deterministic advance outcome."""
        super().__init__([READY])

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Preserve history while releasing the active viewpoint."""
        context = blackboard["context"]
        context.current_viewpoint = None
        context.interrupted_viewpoint = None
        context.candidate_source = ""
        context.confirmed_target = None
        context.scan_indices.clear()
        context.scan_heading_deg = 0.0
        return READY


class TerminalState(State):
    """Special function: Store one semantic terminal result for the action boundary."""

    def __init__(self, code: ResultCode, default_message: str) -> None:
        """Special function: Store terminal code and fallback message."""
        super().__init__([COMPLETE])
        self.code = code
        self.default_message = default_message

    def execute(self, blackboard: Blackboard) -> str:
        """Special function: Finalize context without changing ROS goal status directly."""
        context = blackboard["context"]
        context.terminal_code = self.code
        if not context.terminal_message:
            context.terminal_message = self.default_message
        return COMPLETE


class FindObjectMissionSM(StateMachine):
    """Special function: Own Strategy 2B task decisions while delegating motion to Nav2."""

    def __init__(self, server: "FindObjectSkillServer") -> None:
        """Special function: Build the required compound search state graph."""
        super().__init__(outcomes=[COMPLETE])
        self.set_name("FIND_OBJECT_STRATEGY_2B")
        self.add_state("INITIALIZE", InitialState(server), transitions={READY: "BUILD_SEARCH_PLAN", PERCEPTION_FAILED: "PERCEPTION_FAILURE", CANCELED: "CANCELED", TIMEOUT: "TIMEOUT"})
        self.add_state("BUILD_SEARCH_PLAN", BuildSearchPlanState(server), transitions={READY: "SELECT_VIEWPOINT", NAV_FAILED: "NAVIGATION_FAILURE", PERCEPTION_FAILED: "PERCEPTION_FAILURE", CANCELED: "CANCELED", TIMEOUT: "TIMEOUT"})
        self.add_state("SELECT_VIEWPOINT", SelectViewpointState(server), transitions={READY: "NAVIGATE_AND_MONITOR", EXHAUSTED: "NOT_FOUND", NAV_FAILED: "NAVIGATION_FAILURE", CANCELED: "CANCELED", TIMEOUT: "TIMEOUT"})
        self.add_state("NAVIGATE_AND_MONITOR", NavigateAndMonitorState(server), transitions={ARRIVED: "FAST_SCAN", CANDIDATE: "VERIFY_TARGET", NAV_FAILED: "MARK_NAV_FAILED", PERCEPTION_FAILED: "PERCEPTION_FAILURE", CANCELED: "CANCELED", TIMEOUT: "TIMEOUT"})
        self.add_state("FAST_SCAN", ControlledScanState(server, "fast_scan", "fast_angles_deg"), transitions={CANDIDATE: "VERIFY_TARGET", EXHAUSTED: "EXTENDED_SCAN", NAV_FAILED: "EXTENDED_SCAN", CANCELED: "CANCELED", TIMEOUT: "TIMEOUT"})
        self.add_state("EXTENDED_SCAN", ControlledScanState(server, "extended_scan", "extended_angles_deg"), transitions={CANDIDATE: "VERIFY_TARGET", EXHAUSTED: "FULL_SCAN", NAV_FAILED: "FULL_SCAN", CANCELED: "CANCELED", TIMEOUT: "TIMEOUT"})
        self.add_state("FULL_SCAN", ControlledScanState(server, "full_scan", "exhaustive_angles_deg"), transitions={CANDIDATE: "VERIFY_TARGET", EXHAUSTED: "MARK_VIEWPOINT", NAV_FAILED: "MARK_VIEWPOINT", CANCELED: "CANCELED", TIMEOUT: "TIMEOUT"})
        self.add_state("VERIFY_TARGET", VerifyTargetState(server), transitions={READY: "LOCALIZE_TARGET", "resume_navigation": "SELECT_VIEWPOINT", "resume_fast_scan": "FAST_SCAN", "resume_extended_scan": "EXTENDED_SCAN", "resume_full_scan": "FULL_SCAN", PERCEPTION_FAILED: "PERCEPTION_FAILURE", CANCELED: "CANCELED", TIMEOUT: "TIMEOUT"})
        self.add_state("LOCALIZE_TARGET", LocalizeTargetState(server), transitions={READY: "PLAN_STANDOFF", LOCALIZATION_FAILED: "LOCALIZATION_FAILURE", CANCELED: "CANCELED", TIMEOUT: "TIMEOUT"})
        self.add_state("PLAN_STANDOFF", PlanStandoffState(server), transitions={READY: "APPROACH_TARGET", FOUND_UNREACHABLE: "UNREACHABLE", CANCELED: "CANCELED", TIMEOUT: "TIMEOUT"})
        self.add_state("APPROACH_TARGET", ApproachTargetState(server), transitions={READY: "SUCCESS", FOUND_UNREACHABLE: "UNREACHABLE", CANCELED: "CANCELED", TIMEOUT: "TIMEOUT"})
        self.add_state("MARK_VIEWPOINT", MarkViewpointState(True), transitions={READY: "ADVANCE_SEARCH"})
        self.add_state("MARK_NAV_FAILED", MarkViewpointState(False), transitions={READY: "ADVANCE_SEARCH"})
        self.add_state("ADVANCE_SEARCH", AdvanceSearchState(), transitions={READY: "SELECT_VIEWPOINT"})
        self.add_state("SUCCESS", TerminalState(ResultCode.SUCCESS, "Target found."), transitions={COMPLETE: COMPLETE})
        self.add_state("NOT_FOUND", TerminalState(ResultCode.NOT_FOUND, "All eligible viewpoints were exhausted."), transitions={COMPLETE: COMPLETE})
        self.add_state("UNREACHABLE", TerminalState(ResultCode.FOUND_UNREACHABLE, "Target found but no safe stand-off pose was reachable."), transitions={COMPLETE: COMPLETE})
        self.add_state("LOCALIZATION_FAILURE", TerminalState(ResultCode.LOCALIZATION_FAILED, "Target verified but localization failed."), transitions={COMPLETE: COMPLETE})
        self.add_state("NAVIGATION_FAILURE", TerminalState(ResultCode.NAVIGATION_FAILED, "No eligible viewpoint could be navigated."), transitions={COMPLETE: COMPLETE})
        self.add_state("PERCEPTION_FAILURE", TerminalState(ResultCode.PERCEPTION_FAILED, "Object perception is unavailable."), transitions={COMPLETE: COMPLETE})
        self.add_state("TIMEOUT", TerminalState(ResultCode.TIMEOUT, "FindObject mission timed out."), transitions={COMPLETE: COMPLETE})
        self.add_state("CANCELED", TerminalState(ResultCode.CANCELED, "FindObject mission canceled."), transitions={COMPLETE: COMPLETE})


class FindObjectSkillServer(LifecycleNode):
    """Special function: Expose complete FIND_OBJECT only while lifecycle ACTIVE."""

    def __init__(self) -> None:
        """Special function: Declare stable configuration without allocating skill resources."""
        super().__init__("find_object_skill_server")
        # Section: Lifecycle-independent parameters define resource interface names.
        self.cb_group = ReentrantCallbackGroup()
        self.declare_parameter("action_name", "/gpsr/find_object")
        self.declare_parameter("detections_topic", "/perception/raw_object_detections")
        self.declare_parameter("enable_depth", True)
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("named_navigation_action", "/nav/navigate_named_pose")
        self.declare_parameter("navigate_to_pose_action", "/navigate_to_pose")
        self.declare_parameter("spin_action", "/spin")
        self.declare_parameter("compute_path_action", "/compute_path_to_pose")
        self.declare_parameter("global_costmap_service", "/global_costmap/get_costmap")
        self.declare_parameter("target_status_topic", "/gpsr/find_object/target_status")
        self.declare_parameter("detector_ready_topic", "/perception/object_detector_ready")
        self.declare_parameter("search_config_file", self.default_search_config_path())
        self.declare_parameter("default_timeout_sec", 300.0)

        # Section: Handles stay empty in UNCONFIGURED.
        self.skill_active = False
        self.search_config: dict[str, Any] = {}
        self.enable_depth = True
        self.default_timeout_sec = 300.0
        self.default_stand_off = 0.8
        self.candidates = CandidateStore()
        self.bridge: CvBridge | None = None
        self.detection_sub = None
        self.depth_sub = None
        self.camera_info_sub = None
        self.detector_ready_sub = None
        self.detector_ready_event = threading.Event()
        self.tf_buffer: Buffer | None = None
        self.tf_listener: TransformListener | None = None
        self.named_nav_client = None
        self.nav_pose_client = None
        self.spin_client = None
        self.path_client = None
        self.costmap_client = None
        self.action_server = None
        self.latest_depth: Image | None = None
        self.latest_camera_info: CameraInfo | None = None
        self.sensor_lock = threading.Lock()
        # Section: Protect single-mission admission independently from Nav2 child goals.
        self.goal_lock = threading.Lock()
        self.motion_lock = threading.Lock()
        # Section: ROS client goal handles are unhashable, so immutable UUID bytes own them.
        self.active_motion_goals: dict[bytes, Any] = {}
        # Section: FIND_OBJECT admits exactly one parent action goal at a time.
        self.active_goal_handle: Any | None = None
        self.deactivate_event = threading.Event()
        self.goal_admitted = False
        # Section: Retain the latest target state across transient skill cleanup.
        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.target_status_pub = self.create_publisher(
            String,
            str(self.get_parameter("target_status_topic").value),
            status_qos,
        )
        self.publish_target_status("IDLE")
        self.get_logger().info("FindObject lifecycle server is UNCONFIGURED.")

    def default_search_config_path(self) -> str:
        """Special function: Resolve the installed Strategy 2B YAML path."""
        try:
            return str(Path(get_package_share_directory("robot_yasmin_tasks")) / "config" / "find_object_search.yaml")
        except Exception:
            return ""

    def on_configure(self, _state: LifecycleState) -> TransitionCallbackReturn:
        """Special function: Allocate every FIND_OBJECT-specific ROS resource."""
        try:
            # Section: Load policy before exposing the action server.
            config_path = Path(str(self.get_parameter("search_config_file").value))
            self.search_config = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
            self.enable_depth = bool(self.get_parameter("enable_depth").value)
            self.default_timeout_sec = float(self.get_parameter("default_timeout_sec").value)
            self.default_stand_off = float(
                dict(self.search_config.get("approach", {})).get(
                    "default_stand_off_distance_m",
                    0.8,
                )
            )
            self.candidates = CandidateStore()
            self.bridge = CvBridge() if self.enable_depth else None

            # Section: Subscribe to structured detections and aligned depth only while configured.
            self.detection_sub = self.create_subscription(
                ObjectDetections,
                str(self.get_parameter("detections_topic").value),
                self.detections_callback,
                qos_profile_sensor_data,
                callback_group=self.cb_group,
            )
            # Section: RGB-only operation must not create or await unavailable depth interfaces.
            if self.enable_depth:
                self.depth_sub = self.create_subscription(
                    Image,
                    str(self.get_parameter("depth_topic").value),
                    self.depth_callback,
                    qos_profile_sensor_data,
                    callback_group=self.cb_group,
                )
                self.camera_info_sub = self.create_subscription(
                    CameraInfo,
                    str(self.get_parameter("camera_info_topic").value),
                    self.camera_info_callback,
                    qos_profile_sensor_data,
                    callback_group=self.cb_group,
                )
            # Section: Transient detector readiness distinguishes ACTIVE from live inference.
            ready_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.detector_ready_event.clear()
            self.detector_ready_sub = self.create_subscription(
                Bool,
                str(self.get_parameter("detector_ready_topic").value),
                self.detector_ready_callback,
                ready_qos,
                callback_group=self.cb_group,
            )

            # Section: TF and Nav2 interfaces remain clients of persistent infrastructure.
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
            self.named_nav_client = ActionClient(
                self,
                NavigateNamedPose,
                str(self.get_parameter("named_navigation_action").value),
                callback_group=self.cb_group,
            )
            self.nav_pose_client = ActionClient(
                self,
                NavigateToPose,
                str(self.get_parameter("navigate_to_pose_action").value),
                callback_group=self.cb_group,
            )
            self.spin_client = ActionClient(
                self,
                Spin,
                str(self.get_parameter("spin_action").value),
                callback_group=self.cb_group,
            )
            self.path_client = ActionClient(
                self,
                ComputePathToPose,
                str(self.get_parameter("compute_path_action").value),
                callback_group=self.cb_group,
            )
            self.costmap_client = self.create_client(
                GetCostmap,
                str(self.get_parameter("global_costmap_service").value),
                callback_group=self.cb_group,
            )
            self.action_server = ActionServer(
                self,
                FindObject,
                str(self.get_parameter("action_name").value),
                execute_callback=self.execute_callback,
                goal_callback=self.goal_callback,
                cancel_callback=self.cancel_callback,
                callback_group=self.cb_group,
            )
            self.get_logger().info("[GPSR_LIFECYCLE] FindObject: configured")
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.get_logger().error(f"FindObject configure failed: {exc}")
            self.release_resources()
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, _state: LifecycleState) -> TransitionCallbackReturn:
        """Special function: Accept FIND_OBJECT missions only after activation."""
        self.deactivate_event.clear()
        # Section: Synchronize activation with concurrent action admission checks.
        with self.goal_lock:
            self.skill_active = True
        self.get_logger().info("[GPSR_LIFECYCLE] FindObject: ACTIVE")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, _state: LifecycleState) -> TransitionCallbackReturn:
        """Special function: Cancel motion and missions before becoming inactive."""
        # Section: Close admission before signaling the current mission to stop.
        with self.goal_lock:
            self.skill_active = False
        self.deactivate_event.set()
        self.cancel_all_motion()
        # Section: Never report INACTIVE while a mission still owns configured resources.
        deadline = time.monotonic() + 2.0
        while self.has_active_goal() and time.monotonic() < deadline:
            time.sleep(0.05)
        if self.has_active_goal():
            self.get_logger().error("FindObject mission did not stop during deactivate.")
            return TransitionCallbackReturn.FAILURE
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _state: LifecycleState) -> TransitionCallbackReturn:
        """Special function: Release every resource allocated during configure."""
        # Section: Close admission before checking whether cleanup is safe.
        with self.goal_lock:
            self.skill_active = False
        self.deactivate_event.set()
        self.cancel_all_motion()
        # Section: Cleanup fails closed instead of racing an active mission callback.
        if self.has_active_goal():
            self.get_logger().error("Refusing FindObject cleanup while a mission is active.")
            return TransitionCallbackReturn.FAILURE
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _state: LifecycleState) -> TransitionCallbackReturn:
        """Special function: Release configured resources during process shutdown."""
        self.skill_active = False
        self.deactivate_event.set()
        self.cancel_all_motion()
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, _state: LifecycleState) -> TransitionCallbackReturn:
        """Special function: Fail closed and restore an unconfigured resource baseline."""
        self.skill_active = False
        self.deactivate_event.set()
        self.cancel_all_motion()
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def release_resources(self) -> None:
        """Special function: Destroy action, subscription, TF, and Nav2 client resources."""
        if self.action_server is not None:
            self.action_server.destroy()
            self.action_server = None
        for subscription_name in (
            "detection_sub",
            "depth_sub",
            "camera_info_sub",
            "detector_ready_sub",
        ):
            subscription = getattr(self, subscription_name)
            if subscription is not None:
                self.destroy_subscription(subscription)
                setattr(self, subscription_name, None)
        for client_name in ("named_nav_client", "nav_pose_client", "spin_client", "path_client"):
            client = getattr(self, client_name)
            if client is not None:
                client.destroy()
                setattr(self, client_name, None)
        if self.costmap_client is not None:
            self.destroy_client(self.costmap_client)
            self.costmap_client = None
        if self.tf_listener is not None:
            try:
                self.tf_listener.unregister()
            except Exception:
                pass
        self.tf_listener = None
        self.tf_buffer = None
        self.bridge = None
        self.search_config = {}
        self.candidates = CandidateStore()
        self.detector_ready_event.clear()
        with self.sensor_lock:
            self.latest_depth = None
            self.latest_camera_info = None
        # Section: Reset parent mission ownership only after active callbacks have stopped.
        with self.goal_lock:
            self.active_goal_handle = None
            self.goal_admitted = False
        # Section: Reset UUID-indexed child motion ownership under its dedicated lock.
        with self.motion_lock:
            self.active_motion_goals.clear()

    def detections_callback(self, message: ObjectDetections) -> None:
        """Special function: Cache detector frames only while ACTIVE."""
        if self.skill_active:
            self.candidates.update(message)

    def depth_callback(self, message: Image) -> None:
        """Special function: Cache the latest aligned depth frame while ACTIVE."""
        if self.skill_active:
            with self.sensor_lock:
                self.latest_depth = message

    def camera_info_callback(self, message: CameraInfo) -> None:
        """Special function: Cache color intrinsics while ACTIVE."""
        if self.skill_active:
            with self.sensor_lock:
                self.latest_camera_info = message

    def detector_ready_callback(self, message: Bool) -> None:
        """Special function: Track whether the detector completed live camera inference."""
        if bool(message.data):
            self.detector_ready_event.set()
        else:
            self.detector_ready_event.clear()

    def wait_for_perception_ready(
        self,
        timeout_sec: float,
        goal_handle: Any,
        context: SearchContext,
    ) -> tuple[bool, str]:
        """Special function: Gate search on live RGB plus optional depth-localization inputs."""
        deadline = time.monotonic() + max(0.1, float(timeout_sec))
        missing: list[str] = []
        while time.monotonic() < deadline:
            guard = self.guard_outcome(goal_handle, context)
            if guard:
                return False, f"Perception readiness interrupted by {guard}."
            with self.sensor_lock:
                has_depth = self.latest_depth is not None
                has_camera_info = self.latest_camera_info is not None
            has_detector_ready = self.detector_ready_event.is_set()
            has_detection_frame = self.candidates.has_frame()
            depth_ready = not self.enable_depth or (has_depth and has_camera_info)
            if has_detector_ready and has_detection_frame and depth_ready:
                return True, "Object perception inputs are ready."
            missing = []
            if not has_detector_ready:
                missing.append("detector_ready")
            if not has_detection_frame:
                missing.append("detections")
            if self.enable_depth and not has_depth:
                missing.append("aligned_depth")
            if self.enable_depth and not has_camera_info:
                missing.append("camera_info")
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        return False, f"Object perception readiness timed out; missing: {', '.join(missing)}."

    def goal_callback(self, goal: FindObject.Goal) -> GoalResponse:
        """Special function: Reject inactive, concurrent, or malformed missions."""
        # Section: Validate the request before reserving the single mission slot.
        if not str(goal.object_name).strip() and not str(goal.category).strip():
            return GoalResponse.REJECT
        scope = str(goal.search_scope or ("STRICT_LOCATION" if goal.location else "GLOBAL")).upper()
        if scope not in {"GLOBAL", "STRICT_LOCATION"}:
            return GoalResponse.REJECT
        if scope == "STRICT_LOCATION" and not str(goal.location).strip():
            return GoalResponse.REJECT
        # Section: Make lifecycle state and concurrent-goal admission one atomic decision.
        with self.goal_lock:
            if not self.skill_active or self.goal_admitted:
                return GoalResponse.REJECT
            self.goal_admitted = True
        # Section: Expose the accepted target before mission execution begins.
        try:
            now = time.monotonic()
            accepted_context = SearchContext(
                target=clean_symbol(goal.object_name),
                category=clean_symbol(goal.category),
                scope=scope,
                requested_location=clean_symbol(goal.location),
                approach_target=bool(goal.approach_target),
                stand_off_distance=float(goal.stand_off_distance or self.default_stand_off),
                start_time=now,
                deadline=now + float(goal.timeout_sec or self.default_timeout_sec),
            )
            self.publish_target_status("GOAL_ACCEPTED", accepted_context)
            return GoalResponse.ACCEPT
        except Exception as exc:
            # Section: Release admission if status preparation fails before execution starts.
            with self.goal_lock:
                self.goal_admitted = False
            self.get_logger().error(f"FindObject goal admission failed: {exc}")
            return GoalResponse.REJECT

    def cancel_callback(self, _goal_handle: Any) -> CancelResponse:
        """Special function: Accept cancellation and forward it to every active Nav2 goal."""
        self.cancel_all_motion()
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: Any) -> FindObject.Result:
        """Special function: Execute one YASMIN mission and return its semantic result."""
        try:
            # Section: Store the single ROS server handle without requiring hash support.
            with self.goal_lock:
                self.active_goal_handle = goal_handle
            # Section: Allocate mission state only after finally-protected ownership begins.
            blackboard = Blackboard()
            blackboard["goal_handle"] = goal_handle
            # Section: Search decisions execute in YASMIN; ROS callbacks progress on other executor threads.
            mission = FindObjectMissionSM(self)
            mission(blackboard)
            try:
                context = blackboard["context"]
            except KeyError:
                context = None
            if context is None:
                context = SearchContext("", "", "GLOBAL", "", False, self.default_stand_off, time.monotonic(), time.monotonic())
                context.terminal_code = ResultCode.PERCEPTION_FAILED
                context.terminal_message = "FindObject mission failed before initialization."
            result = self.build_result(context)
            self.publish_target_status(
                ResultCode(context.terminal_code).name,
                context,
                terminal=True,
            )
            if context.terminal_code == ResultCode.CANCELED:
                goal_handle.canceled()
            elif context.terminal_code in {
                ResultCode.SUCCESS,
                ResultCode.NOT_FOUND,
                ResultCode.FOUND_UNREACHABLE,
            }:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result
        except Exception as exc:
            self.get_logger().error(f"FindObject mission exception: {exc}")
            result = FindObject.Result()
            result.result_code = int(ResultCode.PERCEPTION_FAILED)
            result.message = f"FindObject mission exception: {exc}"
            self.publish_target_status("PERCEPTION_FAILED", terminal=True)
            try:
                goal_handle.abort()
            except Exception:
                pass
            return result
        finally:
            self.cancel_all_motion()
            # Section: Release the exact parent goal and permit the next mission on every exit.
            with self.goal_lock:
                if self.active_goal_handle is goal_handle:
                    self.active_goal_handle = None
                self.goal_admitted = False

    def has_active_goal(self) -> bool:
        """Special function: Return whether admission or execution owns the mission slot."""
        # Section: Include the short accepted-before-execute window in lifecycle barriers.
        with self.goal_lock:
            return self.goal_admitted or self.active_goal_handle is not None

    def build_result(self, context: SearchContext) -> FindObject.Result:
        """Special function: Convert SearchContext into the complete authoritative result."""
        result = FindObject.Result()
        result.result_code = int(context.terminal_code)
        result.found = context.terminal_code in {
            ResultCode.SUCCESS,
            ResultCode.FOUND_UNREACHABLE,
            ResultCode.LOCALIZATION_FAILED,
        }
        result.approached = bool(context.approached)
        result.object_pose = context.object_pose
        result.observation_pose = context.observation_pose
        sample = context.confirmed_target
        if sample is not None:
            detection = sample.detection
            result.object_id = (
                f"{clean_symbol(detection.name)}_{int(detection.center_u)}_{int(detection.center_v)}"
            )
        viewpoint = context.current_viewpoint or context.interrupted_viewpoint
        result.found_room = viewpoint.room if viewpoint is not None and result.found else ""
        result.found_viewpoint = viewpoint.name if viewpoint is not None and result.found else ""
        result.viewpoints_searched = list(context.visited_viewpoints)
        result.navigation_failures = len(context.navigation_failures)
        result.message = context.terminal_message
        return result

    def publish_feedback(
        self,
        goal_handle: Any,
        context: SearchContext,
        phase: str,
        confidence: float = 0.0,
    ) -> None:
        """Special function: Publish mission progress without changing decisions."""
        feedback = FindObject.Feedback()
        feedback.phase = str(phase)
        viewpoint = context.current_viewpoint
        feedback.current_room = viewpoint.room if viewpoint is not None else ""
        feedback.current_viewpoint = viewpoint.name if viewpoint is not None else ""
        feedback.search_tier = int(context.current_tier)
        feedback.candidate_confidence = float(confidence)
        feedback.elapsed_sec = float(time.monotonic() - context.start_time)
        goal_handle.publish_feedback(feedback)
        self.publish_target_status(phase, context, confidence=confidence)

    def publish_target_status(
        self,
        phase: str,
        context: SearchContext | None = None,
        confidence: float = 0.0,
        terminal: bool = False,
    ) -> None:
        """Special function: Publish the latest target-specific state for ROS observers."""
        # Section: Add a ROS timestamp without coupling the pure payload helper to a node.
        payload = target_status_payload(phase, context, confidence, terminal)
        now = self.get_clock().now().to_msg()
        payload["stamp"] = {"sec": int(now.sec), "nanosec": int(now.nanosec)}
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self.target_status_pub.publish(message)

    def guard_outcome(self, goal_handle: Any, context: SearchContext) -> str | None:
        """Special function: Convert cancellation, deactivation, and deadline into outcomes."""
        if goal_handle.is_cancel_requested or self.deactivate_event.is_set() or not self.skill_active:
            return CANCELED
        if time.monotonic() >= context.deadline:
            return TIMEOUT
        return None

    def cancellable_wait(self, seconds: float, goal_handle: Any, context: SearchContext) -> str | None:
        """Special function: Wait in short intervals so cancellation is promptly observed."""
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            guard = self.guard_outcome(goal_handle, context)
            if guard:
                return guard
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        return self.guard_outcome(goal_handle, context)

    def target_names(self, context: SearchContext) -> set[str]:
        """Special function: Expand configured category members into detector labels."""
        if context.target:
            return {context.target}
        members = dict(self.search_config.get("category_members", {})).get(context.category, [])
        return {clean_symbol(member) for member in members}

    def wait_future(
        self,
        future: Any,
        timeout_sec: float,
        canceled: Callable[[], bool],
    ) -> Any:
        """Special function: Wait for a ROS future without spinning this lifecycle node."""
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while time.monotonic() < deadline:
            if done.wait(timeout=0.05):
                return future.result()
            if canceled():
                return None
        return future.result() if future.done() else None

    def motion_goal_key(self, goal_handle: Any) -> bytes:
        """Special function: Convert one Nav2 goal UUID into an immutable tracking key."""
        # Section: Both real rclpy ClientGoalHandle and test doubles expose goal_id.uuid.
        goal_id = getattr(goal_handle, "goal_id", None)
        goal_uuid = getattr(goal_id, "uuid", None)
        if goal_uuid is None:
            raise ValueError("Nav2 goal handle does not contain a goal UUID.")
        goal_key = bytes(goal_uuid)
        if not goal_key:
            raise ValueError("Nav2 goal handle contains an empty goal UUID.")
        return goal_key

    def track_motion(self, goal_handle: Any) -> None:
        """Special function: Register one active Nav2 action goal for lifecycle cancellation."""
        # Section: Index the unhashable ROS handle by its immutable protocol identifier.
        goal_key = self.motion_goal_key(goal_handle)
        with self.motion_lock:
            self.active_motion_goals[goal_key] = goal_handle

    def untrack_motion(self, goal_handle: Any) -> None:
        """Special function: Remove one completed Nav2 action goal."""
        # Section: Remove by goal UUID so the ROS handle itself is never hashed.
        goal_key = self.motion_goal_key(goal_handle)
        with self.motion_lock:
            self.active_motion_goals.pop(goal_key, None)

    def cancel_motion(self, goal_handle: Any) -> None:
        """Special function: Request cancellation of one active Nav2 goal."""
        try:
            goal_handle.cancel_goal_async()
        except Exception:
            pass
        self.untrack_motion(goal_handle)

    def cancel_all_motion(self) -> None:
        """Special function: Cancel named navigation, dynamic navigation, Spin, and planning goals."""
        # Section: Copy dictionary values so cancellation can safely mutate tracking state.
        with self.motion_lock:
            goals = list(self.active_motion_goals.values())
        for goal in goals:
            self.cancel_motion(goal)

    def navigate_named_and_monitor(self, goal_handle: Any, context: SearchContext) -> str:
        """Special function: Run named navigation while checking every detector frame."""
        if self.named_nav_client is None or context.current_viewpoint is None:
            return NAV_FAILED
        navigation = dict(self.search_config.get("navigation", {}))
        server_timeout = float(navigation.get("server_timeout_sec", 5.0))
        result_timeout = min(
            float(navigation.get("result_timeout_sec", 120.0)),
            max(0.0, context.deadline - time.monotonic()),
        )
        if not self.named_nav_client.wait_for_server(timeout_sec=server_timeout):
            return NAV_FAILED
        goal = NavigateNamedPose.Goal()
        goal.target_name = context.current_viewpoint.name
        sent_at = time.monotonic()
        send_future = self.named_nav_client.send_goal_async(goal)
        nav_goal = self.wait_future(
            send_future,
            server_timeout,
            lambda: bool(self.guard_outcome(goal_handle, context)),
        )
        if nav_goal is None or not nav_goal.accepted:
            return self.guard_outcome(goal_handle, context) or NAV_FAILED
        self.track_motion(nav_goal)
        result_future = nav_goal.get_result_async()
        deadline = time.monotonic() + result_timeout
        verify = dict(self.search_config.get("verification", {}))
        names = self.target_names(context)
        min_confidence = float(verify.get("min_confidence", 0.35))
        while time.monotonic() < deadline:
            guard = self.guard_outcome(goal_handle, context)
            if guard:
                self.cancel_motion(nav_goal)
                return guard
            sample = self.candidates.latest_match(names, min_confidence, sent_at)
            if sample is not None:
                # Section: Moving detection is only a candidate; verification follows after cancel.
                context.candidate_history.append(sample)
                context.candidate_source = "navigation"
                context.interrupted_viewpoint = context.current_viewpoint
                context.current_viewpoint.status = ViewpointStatus.INTERRUPTED
                self.cancel_motion(nav_goal)
                self.publish_feedback(
                    goal_handle,
                    context,
                    "MOVING_CANDIDATE",
                    float(sample.detection.confidence),
                )
                return CANDIDATE
            if result_future.done():
                wrapped = result_future.result()
                self.untrack_motion(nav_goal)
                success = (
                    int(wrapped.status) == GoalStatus.STATUS_SUCCEEDED
                    and bool(getattr(wrapped.result, "success", True))
                )
                if success:
                    context.current_viewpoint.status = ViewpointStatus.ARRIVED
                    context.interrupted_viewpoint = None
                    self.publish_feedback(goal_handle, context, "ARRIVED")
                    return ARRIVED
                return NAV_FAILED
            time.sleep(0.05)
        self.cancel_motion(nav_goal)
        return TIMEOUT if time.monotonic() >= context.deadline else NAV_FAILED

    def scan_and_monitor(
        self,
        goal_handle: Any,
        context: SearchContext,
        phase: str,
        angles_deg: list[float],
    ) -> str:
        """Special function: Reach configured heading offsets using Nav2 Spin and inspect perception."""
        if self.spin_client is None:
            return NAV_FAILED
        scan_config = dict(self.search_config.get("scan", {}))
        verify = dict(self.search_config.get("verification", {}))
        index = int(context.scan_indices.get(phase, 0))
        context.current_viewpoint.status = ViewpointStatus.SCANNING
        while index < len(angles_deg):
            guard = self.guard_outcome(goal_handle, context)
            if guard:
                return guard
            desired = float(angles_deg[index])
            delta_deg = normalize_delta_degrees(desired - context.scan_heading_deg)
            context.scan_indices[phase] = index + 1
            if abs(delta_deg) > 0.5:
                spin_outcome = self.execute_spin(goal_handle, context, delta_deg)
                if spin_outcome in {CANCELED, TIMEOUT}:
                    return spin_outcome
                if spin_outcome == NAV_FAILED:
                    context.navigation_failures.append(
                        f"spin:{context.current_viewpoint.name}:{phase}:{desired}"
                    )
                    index += 1
                    continue
            context.scan_heading_deg = desired
            settled_at = time.monotonic()
            guard = self.cancellable_wait(
                float(scan_config.get("settle_interval_sec", 0.5)),
                goal_handle,
                context,
            )
            if guard:
                return guard
            sample = self.candidates.wait_for_candidate(
                self.target_names(context),
                float(verify.get("min_confidence", 0.35)),
                float(verify.get("last_seen_timeout_sec", 1.0)),
                lambda: bool(self.guard_outcome(goal_handle, context)),
                settled_at,
            )
            self.publish_feedback(goal_handle, context, phase.upper())
            if sample is not None:
                context.candidate_history.append(sample)
                context.candidate_source = phase
                return CANDIDATE
            index += 1
        return EXHAUSTED

    def execute_spin(self, goal_handle: Any, context: SearchContext, delta_deg: float) -> str:
        """Special function: Execute one relative search rotation through Nav2 Spin."""
        scan_config = dict(self.search_config.get("scan", {}))
        timeout_sec = float(scan_config.get("action_timeout_sec", 20.0))
        if not self.spin_client.wait_for_server(timeout_sec=min(5.0, timeout_sec)):
            return NAV_FAILED
        spin_goal = Spin.Goal()
        spin_goal.target_yaw = math.radians(float(delta_deg))
        spin_goal.time_allowance = duration_msg(timeout_sec)
        future = self.spin_client.send_goal_async(spin_goal)
        motion = self.wait_future(
            future,
            min(5.0, timeout_sec),
            lambda: bool(self.guard_outcome(goal_handle, context)),
        )
        if motion is None or not motion.accepted:
            return self.guard_outcome(goal_handle, context) or NAV_FAILED
        self.track_motion(motion)
        wrapped = self.wait_future(
            motion.get_result_async(),
            timeout_sec,
            lambda: bool(self.guard_outcome(goal_handle, context)),
        )
        self.untrack_motion(motion)
        if wrapped is None:
            self.cancel_motion(motion)
            return self.guard_outcome(goal_handle, context) or NAV_FAILED
        return READY if int(wrapped.status) == GoalStatus.STATUS_SUCCEEDED else NAV_FAILED

    def localize_confirmed_target(self, context: SearchContext) -> tuple[bool, str]:
        """Special function: Project aligned depth and transform the target into map."""
        if context.confirmed_target is None or self.bridge is None or self.tf_buffer is None:
            return False, "Confirmed detection, depth bridge, or TF buffer is missing."
        with self.sensor_lock:
            depth_message = self.latest_depth
            camera_info = self.latest_camera_info
        if depth_message is None or camera_info is None:
            return False, "Aligned depth or CameraInfo is unavailable."
        config = dict(self.search_config.get("localization", {}))
        sample = context.confirmed_target
        now = self.get_clock().now()
        detection_time = Time.from_msg(sample.stamp)
        depth_time = Time.from_msg(depth_message.header.stamp)
        detection_age = max(0.0, (now - detection_time).nanoseconds / 1e9)
        depth_age = max(0.0, (now - depth_time).nanoseconds / 1e9)
        skew = abs((detection_time - depth_time).nanoseconds) / 1e9
        if detection_age > float(config.get("max_detection_age_sec", 0.75)):
            return False, "Confirmed detection is stale."
        if depth_age > float(config.get("max_depth_age_sec", 0.75)):
            return False, "Aligned depth frame is stale."
        if skew > float(config.get("max_timestamp_skew_sec", 0.35)):
            return False, "Detection/depth timestamp skew exceeds the configured limit."
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
        except Exception as exc:
            return False, f"Aligned depth conversion failed: {exc}"
        camera_point, error = self.project_detection(
            depth_image,
            depth_message.encoding,
            depth_message.header,
            sample.detection,
            camera_info,
            config,
        )
        if camera_point is None:
            return False, error
        map_frame = str(config.get("map_frame", "map"))
        try:
            map_point = self.tf_buffer.transform(
                camera_point,
                map_frame,
                timeout=Duration(seconds=float(config.get("tf_timeout_sec", 0.3))),
            )
            object_pose = PoseStamped()
            object_pose.header = map_point.header
            object_pose.pose.position = map_point.point
            object_pose.pose.orientation.w = 1.0
            context.object_pose = object_pose
            context.observation_pose = self.current_observation_pose(map_frame, config)
            return True, ""
        except TransformException as exc:
            return False, f"Target TF transform to {map_frame} failed: {exc}"

    @staticmethod
    def project_detection(
        depth_image: np.ndarray,
        encoding: str,
        header: Any,
        detection: ObjectDetection,
        camera_info: CameraInfo,
        config: dict[str, Any],
    ) -> tuple[PointStamped | None, str]:
        """Special function: Median-project a bbox-center ROI into camera optical coordinates."""
        if depth_image.ndim < 2 or depth_image.size == 0:
            return None, "Aligned depth image is empty."
        height, width = depth_image.shape[:2]
        center_u = int(round(float(detection.center_u)))
        center_v = int(round(float(detection.center_v)))
        roi_width = max(1, int(config.get("depth_roi_width_px", 20)))
        roi_height = max(1, int(config.get("depth_roi_height_px", 20)))
        left = max(0, min(width - 1, center_u - roi_width // 2))
        right = max(left + 1, min(width, center_u + roi_width // 2))
        top = max(0, min(height - 1, center_v - roi_height // 2))
        bottom = max(top + 1, min(height, center_v + roi_height // 2))
        patch = depth_image[top:bottom, left:right].astype(np.float32)
        if encoding in {"16UC1", "mono16"}:
            patch /= 1000.0
        min_range = float(config.get("min_range_m", 0.15))
        max_range = float(config.get("max_range_m", 6.0))
        valid = np.isfinite(patch) & (patch >= min_range) & (patch <= max_range)
        valid_ratio = float(valid.sum()) / float(patch.size) if patch.size else 0.0
        if valid_ratio < float(config.get("min_valid_depth_ratio", 0.5)):
            return None, "Depth ROI has insufficient valid pixels."
        depth_m = float(np.median(patch[valid]))
        intrinsics = list(camera_info.k)
        fx, fy, principal_x, principal_y = intrinsics[0], intrinsics[4], intrinsics[2], intrinsics[5]
        if fx <= 0.0 or fy <= 0.0:
            return None, "CameraInfo focal lengths are invalid."
        point = PointStamped()
        point.header = header
        if not point.header.frame_id:
            point.header.frame_id = str(config.get("camera_frame", "camera_color_optical_frame"))
        point.point.x = (center_u - principal_x) * depth_m / fx
        point.point.y = (center_v - principal_y) * depth_m / fy
        point.point.z = depth_m
        return point, ""

    def current_observation_pose(self, map_frame: str, config: dict[str, Any]) -> PoseStamped:
        """Special function: Transform the robot base origin into the map at observation time."""
        base_pose = PoseStamped()
        base_pose.header.frame_id = str(config.get("base_frame", "base_link"))
        base_pose.header.stamp = self.get_clock().now().to_msg()
        base_pose.pose.orientation.w = 1.0
        try:
            return self.tf_buffer.transform(
                base_pose,
                map_frame,
                timeout=Duration(seconds=float(config.get("tf_timeout_sec", 0.3))),
            )
        except TransformException:
            return PoseStamped()

    def plan_standoff_candidates(self, goal_handle: Any, context: SearchContext) -> str:
        """Special function: Keep only occupancy-safe candidates with a valid Nav2 path."""
        approach = dict(self.search_config.get("approach", {}))
        candidates = StandoffPlanner.generate(
            context.object_pose,
            context.stand_off_distance,
            list(approach.get("candidate_angles_deg", [])),
        )
        costmap = self.fetch_costmap(
            float(approach.get("planner_timeout_sec", 5.0)),
            goal_handle,
            context,
        )
        context.standoff_candidates = []
        for candidate in candidates:
            guard = self.guard_outcome(goal_handle, context)
            if guard:
                return guard
            if costmap is not None and not self.costmap_pose_is_free(
                costmap,
                candidate,
                int(approach.get("costmap_lethal_threshold", 252)),
            ):
                continue
            if self.path_exists(goal_handle, context, candidate):
                context.standoff_candidates.append(candidate)
        return READY if context.standoff_candidates else FOUND_UNREACHABLE

    def fetch_costmap(self, timeout_sec: float, goal_handle: Any, context: SearchContext) -> Any | None:
        """Special function: Fetch the persistent global costmap when available."""
        if self.costmap_client is None or not self.costmap_client.wait_for_service(timeout_sec=min(1.0, timeout_sec)):
            return None
        return self.wait_future(
            self.costmap_client.call_async(GetCostmap.Request()),
            timeout_sec,
            lambda: bool(self.guard_outcome(goal_handle, context)),
        )

    @staticmethod
    def costmap_pose_is_free(response: Any, pose: PoseStamped, lethal_threshold: int) -> bool:
        """Special function: Reject candidates outside or occupied in the known global costmap."""
        costmap = response.map
        metadata = costmap.metadata
        resolution = float(metadata.resolution)
        if resolution <= 0.0:
            return False
        origin = metadata.origin.position
        grid_x = int(math.floor((pose.pose.position.x - origin.x) / resolution))
        grid_y = int(math.floor((pose.pose.position.y - origin.y) / resolution))
        size_x = int(metadata.size_x)
        size_y = int(metadata.size_y)
        if grid_x < 0 or grid_y < 0 or grid_x >= size_x or grid_y >= size_y:
            return False
        value = int(costmap.data[grid_y * size_x + grid_x])
        return value != 255 and value < int(lethal_threshold)

    def path_exists(self, goal_handle: Any, context: SearchContext, candidate: PoseStamped) -> bool:
        """Special function: Ask Nav2 planner whether the stand-off pose is reachable."""
        if self.path_client is None:
            return False
        approach = dict(self.search_config.get("approach", {}))
        timeout_sec = float(approach.get("planner_timeout_sec", 5.0))
        if not self.path_client.wait_for_server(timeout_sec=timeout_sec):
            return False
        goal = ComputePathToPose.Goal()
        candidate.header.stamp = self.get_clock().now().to_msg()
        goal.goal = candidate
        goal.use_start = False
        sent = self.wait_future(
            self.path_client.send_goal_async(goal),
            timeout_sec,
            lambda: bool(self.guard_outcome(goal_handle, context)),
        )
        if sent is None or not sent.accepted:
            return False
        self.track_motion(sent)
        wrapped = self.wait_future(
            sent.get_result_async(),
            timeout_sec,
            lambda: bool(self.guard_outcome(goal_handle, context)),
        )
        self.untrack_motion(sent)
        return bool(
            wrapped is not None
            and int(wrapped.status) == GoalStatus.STATUS_SUCCEEDED
            and list(wrapped.result.path.poses)
        )

    def approach_candidates(self, goal_handle: Any, context: SearchContext) -> str:
        """Special function: Try planned stand-off alternatives through NavigateToPose."""
        if self.nav_pose_client is None:
            return FOUND_UNREACHABLE
        approach = dict(self.search_config.get("approach", {}))
        timeout_sec = float(approach.get("navigation_timeout_sec", 90.0))
        if not self.nav_pose_client.wait_for_server(timeout_sec=min(5.0, timeout_sec)):
            return FOUND_UNREACHABLE
        for candidate in context.standoff_candidates:
            guard = self.guard_outcome(goal_handle, context)
            if guard:
                return guard
            goal = NavigateToPose.Goal()
            goal.pose = candidate
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            sent = self.wait_future(
                self.nav_pose_client.send_goal_async(goal),
                min(5.0, timeout_sec),
                lambda: bool(self.guard_outcome(goal_handle, context)),
            )
            if sent is None or not sent.accepted:
                continue
            self.track_motion(sent)
            wrapped = self.wait_future(
                sent.get_result_async(),
                min(timeout_sec, max(0.0, context.deadline - time.monotonic())),
                lambda: bool(self.guard_outcome(goal_handle, context)),
            )
            self.untrack_motion(sent)
            if wrapped is not None and int(wrapped.status) == GoalStatus.STATUS_SUCCEEDED:
                context.approached = True
                return READY
            self.cancel_motion(sent)
        return FOUND_UNREACHABLE


def main(args: list[str] | None = None) -> None:
    """Special function: Spin the lifecycle FIND_OBJECT server with concurrent callbacks."""
    rclpy.init(args=args)
    node = FindObjectSkillServer()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.release_resources()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
