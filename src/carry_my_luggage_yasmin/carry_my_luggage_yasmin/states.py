"""Reusable ROS-backed states for both Carry My Luggage YASMIN graphs."""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Iterable

import yaml
from geometry_msgs.msg import PoseStamped
from person_follower_master.msg import MasterTarget
from putrabot_interfaces.action import ManagedProcess, PersonFollow
from putrabot_interfaces.srv import KillManagedProcess
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from voice_interfaces.action import Receptionist
from voice_interfaces.srv import SayText
from yasmin import Blackboard, State
from yasmin_ros import ActionState, MonitorState, ServiceState
from yasmin_ros.basic_outcomes import ABORT, CANCEL, SUCCEED, TIMEOUT

from carry_my_luggage_yasmin import outcomes as out


# Section: Runtime defaults mirror the active SMACC2 client behaviors.
DEFAULT_MAX_ATTEMPTS = 3
DEFAULT_FOLLOW_DISTANCE_M = 0.8
DEFAULT_FACE_HOLD_SEC = 5.0


def _trigger_request(_blackboard: Blackboard) -> Trigger.Request:
    """Create an empty request for a Trigger service."""
    return Trigger.Request()


def _trigger_response(_blackboard: Blackboard, response: Trigger.Response) -> str:
    """Convert the standard Trigger response to a YASMIN outcome."""
    return SUCCEED if response.success else ABORT


def make_trigger_state(
    node: Node,
    service_name: str,
    wait_timeout: float = 10.0,
    response_timeout: float = 60.0,
) -> ServiceState:
    """Build a bounded standard Trigger-service state."""
    # Section: YASMIN handles service discovery, cancellation, and response waiting.
    return ServiceState(
        Trigger,
        service_name,
        _trigger_request,
        response_handler=_trigger_response,
        node=node,
        wait_timeout=wait_timeout,
        response_timeout=response_timeout,
        maximum_retry=0,
    )


def _managed_goal(_blackboard: Blackboard) -> ManagedProcess.Goal:
    """Request an idempotent managed-process start."""
    goal = ManagedProcess.Goal()
    goal.force_restart = False
    return goal


def _managed_result(_blackboard: Blackboard, result: ManagedProcess.Result) -> str:
    """Accept a newly started or already-running managed process."""
    return SUCCEED if result.success or result.already_running else ABORT


def make_managed_process_state(
    node: Node,
    action_name: str,
    response_timeout: float = 30.0,
) -> ActionState:
    """Build a state for a phase-supervisor ManagedProcess action."""
    # Section: The action endpoint owns process generation tracking and idempotency.
    return ActionState(
        ManagedProcess,
        action_name,
        _managed_goal,
        result_handler=_managed_result,
        node=node,
        wait_timeout=10.0,
        response_timeout=response_timeout,
        maximum_retry=0,
    )


class RetryState(State):
    """Add a visible bounded-retry policy around another YASMIN state."""

    def __init__(
        self,
        delegate: State,
        key: str,
        max_attempts: int = DEFAULT_MAX_ATTEMPTS,
    ) -> None:
        """Configure the delegated operation and its blackboard counter."""
        # Section: Expose retry exhaustion as a separate transition for safety handling.
        super().__init__({out.SUCCEEDED, out.RETRY, out.EXHAUSTED, out.CANCELED})
        self.delegate = delegate
        self.key = key
        self.max_attempts = max(1, int(max_attempts))

    def execute(self, blackboard: Blackboard) -> str:
        """Run one attempt and classify its result."""
        result = self.delegate(blackboard)
        if result == SUCCEED:
            blackboard[self.key] = 0
            return out.SUCCEEDED
        if result == CANCEL:
            return out.CANCELED

        # Section: Count failed or timed-out attempts without sharing counters across states.
        attempts = int(blackboard.get(self.key, 0)) + 1
        blackboard[self.key] = attempts
        return out.EXHAUSTED if attempts >= self.max_attempts else out.RETRY

    def cancel_state(self) -> None:
        """Forward cancellation to the currently delegated ROS state."""
        self.delegate.cancel_state()
        super().cancel_state()


class SequenceState(State):
    """Execute several state behaviors as one graph state."""

    def __init__(self, delegates: Iterable[State]) -> None:
        """Store the ordered behaviors used by the composite state."""
        super().__init__({SUCCEED, ABORT, CANCEL, TIMEOUT})
        self.delegates = list(delegates)
        self.active_delegate: State | None = None

    def execute(self, blackboard: Blackboard) -> str:
        """Stop the sequence on its first non-success outcome."""
        for delegate in self.delegates:
            self.active_delegate = delegate
            result = delegate(blackboard)
            if result != SUCCEED:
                self.active_delegate = None
                return result
        self.active_delegate = None
        return SUCCEED

    def cancel_state(self) -> None:
        """Cancel the behavior that is active inside the sequence."""
        if self.active_delegate is not None:
            self.active_delegate.cancel_state()
        super().cancel_state()


class BestEffortState(State):
    """Convert optional cleanup failure into forward progress."""

    def __init__(self, delegate: State) -> None:
        """Store the cleanup operation to attempt once."""
        super().__init__({out.SUCCEEDED})
        self.delegate = delegate

    def execute(self, blackboard: Blackboard) -> str:
        """Attempt cleanup and always report completion."""
        self.delegate(blackboard)
        return out.SUCCEEDED


class LoadWaypointsState(State):
    """Parse the configured waypoint YAML into the YASMIN blackboard."""

    def __init__(self, yaml_path: str) -> None:
        """Save the installed YAML path for runtime loading."""
        super().__init__({SUCCEED, ABORT})
        self.yaml_path = Path(yaml_path)

    def execute(self, blackboard: Blackboard) -> str:
        """Validate and cache PoseStamped waypoints without a helper node."""
        try:
            document = yaml.safe_load(self.yaml_path.read_text(encoding="utf-8")) or {}
            entries = document.get("waypoints", [])
            if not entries:
                return ABORT

            # Section: Preserve ROS poses and IDs for later navigation extensions.
            waypoint_ids: list[str] = []
            waypoint_goals: list[PoseStamped] = []
            for entry in entries:
                pose_data = entry["pose"]
                goal = PoseStamped()
                goal.header.frame_id = str(entry["frame_id"])
                goal.pose.position.x = float(pose_data["position"]["x"])
                goal.pose.position.y = float(pose_data["position"]["y"])
                goal.pose.position.z = float(pose_data["position"]["z"])
                goal.pose.orientation.x = float(pose_data["orientation"]["x"])
                goal.pose.orientation.y = float(pose_data["orientation"]["y"])
                goal.pose.orientation.z = float(pose_data["orientation"]["z"])
                goal.pose.orientation.w = float(pose_data["orientation"]["w"])
                waypoint_ids.append(str(entry["id"]))
                waypoint_goals.append(goal)
            blackboard["waypoint_ids"] = waypoint_ids
            blackboard["waypoint_goals"] = waypoint_goals
            return SUCCEED
        except (KeyError, OSError, TypeError, ValueError, yaml.YAMLError):
            return ABORT


def make_bool_monitor(node: Node, topic_name: str, timeout: float | None = None) -> MonitorState:
    """Wait until a Boolean topic publishes true."""

    def monitor_handler(_blackboard: Blackboard, message: Bool) -> str:
        """Keep monitoring false samples and accept a true sample."""
        return SUCCEED if message.data else ABORT

    # Section: A false sample returns ABORT so the containing graph can retry visibly.
    return MonitorState(
        Bool,
        topic_name,
        {SUCCEED, ABORT},
        monitor_handler,
        node=node,
        timeout=timeout,
        maximum_retry=0,
    )


class WaitForStartState(State):
    """Start breadcrumb teaching and wait for the physical task button."""

    def __init__(self, node: Node) -> None:
        """Configure the two phase-one start prerequisites."""
        super().__init__({out.SUCCEEDED, out.FAILED, out.CANCELED})
        self.breadcrumb = make_trigger_state(
            node, "/breadcrumb_teach_repeat/start_teach", response_timeout=15.0
        )
        self.button = make_bool_monitor(node, "/nano_d2_grounded")
        self.active: State | None = None

    def execute(self, blackboard: Blackboard) -> str:
        """Require breadcrumb readiness before accepting the button event."""
        self.active = self.breadcrumb
        breadcrumb_result = self.breadcrumb(blackboard)
        if breadcrumb_result != SUCCEED:
            self.active = None
            return out.CANCELED if breadcrumb_result == CANCEL else out.FAILED
        self.active = self.button
        button_result = self.button(blackboard)
        self.active = None
        if button_result == SUCCEED:
            return out.SUCCEEDED
        return out.CANCELED if button_result == CANCEL else out.FAILED

    def cancel_state(self) -> None:
        """Release a pending service call or button monitor."""
        if self.active is not None:
            self.active.cancel_state()
        super().cancel_state()


class SelectBagState(State):
    """Move the arm ready and wait for bag selection confirmation."""

    def __init__(self, node: Node) -> None:
        """Configure the ready-pose service and bag-ready topic."""
        super().__init__({out.SUCCEEDED, out.POSE_FAILED, out.BAG_FAILED, out.CANCELED})
        self.ready_pose = make_trigger_state(node, "/grasp/execute_ready")
        self.bag_ready = make_bool_monitor(node, "/bag_ready_for_pickup", timeout=10.0)
        self.active: State | None = None

    def execute(self, blackboard: Blackboard) -> str:
        """Preserve distinct recovery edges for pose and selection failures."""
        self.active = self.ready_pose
        pose_result = self.ready_pose(blackboard)
        if pose_result != SUCCEED:
            self.active = None
            return out.CANCELED if pose_result == CANCEL else out.POSE_FAILED
        self.active = self.bag_ready
        bag_result = self.bag_ready(blackboard)
        self.active = None
        if bag_result == SUCCEED:
            return out.SUCCEEDED
        return out.CANCELED if bag_result == CANCEL else out.BAG_FAILED

    def cancel_state(self) -> None:
        """Cancel whichever bag-selection prerequisite is active."""
        if self.active is not None:
            self.active.cancel_state()
        super().cancel_state()


def _kill_request(target: str):
    """Create a KillManagedProcess request factory for one fixed target."""

    def create_request(_blackboard: Blackboard) -> KillManagedProcess.Request:
        """Populate only the supervisor-owned target name."""
        request = KillManagedProcess.Request()
        request.target = target
        request.generation = ""
        return request

    return create_request


def _kill_response(_blackboard: Blackboard, response: KillManagedProcess.Response) -> str:
    """Treat already-stopped managed processes as successful cleanup."""
    already_stopped = "not running" in response.message or "no active launches" in response.message
    return SUCCEED if response.success or already_stopped else ABORT


def make_kill_state(node: Node, target: str) -> ServiceState:
    """Build a fixed-target phase-supervisor cleanup state."""
    return ServiceState(
        KillManagedProcess,
        "/kill",
        _kill_request(target),
        response_handler=_kill_response,
        node=node,
        wait_timeout=5.0,
        response_timeout=10.0,
        maximum_retry=0,
    )


class TargetStillState(State):
    """Detect the full phase-one operator stop gesture."""

    def __init__(self, node: Node) -> None:
        """Prepare the target monitor and optional voice prompt."""
        super().__init__({out.SUCCEEDED, out.FAILED, out.CANCELED})
        self.node = node
        self.condition = threading.Condition()
        self.still_since: float | None = None
        self.last_target_at: float | None = None
        self.subscription = node.create_subscription(
            MasterTarget, "/master_target", self._target_callback, 10
        )
        self.prompt = ServiceState(
            SayText,
            "/voice/tts/say",
            self._prompt_request,
            response_handler=lambda _bb, response: SUCCEED if response.success else ABORT,
            node=node,
            wait_timeout=5.0,
            response_timeout=15.0,
            maximum_retry=0,
        )

    def _prompt_request(self, _blackboard: Blackboard) -> SayText.Request:
        """Ask the operator to stand centered in front of the robot."""
        request = SayText.Request()
        request.text = "Please stand in front of me so I can follow you"
        return request

    def _target_callback(self, message: MasterTarget) -> None:
        """Track the continuous five-second still-gesture window."""
        now = time.monotonic()
        healthy = message.is_locked and message.state != "LOST"
        centered = abs(message.bearing) <= 0.30
        in_range = 0.80 <= message.distance <= 2.00
        still = math.hypot(message.vx, message.vy) <= 0.12
        with self.condition:
            self.last_target_at = now
            if healthy and centered and in_range and still:
                if self.still_since is None:
                    self.still_since = now
            else:
                self.still_since = None
            self.condition.notify_all()

    def execute(self, blackboard: Blackboard) -> str:
        """Wait until the target remains centered and still for five seconds."""
        # Section: Voice availability is advisory and must not block the safety gesture.
        self.prompt(blackboard)
        with self.condition:
            self.still_since = None
            self.last_target_at = None
            while not self.is_canceled():
                now = time.monotonic()
                if self.last_target_at is not None and now - self.last_target_at > 1.0:
                    self.still_since = None
                if self.still_since is not None and now - self.still_since >= 5.0:
                    return out.SUCCEEDED
                self.condition.wait(timeout=0.1)
        return out.CANCELED

    def cancel_state(self) -> None:
        """Wake the gesture wait immediately during state-machine shutdown."""
        super().cancel_state()
        with self.condition:
            self.condition.notify_all()


def make_person_follow_state(node: Node, face_hold_sec: float) -> ActionState:
    """Build the follow-demo destination action state."""

    def create_goal(_blackboard: Blackboard) -> PersonFollow.Goal:
        """Configure automatic selection and unmapped following."""
        goal = PersonFollow.Goal()
        goal.auto_select_if_needed = True
        goal.follow_distance_m = DEFAULT_FOLLOW_DISTANCE_M
        goal.face_hold_sec = float(face_hold_sec)
        return goal

    def handle_result(_blackboard: Blackboard, result: PersonFollow.Result) -> str:
        """Accept success or the behavior-aborted arrival marker."""
        arrived = "behavior_aborted" in result.message
        return SUCCEED if result.success or arrived else ABORT

    # Section: The destination leg is intentionally unbounded until the operator arrival gesture.
    return ActionState(
        PersonFollow,
        "/person_follow/control",
        create_goal,
        result_handler=handle_result,
        node=node,
        wait_timeout=10.0,
        response_timeout=None,
        maximum_retry=0,
    )


def make_receptionist_state(node: Node) -> ActionState:
    """Build the phase-one pass-bag confirmation dialogue."""

    def create_goal(_blackboard: Blackboard) -> Receptionist.Goal:
        """Mirror the active interactive Receptionist goal fields."""
        goal = Receptionist.Goal()
        goal.summary_only = False
        goal.replay_last_profile = False
        goal.record_sec = 4.0
        goal.mic_device = "sysdefault:CARD=Gen"
        goal.piper_model_path = "/home/yanjiun68/piper_models/en_US-lessac-medium.onnx"
        goal.whisper_model = "base"
        goal.whisper_language = "en"
        goal.guest_name = ""
        goal.guest_age = 0
        goal.guest_gender = ""
        goal.favorite_drink = ""
        return goal

    def handle_result(blackboard: Blackboard, result: Receptionist.Result) -> str:
        """Store the handover decision for a following decision state."""
        blackboard["pass_bag"] = bool(result.handover_ok)
        return SUCCEED

    return ActionState(
        Receptionist,
        "/receptionist",
        create_goal,
        result_handler=handle_result,
        node=node,
        wait_timeout=10.0,
        response_timeout=30.0,
        maximum_retry=0,
    )


class PassBagDecisionState(State):
    """Convert the dialogue blackboard value into yes/no graph edges."""

    def __init__(self, dialogue: State) -> None:
        """Store the Receptionist action used for confirmation."""
        super().__init__({out.YES, out.NO, out.FAILED, out.CANCELED})
        self.dialogue = dialogue

    def execute(self, blackboard: Blackboard) -> str:
        """Run the dialogue and expose its boolean handover decision."""
        result = self.dialogue(blackboard)
        if result == CANCEL:
            return out.CANCELED
        if result != SUCCEED:
            return out.FAILED
        return out.YES if bool(blackboard.get("pass_bag", False)) else out.NO

    def cancel_state(self) -> None:
        """Forward cancellation to the active voice action."""
        self.dialogue.cancel_state()
        super().cancel_state()


class DelayState(State):
    """Provide a cancelable delay before releasing the bag."""

    def __init__(self, duration_sec: float = 5.0) -> None:
        """Store the non-negative delay duration."""
        super().__init__({out.SUCCEEDED, out.CANCELED})
        self.duration_sec = max(0.0, float(duration_sec))

    def execute(self, _blackboard: Blackboard) -> str:
        """Sleep in short intervals so cancellation remains responsive."""
        deadline = time.monotonic() + self.duration_sec
        while time.monotonic() < deadline:
            if self.is_canceled():
                return out.CANCELED
            time.sleep(min(0.1, deadline - time.monotonic()))
        return out.SUCCEEDED


class TerminalState(State):
    """Keep completed and aborted states visible in YASMIN Viewer."""

    def __init__(self, terminal_outcome: str, cleanup: State | None = None) -> None:
        """Configure the terminal outcome and optional safety cleanup behavior."""
        super().__init__({terminal_outcome})
        self.terminal_outcome = terminal_outcome
        self.cleanup = cleanup

    def execute(self, blackboard: Blackboard) -> str:
        """Run optional safety cleanup before finishing the machine."""
        if self.cleanup is not None:
            self.cleanup(blackboard)
        return self.terminal_outcome

    def cancel_state(self) -> None:
        """Forward shutdown cancellation to optional terminal cleanup."""
        if self.cleanup is not None:
            self.cleanup.cancel_state()
        super().cancel_state()
