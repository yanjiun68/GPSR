"""Section: ROS action node for deterministic GPSR scheduling."""

from __future__ import annotations

import copy
import json
import math
import threading
import time
import uuid
from pathlib import Path
from typing import Any

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from gpsr_task_scheduler.config import SchedulerConfig, load_config
from gpsr_task_scheduler.clarification import ClarificationSession, merge_clarification
from gpsr_task_scheduler.cycle import GpsrCycleController, SchedulerState
from gpsr_task_scheduler.models import ReturnTarget
from gpsr_task_scheduler.planner import plan_to_dict, step_to_action_step_dict
from gpsr_task_scheduler.scheduler import schedule_command
from gpsr_task_scheduler.scheduler_viewer_sm import SchedulerViewerSM
from robot_ai_interfaces.action import ExecuteActionQueue, ExecuteTaskPlan
from robot_ai_interfaces.msg import ActionQueue, ActionStep
from robot_ai_interfaces.srv import ScheduleCommand
from robot_yasmin_tasks.gpsr.models import summarize_action_queue_failure


def retained_plan_qos() -> QoSProfile:
    """Special function: Retain the latest plan for optional late-joining monitors."""
    # Section: A depth of one keeps only the current accepted or dry-run plan.
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class GpsrTaskSchedulerNode(Node):
    """Special function: Parse, plan, dry-run, and dispatch deterministic GPSR tasks."""

    def __init__(self) -> None:
        """Special function: Initialize scheduler configuration, topics, and action clients."""
        super().__init__("gpsr_task_scheduler")

        # Section: Declare runtime parameters exposed by the scheduler launch file.
        self.declare_parameter("config_file", self.default_config_path())
        self.declare_parameter("skill_registry_file", "")
        self.declare_parameter("allow_simulated_primitives", False)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("command_input_mode", "topic")
        self.declare_parameter("command_topic", "/gpsr/command")
        self.declare_parameter("plan_topic", "/gpsr/task_plan")
        self.declare_parameter("schedule_command_service", "/gpsr/schedule_command")
        self.declare_parameter("reset_state_service", "/gpsr/reset_state")
        self.declare_parameter("execute_task_plan_action", "/gpsr/execute_task_plan")
        self.declare_parameter("execute_action_queue_action", "/execute_action_queue")
        self.declare_parameter("executor_goal_response_timeout_sec", 30.0)
        self.declare_parameter("instruction_point", "")
        self.declare_parameter("return_after_completion", False)
        self.declare_parameter("return_after_failure", False)
        self.declare_parameter("default_skill_timeout_sec", 30.0)
        self.declare_parameter("default_skill_retries", 0)
        self.declare_parameter("enable_navigation", True)
        self.declare_parameter("enable_perception", True)
        self.declare_parameter("enable_manipulation", True)
        self.declare_parameter("enable_tts", True)
        self.declare_parameter("max_commands", 3)
        self.declare_parameter("navigate_to_instruction_point_on_start", False)
        self.declare_parameter("return_target_mode", "command_start_amcl")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("amcl_pose_max_age_sec", 1.0)
        self.declare_parameter("amcl_pose_capture_timeout_sec", 3.0)
        self.declare_parameter("return_fallback_named_location", "instruction_point")
        self.declare_parameter("enable_error_recovery", True)
        self.declare_parameter("error_recovery_max_attempts", 3)
        self.declare_parameter("error_recovery_retry_delay_sec", 5.0)
        self.declare_parameter("state_topic", "/gpsr/state")
        self.declare_parameter("command_result_topic", "/gpsr/command_result")
        self.declare_parameter("clarification_response_topic", "/gpsr/clarification_response")
        self.declare_parameter("max_clarification_attempts", 2)
        self.declare_parameter("clarification_timeout_sec", 40.0)
        self.declare_parameter("enable_scheduler_viewer", True)
        self.declare_parameter("require_executor_ready", True)

        # Section: Load deterministic vocabulary and mission policy.
        self.config = self.load_runtime_config()

        # Section: Guard the complete competition-command lifecycle across callback threads.
        self.cycle = GpsrCycleController(int(self.get_parameter("max_commands").value))
        self.initialization_started = False
        self.error_recovery_attempts = 0
        self.error_recovery_next_attempt_time = 0.0
        self.error_recovery_exhaustion_logged = False
        # Section: Synchronize live localization callbacks with bounded command-start capture.
        self.amcl_pose_condition = threading.Condition()
        self.latest_amcl_pose: PoseWithCovarianceStamped | None = None
        self.latest_amcl_pose_receipt_time = 0.0
        # Section: Freeze return and deferred-report context independently for every admitted command.
        self.command_return_targets: dict[str, ReturnTarget] = {}
        self.pending_post_return: dict[str, dict[str, Any]] = {}
        # Section: Reserve one topic dialogue across executor threads and reject stale replies.
        self.clarification_lock = threading.Lock()
        self.clarification_session: ClarificationSession | None = None
        self.command_intake_busy = False

        # Section: Publish the complete scheduler lifecycle, including ERROR, to YASMIN Viewer.
        self.scheduler_viewer = (
            SchedulerViewerSM(self)
            if bool(self.get_parameter("enable_scheduler_viewer").value)
            else None
        )

        # Section: Publish without waiting for a TUI and retain the latest plan for late subscribers.
        self.plan_pub = self.create_publisher(
            String,
            str(self.get_parameter("plan_topic").value),
            retained_plan_qos(),
        )

        # Section: Publish scheduler readiness and terminal command results for voice/operator nodes.
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.command_result_pub = self.create_publisher(
            String,
            str(self.get_parameter("command_result_topic").value),
            10,
        )
        # Section: Reply and timeout callbacks stay runnable during guarded command work.
        self.clarification_callback_group = ReentrantCallbackGroup()
        self.clarification_response_sub = self.create_subscription(
            String,
            str(self.get_parameter("clarification_response_topic").value),
            self.clarification_response_callback,
            10,
            callback_group=self.clarification_callback_group,
        )

        # Section: A reentrant localization callback remains runnable while command capture waits.
        self.pose_callback_group = ReentrantCallbackGroup()
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("amcl_pose_topic").value),
            self.amcl_pose_callback,
            10,
            callback_group=self.pose_callback_group,
        )

        # Section: Keep action responses runnable while scheduler callbacks await their futures.
        self.executor_callback_group = ReentrantCallbackGroup()
        self.executor_client = ActionClient(
            self,
            ExecuteActionQueue,
            str(self.get_parameter("execute_action_queue_action").value),
            callback_group=self.executor_callback_group,
        )
        # Section: Command-topic intake is created only after runtime dependencies are ready.
        self.command_subscription = None

        # Section: Expose service input for command schedulers that cannot use ROS actions directly.
        self.schedule_service = self.create_service(
            ScheduleCommand,
            str(self.get_parameter("schedule_command_service").value),
            self.schedule_service_callback,
        )

        # Section: Expose an operator interface that clears ERROR and reopens command intake.
        self.reset_state_service = self.create_service(
            Trigger,
            str(self.get_parameter("reset_state_service").value),
            self.reset_state_callback,
        )

        # Section: Expose top-level deterministic task scheduling action.
        self.action_server = ActionServer(
            self,
            ExecuteTaskPlan,
            str(self.get_parameter("execute_task_plan_action").value),
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # Section: Defer initial navigation until the separately launched queue executor is ready.
        self.initialization_timer = self.create_timer(1.0, self.initialize_competition_cycle)
        self.error_recovery_timer = self.create_timer(1.0, self.recover_from_error)
        self.state_timer = self.create_timer(1.0, self.publish_cycle_state)
        self.clarification_timer = self.create_timer(
            0.5, self.check_clarification_timeout,
            callback_group=self.clarification_callback_group,
        )
        self.publish_cycle_state()

        # Section: Log initialization without claiming that command intake is available yet.
        self.get_logger().info(
            "GPSR task scheduler initializing: "
            f"dry_run={self.get_parameter('dry_run').value}, "
            f"command_topic={self.get_parameter('command_topic').value}, "
            f"schedule_service={self.get_parameter('schedule_command_service').value}, "
            f"executor={self.get_parameter('execute_action_queue_action').value}"
        )

    def default_config_path(self) -> str:
        """Special function: Resolve installed scheduler YAML path if available."""
        try:
            return str(Path(get_package_share_directory("gpsr_task_scheduler")) / "config" / "scheduler.yaml")
        except Exception:
            return ""

    def load_runtime_config(self) -> SchedulerConfig:
        """Special function: Apply ROS launch parameters over YAML scheduler defaults."""
        registry_file = str(self.get_parameter("skill_registry_file").value).strip()
        config = load_config(
            str(self.get_parameter("config_file").value),
            registry_file or None,
        )
        instruction_point = str(self.get_parameter("instruction_point").value).strip()
        if instruction_point:
            config.instruction_point = instruction_point
        config.return_after_completion = bool(self.get_parameter("return_after_completion").value) or bool(config.return_after_completion)
        config.return_after_failure = bool(self.get_parameter("return_after_failure").value) or bool(config.return_after_failure)
        config.default_skill_timeout_sec = float(self.get_parameter("default_skill_timeout_sec").value)
        config.default_skill_retries = int(self.get_parameter("default_skill_retries").value)
        config.actions["execute_action_queue"] = str(self.get_parameter("execute_action_queue_action").value)
        config.allow_simulated_primitives = bool(
            self.get_parameter("allow_simulated_primitives").value
        )
        return config

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """Special function: Cache the newest AMCL observation for command-start capture."""
        # Section: Deep-copy middleware-owned data before releasing the callback.
        with self.amcl_pose_condition:
            self.latest_amcl_pose = copy.deepcopy(msg)
            self.latest_amcl_pose_receipt_time = time.monotonic()
            self.amcl_pose_condition.notify_all()

    def capture_return_target(
        self,
        command_id: str,
        dry_run: bool,
    ) -> ReturnTarget:
        """Special function: Freeze a current pose or later-return named fallback."""
        mode = str(self.get_parameter("return_target_mode").value).strip().lower()
        fallback = self.named_fallback_target()
        if dry_run:
            # Section: Dry-run exposes policy without waiting for localization or claiming capture.
            return ReturnTarget(
                source=f"dry_run_{mode}",
                target_location=fallback.target_location if mode == "named_location" else "",
            )
        if mode == "named_location":
            # Section: Preserve named navigation only as an explicit legacy operator policy.
            return ReturnTarget(
                source="named_location",
                target_location=fallback.target_location,
            )
        if mode != "command_start_amcl":
            self.get_logger().warn(
                f"Unknown return_target_mode '{mode}'; storing named return fallback."
            )
            return fallback

        timeout_sec = max(
            0.0,
            float(self.get_parameter("amcl_pose_capture_timeout_sec").value),
        )
        deadline = time.monotonic() + timeout_sec
        while True:
            with self.amcl_pose_condition:
                pose = copy.deepcopy(self.latest_amcl_pose)
                receipt_time = float(self.latest_amcl_pose_receipt_time)
                target_pose = self.validated_amcl_target_pose(pose, receipt_time)
                if target_pose is not None:
                    target = ReturnTarget(
                        source="command_start_amcl",
                        target_pose=target_pose,
                    )
                    self.get_logger().info(
                        f"Captured frozen AMCL return pose for {command_id}."
                    )
                    return target
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self.amcl_pose_condition.wait(timeout=min(0.1, remaining))

        self.get_logger().warn(
            f"Unable to capture fresh return pose for {command_id}; command reception "
            f"remains valid and named return fallback '{fallback.target_location}' is stored."
        )
        return fallback

    def named_fallback_target(self) -> ReturnTarget:
        """Special function: Resolve the explicitly requested legacy named return target."""
        location = str(
            self.get_parameter("return_fallback_named_location").value
            or self.config.instruction_point
        ).strip()
        return ReturnTarget(
            source="instruction_point_fallback",
            target_location=location or self.config.instruction_point,
        )

    def validated_amcl_target_pose(
        self,
        msg: PoseWithCovarianceStamped | None,
        receipt_time: float,
    ) -> dict[str, Any] | None:
        """Special function: Validate and normalize one fresh map-frame AMCL pose."""
        if msg is None:
            return None
        max_age = max(0.0, float(self.get_parameter("amcl_pose_max_age_sec").value))
        if receipt_time <= 0.0 or time.monotonic() - receipt_time > max_age:
            return None
        frame_id = str(msg.header.frame_id).strip().lstrip("/")
        if frame_id != "map":
            return None
        pose = msg.pose.pose
        values = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        if not all(math.isfinite(float(value)) for value in values):
            return None
        quaternion_norm = math.sqrt(
            float(pose.orientation.x) ** 2
            + float(pose.orientation.y) ** 2
            + float(pose.orientation.z) ** 2
            + float(pose.orientation.w) ** 2
        )
        if quaternion_norm <= 1e-9:
            return None
        return {
            "frame_id": "map",
            "position": {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
            },
            "orientation": {
                "x": float(pose.orientation.x) / quaternion_norm,
                "y": float(pose.orientation.y) / quaternion_norm,
                "z": float(pose.orientation.z) / quaternion_norm,
                "w": float(pose.orientation.w) / quaternion_norm,
            },
        }

    @staticmethod
    def return_target_to_dict(target: ReturnTarget) -> dict[str, Any]:
        """Special function: Serialize frozen return policy for plans and diagnostics."""
        return {
            "source": target.source,
            "target_location": target.target_location,
            "target_pose": copy.deepcopy(target.target_pose),
        }

    def command_callback(self, msg: String) -> None:
        """Special function: Admit and execute one topic command through the guarded cycle."""
        # Section: Voice may supply an ID for end-to-end verdict correlation; raw text stays supported.
        command_id = f"topic_{uuid.uuid4().hex}"
        instruction = str(msg.data)
        try:
            envelope = json.loads(instruction)
        except (TypeError, ValueError):
            envelope = None
        if isinstance(envelope, dict) and envelope.get("type") == "gpsr_voice_command":
            command_id = str(envelope.get("command_id") or command_id)
            instruction = str(envelope.get("instruction") or "")
        # Section: A pending clarification owns the next transcript, never a new task.
        lock = getattr(self, "clarification_lock", None)
        if lock is not None:
            with lock:
                if self.command_intake_busy or self.clarification_session is not None:
                    self.publish_command_result({
                        "accepted": False, "success": False, "command_id": command_id,
                        "source": "topic", "message": "clarification_pending",
                    })
                    return
                self.command_intake_busy = True
        dry_run = bool(self.get_parameter("dry_run").value)
        timeout_sec = float(self.config.default_skill_timeout_sec or 300.0)
        try:
            result = (
                self.process_one_command(command_id, instruction, dry_run, timeout_sec, from_topic=True)
                if lock is not None else
                self.process_one_command(command_id, instruction, dry_run, timeout_sec)
            )
            # Section: Keep only the partial command when semantic clarification is required.
            validation = ((result.get("plan") or {}).get("command_context") or {}).get("validation") or {}
            if validation.get("status") in {"MISSING_ACTION", "MISSING_TARGET", "MISSING_LOCATION", "MULTIPLE_MISSING"}:
                self.start_clarification(result, instruction, dry_run, timeout_sec, validation)
        finally:
            if lock is not None:
                with lock:
                    self.command_intake_busy = False
        result["source"] = "topic"
        # Section: Executed commands publish inside process_one_command; early outcomes publish here.
        if not isinstance(result.get("execution"), dict):
            self.publish_command_result(result)
        if "clarification" in result:
            return
        if not result["accepted"]:
            self.get_logger().warn(f"GPSR command rejected: {result['message']}")
        elif not result["success"]:
            self.get_logger().warn(f"GPSR command failed: {result['message']}")

    def start_clarification(
        self,
        result: dict[str, Any],
        instruction: str,
        dry_run: bool,
        timeout_sec: float,
        validation: dict[str, Any],
    ) -> None:
        """Special function: Publish a correlated prompt while preserving parsed context."""
        # Section: A disabled dialogue limit returns the ordinary invalid-plan verdict.
        maximum = max(0, int(self.get_parameter("max_clarification_attempts").value))
        if maximum == 0:
            return
        deadline = time.monotonic() + max(1.0, float(self.get_parameter("clarification_timeout_sec").value))
        session = ClarificationSession(str(result["command_id"]), instruction, 1, deadline, dry_run, timeout_sec, dict(validation))
        with self.clarification_lock:
            self.clarification_session = session
        result["clarification"] = {
            "command_id": session.command_id,
            "attempt": session.attempt,
            "max_attempts": maximum,
            "status": str(validation["status"]),
            "missing_fields": list(validation.get("missing_fields", [])),
            "prompt": str(validation["clarification_prompt"]),
        }
        self.get_logger().info(
            f"GPSR clarification {session.attempt}/{maximum}: "
            f"{validation['status']} {validation.get('missing_fields', [])}"
        )
        self.publish_cycle_state()

    def clarification_response_callback(self, msg: String) -> None:
        """Special function: Reparse one reply only for its matching dialogue attempt."""
        try:
            payload = json.loads(str(msg.data))
        except (TypeError, ValueError):
            return
        if not isinstance(payload, dict):
            return
        with self.clarification_lock:
            session = self.clarification_session
            if (
                session is None or self.command_intake_busy
                or str(payload.get("command_id", "")) != session.command_id
                or payload.get("attempt") != session.attempt
            ):
                return
            self.command_intake_busy = True
        try:
            # Section: Expired or empty replies cannot accidentally complete a task.
            if time.monotonic() >= session.deadline:
                self.finish_clarification(session, "clarification_timeout")
                return
            transcript = str(payload.get("transcript") or "").strip()
            self.get_logger().info(f"GPSR clarification reply received: attempt {session.attempt}.")
            merged = merge_clarification(session.instruction, transcript, self.config)
            if transcript and merged != session.instruction:
                result = self.process_one_command(
                    session.command_id, merged, session.dry_run, session.timeout_sec,
                    from_clarification=True,
                )
                validation = ((result.get("plan") or {}).get("command_context") or {}).get("validation") or {}
                if validation.get("status") not in {"MISSING_ACTION", "MISSING_TARGET", "MISSING_LOCATION", "MULTIPLE_MISSING"}:
                    with self.clarification_lock:
                        self.clarification_session = None
                    result["source"] = "topic"
                    self.publish_command_result(result)
                    self.publish_cycle_state()
                    return
                session.instruction = merged
                session.validation = dict(validation)
            else:
                validation = session.validation
            maximum = max(0, int(self.get_parameter("max_clarification_attempts").value))
            if session.attempt >= maximum:
                self.finish_clarification(session, "clarification_retry_exhausted")
                return
            session.attempt += 1
            session.deadline = time.monotonic() + max(1.0, float(self.get_parameter("clarification_timeout_sec").value))
            result = {
                "accepted": False, "success": False, "command_id": session.command_id,
                "source": "topic", "message": "clarification_required",
                "clarification": {
                    "command_id": session.command_id,
                    "attempt": session.attempt,
                    "max_attempts": maximum,
                    "status": str(validation.get("status", "MULTIPLE_MISSING")),
                    "missing_fields": list(validation.get("missing_fields", [])),
                    "prompt": str(validation.get("clarification_prompt", "")),
                },
            }
            self.publish_command_result(result)
            self.publish_cycle_state()
        finally:
            with self.clarification_lock:
                self.command_intake_busy = False

    def finish_clarification(self, session: ClarificationSession, reason: str, require_idle: bool = False) -> None:
        """Special function: Abandon a timed-out or exhausted dialogue without dispatch."""
        with self.clarification_lock:
            if self.clarification_session is not session or (require_idle and self.command_intake_busy):
                return
            self.clarification_session = None
        self.get_logger().warn(f"GPSR clarification abandoned: {reason}.")
        self.publish_command_result({
            "accepted": False, "success": False, "command_id": session.command_id,
            "source": "topic", "message": reason, "failed_step": "clarification",
        })
        self.publish_cycle_state()

    def check_clarification_timeout(self) -> None:
        """Special function: End a silent clarification window on the ROS timer."""
        with self.clarification_lock:
            session = self.clarification_session if not self.command_intake_busy else None
        if session is not None and time.monotonic() >= session.deadline:
            self.finish_clarification(session, "clarification_timeout", require_idle=True)

    def publish_command_result(self, result: dict[str, Any]) -> None:
        """Special function: Publish one structured command verdict for voice and operators."""
        # Section: Preserve the complete scheduler result as deterministic JSON.
        result_msg = String()
        result_msg.data = json.dumps(result, sort_keys=True)
        self.command_result_pub.publish(result_msg)

    def schedule_service_callback(
        self,
        request: ScheduleCommand.Request,
        response: ScheduleCommand.Response,
    ) -> ScheduleCommand.Response:
        """Special function: Process one service command without bypassing cycle admission."""
        command_id = str(request.command_id or f"service_{uuid.uuid4().hex}")
        instruction = str(request.instruction or "")
        global_dry_run = bool(self.get_parameter("dry_run").value)
        execute_requested = bool(request.execute)
        dry_run = bool(request.dry_run) or global_dry_run or not execute_requested
        processed = self.process_one_command(
            command_id,
            instruction,
            dry_run,
            max(1.0, float(request.overall_timeout_sec or 300.0)),
        )
        # Section: Service callers receive JSON, while operators also need an exact terminal log.
        if not bool(processed["accepted"]):
            self.get_logger().warn(
                f"GPSR service command rejected: command_id={command_id} "
                f"reason={processed['message']}"
            )
        elif not bool(processed["success"]):
            self.get_logger().error(
                f"GPSR service command failed: command_id={command_id} "
                f"failed_step={processed.get('failed_step', 'unknown')} "
                f"reason={processed['message']}"
            )
        response.accepted = bool(processed["accepted"])
        response.success = bool(processed["success"])
        response.message = str(processed["message"])
        response.plan_json = json.dumps(processed.get("plan", {}), sort_keys=True)
        response.execution_log_json = json.dumps(processed, sort_keys=True)
        return response

    def reset_state_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Special function: Reset ERROR directly to command intake without recovery motion."""
        snapshot = self.cycle.snapshot()
        if snapshot.state != SchedulerState.ERROR:
            response.success = False
            response.message = (
                f"Reset rejected while scheduler is {snapshot.state.value}; "
                "only ERROR can be reset."
            )
            return response
        # Section: Clear exhausted automatic-recovery bookkeeping before the operator override.
        self.error_recovery_attempts = 0
        self.error_recovery_next_attempt_time = 0.0
        self.error_recovery_exhaustion_logged = False
        # Section: Abandon frozen motion/report context together with stale command ownership.
        self.command_return_targets.clear()
        self.pending_post_return.clear()
        response.success = bool(self.cycle.reset_error())
        response.message = "GPSR ERROR reset; scheduler is WAITING_FOR_COMMAND."
        self.get_logger().warn(response.message)
        self.publish_cycle_state()
        return response

    def goal_callback(self, goal_request: ExecuteTaskPlan.Goal) -> GoalResponse:
        """Special function: Reject empty or concurrently submitted top-level goals."""
        if not str(goal_request.original_instruction or goal_request.structured_task_json or "").strip():
            return GoalResponse.REJECT
        # Section: An action goal cannot overtake an active spoken clarification.
        with self.clarification_lock:
            if self.clarification_session is not None or self.command_intake_busy:
                return GoalResponse.REJECT
        return (
            GoalResponse.ACCEPT
            if self.cycle.snapshot().state == SchedulerState.WAITING_FOR_COMMAND
            else GoalResponse.REJECT
        )

    def cancel_callback(self, _goal_handle: Any) -> CancelResponse:
        """Special function: Allow cancellation before or during queue dispatch."""
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: Any) -> ExecuteTaskPlan.Result:
        """Special function: Execute one admitted action command through the guarded cycle."""
        goal = goal_handle.request
        dry_run = bool(goal.dry_run) or bool(self.get_parameter("dry_run").value)
        command_id = str(goal.command_id or f"goal_{uuid.uuid4().hex}")
        processed = self.process_one_command(
            command_id,
            str(goal.original_instruction),
            dry_run,
            max(1.0, float(goal.overall_timeout_sec or 300.0)),
        )
        feedback = ExecuteTaskPlan.Feedback()
        feedback.current_step = ""
        feedback.current_skill = ""
        feedback.completed_count = int(processed.get("completed_steps", 0))
        feedback.total_count = len(processed.get("plan", {}).get("steps", []))
        feedback.status_message = str(processed["message"])
        goal_handle.publish_feedback(feedback)
        result = ExecuteTaskPlan.Result()
        result.success = bool(processed["success"])
        result.completed_steps = int(processed.get("completed_steps", 0))
        result.failed_step = str(processed.get("failed_step", ""))
        result.failure_reason = "" if result.success else str(processed["message"])
        result.execution_log_json = json.dumps(processed, sort_keys=True)
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def initialize_competition_cycle(self) -> None:
        """Special function: Wait for internal runtime before opening command intake."""
        if self.initialization_started:
            return
        # Section: The executor action appears only after its lifecycle clients are ready.
        require_executor = bool(self.get_parameter("require_executor_ready").value)
        if require_executor and not self.executor_client.server_is_ready():
            self.get_logger().info(
                "GPSR scheduler waiting for the ActionQueue executor and lifecycle manager.",
                throttle_duration_sec=10.0,
            )
            return
        self.initialization_started = True
        self.initialization_timer.cancel()
        # Section: Default AMCL-return mode must not move before the first command begins.
        dry_run = bool(self.get_parameter("dry_run").value)
        navigate_on_start = (
            bool(self.get_parameter("navigate_to_instruction_point_on_start").value)
            and not dry_run
        )
        if not navigate_on_start:
            self.cycle.finish_initialization_without_navigation()
            self.enable_command_topic_intake()
            self.publish_cycle_state()
            self.log_command_ready()
            return

        # Section: Backward-compatible named startup navigation remains explicit opt-in behavior.
        self.cycle.start_initial_return()
        self.publish_cycle_state()
        return_result = self.submit_return_to_instruction_point("gpsr_initialization")
        self.cycle.finish_initial_return(bool(return_result["success"]))
        if bool(return_result["success"]):
            self.enable_command_topic_intake()
        self.publish_cycle_state()
        if bool(return_result["success"]):
            self.log_command_ready()

    def enable_command_topic_intake(self) -> None:
        """Special function: Subscribe only after the scheduler reaches its readiness barrier."""
        # Section: Action/service input remains state-gated; topic input must not lose early commands.
        if self.command_subscription is not None:
            return
        if str(self.get_parameter("command_input_mode").value) != "topic":
            return
        self.command_subscription = self.create_subscription(
            String,
            str(self.get_parameter("command_topic").value),
            self.command_callback,
            10,
        )

    def log_command_ready(self) -> None:
        """Special function: Announce readiness only after every internal startup barrier passes."""
        self.get_logger().info(
            "GPSR task scheduler ready to receive commands: "
            f"topic={self.get_parameter('command_topic').value}, "
            f"service={self.get_parameter('schedule_command_service').value}, "
            f"executor={self.get_parameter('execute_action_queue_action').value}"
        )

    def recover_from_error(self) -> None:
        """Special function: Retry the frozen return target and resume pending reporting."""
        snapshot = self.cycle.snapshot()
        if snapshot.state != SchedulerState.ERROR:
            return
        if not bool(self.get_parameter("enable_error_recovery").value):
            return

        # Section: Bound automatic motion retries while allowing zero to mean unlimited attempts.
        max_attempts = max(
            0,
            int(self.get_parameter("error_recovery_max_attempts").value),
        )
        if max_attempts and self.error_recovery_attempts >= max_attempts:
            if not self.error_recovery_exhaustion_logged:
                self.get_logger().error(
                    "GPSR error recovery exhausted; scheduler remains in ERROR."
                )
                self.error_recovery_exhaustion_logged = True
            return
        if time.monotonic() < self.error_recovery_next_attempt_time:
            return
        if not self.cycle.start_error_recovery():
            return

        # Section: RECOVERING rejects commands while the normal executor cleanup barrier runs.
        self.error_recovery_attempts += 1
        attempt = self.error_recovery_attempts
        self.publish_cycle_state()
        self.get_logger().warn(
            f"GPSR recovery attempt {attempt}: returning to the frozen command start."
        )
        command_id = str(snapshot.active_command_id)
        return_result = (
            self.submit_return_to_command_start(
                command_id,
                queue_id=f"{command_id}_recovery_{attempt}",
            )
            if command_id
            else self.submit_return_to_instruction_point(
                f"gpsr_error_recovery_{attempt}"
            )
        )
        success = bool(return_result.get("success", False))
        pending = self.pending_post_return.get(command_id)
        # Section: Any stored report context includes explicit speech or a mandatory miss report.
        report_pending = pending is not None
        self.cycle.finish_error_recovery(success, report_pending=report_pending)
        if success:
            # Section: A delayed report remains gated on confirmed recovery arrival.
            if report_pending and pending is not None:
                report_result = self.execute_post_return_report(
                    command_id,
                    pending["planned"],
                    pending["task_result"],
                    float(pending["timeout_sec"]),
                )
                self.cycle.finish_reporting(command_id)
                self.get_logger().info(
                    "GPSR recovery reached the saved pose; pending report finished "
                    f"with {report_result.get('outcome', 'unknown')}."
                )
            else:
                self.get_logger().info(
                    "GPSR recovery succeeded; command intake is available again."
                )
            self.clear_command_context(command_id)
            self.error_recovery_attempts = 0
            self.error_recovery_next_attempt_time = 0.0
            self.error_recovery_exhaustion_logged = False
        else:
            retry_delay = max(
                0.1,
                float(self.get_parameter("error_recovery_retry_delay_sec").value),
            )
            self.error_recovery_next_attempt_time = time.monotonic() + retry_delay
            self.get_logger().warn(
                f"GPSR recovery attempt {attempt} failed: "
                f"{return_result.get('message', 'unknown return failure')}; "
                f"retrying after {retry_delay:.1f}s."
            )
        self.publish_cycle_state()

    def process_one_command(
        self,
        command_id: str,
        instruction: str,
        dry_run: bool,
        timeout_sec: float,
        from_clarification: bool = False,
        from_topic: bool = False,
    ) -> dict[str, Any]:
        """Special function: Plan, execute, clean up, return, and reset one command."""
        # Section: Service/action callers cannot replace a pending topic transaction.
        lock = getattr(self, "clarification_lock", None)
        if lock is not None and not from_clarification:
            with lock:
                pending = self.clarification_session
            if pending is not None or (self.command_intake_busy and not from_topic):
                return {
                    "accepted": False, "success": False, "command_id": command_id,
                    "message": "clarification_pending", "failed_step": "clarification",
                }
        if not self.cycle.try_accept_command(command_id):
            snapshot = self.cycle.snapshot()
            return {
                "accepted": False,
                "success": False,
                "command_id": command_id,
                "message": f"Scheduler is {snapshot.state.value}; command not accepted.",
                "state": snapshot.state.value,
            }
        self.publish_cycle_state()

        # Section: Freeze exactly one return destination before planning or robot execution.
        return_target = self.capture_return_target(command_id, dry_run)
        self.command_return_targets[command_id] = return_target
        self.cycle.finish_return_pose_capture(command_id)
        self.publish_cycle_state()

        # Section: Generate exactly one task plan from the admitted command.
        planned = self.schedule(command_id, instruction, dry_run)
        if not planned.get("valid"):
            self.cycle.finish_planning(command_id, False)
            self.clear_command_context(command_id)
            self.publish_cycle_state()
            return {
                "accepted": False,
                "success": False,
                "command_id": command_id,
                "message": json.dumps(planned.get("issues", []), sort_keys=True),
                "failed_step": "planning",
                "plan": planned,
            }
        if dry_run:
            self.cycle.finish_planning(command_id, False)
            self.clear_command_context(command_id)
            self.publish_cycle_state()
            return {
                "accepted": True,
                "success": True,
                "command_id": command_id,
                "message": "Plan generated; execution skipped because dry_run is true.",
                "completed_steps": 0,
                "plan": planned,
            }

        # Section: Execute all internal skills before initiating the single return transition.
        self.cycle.finish_planning(command_id, True)
        self.publish_cycle_state()
        executable_plan = self.without_embedded_return(planned)
        queue_result = self.submit_action_queue(command_id, executable_plan, timeout_sec)
        self.cycle.finish_execution(command_id)
        self.publish_cycle_state()

        # Section: Preserve requested reports and always announce an exhaustive FIND_OBJECT miss.
        report_pending = bool(planned.get("post_return_steps")) or GpsrTaskSchedulerNode.requires_not_found_report(
            planned,
            queue_result,
        )
        if report_pending:
            self.pending_post_return[command_id] = {
                "planned": copy.deepcopy(planned),
                "task_result": copy.deepcopy(queue_result),
                "timeout_sec": float(timeout_sec),
            }
        return_result = self.submit_return_to_command_start(command_id)
        return_success = bool(return_result.get("success", False))
        self.cycle.finish_return(
            command_id,
            return_success,
            report_pending=report_pending,
        )
        self.publish_cycle_state()

        # Section: Submit speech only after confirmed return; return failure leaves it pending.
        report_result: dict[str, Any] | None = None
        if return_success and report_pending:
            report_result = self.execute_post_return_report(
                command_id,
                planned,
                queue_result,
                timeout_sec,
            )
            self.cycle.finish_reporting(command_id)
            self.publish_cycle_state()

        task_success = bool(queue_result.get("success", False))
        report_success = bool(report_result is None or report_result.get("success", False))
        success = bool(task_success and return_success and report_success)
        message = self.command_completion_message(
            task_result=queue_result,
            return_result=return_result,
            report_result=report_result,
        )
        processed = {
            "accepted": True,
            "success": success,
            "command_id": command_id,
            "message": message,
            "completed_steps": (
                int(queue_result.get("completed_steps", 0))
                + int(return_result.get("completed_steps", 0))
                + int((report_result or {}).get("completed_steps", 0))
            ),
            "failed_step": "" if success else (
                "execute_action_queue"
                if not task_success
                else (
                    "return_to_command_start"
                    if not return_success
                    else "post_return_report"
                )
            ),
            "plan": planned,
            "execution": queue_result,
            "return": return_result,
            "post_return_report": report_result,
            "state": self.cycle.snapshot().state.value,
        }
        # Section: A clarified execution reports only after its session is released.
        if not from_clarification:
            result_msg = String()
            result_msg.data = json.dumps(processed, sort_keys=True)
            self.command_result_pub.publish(result_msg)
        if return_success:
            self.clear_command_context(command_id)
        return processed

    def without_embedded_return(self, planned: dict[str, Any]) -> dict[str, Any]:
        """Special function: Keep return navigation outside the command's internal skill plan."""
        executable = dict(planned)
        executable["steps"] = [
            step
            for step in planned.get("steps", [])
            if str(step.get("step_id", "")) != "return_to_instruction_point"
        ]
        return executable

    def submit_return_to_instruction_point(self, command_id: str) -> dict[str, Any]:
        """Special function: Request return navigation through the existing queue executor."""
        target = ReturnTarget(
            source="startup_instruction_point",
            target_location=self.config.instruction_point,
        )
        return_plan = self.build_return_plan(target)
        return self.submit_action_queue(
            f"{command_id}_return",
            return_plan,
            max(1.0, float(self.config.timeout_for("NAVIGATE"))),
        )

    def submit_return_to_command_start(
        self,
        command_id: str,
        queue_id: str | None = None,
    ) -> dict[str, Any]:
        """Special function: Navigate to the immutable return target captured for one command."""
        target = self.command_return_targets.get(command_id, self.named_fallback_target())
        return_plan = self.build_return_plan(target)
        return self.submit_action_queue(
            queue_id or f"{command_id}_return",
            return_plan,
            max(1.0, float(self.config.timeout_for("NAVIGATE"))),
        )

    def build_return_plan(self, target: ReturnTarget) -> dict[str, Any]:
        """Special function: Build one raw-pose or named NAVIGATE return queue."""
        arguments: dict[str, Any] = {"target_source": target.source}
        if target.target_pose is not None:
            arguments["target_pose"] = copy.deepcopy(target.target_pose)
        else:
            arguments["target_location"] = target.target_location
        return {
            "steps": [{
                "step_id": "return_to_command_start",
                "skill_name": "NAVIGATE",
                "arguments": arguments,
                "dependencies": [],
                "timeout": float(self.config.timeout_for("NAVIGATE")),
                "retry_count": int(self.config.retries_for("NAVIGATE")),
                "result_variable": None,
            }]
        }

    def execute_post_return_report(
        self,
        command_id: str,
        planned: dict[str, Any],
        task_result: dict[str, Any],
        timeout_sec: float,
    ) -> dict[str, Any]:
        """Special function: Submit deferred speech after confirmed return arrival."""
        report_plan = self.build_post_return_plan(planned, task_result)
        if not report_plan["steps"]:
            return {
                "success": True,
                "completed_steps": 0,
                "message": "No post-return report requested.",
                "outcome": "success",
                "results": [],
            }
        return self.submit_action_queue(
            f"{command_id}_report",
            report_plan,
            max(1.0, float(timeout_sec)),
        )

    def build_post_return_plan(
        self,
        planned: dict[str, Any],
        task_result: dict[str, Any],
    ) -> dict[str, Any]:
        """Special function: Rebase report dependencies and inject cross-queue result data."""
        steps = copy.deepcopy(list(planned.get("post_return_steps", [])))
        # Section: Synthesize mandatory target-specific speech when FIND_OBJECT exhausts its scope.
        if not steps and GpsrTaskSchedulerNode.requires_not_found_report(planned, task_result):
            steps = [{
                "step_id": "report_find_object_not_found",
                "skill_name": "SAY",
                "arguments": {"text": GpsrTaskSchedulerNode.failure_report_text(planned, task_result)},
                "dependencies": [],
                "timeout": float(self.config.timeout_for("SAY")),
                "retry_count": int(self.config.retries_for("SAY")),
                "result_variable": None,
            }]
        task_success = bool(task_result.get("success", False))
        result_variables = GpsrTaskSchedulerNode.extract_result_variables(planned, task_result)
        for step in steps:
            # Section: Scheduler success/failure gates replace dependencies from the prior queue.
            step["dependencies"] = []
            arguments = dict(step.get("arguments", {}))
            substitution = str(arguments.get("substitution", "")).strip()
            if task_success:
                if substitution and substitution in result_variables:
                    arguments["substitution_data"] = result_variables[substitution]
            else:
                arguments["text"] = GpsrTaskSchedulerNode.failure_report_text(planned, task_result)
                arguments.pop("substitution", None)
                arguments.pop("substitution_data", None)
            step["arguments"] = arguments
        return {"steps": steps}

    @staticmethod
    def requires_not_found_report(
        planned: dict[str, Any],
        task_result: dict[str, Any],
    ) -> bool:
        """Special function: Require speech for an exhaustive FIND_OBJECT miss."""
        request = dict(planned.get("task_request") or {})
        outcomes = {
            str(item.get("outcome", "")).strip().lower()
            for item in task_result.get("results", [])
        }
        return request.get("task_type") == "find_object" and "not_found" in outcomes

    @staticmethod
    def extract_result_variables(
        planned: dict[str, Any],
        task_result: dict[str, Any],
    ) -> dict[str, dict[str, Any]]:
        """Special function: Match successful executor results to declared result variables."""
        available = list(task_result.get("results", []))
        variables: dict[str, dict[str, Any]] = {}
        cursor = 0
        for step in planned.get("steps", []):
            variable = str(step.get("result_variable") or "").strip()
            if not variable:
                continue
            action_type = str(step.get("skill_name", ""))
            for index in range(cursor, len(available)):
                candidate = dict(available[index])
                if (
                    str(candidate.get("action_type", "")) == action_type
                    and str(candidate.get("outcome", "")) == "success"
                ):
                    variables[variable] = dict(candidate.get("result", {}))
                    cursor = index + 1
                    break
        return variables

    @staticmethod
    def failure_report_text(
        planned: dict[str, Any],
        task_result: dict[str, Any],
    ) -> str:
        """Special function: Produce deterministic post-return failure speech."""
        request = dict(planned.get("task_request") or {})
        target = dict(request.get("target") or {})
        outcomes = {str(item.get("outcome", "")) for item in task_result.get("results", [])}
        if request.get("task_type") == "find_object" and "not_found" in outcomes:
            object_name = str(
                target.get("object_name") or target.get("category") or "object"
            ).strip() or "object"
            return f"I could not find the {object_name}."
        return "I could not complete the requested task."

    @staticmethod
    def command_completion_message(
        task_result: dict[str, Any],
        return_result: dict[str, Any],
        report_result: dict[str, Any] | None,
    ) -> str:
        """Special function: Summarize task, return, and deferred-report outcomes."""
        return (
            f"task={task_result.get('message', 'unknown')}; "
            f"return={return_result.get('message', 'unknown')}; "
            f"report={(report_result or {}).get('message', 'not_requested_or_pending')}"
        )

    def clear_command_context(self, command_id: str) -> None:
        """Special function: Release frozen pose and deferred report after terminal handling."""
        self.command_return_targets.pop(str(command_id), None)
        self.pending_post_return.pop(str(command_id), None)

    def publish_cycle_state(self) -> None:
        """Special function: Publish current readiness, identity, and competition count."""
        snapshot = self.cycle.snapshot()
        # Section: Keep the YASMIN supervisory state synchronized with the guarded cycle.
        if self.scheduler_viewer is not None:
            self.scheduler_viewer.update(snapshot.state)
        # Section: Keep established state values while exposing dialogue progress to monitors.
        with self.clarification_lock:
            session = self.clarification_session
            clarification_status = "WAITING_FOR_CLARIFICATION" if session is not None else ""
            clarification_command_id = session.command_id if session is not None else ""
        msg = String()
        msg.data = json.dumps({
            "state": snapshot.state.value,
            "clarification_status": clarification_status,
            "pending_clarification_command_id": clarification_command_id,
            "active_command_id": snapshot.active_command_id,
            "completed_command_count": snapshot.completed_command_count,
            "max_commands": snapshot.max_commands,
            "error_recovery_attempts": self.error_recovery_attempts,
            "error_recovery_max_attempts": max(
                0,
                int(self.get_parameter("error_recovery_max_attempts").value),
            ),
        }, sort_keys=True)
        self.state_pub.publish(msg)

    def destroy_node(self) -> Any:
        """Special function: Stop the scheduler viewer before destroying ROS resources."""
        # Section: Prevent the YASMIN worker and publisher timer from outliving this node.
        if self.scheduler_viewer is not None:
            self.scheduler_viewer.shutdown()
            self.scheduler_viewer = None
        return super().destroy_node()

    def schedule(self, command_id: str, instruction: str, dry_run: bool) -> dict[str, Any]:
        """Special function: Build and publish one deterministic plan dictionary."""
        result = schedule_command(command_id, instruction, self.config)
        payload = plan_to_dict(result)
        payload["dry_run"] = bool(dry_run)
        # Section: Expose the frozen target or dry-run policy beside the generated task plan.
        target = self.command_return_targets.get(command_id)
        payload["return_policy"] = (
            self.return_target_to_dict(target)
            if target is not None
            else {"source": "not_captured", "target_location": "", "target_pose": None}
        )
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.plan_pub.publish(msg)
        self.get_logger().info(msg.data)
        return payload

    async def dispatch_queue(self, goal_handle: Any, command_id: str, planned: dict[str, Any]) -> ExecuteTaskPlan.Result:
        """Special function: Submit a planned queue to the existing YASMIN executor."""
        result = ExecuteTaskPlan.Result()
        result.execution_log_json = json.dumps(planned, sort_keys=True)
        queue_result = self.submit_action_queue(command_id, planned, max(1.0, float(goal_handle.request.overall_timeout_sec or 300.0)))
        result.success = bool(queue_result["success"])
        result.completed_steps = int(queue_result["completed_steps"])
        result.failed_step = "" if result.success else "execute_action_queue"
        result.failure_reason = "" if result.success else str(queue_result["message"])
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def submit_action_queue(self, command_id: str, planned: dict[str, Any], timeout_sec: float) -> dict[str, Any]:
        """Special function: Submit an ActionQueue goal and return a JSON-friendly execution summary."""
        # Section: Defensive dispatch gate covers all command plans, including direct callers.
        if planned.get("valid") is False:
            return {
                "success": False, "completed_steps": 0,
                "message": "invalid_or_incomplete_plan", "outcome": "invalid_plan",
            }
        if not self.executor_client.wait_for_server(timeout_sec=5.0):
            return {
                "success": False,
                "completed_steps": 0,
                "message": f"Executor action unavailable: {self.config.actions.get('execute_action_queue', '/execute_action_queue')}",
                "outcome": "executor_unavailable",
            }
        queue_goal = ExecuteActionQueue.Goal()
        queue_goal.queue = self.build_action_queue(command_id, planned)
        queue_future = self.executor_client.send_goal_async(queue_goal)
        # Section: Allow a deployment-configurable window for executor goal acknowledgement.
        goal_response_timeout_sec = max(
            0.1,
            float(self.get_parameter("executor_goal_response_timeout_sec").value),
        )
        if not self.wait_for_future(queue_future, goal_response_timeout_sec):
            return {
                "success": False,
                "completed_steps": 0,
                "message": "Executor goal response timed out.",
                "outcome": "goal_response_timeout",
            }
        queue_handle = queue_future.result()
        if queue_handle is None or not queue_handle.accepted:
            return {
                "success": False,
                "completed_steps": 0,
                "message": "Executor rejected the queue goal.",
                "outcome": "goal_rejected",
            }
        wrapped_future = queue_handle.get_result_async()
        if not self.wait_for_future(wrapped_future, max(1.0, timeout_sec)):
            queue_handle.cancel_goal_async()
            return {
                "success": False,
                "completed_steps": 0,
                "message": "Executor result timed out.",
                "outcome": "result_timeout",
            }
        wrapped = wrapped_future.result()
        queue_result = wrapped.result
        outcome = str(queue_result.outcome)
        success = bool(queue_result.success)
        # Section: Preserve structured skill payloads for post-return result reporting.
        results = [self.skill_result_to_dict(item) for item in list(queue_result.results)]
        message = (
            "Execution succeeded."
            if success
            else summarize_action_queue_failure(outcome, results)
        )
        if not success:
            self.get_logger().error(
                f"GPSR ActionQueue failed: command_id={command_id} {message}"
            )
        return {
            "success": success,
            "completed_steps": len(results),
            "message": message,
            "outcome": outcome,
            "results": results,
        }

    @staticmethod
    def skill_result_to_dict(result: Any) -> dict[str, Any]:
        """Special function: Convert one executor SkillResult into JSON-safe diagnostics."""
        try:
            payload = json.loads(str(getattr(result, "result_json", "") or "{}"))
        except json.JSONDecodeError:
            payload = {"raw": str(getattr(result, "result_json", ""))}
        return {
            "action_type": str(getattr(result, "action_type", "")),
            "outcome": str(getattr(result, "outcome", "")),
            "message": str(getattr(result, "message", "")),
            "result": payload if isinstance(payload, dict) else {"value": payload},
            "recoverable": bool(getattr(result, "recoverable", False)),
            "fatal": bool(getattr(result, "fatal", False)),
        }

    def build_action_queue(self, command_id: str, planned: dict[str, Any]) -> ActionQueue:
        """Special function: Convert scheduler plan JSON into an ActionQueue message."""
        queue = ActionQueue()
        queue.task_id = command_id
        queue.steps = [self.build_action_step(command_id, step) for step in planned.get("steps", [])]
        return queue

    def build_action_step(self, command_id: str, step: dict[str, Any]) -> ActionStep:
        """Special function: Convert one scheduled step into an ActionStep message."""
        converted = step_to_action_step_dict(_dict_to_skill_step(step), command_id)
        msg = ActionStep()
        msg.step_id = converted["step_id"]
        msg.action_type = converted["action_type"]
        msg.task_id = converted["task_id"]
        msg.task_family = converted["task_family"]
        msg.target_object = converted["target_object"]
        msg.target_person = converted["target_person"]
        msg.target_location = converted["target_location"]
        msg.target_category = converted["target_category"]
        msg.parameters_json = converted["parameters_json"]
        msg.timeout_sec = converted["timeout_sec"]
        msg.max_retries = converted["max_retries"]
        msg.requires_success = converted["requires_success"]
        msg.requires_data = converted["requires_data"]
        msg.result_variable = converted["result_variable"]
        return msg

    def wait_for_future(self, future: Any, timeout_sec: float) -> bool:
        """Special function: Wait for a ROS future while executor threads continue spinning."""
        deadline = time.monotonic() + max(0.1, timeout_sec)
        while time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.02)
        return False


def _dict_to_skill_step(data: dict[str, Any]):
    """Special function: Rehydrate a SkillStep from JSON-friendly plan data."""
    from gpsr_task_scheduler.models import SkillStep
    return SkillStep(
        step_id=str(data.get("step_id", "")),
        skill_name=str(data.get("skill_name", "")),
        arguments=dict(data.get("arguments", {})),
        dependencies=list(data.get("dependencies", [])),
        timeout=float(data.get("timeout", 0.0) or 0.0),
        retry_count=int(data.get("retry_count", 0) or 0),
        result_variable=data.get("result_variable"),
    )


def main(args: list[str] | None = None) -> None:
    """Special function: Run the deterministic GPSR scheduler ROS node."""
    rclpy.init(args=args)
    node = GpsrTaskSchedulerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        # Section: SIGINT may already have shut down the default ROS context.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
