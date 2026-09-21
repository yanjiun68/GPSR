#!/usr/bin/env python3
"""Section: Lifecycle-managed adapter from GPSR PersonFollow to person_follower_master."""

from __future__ import annotations

import asyncio
import math
import threading
import time
from typing import Any

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from person_follower_master.msg import MasterTarget
from pfm_utils.follow_action_policy import master_has_moved, master_is_facing
from putrabot_interfaces.action import PersonFollow


# Section: Tracker state names are part of person_follower_master's published contract.
STATE_FOLLOWING = "FOLLOWING"
STATE_LOST = "LOST"


class FollowPersonSkillServer(LifecycleNode):
    """Special function: Own safe enrollment, feedback, completion, and reset for FOLLOW_PERSON."""

    def __init__(self) -> None:
        """Special function: Declare adapter policy while leaving task interfaces unconfigured."""
        super().__init__("follow_person_skill_server")

        # Section: Stable endpoints bridge the existing GPSR and master-follow contracts.
        self.cb_group = ReentrantCallbackGroup()
        # Section: Keep the master-stack provider distinct from the legacy follow server.
        self.declare_parameter("action_name", "/person_follow/master")
        self.declare_parameter("master_topic", "/master_target")
        self.declare_parameter("enroll_service", "/enroll_master")
        self.declare_parameter("reset_service", "/reset_master")
        self.declare_parameter(
            "controller_parameter_service",
            "/follow_controller_node/set_parameters",
        )
        self.declare_parameter("follow_cmd_topic", "/cmd_vel_follow")
        self.declare_parameter("output_cmd_topic", "/cmd_vel")

        # Section: Bounded enrollment, loss, and feedback policy prevents endless GPSR goals.
        self.declare_parameter("default_follow_distance_m", 1.2)
        self.declare_parameter("default_face_hold_sec", 5.0)
        self.declare_parameter("enrollment_timeout_sec", 15.0)
        self.declare_parameter("lost_abort_sec", 10.0)
        self.declare_parameter("master_stream_timeout_sec", 1.0)
        self.declare_parameter("feedback_rate_hz", 10.0)

        # Section: A stable, close, centered person acts as the stop-and-face completion gesture.
        self.declare_parameter("face_distance_margin_m", 0.25)
        self.declare_parameter("face_bearing_tolerance_deg", 10.0)
        self.declare_parameter("face_max_speed_mps", 0.12)
        self.declare_parameter("require_motion_before_completion", True)
        self.declare_parameter("completion_arm_distance_delta_m", 0.30)
        self.declare_parameter("completion_arm_bearing_deg", 15.0)
        self.declare_parameter("completion_arm_speed_mps", 0.18)

        # Section: Resource handles are allocated only during CONFIGURE.
        self.action_server: ActionServer | None = None
        self.master_subscription: Any | None = None
        self.enroll_client: Any | None = None
        self.reset_client: Any | None = None
        self.parameter_client: Any | None = None
        self.follow_cmd_subscription: Any | None = None
        self.output_cmd_publisher: Any | None = None
        self.skill_active = False
        self.motion_enabled = False
        self.latest_master: MasterTarget | None = None
        self.latest_master_monotonic = 0.0
        self.active_goal_handles: set[Any] = set()
        self.goal_lock = threading.Lock()
        self.goal_reserved = False
        self.get_logger().info("FollowPerson adapter is UNCONFIGURED.")

    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Allocate ROS interfaces required by the follow adapter."""
        try:
            # Section: Transient-local QoS receives the tracker's most recent master state immediately.
            master_qos = QoSProfile(depth=1)
            master_qos.reliability = ReliabilityPolicy.RELIABLE
            master_qos.history = HistoryPolicy.KEEP_LAST
            master_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.master_subscription = self.create_subscription(
                MasterTarget,
                str(self.get_parameter("master_topic").value),
                self.master_callback,
                master_qos,
                callback_group=self.cb_group,
            )
            self.enroll_client = self.create_client(
                Trigger,
                str(self.get_parameter("enroll_service").value),
                callback_group=self.cb_group,
            )
            self.reset_client = self.create_client(
                Trigger,
                str(self.get_parameter("reset_service").value),
                callback_group=self.cb_group,
            )
            self.parameter_client = self.create_client(
                SetParameters,
                str(self.get_parameter("controller_parameter_service").value),
                callback_group=self.cb_group,
            )
            # Section: Forward follow velocity only while this action owns the motion resource.
            command_qos = QoSProfile(depth=10)
            command_qos.reliability = ReliabilityPolicy.RELIABLE
            command_qos.history = HistoryPolicy.KEEP_LAST
            self.output_cmd_publisher = self.create_publisher(
                Twist,
                str(self.get_parameter("output_cmd_topic").value),
                command_qos,
            )
            self.follow_cmd_subscription = self.create_subscription(
                Twist,
                str(self.get_parameter("follow_cmd_topic").value),
                self.follow_command_callback,
                command_qos,
                callback_group=self.cb_group,
            )
            self.action_server = ActionServer(
                self,
                PersonFollow,
                str(self.get_parameter("action_name").value),
                execute_callback=self.execute_callback,
                goal_callback=self.goal_callback,
                cancel_callback=self.cancel_callback,
                callback_group=self.cb_group,
            )
            self.get_logger().info("[GPSR_LIFECYCLE] FollowPerson adapter configured")
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"FollowPerson configure failed: {exc}")
            self.release_resources()
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Permit exactly one GPSR follow goal while ACTIVE."""
        self.skill_active = True
        self.get_logger().info("[GPSR_LIFECYCLE] FollowPerson adapter ACTIVE")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Fail closed, reset motion, and stop accepting follow goals."""
        self.skill_active = False
        self.disable_motion()
        self.request_reset_without_waiting()
        for goal_handle in list(self.active_goal_handles):
            try:
                goal_handle.abort()
            except Exception:  # noqa: BLE001
                pass
        with self.goal_lock:
            self.goal_reserved = False
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Release all interfaces allocated during CONFIGURE."""
        self.skill_active = False
        self.disable_motion()
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Stop motion and release configured interfaces during shutdown."""
        self.skill_active = False
        self.disable_motion()
        self.request_reset_without_waiting()
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Return to a clean, non-moving baseline after lifecycle errors."""
        self.skill_active = False
        self.disable_motion()
        self.request_reset_without_waiting()
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def release_resources(self) -> None:
        """Special function: Destroy adapter action, subscription, and service clients."""
        if self.action_server is not None:
            self.action_server.destroy()
            self.action_server = None
        if self.master_subscription is not None:
            self.destroy_subscription(self.master_subscription)
            self.master_subscription = None
        if self.follow_cmd_subscription is not None:
            self.destroy_subscription(self.follow_cmd_subscription)
            self.follow_cmd_subscription = None
        if self.output_cmd_publisher is not None:
            self.destroy_publisher(self.output_cmd_publisher)
            self.output_cmd_publisher = None
        for attribute_name in ("enroll_client", "reset_client", "parameter_client"):
            client = getattr(self, attribute_name)
            if client is not None:
                self.destroy_client(client)
                setattr(self, attribute_name, None)
        self.latest_master = None
        self.latest_master_monotonic = 0.0
        self.active_goal_handles.clear()
        with self.goal_lock:
            self.goal_reserved = False

    def master_callback(self, message: MasterTarget) -> None:
        """Special function: Cache the latest master state for feedback and safety decisions."""
        self.latest_master = message
        self.latest_master_monotonic = time.monotonic()

    def follow_command_callback(self, message: Twist) -> None:
        """Special function: Forward controller velocity only during an accepted follow goal."""
        if self.motion_enabled and self.skill_active and self.output_cmd_publisher is not None:
            self.output_cmd_publisher.publish(message)

    def goal_callback(self, goal_request: PersonFollow.Goal) -> GoalResponse:
        """Special function: Reject inactive, invalid, or concurrent follow requests."""
        if not self.skill_active:
            return GoalResponse.REJECT
        if float(goal_request.follow_distance_m) == 0.0:
            pass
        elif float(goal_request.follow_distance_m) < 0.3:
            return GoalResponse.REJECT
        if float(goal_request.face_hold_sec) < 0.0:
            return GoalResponse.REJECT
        with self.goal_lock:
            if self.goal_reserved:
                return GoalResponse.REJECT
            self.goal_reserved = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle: Any) -> CancelResponse:
        """Special function: Accept cancellation so the execute callback can reset the tracker."""
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: Any) -> PersonFollow.Result:
        """Special function: Enroll, follow, report feedback, and stop on gesture or failure."""
        self.active_goal_handles.add(goal_handle)
        try:
            # Section: Refuse execution after a lifecycle transition race.
            if not self.skill_active:
                return await self.abort_result(
                    goal_handle,
                    "adapter_inactive: FollowPerson lifecycle node is not ACTIVE",
                )

            follow_distance_m = float(goal_handle.request.follow_distance_m) or float(
                self.get_parameter("default_follow_distance_m").value)
            face_hold_sec = float(goal_handle.request.face_hold_sec) or float(
                self.get_parameter("default_face_hold_sec").value)

            # Section: Validate the master stack before changing any robot state.
            clients = (self.enroll_client, self.reset_client, self.parameter_client)
            if any(client is None for client in clients):
                return await self.abort_result(
                    goal_handle,
                    "stack_unavailable: FollowPerson adapter clients are not configured",
                )
            for client in clients:
                if not client.wait_for_service(timeout_sec=2.0):
                    return await self.abort_result(
                        goal_handle,
                        f"stack_unavailable: required service {client.srv_name} is unavailable",
                    )

            # Section: Apply the requested following distance directly to the live controller.
            parameter_ok, parameter_message = await self.set_follow_distance(follow_distance_m)
            if not parameter_ok:
                return await self.abort_result(
                    goal_handle,
                    f"parameter_error: {parameter_message}",
                )

            # Section: Auto-select starts from a clean identity and repeatedly enrolls the centered person.
            if bool(goal_handle.request.auto_select_if_needed):
                await self.call_trigger(self.reset_client)
                enrolled = await self.enroll_centered_person(goal_handle)
                if not enrolled:
                    if goal_handle.is_cancel_requested:
                        return await self.cancel_result(goal_handle)
                    return await self.abort_result(
                        goal_handle,
                        "person_not_found: no confirmed person entered the enrollment window",
                    )
            elif not self.target_visible():
                return await self.abort_result(
                    goal_handle,
                    "person_not_selected: auto selection is disabled and no master is locked",
                )

            # Section: Motion forwarding begins only after the requested target is locked.
            self.motion_enabled = True

            # Section: Natural success requires real follow motion before the person stops and faces the robot.
            motion_armed = not bool(
                self.get_parameter("require_motion_before_completion").value)
            facing_since: float | None = None
            feedback_period = 1.0 / max(
                1.0,
                float(self.get_parameter("feedback_rate_hz").value),
            )
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    return await self.cancel_result(goal_handle)
                if not self.skill_active:
                    return await self.abort_result(
                        goal_handle,
                        "adapter_inactive: FollowPerson was deactivated",
                    )

                master = self.latest_master
                if not motion_armed and master_has_moved(
                    master,
                    follow_distance_m,
                    float(self.get_parameter("completion_arm_distance_delta_m").value),
                    math.radians(float(self.get_parameter("completion_arm_bearing_deg").value)),
                    float(self.get_parameter("completion_arm_speed_mps").value),
                ):
                    motion_armed = True

                facing = master_is_facing(
                    master,
                    follow_distance_m,
                    float(self.get_parameter("face_distance_margin_m").value),
                    math.radians(float(self.get_parameter("face_bearing_tolerance_deg").value)),
                    float(self.get_parameter("face_max_speed_mps").value),
                )
                now = time.monotonic()
                facing_since = now if facing and facing_since is None else facing_since
                if not facing:
                    facing_since = None

                # Section: Publish the standard action feedback expected by GPSR and CLI clients.
                self.publish_feedback(goal_handle, facing, motion_armed)

                # Section: A stale stream or prolonged LOST state is a recoverable person-lost failure.
                if self.master_stream_age() > float(
                    self.get_parameter("master_stream_timeout_sec").value
                ):
                    return await self.abort_result(
                        goal_handle,
                        "person_lost: master target stream became stale",
                    )
                if (
                    master is not None
                    and str(master.state) == STATE_LOST
                    and float(master.time_since_last_seen)
                    >= float(self.get_parameter("lost_abort_sec").value)
                ):
                    return await self.abort_result(
                        goal_handle,
                        "person_lost: master could not be reacquired",
                    )

                # Section: Stop safely when the followed person holds the completion gesture.
                if (
                    motion_armed
                    and facing_since is not None
                    and now - facing_since >= face_hold_sec
                ):
                    self.disable_motion()
                    await self.call_trigger(self.reset_client)
                    result = PersonFollow.Result()
                    result.success = True
                    result.message = "follow_complete: person stopped and faced the robot"
                    goal_handle.succeed()
                    return result
                await asyncio.sleep(feedback_period)
        finally:
            # Section: Unexpected action exits also fail closed at the velocity boundary.
            self.disable_motion()
            self.active_goal_handles.discard(goal_handle)
            with self.goal_lock:
                self.goal_reserved = False

    async def enroll_centered_person(self, goal_handle: Any) -> bool:
        """Special function: Retry immediate enrollment until a valid centered track is ready."""
        deadline = time.monotonic() + float(
            self.get_parameter("enrollment_timeout_sec").value)
        while time.monotonic() < deadline and rclpy.ok():
            if goal_handle.is_cancel_requested or not self.skill_active:
                return False
            if self.target_visible():
                return True
            response = await self.call_trigger(self.enroll_client)
            if response is not None and bool(response.success):
                await asyncio.sleep(0.1)
                if self.target_visible():
                    return True
            self.publish_feedback(goal_handle, facing=False, motion_armed=False)
            await asyncio.sleep(0.5)
        return self.target_visible()

    async def set_follow_distance(self, follow_distance_m: float) -> tuple[bool, str]:
        """Special function: Update follow_controller_node's dynamic target distance."""
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name="d_target",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=float(follow_distance_m),
                ),
            )
        ]
        try:
            response = await self.parameter_client.call_async(request)
        except Exception as exc:  # noqa: BLE001
            return False, str(exc)
        if not response.results:
            return False, "controller returned no parameter result"
        failed = [result.reason for result in response.results if not result.successful]
        return not failed, "; ".join(failed) if failed else "updated"

    async def call_trigger(self, client: Any) -> Any | None:
        """Special function: Invoke a Trigger client and convert ROS errors into a null result."""
        if client is None:
            return None
        try:
            return await client.call_async(Trigger.Request())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Trigger call failed: {exc}")
            return None

    def request_reset_without_waiting(self) -> None:
        """Special function: Request a best-effort tracker reset from synchronous lifecycle hooks."""
        if self.reset_client is None:
            return
        try:
            self.reset_client.call_async(Trigger.Request())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Best-effort reset failed: {exc}")

    def disable_motion(self) -> None:
        """Special function: Close the velocity gate and publish one explicit base stop."""
        was_enabled = self.motion_enabled
        self.motion_enabled = False
        if was_enabled and self.output_cmd_publisher is not None:
            self.output_cmd_publisher.publish(Twist())

    def target_visible(self) -> bool:
        """Special function: Return true only for a fresh locked FOLLOWING master."""
        master = self.latest_master
        return bool(
            master is not None
            and bool(master.is_locked)
            and str(master.state) == STATE_FOLLOWING
            and self.master_stream_age()
            <= float(self.get_parameter("master_stream_timeout_sec").value)
        )

    def master_stream_age(self) -> float:
        """Special function: Measure wall-clock age of the last master status message."""
        if self.latest_master_monotonic <= 0.0:
            return math.inf
        return max(0.0, time.monotonic() - self.latest_master_monotonic)

    def publish_feedback(
        self,
        goal_handle: Any,
        facing: bool,
        motion_armed: bool,
    ) -> None:
        """Special function: Publish PersonFollow feedback derived from master tracker state."""
        master = self.latest_master
        feedback = PersonFollow.Feedback()
        feedback.active = bool(self.skill_active and self.motion_enabled)
        feedback.selected_track_id = int(master.track_id) if master is not None else -1
        feedback.target_visible = self.target_visible()
        feedback.target_age_sec = float(
            max(
                self.master_stream_age(),
                float(master.time_since_last_seen) if master is not None else 0.0,
            )
        )
        feedback.facing_robot = bool(facing)
        if not feedback.target_visible:
            feedback.stop_reason = "waiting_for_target"
        elif not motion_armed:
            feedback.stop_reason = "waiting_for_follow_motion"
        elif facing:
            feedback.stop_reason = "completion_gesture_detected"
        else:
            feedback.stop_reason = "following"
        goal_handle.publish_feedback(feedback)

    async def cancel_result(self, goal_handle: Any) -> PersonFollow.Result:
        """Special function: Reset motion and return a proper canceled action result."""
        self.disable_motion()
        await self.call_trigger(self.reset_client)
        result = PersonFollow.Result()
        result.success = False
        result.message = "canceled: follow goal canceled by GPSR"
        try:
            goal_handle.canceled()
        except Exception:  # noqa: BLE001
            pass
        return result

    async def abort_result(self, goal_handle: Any, message: str) -> PersonFollow.Result:
        """Special function: Reset motion and abort a failed action with a classified message."""
        self.disable_motion()
        await self.call_trigger(self.reset_client)
        result = PersonFollow.Result()
        result.success = False
        result.message = str(message)
        try:
            goal_handle.abort()
        except Exception:  # noqa: BLE001
            pass
        return result


def main(args: list[str] | None = None) -> None:
    """Special function: Run the lifecycle-managed FollowPerson adapter."""
    rclpy.init(args=args)
    node = FollowPersonSkillServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
