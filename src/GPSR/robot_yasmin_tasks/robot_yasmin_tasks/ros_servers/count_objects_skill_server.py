#!/usr/bin/env python3
"""Section: Lifecycle-managed GPSR count-object action server."""

from __future__ import annotations

import json

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

from robot_ai_interfaces.action import CountObjects
from robot_ai_interfaces.srv import CountObjects as CountObjectsSrv


class CountObjectsSkillServer(LifecycleNode):
    """Special function: Expose object counting only while lifecycle ACTIVE."""

    def __init__(self) -> None:
        """Special function: Declare configuration without allocating task interfaces."""
        super().__init__("count_objects_skill_server")

        # Section: Declare stable ROS interface names and timeout policy.
        self.cb_group = ReentrantCallbackGroup()
        self.declare_parameter("action_name", "/gpsr/count_objects")
        self.declare_parameter("perception_service", "/perception/count_objects")
        self.declare_parameter("default_timeout_sec", 5.0)
        self.action_name = str(self.get_parameter("action_name").value)
        self.service_name = str(self.get_parameter("perception_service").value)

        # Section: Resource handles remain empty until CONFIGURE.
        self.count_client = None
        self.action_server = None
        self.active_goal_handles: set[object] = set()
        self.skill_active = False
        self.get_logger().info("CountObjects lifecycle server is UNCONFIGURED.")

    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Create the perception client and GPSR action server."""
        try:
            # Section: Allocate interfaces without allowing normal goals while INACTIVE.
            self.count_client = self.create_client(
                CountObjectsSrv,
                self.service_name,
                callback_group=self.cb_group,
            )
            self.action_server = ActionServer(
                self,
                CountObjects,
                self.action_name,
                execute_callback=self.execute_callback,
                goal_callback=self.goal_callback,
                cancel_callback=self.cancel_callback,
                callback_group=self.cb_group,
            )
            self.get_logger().info("[GPSR_LIFECYCLE] CountObjects: configured")
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.get_logger().error(f"CountObjects configure failed: {exc}")
            self.release_resources()
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Permit GPSR action goals while ACTIVE."""
        self.skill_active = True
        self.get_logger().info("[GPSR_LIFECYCLE] CountObjects: ACTIVE")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Stop accepting work and abort unexpected in-flight goals."""
        self.skill_active = False
        for goal_handle in list(self.active_goal_handles):
            try:
                goal_handle.abort()
            except Exception:
                pass
        self.active_goal_handles.clear()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Release every resource allocated by CONFIGURE."""
        self.skill_active = False
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Release configured resources during shutdown."""
        self.skill_active = False
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Fail closed and reset to a clean resource baseline."""
        self.skill_active = False
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def release_resources(self) -> None:
        """Special function: Destroy the action server and perception client."""
        if self.action_server is not None:
            self.action_server.destroy()
            self.action_server = None
        if self.count_client is not None:
            self.destroy_client(self.count_client)
            self.count_client = None
        self.active_goal_handles.clear()

    def goal_callback(self, goal_request: CountObjects.Goal) -> GoalResponse:
        """Special function: Reject inactive or invalid GPSR goals."""
        if not self.skill_active:
            self.get_logger().warn("Rejected CountObjects goal while not ACTIVE.")
            return GoalResponse.REJECT
        if float(goal_request.timeout_sec) < 0.0:
            self.get_logger().warn("Rejected CountObjects goal: negative timeout.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle: object) -> CancelResponse:
        """Special function: Allow the executor to cancel a pending action."""
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: object) -> CountObjects.Result:
        """Special function: Return only the final perception response as action result."""
        goal = goal_handle.request
        result = CountObjects.Result()
        self.active_goal_handles.add(goal_handle)
        try:
            # Section: Explicit ACTIVE gating protects against transition races.
            if not self.skill_active or self.count_client is None:
                return self.abort_result(goal_handle, result, "CountObjects lifecycle node is not ACTIVE.")

            # Section: Feedback is informational and never advances the ActionQueue.
            feedback = CountObjects.Feedback()
            feedback.status = f"Counting {self.clean_text(goal.category)} at {self.clean_text(goal.location) or 'current view'}"
            goal_handle.publish_feedback(feedback)

            # Section: Wait for the configured perception service within the goal timeout.
            timeout_sec = float(goal.timeout_sec) or float(
                self.get_parameter("default_timeout_sec").value
            )
            if not self.count_client.wait_for_service(timeout_sec=timeout_sec):
                return self.abort_result(
                    goal_handle,
                    result,
                    "Perception count_objects service is unavailable.",
                )

            # Section: Forward normalized goal fields to the existing service interface.
            request = CountObjectsSrv.Request()
            request.category = self.clean_text(goal.category)
            request.location = self.clean_text(goal.location)
            request.filter_json = self.normalize_filter_json(goal.filter_json)
            response = await self.count_client.call_async(request)

            # Section: Cancellation and deactivation are terminal action outcomes.
            if goal_handle.is_cancel_requested:
                result.success = False
                result.count = 0
                result.result_json = "{}"
                result.message = "CountObjects goal was cancelled."
                goal_handle.canceled()
                return result
            if not self.skill_active:
                return self.abort_result(
                    goal_handle,
                    result,
                    "CountObjects deactivated before its final result.",
                )

            # Section: The final service response authoritatively determines action status.
            result.success = bool(response.success)
            result.count = int(response.count)
            result.result_json = str(response.result_json)
            result.message = str(response.message)
            if response.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result
        finally:
            self.active_goal_handles.discard(goal_handle)

    def abort_result(
        self,
        goal_handle: object,
        result: CountObjects.Result,
        message: str,
    ) -> CountObjects.Result:
        """Special function: Fill and safely abort a failed action result."""
        result.success = False
        result.count = 0
        result.result_json = "{}"
        result.message = str(message)
        try:
            goal_handle.abort()
        except Exception:
            pass
        return result

    def normalize_filter_json(self, raw_filter_json: str) -> str:
        """Special function: Guarantee valid JSON filter text for the service request."""
        text = str(raw_filter_json).strip()
        if not text:
            return "{}"
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid filter_json replaced with empty object: {text}")
            return "{}"
        return json.dumps(
            parsed if isinstance(parsed, dict) else {"value": parsed},
            sort_keys=True,
        )

    @staticmethod
    def clean_text(value: object) -> str:
        """Special function: Normalize empty ROS text values."""
        text = str(value).strip()
        return "" if text.lower() in {"", "none", "null"} else text


def main(args: list[str] | None = None) -> None:
    """Special function: Run the lifecycle-managed CountObjects action server."""
    rclpy.init(args=args)
    node = CountObjectsSkillServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
