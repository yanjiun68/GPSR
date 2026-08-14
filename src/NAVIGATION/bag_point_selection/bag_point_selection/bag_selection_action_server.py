#!/usr/bin/env python3
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from putrabot_interfaces.action import SelectBag


class BagSelectionActionServer(Node):
    def __init__(self):
        super().__init__("bag_selection_action_server")
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter("bag_selection_action_name", "/bag_selection/select")
        self.declare_parameter("bag_selection_default_timeout_sec", 30.0)
        self.declare_parameter("bag_selection_feedback_heartbeat_sec", 0.5)

        self.action_name = str(self.get_parameter("bag_selection_action_name").value)
        self.default_timeout_sec = max(0.1, float(self.get_parameter("bag_selection_default_timeout_sec").value))
        self.feedback_heartbeat_sec = max(0.1, float(self.get_parameter("bag_selection_feedback_heartbeat_sec").value))

        self.current_state = "IDLE"
        self.selected_side = ""
        self.motion_status = "idle"
        self.bag_ready = False
        self.failure_reason = ""
        self.latest_bag_point: Optional[PointStamped] = None
        self.latest_front_goal: Optional[PoseStamped] = None
        self._active_goal_handle = None

        self.create_subscription(
            String, "/selection_state", self.state_callback, 10, callback_group=self.cb_group
        )
        self.create_subscription(
            String, "/selected_side", self.side_callback, 10, callback_group=self.cb_group
        )
        self.create_subscription(
            PointStamped, "/selected_bag_fused_center", self.bag_point_callback, 10, callback_group=self.cb_group
        )
        self.create_subscription(
            String, "/bag_motion_status", self.motion_status_callback, 10, callback_group=self.cb_group
        )
        self.create_subscription(
            Bool, "/bag_ready_for_pickup", self.ready_callback, 10, callback_group=self.cb_group
        )
        self.create_subscription(
            String, "/bag_selection/failure_reason", self.failure_callback, 10, callback_group=self.cb_group
        )

        self.start_pub = self.create_publisher(Bool, "/bag_selection/start", 10)
        self.reset_pub = self.create_publisher(Bool, "/bag_selection/reset", 10)
        self.announce_enabled_pub = self.create_publisher(Bool, "/bag_selection/announce_selection_enabled", 10)

        self.action_server = ActionServer(
            self,
            SelectBag,
            self.action_name,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=self.cb_group,
        )

    def state_callback(self, msg: String) -> None:
        self.current_state = msg.data

    def side_callback(self, msg: String) -> None:
        self.selected_side = msg.data

    def bag_point_callback(self, msg: PointStamped) -> None:
        self.latest_bag_point = msg

    def motion_status_callback(self, msg: String) -> None:
        self.motion_status = msg.data

    def ready_callback(self, msg: Bool) -> None:
        self.bag_ready = bool(msg.data)

    def failure_callback(self, msg: String) -> None:
        self.failure_reason = msg.data.strip()

    def goal_callback(self, goal_request: SelectBag.Goal):
        del goal_request
        if self._active_goal_handle is not None:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        del goal_handle
        return CancelResponse.ACCEPT

    def clear_observed_state(self) -> None:
        self.current_state = "IDLE"
        self.selected_side = ""
        self.motion_status = "idle"
        self.bag_ready = False
        self.failure_reason = ""
        self.latest_bag_point = None
        self.latest_front_goal = None

    def publish_bool(self, publisher, value: bool) -> None:
        publisher.publish(Bool(data=bool(value)))

    def publish_session_reset(self) -> None:
        self.publish_bool(self.reset_pub, True)

    def publish_session_start(self) -> None:
        self.publish_bool(self.start_pub, True)

    def configure_session(self, announce_selection: bool) -> None:
        self.publish_bool(self.announce_enabled_pub, announce_selection)

    def feedback_detail_message(self) -> str:
        if self.failure_reason:
            return self.failure_reason
        mapping = {
            "IDLE": "idle",
            "WAIT_FOR_TWO_BAGS": "waiting for exactly two bag detections",
            "WAIT_FOR_SELECTION": "waiting for stable pointing selection",
            "SELECT_BAG": "selecting confirmed bag",
            "EXTRACT_CAMERA_CLUSTER": "waiting for camera cluster",
            "MATCH_OR_FUSE_TARGET": "waiting for fused target",
            "APPROACH_TARGET": "approaching selected bag",
            "FINAL_ALIGN": "running final visual alignment",
            "READY_FOR_PICKUP": "ready for pickup",
        }
        if self.current_state in ("APPROACH_TARGET", "FINAL_ALIGN") and self.motion_status not in ("", "idle"):
            return self.motion_status
        return mapping.get(self.current_state, self.current_state.lower())

    def build_feedback(self) -> SelectBag.Feedback:
        feedback = SelectBag.Feedback()
        feedback.current_state = str(self.current_state)
        feedback.selected_side = str(self.selected_side)
        feedback.nav_status = str(self.motion_status)
        feedback.bag_ready = bool(self.bag_ready)
        feedback.detail_message = self.feedback_detail_message()
        return feedback

    def build_result(self, success: bool, message: str) -> SelectBag.Result:
        result = SelectBag.Result()
        result.success = bool(success)
        result.selected_side = str(self.selected_side)
        result.final_state = str(self.current_state)
        result.message = str(message)
        result.has_bag_point = self.latest_bag_point is not None
        if self.latest_bag_point is not None:
            result.bag_point = self.latest_bag_point
        result.has_front_goal = False
        if self.latest_front_goal is not None:
            result.front_goal = self.latest_front_goal
        return result

    def execute_callback(self, goal_handle):
        self._active_goal_handle = goal_handle
        goal = goal_handle.request
        timeout_sec = float(goal.timeout_sec) if float(goal.timeout_sec) > 0.0 else self.default_timeout_sec
        navigate_to_bag = bool(goal.navigate_to_bag)
        announce_selection = bool(goal.announce_selection)
        start_sec = time.monotonic()
        last_feedback_sec = 0.0
        last_feedback_key = None

        self.publish_session_reset()
        time.sleep(0.05)
        self.clear_observed_state()
        self.configure_session(announce_selection)
        self.publish_session_start()

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.publish_session_reset()
                    goal_handle.canceled()
                    return self.build_result(False, "canceled by client")

                if self.failure_reason:
                    self.publish_session_reset()
                    goal_handle.abort()
                    return self.build_result(False, self.failure_reason)

                if navigate_to_bag:
                    if self.current_state == "READY_FOR_PICKUP" and self.bag_ready:
                        self.publish_session_reset()
                        goal_handle.succeed()
                        return self.build_result(True, "bag selection and local approach completed")
                else:
                    if (
                        self.current_state in ("APPROACH_TARGET", "FINAL_ALIGN", "READY_FOR_PICKUP")
                        and self.latest_bag_point is not None
                    ):
                        self.publish_session_reset()
                        goal_handle.succeed()
                        return self.build_result(True, "bag selected and fused target acquired")

                if (time.monotonic() - start_sec) >= timeout_sec:
                    self.publish_session_reset()
                    goal_handle.abort()
                    message = self.failure_reason or "select bag action timed out"
                    return self.build_result(False, message)

                feedback = self.build_feedback()
                feedback_key = (
                    feedback.current_state,
                    feedback.selected_side,
                    feedback.nav_status,
                    feedback.bag_ready,
                    feedback.detail_message,
                )
                now_sec = time.monotonic()
                state_changed = feedback_key != last_feedback_key
                heartbeat_due = (now_sec - last_feedback_sec) >= self.feedback_heartbeat_sec
                if state_changed or heartbeat_due:
                    goal_handle.publish_feedback(feedback)
                    last_feedback_sec = now_sec
                    last_feedback_key = feedback_key

                time.sleep(0.05)
        finally:
            self._active_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = BagSelectionActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
