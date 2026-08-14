#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool
from putrabot_interfaces.action import AnnounceFollowReady
from putrabot_interfaces.msg import SafetyStatus, TrackedPose
from voice_interfaces.srv import SayText


class FollowReadyVoiceServer(Node):
    def __init__(self):
        super().__init__('follow_ready_voice_server')
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('session_active_topic', '/person_follow/session_active')
        self.declare_parameter('safety_status_topic', '/person_follow/safety_status')
        self.declare_parameter('target_topic', '/person_target_locked')
        self.declare_parameter('action_name', '/person_follow/announce_ready')
        self.declare_parameter('default_text', 'I am able to follow you')
        self.declare_parameter('default_timeout_sec', 10.0)
        self.declare_parameter('wait_poll_sec', 0.1)
        self.declare_parameter('feedback_heartbeat_sec', 0.5)
        self.declare_parameter('tts_service_name', '/voice/tts/say')
        self.declare_parameter('tts_service_timeout_sec', 5.0)

        self.session_active_topic = str(self.get_parameter('session_active_topic').value)
        self.safety_status_topic = str(self.get_parameter('safety_status_topic').value)
        self.target_topic = str(self.get_parameter('target_topic').value)
        self.action_name = str(self.get_parameter('action_name').value)
        self.default_text = str(self.get_parameter('default_text').value)
        self.default_timeout_sec = max(0.1, float(self.get_parameter('default_timeout_sec').value))
        self.wait_poll_sec = max(0.02, float(self.get_parameter('wait_poll_sec').value))
        self.feedback_heartbeat_sec = max(0.1, float(self.get_parameter('feedback_heartbeat_sec').value))
        self.tts_service_name = str(self.get_parameter('tts_service_name').value)
        self.tts_service_timeout_sec = max(0.1, float(self.get_parameter('tts_service_timeout_sec').value))

        self.latest_session_active = False
        self.latest_safety: Optional[SafetyStatus] = None
        self.latest_target: Optional[TrackedPose] = None
        self._active_goal_handle = None
        self.tts_client = self.create_client(SayText, self.tts_service_name, callback_group=self.cb_group)

        self.create_subscription(Bool, self.session_active_topic, self._session_cb, 10, callback_group=self.cb_group)
        self.create_subscription(SafetyStatus, self.safety_status_topic, self._safety_cb, 10, callback_group=self.cb_group)
        self.create_subscription(TrackedPose, self.target_topic, self._target_cb, 10, callback_group=self.cb_group)

        self.action_server = ActionServer(
            self,
            AnnounceFollowReady,
            self.action_name,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info('[FollowReadyVoiceServer] Started')
        self.get_logger().info(f'  action_name         : {self.action_name}')
        self.get_logger().info(f'  session_active_topic: {self.session_active_topic}')
        self.get_logger().info(f'  safety_status_topic : {self.safety_status_topic}')
        self.get_logger().info(f'  target_topic        : {self.target_topic}')
        self.get_logger().info(f'  tts_service_name    : {self.tts_service_name}')

    def _session_cb(self, msg: Bool):
        self.latest_session_active = bool(msg.data)

    def _safety_cb(self, msg: SafetyStatus):
        self.latest_safety = msg

    def _target_cb(self, msg: TrackedPose):
        self.latest_target = msg

    def goal_callback(self, goal_request: AnnounceFollowReady.Goal):
        del goal_request
        if self._active_goal_handle is not None:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        del goal_handle
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self._active_goal_handle = goal_handle
        goal = goal_handle.request
        text = str(goal.text).strip() or self.default_text
        timeout_sec = float(goal.timeout_sec) if float(goal.timeout_sec) > 0.0 else self.default_timeout_sec
        start_sec = self._now_sec()
        last_feedback_sec = 0.0
        last_feedback_key = None

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._result(False, 'canceled')

                session_active, safety_ok, target_locked = self._readiness_state()
                waiting_on = self._waiting_on(session_active, safety_ok, target_locked)
                feedback_key = (session_active, safety_ok, target_locked, False, waiting_on)
                now_sec = self._now_sec()
                state_changed = feedback_key != last_feedback_key
                heartbeat_due = (now_sec - last_feedback_sec) >= self.feedback_heartbeat_sec

                if state_changed or heartbeat_due:
                    goal_handle.publish_feedback(
                        self._feedback(
                            session_active=session_active,
                            safety_ok=safety_ok,
                            target_locked=target_locked,
                            speaking=False,
                            waiting_on=waiting_on,
                        )
                    )
                    last_feedback_sec = now_sec
                    last_feedback_key = feedback_key

                if session_active and safety_ok and target_locked:
                    goal_handle.publish_feedback(
                        self._feedback(
                            session_active=True,
                            safety_ok=True,
                            target_locked=True,
                            speaking=True,
                            waiting_on='',
                        )
                    )
                    ok, message = self._speak(text)
                    if ok:
                        goal_handle.succeed()
                        return self._result(True, message)
                    goal_handle.abort()
                    return self._result(False, message)

                if (now_sec - start_sec) >= timeout_sec:
                    goal_handle.abort()
                    return self._result(False, f'timed out waiting for readiness: {waiting_on}')

                time.sleep(self.wait_poll_sec)
        finally:
            self._active_goal_handle = None

    def _readiness_state(self) -> tuple[bool, bool, bool]:
        session_active = bool(self.latest_session_active)
        safety_ok = False
        if self.latest_safety is not None:
            safety_ok = (
                int(self.latest_safety.health_state) == int(SafetyStatus.HEALTH_OK)
                and not bool(self.latest_safety.inhibit_active)
            )

        target_locked = False
        if self.latest_target is not None:
            target_locked = (
                bool(self.latest_target.selected)
                and bool(self.latest_target.valid)
                and int(self.latest_target.track_id) > 0
            )
        return session_active, safety_ok, target_locked

    @staticmethod
    def _waiting_on(session_active: bool, safety_ok: bool, target_locked: bool) -> str:
        if not session_active:
            return 'session_active'
        if not safety_ok:
            return 'safety_ok'
        if not target_locked:
            return 'target_locked'
        return ''

    def _speak(self, text: str) -> tuple[bool, str]:
        self.get_logger().info(f'[FollowReadyVoiceServer] Speaking: "{text}"')
        if not text.strip():
            return False, 'empty text'
        if not self.tts_client.wait_for_service(timeout_sec=self.tts_service_timeout_sec):
            return False, f'tts service unavailable: {self.tts_service_name}'

        request = SayText.Request()
        request.text = text
        future = self.tts_client.call_async(request)
        deadline = time.monotonic() + self.tts_service_timeout_sec

        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(self.wait_poll_sec)

        if not future.done():
            return False, f'tts service call timed out: {self.tts_service_name}'

        try:
            response = future.result()
        except Exception as exc:
            return False, f'tts service call failed: {exc}'

        if response is None:
            return False, 'tts service returned no response'
        if not response.success:
            return False, response.message or 'tts service returned failure'
        return True, response.message or 'announcement completed'

    @staticmethod
    def _feedback(
        session_active: bool,
        safety_ok: bool,
        target_locked: bool,
        speaking: bool,
        waiting_on: str,
    ) -> AnnounceFollowReady.Feedback:
        feedback = AnnounceFollowReady.Feedback()
        feedback.session_active = bool(session_active)
        feedback.safety_ok = bool(safety_ok)
        feedback.target_locked = bool(target_locked)
        feedback.speaking = bool(speaking)
        feedback.waiting_on = str(waiting_on)
        return feedback

    @staticmethod
    def _result(success: bool, message: str) -> AnnounceFollowReady.Result:
        result = AnnounceFollowReady.Result()
        result.success = bool(success)
        result.message = str(message)
        return result

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = FollowReadyVoiceServer()
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


if __name__ == '__main__':
    main()
