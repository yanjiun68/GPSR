#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose


def _yaw_from_quat_xyzw(x: float, y: float, z: float, w: float) -> float:
    # yaw (Z) from quaternion (x,y,z,w)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    # returns (x,y,z,w)
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class PersonFollowNav2Goal(Node):
    def __init__(self):
        super().__init__('person_follow_nav2_goal')

        # ---------------- Parameters ----------------
        self.declare_parameter('target_topic', '/person_target_locked')
        self.declare_parameter('nav2_action_name', '/navigate_to_pose')
        self.declare_parameter('global_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('cmd_vel_raw_topic', '/cmd_vel')

        self.declare_parameter('follow_distance_m', 0.7)
        self.declare_parameter('target_timeout_sec', 0.6)
        self.declare_parameter('max_goal_distance_m', 2.5)
        self.declare_parameter('goal_update_hz', 2.0)
        self.declare_parameter('min_goal_step_m', 0.25)
        self.declare_parameter('goal_resend_sec', 2.0)
        self.declare_parameter('publish_zero_on_lost', True)

        self.declare_parameter('stop_distance_m', 0.45)      # stop if closer than this
        self.declare_parameter('deadband_m', 0.10)          # hysteresis to prevent flicker

        self.stop_distance_m = float(self.get_parameter('stop_distance_m').value)
        self.deadband_m = float(self.get_parameter('deadband_m').value)

        # ---------------- Read params ----------------
        self.target_topic = str(self.get_parameter('target_topic').value)
        self.nav2_action_name = str(self.get_parameter('nav2_action_name').value)
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.cmd_vel_raw_topic = str(self.get_parameter('cmd_vel_raw_topic').value)

        self.follow_distance_m = float(self.get_parameter('follow_distance_m').value)
        self.target_timeout_sec = float(self.get_parameter('target_timeout_sec').value)
        self.max_goal_distance_m = float(self.get_parameter('max_goal_distance_m').value)
        self.goal_update_hz = float(self.get_parameter('goal_update_hz').value)
        self.min_goal_step_m = float(self.get_parameter('min_goal_step_m').value)
        self.goal_resend_sec = float(self.get_parameter('goal_resend_sec').value)
        self.publish_zero_on_lost = bool(self.get_parameter('publish_zero_on_lost').value)

        self.goal_period_sec = 1.0 / max(0.1, self.goal_update_hz)

        # ---------------- TF ----------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- Nav2 action client ----------------
        self.nav2_client = ActionClient(self, NavigateToPose, self.nav2_action_name)
        self.goal_handle: Optional[rclpy.action.client.ClientGoalHandle] = None

        # ---------------- ROS I/O ----------------
        self.sub_target = self.create_subscription(PoseStamped, self.target_topic, self.target_cb, 10)
        self.pub_cmd_vel_raw = self.create_publisher(Twist, self.cmd_vel_raw_topic, 10)

        self.timer = self.create_timer(self.goal_period_sec, self.timer_cb)

        # ---------------- State ----------------
        self.latest_target: Optional[PoseStamped] = None
        self.last_target_rx_sec: float = 0.0

        self.last_goal_sent_xy: Optional[Tuple[float, float]] = None
        self.last_goal_send_sec: float = 0.0

        self._lost_state = True
        self._sent_zero_on_lost = False
        self._last_server_warn_sec = 0.0

        self.get_logger().info('[PersonFollowNav2Goal] Started')
        self.get_logger().info(f'  target_topic     : {self.target_topic}')
        self.get_logger().info(f'  nav2_action_name : {self.nav2_action_name}')
        self.get_logger().info(f'  frames           : {self.global_frame} -> {self.base_frame}')
        self.get_logger().info(f'  cmd_vel_raw      : {self.cmd_vel_raw_topic}')

    # ---------------- callbacks ----------------
    def target_cb(self, msg: PoseStamped):
        self.latest_target = msg
        self.last_target_rx_sec = self._now_sec()

    # ---------------- main loop ----------------
    def timer_cb(self):
        now = self._now_sec()
        has_fresh_target = (self.latest_target is not None) and ((now - self.last_target_rx_sec) <= self.target_timeout_sec)

        if not has_fresh_target:
            self._handle_lost_target(now)
            return

        self._lost_state = False
        self._sent_zero_on_lost = False

        msg = self.latest_target
        if msg is None:
            self._handle_lost_target(now)
            return

        px = float(msg.pose.position.x)
        py = float(msg.pose.position.y)

        # ignore invalid / behind
        if not math.isfinite(px) or not math.isfinite(py) or px <= 0.05:
            self._handle_lost_target(now)
            return

        d = float(math.hypot(px, py))
        if d < 1e-6:
            self._handle_lost_target(now)
            return

        # --- distance control with hysteresis ---
        # stop zone (very close)
        if d <= self.stop_distance_m:
            self._cancel_goal_if_any()
            if self.publish_zero_on_lost:
                self._publish_zero_cmd_vel()
            return

        # deadband zone: close enough -> don't keep sending/canceling goals
        # (prevents goal "chatter")
        if d <= (self.follow_distance_m + self.deadband_m):
            # optional: ensure any running goal is canceled so robot doesn't creep
            self._cancel_goal_if_any()
            return

        # compute stand-off goal in base frame
        ux = px / d
        uy = py / d
        goal_bx = px - self.follow_distance_m * ux
        goal_by = py - self.follow_distance_m * uy

        # clamp max distance
        gbd = float(math.hypot(goal_bx, goal_by))
        if gbd > self.max_goal_distance_m and gbd > 1e-6:
            s = self.max_goal_distance_m / gbd
            goal_bx *= s
            goal_by *= s

        goal_map = self._goal_in_map(goal_bx, goal_by)
        if goal_map is None:
            self._handle_lost_target(now)
            return

        gx, gy, yaw_map = goal_map

        if not self._should_send_goal(now, gx, gy):
            return

        if not self.nav2_client.wait_for_server(timeout_sec=0.1):
            self._warn_nav2_server(now)
            return

        self._send_goal(gx, gy, yaw_map, now)

    # ---------------- helpers ----------------
    def _warn_nav2_server(self, now_sec: float):
        # throttle to ~2s
        if (now_sec - self._last_server_warn_sec) < 2.0:
            return
        self._last_server_warn_sec = now_sec
        self.get_logger().warn(f'[PersonFollowNav2Goal] Nav2 action server not available: {self.nav2_action_name}')

    def _handle_lost_target(self, now_sec: float):
        if not self._lost_state:
            self.get_logger().info('[PersonFollowNav2Goal] Target lost -> cancel goal')
        self._lost_state = True

        self._cancel_goal_if_any()

        if self.publish_zero_on_lost and not self._sent_zero_on_lost:
            self._publish_zero_cmd_vel()
            self._sent_zero_on_lost = True

    def _publish_zero_cmd_vel(self):
        t = Twist()
        self.pub_cmd_vel_raw.publish(t)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _cancel_goal_if_any(self):
        gh = self.goal_handle
        if gh is None:
            return
        try:
            gh.cancel_goal_async()
        except Exception as e:
            self.get_logger().warn(f'[PersonFollowNav2Goal] cancel_goal_async failed: {e}')
        self.goal_handle = None

    def _should_send_goal(self, now_sec: float, gx: float, gy: float) -> bool:
        if self.last_goal_sent_xy is None:
            return True

        dt = now_sec - self.last_goal_send_sec
        if dt >= self.goal_resend_sec:
            return True

        dx = gx - self.last_goal_sent_xy[0]
        dy = gy - self.last_goal_sent_xy[1]
        return float(math.hypot(dx, dy)) >= self.min_goal_step_m

    def _goal_in_map(self, goal_bx: float, goal_by: float) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, Time())
        except Exception as e:
            self.get_logger().warn(f'[PersonFollowNav2Goal] TF lookup failed ({self.global_frame}<-{self.base_frame}): {e}')
            return None

        tx = float(tf.transform.translation.x)
        ty = float(tf.transform.translation.y)
        q = tf.transform.rotation
        yaw = _yaw_from_quat_xyzw(float(q.x), float(q.y), float(q.z), float(q.w))

        cy = math.cos(yaw)
        sy = math.sin(yaw)
        gx = tx + cy * goal_bx - sy * goal_by
        gy = ty + sy * goal_bx + cy * goal_by
        return (gx, gy, yaw)

    def _send_goal(self, gx: float, gy: float, yaw_map: float, now_sec: float):
        # cancel previous goal before sending a new one
        self._cancel_goal_if_any()

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        goal.pose.pose.position.z = 0.0

        qx, qy, qz, qw = _quat_from_yaw(yaw_map)
        goal.pose.pose.orientation.x = float(qx)
        goal.pose.pose.orientation.y = float(qy)
        goal.pose.pose.orientation.z = float(qz)
        goal.pose.pose.orientation.w = float(qw)

        send_future = self.nav2_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

        self.last_goal_sent_xy = (float(gx), float(gy))
        self.last_goal_send_sec = float(now_sec)

    def _on_goal_response(self, future):
        try:
            gh = future.result()
        except Exception as e:
            self.get_logger().warn(f'[PersonFollowNav2Goal] send_goal_async failed: {e}')
            return
        if not gh.accepted:
            self.get_logger().warn('[PersonFollowNav2Goal] Nav2 goal rejected')
            return
        self.goal_handle = gh


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowNav2Goal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

