#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String as StringMsg

from putrabot_interfaces.msg import BehaviorStatus, SafetyStatus, TrackedPose, TrackedPoseArray
from person_following.utils.follow_mode import MODE_MAPPED, MODE_UNMAPPED, normalize_mode


class SafetySupervisor(Node):
    STOP_REASON_LABELS = {
        SafetyStatus.STOP_NONE: '',
        SafetyStatus.STOP_DISABLED_BY_SERVER: 'disabled_by_server',
        SafetyStatus.STOP_TRACKER_HEARTBEAT_TIMEOUT: 'tracker_heartbeat_timeout',
        SafetyStatus.STOP_TF_UNAVAILABLE_TIMEOUT: 'tf_unavailable_timeout',
        SafetyStatus.STOP_NAV2_PENDING: 'nav2_pending',
        SafetyStatus.STOP_NAV2_UNAVAILABLE: 'nav2_unavailable',
        SafetyStatus.STOP_OBSTACLE_INHIBIT: 'obstacle_inhibit',
        SafetyStatus.STOP_OBSTACLE_INHIBIT_TIMEOUT: 'obstacle_inhibit_timeout',
        SafetyStatus.STOP_WAITING_FOR_TARGET: 'waiting_for_target',
        SafetyStatus.STOP_SELECTED_TARGET_PENDING: 'selected_target_pending',
        SafetyStatus.STOP_SELECTED_TARGET_STALE_TIMEOUT: 'selected_target_stale_timeout',
        SafetyStatus.STOP_BEHAVIOR_ABORTED: 'behavior_aborted',
    }
    STOP_SOURCE_LABELS = {
        SafetyStatus.SOURCE_NONE: '',
        SafetyStatus.SOURCE_SERVER: 'server',
        SafetyStatus.SOURCE_TRACKER: 'tracker',
        SafetyStatus.SOURCE_TF: 'tf',
        SafetyStatus.SOURCE_NAVIGATION: 'navigation',
        SafetyStatus.SOURCE_SAFETY: 'safety',
        SafetyStatus.SOURCE_TARGET_LOCK: 'target_lock',
        SafetyStatus.SOURCE_BEHAVIOR: 'behavior',
    }

    def __init__(self):
        super().__init__('safety_supervisor')

        self.declare_parameter('follow_enable_topic', '/person_follow/enable')
        self.declare_parameter('follow_mode_topic', '/person_follow/mode')
        self.declare_parameter('tracks_state_topic', '/person_tracks_state')
        self.declare_parameter('target_topic', '/person_target_locked')
        self.declare_parameter('behavior_status_topic', '/person_follow/behavior_status')
        self.declare_parameter('safety_status_topic', '/person_follow/safety_status')
        self.declare_parameter('stop_reason_topic', '/person_follow/stop_reason')
        self.declare_parameter('nav_available_topic', '/person_follow/nav2_available')
        self.declare_parameter('tf_ok_topic', '/person_follow/tf_ok')
        self.declare_parameter('obstacle_inhibit_topic', '/person_follow/obstacle_inhibit')
        self.declare_parameter('local_sensing_ok_topic', '/person_follow/local_sensing_ok')
        self.declare_parameter('nav_backend_status_topic', '/person_follow/nav_backend_status')
        self.declare_parameter('local_backend_status_topic', '/person_follow/local_backend_status')
        self.declare_parameter('track_frame', 'base_footprint')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('tracker_timeout_sec', 1.0)
        self.declare_parameter('target_soft_timeout_sec', 1.2)
        self.declare_parameter('target_hard_timeout_sec', 2.5)
        self.declare_parameter('tf_timeout_sec', 0.75)
        self.declare_parameter('nav_timeout_sec', 1.5)
        self.declare_parameter('obstacle_fault_timeout_sec', 3.0)
        self.declare_parameter('local_sensing_timeout_sec', 1.0)
        self.declare_parameter('default_follow_mode', MODE_MAPPED)

        self.follow_enable_topic = str(self.get_parameter('follow_enable_topic').value)
        self.follow_mode_topic = str(self.get_parameter('follow_mode_topic').value)
        self.tracks_state_topic = str(self.get_parameter('tracks_state_topic').value)
        self.target_topic = str(self.get_parameter('target_topic').value)
        self.behavior_status_topic = str(self.get_parameter('behavior_status_topic').value)
        self.safety_status_topic = str(self.get_parameter('safety_status_topic').value)
        self.stop_reason_topic = str(self.get_parameter('stop_reason_topic').value)
        self.nav_available_topic = str(self.get_parameter('nav_available_topic').value)
        self.tf_ok_topic = str(self.get_parameter('tf_ok_topic').value)
        self.obstacle_inhibit_topic = str(self.get_parameter('obstacle_inhibit_topic').value)
        self.local_sensing_ok_topic = str(self.get_parameter('local_sensing_ok_topic').value)
        self.nav_backend_status_topic = str(self.get_parameter('nav_backend_status_topic').value)
        self.local_backend_status_topic = str(self.get_parameter('local_backend_status_topic').value)
        self.track_frame = str(self.get_parameter('track_frame').value)
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.tracker_timeout_sec = max(0.1, float(self.get_parameter('tracker_timeout_sec').value))
        self.target_soft_timeout_sec = max(0.1, float(self.get_parameter('target_soft_timeout_sec').value))
        self.target_hard_timeout_sec = max(self.target_soft_timeout_sec, float(self.get_parameter('target_hard_timeout_sec').value))
        self.tf_timeout_sec = max(0.1, float(self.get_parameter('tf_timeout_sec').value))
        self.nav_timeout_sec = max(0.1, float(self.get_parameter('nav_timeout_sec').value))
        self.obstacle_fault_timeout_sec = max(0.1, float(self.get_parameter('obstacle_fault_timeout_sec').value))
        self.local_sensing_timeout_sec = max(0.1, float(self.get_parameter('local_sensing_timeout_sec').value))
        self.current_mode = normalize_mode(self.get_parameter('default_follow_mode').value)

        self.follow_enabled = False
        self.last_tracker_rx_sec = 0.0
        self.last_target_rx_sec = 0.0
        self.latest_target: TrackedPose | None = None
        self.latest_behavior: BehaviorStatus | None = None
        self.nav_available = False
        self.last_nav_ok_sec = 0.0
        self.tf_healthy = False
        self.last_tf_ok_sec = 0.0
        self.obstacle_inhibit = False
        self.obstacle_start_sec = 0.0
        self.local_sensing_ok = False
        self.last_local_sensing_ok_sec = 0.0
        self.nav_backend = {'state': 'IDLE', 'authority': False, 'ready': False, 'fault': False, 'reason': ''}
        self.local_backend = {'state': 'IDLE', 'authority': False, 'ready': False, 'fault': False, 'reason': ''}

        self.create_subscription(Bool, self.follow_enable_topic, self._follow_enable_cb, 10)
        self.create_subscription(StringMsg, self.follow_mode_topic, self._mode_cb, 10)
        self.create_subscription(TrackedPoseArray, self.tracks_state_topic, self._tracks_cb, 10)
        self.create_subscription(TrackedPose, self.target_topic, self._target_cb, 10)
        self.create_subscription(BehaviorStatus, self.behavior_status_topic, self._behavior_cb, 10)
        self.create_subscription(Bool, self.nav_available_topic, self._nav_cb, 10)
        self.create_subscription(Bool, self.tf_ok_topic, self._tf_cb, 10)
        self.create_subscription(Bool, self.obstacle_inhibit_topic, self._obstacle_cb, 10)
        self.create_subscription(Bool, self.local_sensing_ok_topic, self._local_sensing_cb, 10)
        self.create_subscription(StringMsg, self.nav_backend_status_topic, self._nav_backend_status_cb, 10)
        self.create_subscription(StringMsg, self.local_backend_status_topic, self._local_backend_status_cb, 10)

        self.pub_status = self.create_publisher(SafetyStatus, self.safety_status_topic, 10)
        self.pub_reason = self.create_publisher(StringMsg, self.stop_reason_topic, 10)
        self.timer = self.create_timer(0.1, self._timer_cb)

        self.get_logger().info(f'[SafetySupervisor] Started mode={self.current_mode}')

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _follow_enable_cb(self, msg: Bool):
        self.follow_enabled = bool(msg.data)

    def _mode_cb(self, msg: StringMsg):
        next_mode = normalize_mode(msg.data, default='')
        if not next_mode:
            self.get_logger().warn(f'[SafetySupervisor] Ignoring invalid follow mode: {msg.data!r}')
            return
        if next_mode != self.current_mode:
            self.get_logger().info(f'[SafetySupervisor] mode: {self.current_mode} -> {next_mode}')
            self.current_mode = next_mode

    def _tracks_cb(self, msg: TrackedPoseArray):
        del msg
        self.last_tracker_rx_sec = self._now_sec()

    def _target_cb(self, msg: TrackedPose):
        self.latest_target = msg
        self.last_target_rx_sec = self._now_sec()

    def _behavior_cb(self, msg: BehaviorStatus):
        self.latest_behavior = msg

    def _nav_cb(self, msg: Bool):
        self.nav_available = bool(msg.data)
        if self.nav_available:
            self.last_nav_ok_sec = self._now_sec()

    def _tf_cb(self, msg: Bool):
        self.tf_healthy = bool(msg.data)
        if self.tf_healthy:
            self.last_tf_ok_sec = self._now_sec()

    def _obstacle_cb(self, msg: Bool):
        active = bool(msg.data)
        now = self._now_sec()
        if active and not self.obstacle_inhibit:
            self.obstacle_start_sec = now
        if not active:
            self.obstacle_start_sec = 0.0
        self.obstacle_inhibit = active

    def _local_sensing_cb(self, msg: Bool):
        self.local_sensing_ok = bool(msg.data)
        if self.local_sensing_ok:
            self.last_local_sensing_ok_sec = self._now_sec()

    def _nav_backend_status_cb(self, msg: StringMsg):
        self.nav_backend = self._parse_backend_status(msg.data)

    def _local_backend_status_cb(self, msg: StringMsg):
        self.local_backend = self._parse_backend_status(msg.data)

    def _timer_cb(self):
        now = self._now_sec()
        status = self._build_status(now)
        self.pub_status.publish(status)

        reason = StringMsg()
        reason.data = f'{self.current_mode}:{status.stop_reason}'
        self.pub_reason.publish(reason)

    def _build_status(self, now: float) -> SafetyStatus:
        msg = SafetyStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.follow_enabled = bool(self.follow_enabled)
        msg.nav_available = bool(self.nav_available)
        msg.tf_healthy = bool(self.tf_healthy)
        msg.obstacle_inhibit = bool(self.obstacle_inhibit)
        msg.tracker_age_sec = self._age(now, self.last_tracker_rx_sec)
        msg.target_age_sec = self._target_age(now)
        msg.tf_age_sec = self._age(now, self.last_tf_ok_sec)
        msg.target_valid = self._target_valid()

        if not self.follow_enabled:
            return self._fill(msg, SafetyStatus.HEALTH_WAITING, True, SafetyStatus.STOP_DISABLED_BY_SERVER, SafetyStatus.SOURCE_SERVER)
        if self.latest_behavior is not None and int(self.latest_behavior.state) == int(BehaviorStatus.STATE_ABORTED):
            return self._fill(msg, SafetyStatus.HEALTH_FAULT, True, SafetyStatus.STOP_BEHAVIOR_ABORTED, SafetyStatus.SOURCE_BEHAVIOR)
        if self.last_tracker_rx_sec <= 0.0 or msg.tracker_age_sec > self.tracker_timeout_sec:
            return self._fill(msg, SafetyStatus.HEALTH_FAULT, True, SafetyStatus.STOP_TRACKER_HEARTBEAT_TIMEOUT, SafetyStatus.SOURCE_TRACKER)
        if self.nav_backend['authority'] and self.local_backend['authority']:
            return self._fill(
                msg,
                SafetyStatus.HEALTH_FAULT,
                True,
                SafetyStatus.STOP_OBSTACLE_INHIBIT_TIMEOUT,
                SafetyStatus.SOURCE_SAFETY,
                reason='backend_conflict',
            )
        if self.current_mode == MODE_MAPPED:
            if self.local_backend['authority'] or self.local_backend['state'] == 'PAUSING':
                return self._fill(msg, SafetyStatus.HEALTH_WAITING, True, SafetyStatus.STOP_NAV2_PENDING, SafetyStatus.SOURCE_NAVIGATION, reason='handoff_wait_nav')
            if self.last_tf_ok_sec <= 0.0 or msg.tf_age_sec > self.tf_timeout_sec:
                return self._fill(msg, SafetyStatus.HEALTH_FAULT, True, SafetyStatus.STOP_TF_UNAVAILABLE_TIMEOUT, SafetyStatus.SOURCE_TF)
            if self.last_nav_ok_sec <= 0.0 and not self.nav_available:
                return self._fill(msg, SafetyStatus.HEALTH_WAITING, True, SafetyStatus.STOP_NAV2_PENDING, SafetyStatus.SOURCE_NAVIGATION)
            if not self.nav_available and self._age(now, self.last_nav_ok_sec) > self.nav_timeout_sec:
                return self._fill(msg, SafetyStatus.HEALTH_FAULT, True, SafetyStatus.STOP_NAV2_UNAVAILABLE, SafetyStatus.SOURCE_NAVIGATION)
            if self.nav_backend['fault']:
                return self._fill(msg, SafetyStatus.HEALTH_FAULT, True, SafetyStatus.STOP_NAV2_UNAVAILABLE, SafetyStatus.SOURCE_NAVIGATION, reason='nav_backend_fault')
        else:
            if self.nav_backend['authority'] or self.nav_backend['state'] == 'PAUSING':
                return self._fill(msg, SafetyStatus.HEALTH_WAITING, True, SafetyStatus.STOP_SELECTED_TARGET_PENDING, SafetyStatus.SOURCE_SAFETY, reason='handoff_wait_local')
            sensing_age = self._age(now, self.last_local_sensing_ok_sec)
            if not self.local_sensing_ok and (self.last_local_sensing_ok_sec <= 0.0 or sensing_age > self.local_sensing_timeout_sec):
                return self._fill(
                    msg,
                    SafetyStatus.HEALTH_FAULT,
                    True,
                    SafetyStatus.STOP_OBSTACLE_INHIBIT_TIMEOUT,
                    SafetyStatus.SOURCE_SAFETY,
                    reason='local_sensing_unavailable',
                )
            if self.local_backend['fault']:
                return self._fill(
                    msg,
                    SafetyStatus.HEALTH_FAULT,
                    True,
                    SafetyStatus.STOP_OBSTACLE_INHIBIT_TIMEOUT,
                    SafetyStatus.SOURCE_SAFETY,
                    reason='local_backend_fault',
                )
        if self.obstacle_inhibit:
            obstacle_dt = self._age(now, self.obstacle_start_sec)
            if obstacle_dt > self.obstacle_fault_timeout_sec:
                return self._fill(msg, SafetyStatus.HEALTH_FAULT, True, SafetyStatus.STOP_OBSTACLE_INHIBIT_TIMEOUT, SafetyStatus.SOURCE_SAFETY)
            return self._fill(msg, SafetyStatus.HEALTH_INHIBITED, True, SafetyStatus.STOP_OBSTACLE_INHIBIT, SafetyStatus.SOURCE_SAFETY)
        if self.latest_target is None or not bool(self.latest_target.selected):
            return self._fill(msg, SafetyStatus.HEALTH_WAITING, True, SafetyStatus.STOP_WAITING_FOR_TARGET, SafetyStatus.SOURCE_TARGET_LOCK)
        if msg.target_age_sec > self.target_hard_timeout_sec:
            return self._fill(msg, SafetyStatus.HEALTH_FAULT, True, SafetyStatus.STOP_SELECTED_TARGET_STALE_TIMEOUT, SafetyStatus.SOURCE_TARGET_LOCK)
        if msg.target_age_sec > self.target_soft_timeout_sec or not msg.target_valid:
            return self._fill(msg, SafetyStatus.HEALTH_WAITING, True, SafetyStatus.STOP_SELECTED_TARGET_PENDING, SafetyStatus.SOURCE_TARGET_LOCK)
        return self._fill(msg, SafetyStatus.HEALTH_OK, False, SafetyStatus.STOP_NONE, SafetyStatus.SOURCE_NONE)

    def _target_valid(self) -> bool:
        if self.latest_target is None:
            return False
        if not bool(self.latest_target.selected):
            return False
        if not bool(self.latest_target.valid):
            return False
        if self.current_mode == MODE_UNMAPPED and not bool(self.latest_target.publishable):
            return False
        return True

    def _fill(self, msg: SafetyStatus, health: int, inhibit: bool, reason_code: int, source_code: int, reason: str = '', source: str = '') -> SafetyStatus:
        msg.health_state = int(health)
        msg.stop_reason_code = int(reason_code)
        msg.stop_source_code = int(source_code)
        msg.inhibit_active = bool(inhibit)
        msg.stop_reason = str(reason or self.STOP_REASON_LABELS.get(reason_code, ''))
        msg.stop_source = str(source or self.STOP_SOURCE_LABELS.get(source_code, ''))
        msg.health_label = self._health_label(health)
        return msg

    @staticmethod
    def _health_label(health: int) -> str:
        if health == int(SafetyStatus.HEALTH_OK):
            return 'OK'
        if health == int(SafetyStatus.HEALTH_WAITING):
            return 'WAITING'
        if health == int(SafetyStatus.HEALTH_INHIBITED):
            return 'INHIBITED'
        return 'FAULT'

    @staticmethod
    def _age(now: float, stamp: float) -> float:
        if stamp <= 0.0:
            return 0.0
        return max(0.0, now - stamp)

    def _target_age(self, now: float) -> float:
        if self.latest_target is None:
            return 0.0
        if float(self.latest_target.staleness_sec) > 0.0:
            return float(self.latest_target.staleness_sec)
        return self._age(now, self.last_target_rx_sec)

    @staticmethod
    def _parse_backend_status(raw: str):
        result = {'state': 'IDLE', 'authority': False, 'ready': False, 'fault': False, 'reason': ''}
        for part in raw.split(';'):
            if '=' not in part:
                continue
            key, value = part.split('=', 1)
            key = key.strip()
            value = value.strip()
            if key == 'state':
                result['state'] = value or 'IDLE'
            elif key in ('authority', 'ready', 'fault'):
                result[key] = value == '1'
            elif key == 'reason':
                result['reason'] = value
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
