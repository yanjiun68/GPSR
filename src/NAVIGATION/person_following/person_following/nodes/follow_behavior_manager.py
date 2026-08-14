#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool, String
from std_msgs.msg import Float32

from putrabot_interfaces.msg import BehaviorStatus, MotionIntent, SafetyStatus, TrackedPose


class FollowBehaviorManager(Node):
    STATE_TO_LABEL = {
        BehaviorStatus.STATE_IDLE: 'IDLE',
        BehaviorStatus.STATE_WAIT_FOR_TARGET: 'WAIT_FOR_TARGET',
        BehaviorStatus.STATE_LOCKING_TARGET: 'LOCKING_TARGET',
        BehaviorStatus.STATE_FOLLOWING: 'FOLLOWING',
        BehaviorStatus.STATE_FOLLOWING_CAUTIOUS: 'FOLLOWING_CAUTIOUS',
        BehaviorStatus.STATE_HOLD_LOCKED: 'HOLD_LOCKED',
        BehaviorStatus.STATE_TEMP_OCCLUDED: 'TEMP_OCCLUDED',
        BehaviorStatus.STATE_REACQUIRE: 'REACQUIRE',
        BehaviorStatus.STATE_LOCAL_SEARCH: 'LOCAL_SEARCH',
        BehaviorStatus.STATE_SAFE_STOP: 'SAFE_STOP',
        BehaviorStatus.STATE_ABORTED: 'ABORTED',
    }
    MODE_TO_LABEL = {
        MotionIntent.MODE_HOLD: 'HOLD',
        MotionIntent.MODE_ROTATE: 'ROTATE',
        MotionIntent.MODE_FOLLOW_POSE: 'FOLLOW_POSE',
        MotionIntent.MODE_SEARCH: 'SEARCH',
    }
    REASON_TO_LABEL = {
        BehaviorStatus.REASON_NONE: '',
        BehaviorStatus.REASON_FOLLOW_DISABLED: 'follow_disabled',
        BehaviorStatus.REASON_WAITING_FOR_TARGET: 'waiting_for_target',
        BehaviorStatus.REASON_LOCK_DWELL: 'lock_dwell',
        BehaviorStatus.REASON_LOCK_CONFIRMED: 'lock_confirmed',
        BehaviorStatus.REASON_CONFIDENCE_HIGH: 'confidence_high',
        BehaviorStatus.REASON_CONFIDENCE_DEGRADED: 'confidence_degraded',
        BehaviorStatus.REASON_TARGET_FACING_ROBOT: 'target_facing_robot',
        BehaviorStatus.REASON_STANDOFF_REACHED: 'standoff_reached',
        BehaviorStatus.REASON_TARGET_PREDICTED: 'target_predicted',
        BehaviorStatus.REASON_TARGET_CONTRADICTORY: 'target_contradictory',
        BehaviorStatus.REASON_TARGET_SOFT_STALE: 'target_soft_stale',
        BehaviorStatus.REASON_TARGET_HARD_STALE: 'target_hard_stale',
        BehaviorStatus.REASON_SEARCH_BUDGET: 'search_budget',
        BehaviorStatus.REASON_SAFETY_INHIBIT: 'safety_inhibit',
        BehaviorStatus.REASON_ABORT_TRIGGER: 'abort_trigger',
    }

    def __init__(self):
        super().__init__('follow_behavior_manager')

        self.declare_parameter('target_topic', '/person_target_locked')
        self.declare_parameter('facing_topic', '/person_target_facing_robot')
        self.declare_parameter('follow_enable_topic', '/person_follow/enable')
        self.declare_parameter('safety_status_topic', '/person_follow/safety_status')
        self.declare_parameter('behavior_status_topic', '/person_follow/behavior_status')
        self.declare_parameter('motion_intent_topic', '/person_follow/motion_intent')
        self.declare_parameter('nav_backend_status_topic', '/person_follow/nav_backend_status')
        self.declare_parameter('local_backend_status_topic', '/person_follow/local_backend_status')
        self.declare_parameter('target_distance_topic', '/person_follow/target_distance_m')
        self.declare_parameter('track_frame', 'base_footprint')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('follow_distance_m', 0.8)
        self.declare_parameter('face_hold_sec', 10.0)
        self.declare_parameter('lock_confirm_sec', 0.3)
        self.declare_parameter('target_soft_timeout_sec', 1.2)
        self.declare_parameter('target_hard_timeout_sec', 2.0)
        self.declare_parameter('search_timeout_sec', 8.0)
        self.declare_parameter('follow_confidence_min', 0.72)
        self.declare_parameter('cautious_confidence_min', 0.52)
        self.declare_parameter('translate_staleness_sec', 0.6)
        self.declare_parameter('rotate_bearing_deg', 25.0)
        self.declare_parameter('contradiction_rotate_delay_sec', 0.5)
        self.declare_parameter('rotate_yaw_enter_deg', 8.0)
        self.declare_parameter('rotate_yaw_exit_deg', 5.0)
        self.declare_parameter('stop_distance_m', 0.45)
        self.declare_parameter('hold_distance_m', 1.0)
        self.declare_parameter('nominal_speed_mps', 0.55)
        self.declare_parameter('cautious_speed_mps', 0.3)
        self.declare_parameter('search_rotate_speed_rad_s', 0.45)

        self.target_topic = str(self.get_parameter('target_topic').value)
        self.facing_topic = str(self.get_parameter('facing_topic').value)
        self.follow_enable_topic = str(self.get_parameter('follow_enable_topic').value)
        self.safety_status_topic = str(self.get_parameter('safety_status_topic').value)
        self.behavior_status_topic = str(self.get_parameter('behavior_status_topic').value)
        self.motion_intent_topic = str(self.get_parameter('motion_intent_topic').value)
        self.nav_backend_status_topic = str(self.get_parameter('nav_backend_status_topic').value)
        self.local_backend_status_topic = str(self.get_parameter('local_backend_status_topic').value)
        self.target_distance_topic = str(self.get_parameter('target_distance_topic').value)
        self.track_frame = str(self.get_parameter('track_frame').value)
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.follow_distance_m = float(self.get_parameter('follow_distance_m').value)
        self.face_hold_sec = float(self.get_parameter('face_hold_sec').value)
        self.lock_confirm_sec = max(0.1, float(self.get_parameter('lock_confirm_sec').value))
        self.target_soft_timeout_sec = max(0.1, float(self.get_parameter('target_soft_timeout_sec').value))
        self.target_hard_timeout_sec = max(self.target_soft_timeout_sec, float(self.get_parameter('target_hard_timeout_sec').value))
        self.search_timeout_sec = max(0.5, float(self.get_parameter('search_timeout_sec').value))
        self.follow_confidence_min = float(self.get_parameter('follow_confidence_min').value)
        self.cautious_confidence_min = float(self.get_parameter('cautious_confidence_min').value)
        self.translate_staleness_sec = max(0.0, float(self.get_parameter('translate_staleness_sec').value))
        self.rotate_bearing_deg = float(self.get_parameter('rotate_bearing_deg').value)
        self.contradiction_rotate_delay_sec = max(0.0, float(self.get_parameter('contradiction_rotate_delay_sec').value))
        self.rotate_yaw_enter_deg = max(0.0, float(self.get_parameter('rotate_yaw_enter_deg').value))
        self.rotate_yaw_exit_deg = max(0.0, min(self.rotate_yaw_enter_deg, float(self.get_parameter('rotate_yaw_exit_deg').value)))
        self.stop_distance_m = float(self.get_parameter('stop_distance_m').value)
        self.hold_distance_m = float(self.get_parameter('hold_distance_m').value)
        self.nominal_speed_mps = float(self.get_parameter('nominal_speed_mps').value)
        self.cautious_speed_mps = float(self.get_parameter('cautious_speed_mps').value)
        self.search_rotate_speed_rad_s = float(self.get_parameter('search_rotate_speed_rad_s').value)

        self.follow_enabled = False
        self.facing_robot = False
        self.latest_target: Optional[TrackedPose] = None
        self.last_target_rx_sec = 0.0
        self.latest_safety: Optional[SafetyStatus] = None
        self.state = BehaviorStatus.STATE_IDLE
        self.previous_state = BehaviorStatus.STATE_IDLE
        self.transition_reason = BehaviorStatus.REASON_NONE
        self._lock_candidate_since_sec = 0.0
        self._loss_start_sec = 0.0
        self._face_hold_start_sec: Optional[float] = None
        self._last_target_id = 0
        self._contradiction_since_sec = 0.0
        self._publishable_drop_since_sec = 0.0
        self._align_turn_active = False
        self._reason_label_override = ''
        self._nav_backend = {'state': 'IDLE', 'authority': False, 'ready': False, 'fault': False, 'reason': ''}
        self._local_backend = {'state': 'IDLE', 'authority': False, 'ready': False, 'fault': False, 'reason': ''}

        self.create_subscription(TrackedPose, self.target_topic, self._target_cb, 10)
        self.create_subscription(Bool, self.facing_topic, self._facing_cb, 10)
        self.create_subscription(Bool, self.follow_enable_topic, self._follow_enable_cb, 10)
        self.create_subscription(SafetyStatus, self.safety_status_topic, self._safety_cb, 10)
        self.create_subscription(String, self.nav_backend_status_topic, self._nav_backend_cb, 10)
        self.create_subscription(String, self.local_backend_status_topic, self._local_backend_cb, 10)

        self.pub_behavior = self.create_publisher(BehaviorStatus, self.behavior_status_topic, 10)
        self.pub_motion = self.create_publisher(MotionIntent, self.motion_intent_topic, 10)
        self.pub_target_distance = self.create_publisher(Float32, self.target_distance_topic, 10)
        self.timer = self.create_timer(0.1, self._timer_cb)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info('[FollowBehaviorManager] Started')

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _target_cb(self, msg: TrackedPose):
        self.latest_target = msg
        self.last_target_rx_sec = self._now_sec()
        track_id = int(msg.track_id)
        if track_id != self._last_target_id:
            self._lock_candidate_since_sec = 0.0
            self._last_target_id = track_id

    def _facing_cb(self, msg: Bool):
        self.facing_robot = bool(msg.data)

    def _follow_enable_cb(self, msg: Bool):
        self.follow_enabled = bool(msg.data)
        if not self.follow_enabled:
            self._face_hold_start_sec = None
            self._loss_start_sec = 0.0
            self._lock_candidate_since_sec = 0.0

    def _safety_cb(self, msg: SafetyStatus):
        self.latest_safety = msg

    def _nav_backend_cb(self, msg: String):
        self._nav_backend = self._parse_backend_status(msg.data)

    def _local_backend_cb(self, msg: String):
        self._local_backend = self._parse_backend_status(msg.data)

    def _on_set_parameters(self, params) -> SetParametersResult:
        updates = {}
        for param in params:
            if param.name == 'follow_distance_m':
                value = float(param.value)
                if value <= 0.0:
                    return SetParametersResult(successful=False, reason='follow_distance_m must be > 0')
                updates['follow_distance_m'] = value
            elif param.name == 'face_hold_sec':
                value = float(param.value)
                if value <= 0.0:
                    return SetParametersResult(successful=False, reason='face_hold_sec must be > 0')
                updates['face_hold_sec'] = value

        if 'follow_distance_m' in updates:
            self.follow_distance_m = updates['follow_distance_m']
        if 'face_hold_sec' in updates:
            self.face_hold_sec = updates['face_hold_sec']
        return SetParametersResult(successful=True)

    def _timer_cb(self):
        now = self._now_sec()
        state, reason = self._decide_state(now)
        motion = self._build_motion_intent(state)
        self._publish_behavior(now, state, reason, motion)
        self._publish_target_distance(self._current_target_distance())
        self.pub_motion.publish(motion)

    def _decide_state(self, now: float) -> tuple[int, int]:
        if not self.follow_enabled:
            self._reset_tracking_state()
            return self._transition(BehaviorStatus.STATE_IDLE, BehaviorStatus.REASON_FOLLOW_DISABLED)

        target = self.latest_target
        target_fresh = target is not None and (now - self.last_target_rx_sec) <= self.target_soft_timeout_sec and bool(target.selected)

        if self.latest_safety is not None and bool(self.latest_safety.inhibit_active):
            code = int(self.latest_safety.stop_reason_code)
            if code not in (SafetyStatus.STOP_WAITING_FOR_TARGET, SafetyStatus.STOP_SELECTED_TARGET_PENDING):
                return self._transition(BehaviorStatus.STATE_SAFE_STOP, BehaviorStatus.REASON_SAFETY_INHIBIT)

        backend_gate = self._backend_gate_state()
        if backend_gate is not None:
            return backend_gate

        if not target_fresh or target is None or not bool(target.valid):
            self._face_hold_start_sec = None
            self._align_turn_active = False
            self._contradiction_since_sec = 0.0
            self._publishable_drop_since_sec = 0.0
            if self._lock_candidate_since_sec <= 0.0:
                return self._transition(BehaviorStatus.STATE_WAIT_FOR_TARGET, BehaviorStatus.REASON_WAITING_FOR_TARGET)
            if self._loss_start_sec <= 0.0:
                self._loss_start_sec = now
            lost_dt = now - self._loss_start_sec
            if lost_dt <= self.target_soft_timeout_sec:
                return self._transition(BehaviorStatus.STATE_TEMP_OCCLUDED, BehaviorStatus.REASON_TARGET_SOFT_STALE, reason_label_override='target_stale')
            if lost_dt <= self.target_hard_timeout_sec:
                return self._transition(BehaviorStatus.STATE_REACQUIRE, BehaviorStatus.REASON_TARGET_SOFT_STALE, reason_label_override='target_stale')
            if lost_dt <= self.search_timeout_sec:
                return self._transition(BehaviorStatus.STATE_LOCAL_SEARCH, BehaviorStatus.REASON_SEARCH_BUDGET, reason_label_override='target_lost')
            return self._transition(BehaviorStatus.STATE_SAFE_STOP, BehaviorStatus.REASON_TARGET_HARD_STALE, reason_label_override='target_lost')

        self._loss_start_sec = 0.0
        if self._lock_candidate_since_sec <= 0.0:
            self._lock_candidate_since_sec = now
        if (now - self._lock_candidate_since_sec) < self.lock_confirm_sec:
            return self._transition(BehaviorStatus.STATE_LOCKING_TARGET, BehaviorStatus.REASON_LOCK_DWELL)

        d = math.hypot(float(target.pose.position.x), float(target.pose.position.y))
        bearing_deg = abs(math.degrees(math.atan2(float(target.pose.position.y), float(target.pose.position.x))))
        stale = float(target.staleness_sec)
        combined = self._combined_confidence(target)
        contradiction = bool(target.contradictory_candidate)
        reacquire_active = bool(target.reacquire_active)
        publishable = bool(target.publishable)
        ambiguity_ok = bool(target.selected) and bool(target.valid) and stale <= self.target_soft_timeout_sec and combined >= self.cautious_confidence_min
        needs_align = self._should_align_turn(bearing_deg, publishable, d)

        if self.facing_robot and d <= self.hold_distance_m:
            if self._face_hold_start_sec is None:
                self._face_hold_start_sec = now
            elif (now - self._face_hold_start_sec) >= self.face_hold_sec:
                return self._transition(BehaviorStatus.STATE_ABORTED, BehaviorStatus.REASON_ABORT_TRIGGER)
        else:
            self._face_hold_start_sec = None

        if bool(target.predicted) and stale > self.translate_staleness_sec:
            self._align_turn_active = False
            self._contradiction_since_sec = 0.0
            self._publishable_drop_since_sec = 0.0
            return self._transition(BehaviorStatus.STATE_TEMP_OCCLUDED, BehaviorStatus.REASON_TARGET_PREDICTED)
        if d <= self.stop_distance_m:
            self._align_turn_active = False
            self._contradiction_since_sec = 0.0
            self._publishable_drop_since_sec = 0.0
            return self._transition(BehaviorStatus.STATE_HOLD_LOCKED, BehaviorStatus.REASON_STANDOFF_REACHED)
        if self.facing_robot and d <= self.hold_distance_m:
            self._align_turn_active = False
            self._contradiction_since_sec = 0.0
            self._publishable_drop_since_sec = 0.0
            return self._transition(BehaviorStatus.STATE_HOLD_LOCKED, BehaviorStatus.REASON_TARGET_FACING_ROBOT)

        if reacquire_active:
            self._align_turn_active = False
            self._contradiction_since_sec = 0.0
            self._publishable_drop_since_sec = 0.0
            return self._transition(BehaviorStatus.STATE_REACQUIRE, BehaviorStatus.REASON_TARGET_CONTRADICTORY, reason_label_override='target_lost')

        if contradiction:
            if self._contradiction_since_sec <= 0.0:
                self._contradiction_since_sec = now
            if ambiguity_ok and (now - self._contradiction_since_sec) < self.contradiction_rotate_delay_sec:
                self._align_turn_active = False
                return self._transition(BehaviorStatus.STATE_HOLD_LOCKED, BehaviorStatus.REASON_CONFIDENCE_DEGRADED, reason_label_override='contradiction_dwell')
            self._align_turn_active = False
            self._publishable_drop_since_sec = 0.0
            return self._transition(BehaviorStatus.STATE_REACQUIRE, BehaviorStatus.REASON_TARGET_CONTRADICTORY, reason_label_override='contradiction_timeout')
        self._contradiction_since_sec = 0.0

        if bool(target.valid) and not publishable:
            if self._publishable_drop_since_sec <= 0.0:
                self._publishable_drop_since_sec = now
            if ambiguity_ok and (now - self._publishable_drop_since_sec) < self.contradiction_rotate_delay_sec:
                self._align_turn_active = False
                return self._transition(BehaviorStatus.STATE_HOLD_LOCKED, BehaviorStatus.REASON_CONFIDENCE_DEGRADED, reason_label_override='contradiction_dwell')
            self._align_turn_active = False
            if ambiguity_ok:
                return self._transition(BehaviorStatus.STATE_HOLD_LOCKED, BehaviorStatus.REASON_CONFIDENCE_DEGRADED)
            return self._transition(BehaviorStatus.STATE_REACQUIRE, BehaviorStatus.REASON_TARGET_SOFT_STALE, reason_label_override='target_stale')
        self._publishable_drop_since_sec = 0.0

        if needs_align:
            return self._transition(BehaviorStatus.STATE_LOCKING_TARGET, BehaviorStatus.REASON_LOCK_CONFIRMED, reason_label_override='align_bearing')

        self._align_turn_active = False
        if combined >= self.follow_confidence_min and stale <= self.translate_staleness_sec and not bool(target.predicted):
            return self._transition(BehaviorStatus.STATE_FOLLOWING, BehaviorStatus.REASON_CONFIDENCE_HIGH)
        if combined >= self.cautious_confidence_min and stale <= self.target_soft_timeout_sec:
            return self._transition(BehaviorStatus.STATE_FOLLOWING_CAUTIOUS, BehaviorStatus.REASON_CONFIDENCE_DEGRADED)
        return self._transition(BehaviorStatus.STATE_HOLD_LOCKED, BehaviorStatus.REASON_CONFIDENCE_DEGRADED)

    def _build_motion_intent(self, state: int) -> MotionIntent:
        msg = MotionIntent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.target_track_id = int(self.latest_target.track_id) if self.latest_target is not None else 0
        msg.follow_distance_m = float(self.follow_distance_m)
        msg.speed_limit_mps = 0.0
        msg.rotate_speed_rad_s = 0.0
        msg.translation_allowed = False
        msg.desired_yaw_rad = 0.0

        if self.latest_target is not None:
            msg.desired_yaw_rad = float(math.atan2(float(self.latest_target.pose.position.y), float(self.latest_target.pose.position.x)))

        if state in (BehaviorStatus.STATE_IDLE, BehaviorStatus.STATE_WAIT_FOR_TARGET, BehaviorStatus.STATE_SAFE_STOP, BehaviorStatus.STATE_ABORTED):
            msg.mode = MotionIntent.MODE_HOLD
        elif state in (BehaviorStatus.STATE_LOCKING_TARGET, BehaviorStatus.STATE_TEMP_OCCLUDED, BehaviorStatus.STATE_REACQUIRE):
            msg.mode = MotionIntent.MODE_ROTATE
        elif state == BehaviorStatus.STATE_FOLLOWING:
            msg.mode = MotionIntent.MODE_FOLLOW_POSE
            msg.translation_allowed = True
            msg.speed_limit_mps = float(self.nominal_speed_mps)
        elif state == BehaviorStatus.STATE_FOLLOWING_CAUTIOUS:
            msg.mode = MotionIntent.MODE_FOLLOW_POSE
            msg.translation_allowed = True
            msg.speed_limit_mps = float(self.cautious_speed_mps)
        elif state == BehaviorStatus.STATE_LOCAL_SEARCH:
            msg.mode = MotionIntent.MODE_SEARCH
            msg.rotate_speed_rad_s = float(self.search_rotate_speed_rad_s)
        else:
            msg.mode = MotionIntent.MODE_HOLD

        msg.mode_label = self.MODE_TO_LABEL.get(msg.mode, 'HOLD')
        return msg

    def _current_target_distance(self) -> float:
        if self.latest_target is None:
            return 0.0
        return math.hypot(float(self.latest_target.pose.position.x), float(self.latest_target.pose.position.y)) - self.follow_distance_m

    def _publish_behavior(self, now: float, state: int, reason: int, motion: MotionIntent):
        msg = BehaviorStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = int(state)
        msg.previous_state = int(self.previous_state)
        msg.motion_mode = int(motion.mode)
        msg.transition_reason = int(reason)
        msg.selected_track_id = int(self.latest_target.track_id) if self.latest_target is not None else 0
        msg.translation_allowed = bool(motion.translation_allowed)
        msg.target_visible = bool(self.latest_target is not None and self.latest_target.camera_visible and self.latest_target.valid)
        msg.target_age_sec = self._target_age(now)

        if self.latest_target is not None:
            msg.detection_confidence = float(self.latest_target.detection_confidence)
            msg.geometry_confidence = float(self.latest_target.geometry_confidence)
            msg.identity_confidence = float(self.latest_target.identity_confidence)
            msg.motion_confidence = float(self.latest_target.motion_confidence)
            msg.visibility_confidence = float(self.latest_target.visibility_confidence)
            msg.freshness_confidence = float(self.latest_target.freshness_confidence)
            msg.combined_confidence = float(self._combined_confidence(self.latest_target))

        msg.state_label = self.STATE_TO_LABEL.get(state, 'IDLE')
        msg.previous_state_label = self.STATE_TO_LABEL.get(self.previous_state, 'IDLE')
        msg.motion_mode_label = motion.mode_label
        msg.transition_reason_label = self._reason_label_override or self.REASON_TO_LABEL.get(reason, '')
        self.pub_behavior.publish(msg)

    def _publish_target_distance(self, distance_m: float):
        msg = Float32()
        msg.data = float(max(0.0, distance_m))
        self.pub_target_distance.publish(msg)

    def _transition(self, new_state: int, reason: int, reason_label_override: str = '') -> tuple[int, int]:
        reason_changed = reason != self.transition_reason or reason_label_override != self._reason_label_override
        if new_state != self.state:
            old_state = self.state
            self.previous_state = old_state
            self.state = new_state
            self.transition_reason = reason
            self._reason_label_override = reason_label_override
            self.get_logger().info(
                f'[FollowBehaviorManager] state: {self.STATE_TO_LABEL.get(old_state)} -> {self.STATE_TO_LABEL.get(new_state)} ({reason_label_override or self.REASON_TO_LABEL.get(reason, "")})'
            )
        elif reason_changed:
            self.transition_reason = reason
            self._reason_label_override = reason_label_override
        return self.state, reason

    def _target_age(self, now: float) -> float:
        if self.latest_target is None:
            return 0.0
        if float(self.latest_target.staleness_sec) > 0.0:
            return float(self.latest_target.staleness_sec)
        if self.last_target_rx_sec <= 0.0:
            return 0.0
        return max(0.0, now - self.last_target_rx_sec)

    def _reset_tracking_state(self):
        self._face_hold_start_sec = None
        self._loss_start_sec = 0.0
        self._lock_candidate_since_sec = 0.0
        self._contradiction_since_sec = 0.0
        self._publishable_drop_since_sec = 0.0
        self._align_turn_active = False
        self._reason_label_override = ''

    def _backend_gate_state(self) -> Optional[tuple[int, int]]:
        nav = self._nav_backend
        local = self._local_backend
        if nav['authority'] and local['authority']:
            return self._transition(BehaviorStatus.STATE_SAFE_STOP, BehaviorStatus.REASON_SAFETY_INHIBIT)

        if self.latest_target is None:
            return None

        if self.latest_target.publishable and self.latest_target.valid:
            target_ok = True
        else:
            target_ok = False

        if not target_ok:
            return None

        if self.latest_safety is not None and bool(self.latest_safety.inhibit_active):
            return None

        if nav['fault'] or local['fault']:
            return self._transition(BehaviorStatus.STATE_HOLD_LOCKED, BehaviorStatus.REASON_CONFIDENCE_DEGRADED)

        if nav['state'] == 'PAUSING' or local['state'] == 'PAUSING':
            return self._transition(BehaviorStatus.STATE_HOLD_LOCKED, BehaviorStatus.REASON_CONFIDENCE_DEGRADED)

        return None

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

    @staticmethod
    def _combined_confidence(target: TrackedPose) -> float:
        if float(target.combined_confidence) > 0.0:
            return float(target.combined_confidence)
        values = [
            float(target.lock_confidence),
            float(target.identity_confidence),
            float(target.motion_confidence),
            float(target.detection_confidence),
            float(target.geometry_confidence),
            float(target.visibility_confidence),
            float(target.freshness_confidence),
        ]
        return max(0.0, min(1.0, sum(values) / max(1, len(values))))

    def _should_align_turn(self, bearing_deg: float, publishable: bool, distance_m: float) -> bool:
        if not publishable or distance_m <= self.stop_distance_m:
            self._align_turn_active = False
            return False
        if self._align_turn_active:
            self._align_turn_active = bearing_deg > self.rotate_yaw_exit_deg
            return self._align_turn_active
        self._align_turn_active = bearing_deg >= max(self.rotate_yaw_enter_deg, self.rotate_bearing_deg)
        return self._align_turn_active


def main(args=None):
    rclpy.init(args=args)
    node = FollowBehaviorManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
