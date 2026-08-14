#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from enum import Enum
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from putrabot_interfaces.msg import TargetLockDebug, TrackedPose, TrackedPoseArray


class LockTier(str, Enum):
    LOCK_STRONG = 'lock_strong'
    LOCK_WEAK = 'lock_weak'
    LOCK_UNSURE = 'lock_unsure'
    LOCK_LOST = 'lock_lost'


class TargetLockManager(Node):
    def __init__(self):
        super().__init__('target_lock_manager')

        self.declare_parameter('tracks_state_topic', '/person_tracks_state')
        self.declare_parameter('target_lock_debug_topic', '/person_target_lock_debug')
        self.declare_parameter('target_topic', '/person_target_locked')
        self.declare_parameter('target_facing_topic', '/person_target_facing_robot')
        self.declare_parameter('track_frame', 'base_footprint')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('target_timeout_sec', 1.0)
        self.declare_parameter('publishable_timeout_sec', 0.70)
        self.declare_parameter('combined_publishable_min', 0.40)
        self.declare_parameter('relock_stability_cycles', 3)
        self.declare_parameter('relock_cooldown_sec', 0.3)
        self.declare_parameter('current_lock_stale_hold_sec', 0.45)
        self.declare_parameter('same_target_position_gate_m', 0.18)
        self.declare_parameter('same_target_staleness_gate_sec', 0.70)
        self.declare_parameter('publish_timer_period_sec', 0.05)
        self.declare_parameter('lock_score_w_track', 0.22)
        self.declare_parameter('lock_score_w_confidence', 0.25)
        self.declare_parameter('lock_score_w_identity', 0.16)
        self.declare_parameter('lock_score_w_motion', 0.08)
        self.declare_parameter('lock_score_w_freshness', 0.14)
        self.declare_parameter('lock_score_w_facing', 0.05)
        self.declare_parameter('lock_score_w_history', 0.10)
        self.declare_parameter('lock_score_selected_bonus', 0.12)
        self.declare_parameter('lock_score_publishable_bonus', 0.05)
        self.declare_parameter('lock_score_contradiction_penalty', 0.25)
        self.declare_parameter('lock_score_crossover_penalty', 0.16)
        self.declare_parameter('lock_score_stale_penalty', 0.18)
        self.declare_parameter('lock_score_visibility_penalty', 0.10)
        self.declare_parameter('lock_score_switch_margin', 0.08)
        self.declare_parameter('lock_score_retain_margin', 0.03)
        self.declare_parameter('lock_score_strong_min', 0.70)
        self.declare_parameter('lock_score_weak_min', 0.52)
        self.declare_parameter('lock_score_unsure_min', 0.35)

        self.tracks_state_topic = str(self.get_parameter('tracks_state_topic').value)
        self.target_lock_debug_topic = str(self.get_parameter('target_lock_debug_topic').value)
        self.target_topic = str(self.get_parameter('target_topic').value)
        self.target_facing_topic = str(self.get_parameter('target_facing_topic').value)
        self.track_frame = str(self.get_parameter('track_frame').value)
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.target_timeout_sec = max(0.1, float(self.get_parameter('target_timeout_sec').value))
        self.publishable_timeout_sec = max(0.05, float(self.get_parameter('publishable_timeout_sec').value))
        self.combined_publishable_min = max(0.0, min(1.0, float(self.get_parameter('combined_publishable_min').value)))
        self.relock_stability_cycles = max(1, int(self.get_parameter('relock_stability_cycles').value))
        self.relock_cooldown_sec = max(0.0, float(self.get_parameter('relock_cooldown_sec').value))
        self.current_lock_stale_hold_sec = max(0.0, float(self.get_parameter('current_lock_stale_hold_sec').value))
        self.same_target_position_gate_m = max(0.01, float(self.get_parameter('same_target_position_gate_m').value))
        self.same_target_staleness_gate_sec = max(0.0, float(self.get_parameter('same_target_staleness_gate_sec').value))
        self.publish_timer_period_sec = max(0.01, float(self.get_parameter('publish_timer_period_sec').value))
        self.lock_score_w_track = float(self.get_parameter('lock_score_w_track').value)
        self.lock_score_w_confidence = float(self.get_parameter('lock_score_w_confidence').value)
        self.lock_score_w_identity = float(self.get_parameter('lock_score_w_identity').value)
        self.lock_score_w_motion = float(self.get_parameter('lock_score_w_motion').value)
        self.lock_score_w_freshness = float(self.get_parameter('lock_score_w_freshness').value)
        self.lock_score_w_facing = float(self.get_parameter('lock_score_w_facing').value)
        self.lock_score_w_history = float(self.get_parameter('lock_score_w_history').value)
        self.lock_score_selected_bonus = float(self.get_parameter('lock_score_selected_bonus').value)
        self.lock_score_publishable_bonus = float(self.get_parameter('lock_score_publishable_bonus').value)
        self.lock_score_contradiction_penalty = float(self.get_parameter('lock_score_contradiction_penalty').value)
        self.lock_score_crossover_penalty = float(self.get_parameter('lock_score_crossover_penalty').value)
        self.lock_score_stale_penalty = float(self.get_parameter('lock_score_stale_penalty').value)
        self.lock_score_visibility_penalty = float(self.get_parameter('lock_score_visibility_penalty').value)
        self.lock_score_switch_margin = max(0.0, float(self.get_parameter('lock_score_switch_margin').value))
        self.lock_score_retain_margin = max(0.0, float(self.get_parameter('lock_score_retain_margin').value))
        self.lock_score_strong_min = float(self.get_parameter('lock_score_strong_min').value)
        self.lock_score_weak_min = float(self.get_parameter('lock_score_weak_min').value)
        self.lock_score_unsure_min = float(self.get_parameter('lock_score_unsure_min').value)

        self._last_tracks_rx_sec = 0.0
        self._tracks_by_id: Dict[int, TrackedPose] = {}
        self._selected_track: Optional[TrackedPose] = None
        self._selected_track_id = 0
        self._source_track_id = 0
        self._candidate_track_id = 0
        self._candidate_cycles = 0
        self._last_debug: Optional[TargetLockDebug] = None
        self._last_lock_change_sec = 0.0
        self._current_tier = LockTier.LOCK_LOST
        self._last_eval_reason = 'startup'
        self._last_best_score = 0.0
        self._last_second_score = 0.0
        self._last_identity_agreement = 0.0
        self._last_published_state: Tuple[bool, Optional[int]] = (False, None)
        self._last_publish_sec = 0.0
        self._publish_dedupe_sec = min(0.03, max(0.01, self.publish_timer_period_sec * 0.5))

        self.create_subscription(TrackedPoseArray, self.tracks_state_topic, self._tracks_state_cb, 10)
        self.create_subscription(TargetLockDebug, self.target_lock_debug_topic, self._debug_cb, 10)
        self.pub_target = self.create_publisher(TrackedPose, self.target_topic, 10)
        self.pub_facing = self.create_publisher(Bool, self.target_facing_topic, 10)
        self.timer = self.create_timer(self.publish_timer_period_sec, self._timer_cb)

        self.get_logger().info('[TargetLockManager] Started')

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tracks_state_cb(self, msg: TrackedPoseArray):
        now = self._now_sec()
        self._last_tracks_rx_sec = now
        self._tracks_by_id = {int(track.track_id): track for track in msg.tracks if int(track.track_id) > 0}
        self._evaluate_lock(now, msg.tracks)
        current_state = self._current_publish_state(now)
        if self._is_material_transition(self._last_published_state, current_state):
            self._publish_current_target(now)

    def _evaluate_lock(self, now: float, tracks):
        current_locked = self._tracks_by_id.get(self._source_track_id) if self._source_track_id > 0 else None
        if current_locked is not None:
            self._selected_track = current_locked

        candidates = [track for track in tracks if self._is_lock_candidate(track)]
        if not candidates:
            if self._should_hold_current_lock(current_locked):
                self._current_tier = self._tier_for_track(current_locked)
                self._last_eval_reason = 'hold_current_no_candidates'
                return
            self._reset_lock('no_candidates')
            return

        scored = sorted(((self._score_track(track), track) for track in candidates), key=lambda item: item[0], reverse=True)
        best_score, best_track = scored[0]
        second_score = scored[1][0] if len(scored) > 1 else 0.0
        self._last_best_score = best_score
        self._last_second_score = second_score
        self._last_identity_agreement = float(best_track.identity_confidence)

        if self._selected_track_id <= 0:
            self._current_tier = self._tier_from_score(best_track, best_score)
            self._select_track(best_track, now, 'initial_lock')
            return

        if current_locked is None or not self._is_lock_candidate(current_locked):
            if best_score < self.lock_score_unsure_min:
                self._reset_lock('current_invalid_no_replacement')
                return
            self._current_tier = self._tier_from_score(best_track, best_score)
            self._select_track(best_track, now, 'replace_invalid_current')
            return

        current_score = self._score_track(current_locked)
        if int(best_track.track_id) == int(current_locked.track_id):
            self._candidate_track_id = 0
            self._candidate_cycles = 0
            self._selected_track = best_track
            self._source_track_id = int(best_track.track_id)
            self._current_tier = self._tier_from_score(best_track, best_score)
            self._last_eval_reason = 'retain_same_track'
            return

        if self._is_same_target_as_locked(best_track):
            self._source_track_id = int(best_track.track_id)
            self._selected_track_id = int(best_track.track_id)
            self._selected_track = best_track
            self._candidate_track_id = 0
            self._candidate_cycles = 0
            self._current_tier = self._tier_from_score(best_track, best_score)
            self._last_eval_reason = 'retain_same_person'
            return

        if self._should_hold_current_lock(current_locked):
            if (current_score + self.lock_score_retain_margin) >= best_score:
                self._candidate_track_id = 0
                self._candidate_cycles = 0
                self._selected_track = current_locked
                self._current_tier = self._tier_from_score(current_locked, current_score)
                self._last_eval_reason = 'retain_hysteresis'
                return

        if (now - self._last_lock_change_sec) < self.relock_cooldown_sec:
            self._selected_track = current_locked
            self._current_tier = self._tier_from_score(current_locked, current_score)
            self._last_eval_reason = 'relock_cooldown'
            return

        if best_score < max(self.lock_score_weak_min, current_score + self.lock_score_switch_margin):
            self._candidate_track_id = 0
            self._candidate_cycles = 0
            self._selected_track = current_locked
            self._current_tier = self._tier_from_score(current_locked, current_score)
            self._last_eval_reason = 'switch_blocked_margin'
            return

        if int(best_track.track_id) != self._candidate_track_id:
            self._candidate_track_id = int(best_track.track_id)
            self._candidate_cycles = 1
            self._selected_track = current_locked
            self._current_tier = self._tier_from_score(current_locked, current_score)
            self._last_eval_reason = 'switch_candidate_started'
            return

        self._candidate_cycles += 1
        if self._candidate_cycles < self.relock_stability_cycles:
            self._selected_track = current_locked
            self._current_tier = self._tier_from_score(current_locked, current_score)
            self._last_eval_reason = 'switch_candidate_pending'
            return

        self._current_tier = self._tier_from_score(best_track, best_score)
        self._select_track(best_track, now, 'switch_confirmed')

    def _select_track(self, track: TrackedPose, now: float, reason: str):
        track_id = int(track.track_id)
        self._selected_track_id = track_id
        self._source_track_id = track_id
        self._selected_track = track
        self._candidate_track_id = 0
        self._candidate_cycles = 0
        self._last_lock_change_sec = now
        self._last_eval_reason = reason
        self.get_logger().info(f'[TargetLockManager] lock -> track={track_id} reason={reason} tier={self._current_tier.value}')

    def _reset_lock(self, reason: str):
        self._selected_track = None
        self._selected_track_id = 0
        self._source_track_id = 0
        self._candidate_track_id = 0
        self._candidate_cycles = 0
        self._current_tier = LockTier.LOCK_LOST
        self._last_eval_reason = reason

    def _debug_cb(self, msg: TargetLockDebug):
        self._last_debug = msg

    def _timer_cb(self):
        now = self._now_sec()
        if self._last_publish_sec > 0.0 and (now - self._last_publish_sec) < self._publish_dedupe_sec:
            return
        self._publish_current_target(now)

    def _publish_current_target(self, now: float):
        msg = self._current_target_msg(now)
        self.pub_target.publish(msg)

        facing = Bool()
        facing.data = bool(msg.valid and msg.facing_robot)
        self.pub_facing.publish(facing)
        self._last_publish_sec = now
        self._last_published_state = self._state_from_msg(msg)

    def _build_selected_target(self, now: float, selected: TrackedPose) -> TrackedPose:
        msg = TrackedPose()
        msg.header = selected.header
        msg.track_id = int(self._selected_track_id if self._selected_track_id > 0 else selected.track_id)
        msg.pose = selected.pose
        msg.track_state = int(selected.track_state)
        msg.track_score = float(selected.track_score)
        msg.conf_app = float(selected.conf_app)
        msg.conf_motion = float(selected.conf_motion)
        msg.identity_confidence = float(selected.identity_confidence)
        msg.motion_confidence = float(selected.motion_confidence)
        msg.staleness_sec = float(selected.staleness_sec)
        msg.predicted = bool(selected.predicted)
        msg.reacquire_active = bool(selected.reacquire_active or self._current_tier == LockTier.LOCK_UNSURE)
        msg.contradictory_candidate = bool(selected.contradictory_candidate or self._contradictory_candidate())
        msg.crossover_active = bool(selected.crossover_active)
        msg.selected = True
        msg.valid = True
        msg.camera_visible = bool(selected.camera_visible)
        msg.lidar_support = bool(selected.lidar_support)
        msg.facing01 = self._clamp01(float(selected.facing01))
        msg.facing_robot = bool(selected.facing_robot)
        msg.lock_state = int(self._lock_state_from_tier(self._current_tier))
        msg.suppression_code = int(selected.suppression_code)

        msg.detection_confidence = self._clamp01(float(selected.track_score))
        msg.geometry_confidence = self._geometry_confidence(selected)
        msg.visibility_confidence = self._visibility_confidence(selected)
        msg.freshness_confidence = self._freshness_confidence(float(selected.staleness_sec))
        lock_score = self._score_track(selected)
        msg.lock_confidence = self._clamp01(lock_score)
        msg.combined_confidence = self._combined_confidence(msg)

        publishable = (
            self._current_tier in (LockTier.LOCK_STRONG, LockTier.LOCK_WEAK)
            and not msg.contradictory_candidate
            and not bool(msg.crossover_active and self._current_tier != LockTier.LOCK_STRONG)
            and float(msg.staleness_sec) <= self.publishable_timeout_sec
            and float(msg.combined_confidence) >= self.combined_publishable_min
        )
        msg.publishable = bool(publishable)

        if self._current_tier == LockTier.LOCK_LOST:
            msg.valid = False
            msg.publishable = False
            msg.suppression_code = int(TrackedPose.SUPPRESS_INVALID_TARGET)
        elif msg.contradictory_candidate:
            msg.publishable = False
            msg.suppression_code = int(TrackedPose.SUPPRESS_CONTRADICTION)
        elif bool(msg.crossover_active) and self._current_tier != LockTier.LOCK_STRONG:
            msg.publishable = False
            msg.suppression_code = int(TrackedPose.SUPPRESS_AMBIGUOUS)
        elif float(msg.staleness_sec) > self.publishable_timeout_sec:
            msg.publishable = False
            msg.suppression_code = int(TrackedPose.SUPPRESS_STALE)
        elif float(msg.combined_confidence) < self.combined_publishable_min:
            msg.publishable = False
            msg.suppression_code = int(TrackedPose.SUPPRESS_LOW_CONF)
        else:
            msg.suppression_code = int(TrackedPose.PUB_OK)

        if not msg.publishable and self._current_tier == LockTier.LOCK_UNSURE:
            msg.valid = True
        if not math.isfinite(float(msg.pose.position.x)) or not math.isfinite(float(msg.pose.position.y)):
            msg.valid = False
            msg.publishable = False
            msg.suppression_code = int(TrackedPose.SUPPRESS_INVALID_TARGET)

        if not msg.valid:
            return self._invalid_target_msg(now)

        return msg

    def _publish_invalid(self, now: float):
        self.pub_target.publish(self._invalid_target_msg(now))
        facing = Bool()
        facing.data = False
        self.pub_facing.publish(facing)

    def _current_target_msg(self, now: float) -> TrackedPose:
        if self._last_tracks_rx_sec <= 0.0 or (now - self._last_tracks_rx_sec) > self.target_timeout_sec:
            return self._invalid_target_msg(now)

        selected = self._selected_track
        if selected is None or self._selected_track_id <= 0:
            return self._invalid_target_msg(now)

        return self._build_selected_target(now, selected)

    def _current_publish_state(self, now: float) -> Tuple[bool, Optional[int]]:
        return self._state_from_msg(self._current_target_msg(now))

    @staticmethod
    def _state_from_msg(msg: TrackedPose) -> Tuple[bool, Optional[int]]:
        is_valid = bool(msg.valid)
        return is_valid, int(msg.track_id) if is_valid and int(msg.track_id) > 0 else None

    @staticmethod
    def _is_material_transition(
        previous: Tuple[bool, Optional[int]],
        current: Tuple[bool, Optional[int]],
    ) -> bool:
        prev_valid, prev_track_id = previous
        curr_valid, curr_track_id = current
        if prev_valid != curr_valid:
            return True
        if prev_valid and curr_valid and prev_track_id != curr_track_id:
            return True
        return False

    def _invalid_target_msg(self, now: float) -> TrackedPose:
        msg = TrackedPose()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.track_frame
        msg.track_id = 0
        msg.pose.orientation.w = 1.0
        msg.track_state = int(TrackedPose.TRACK_DELETED)
        msg.lock_state = int(TrackedPose.LOCK_NONE)
        msg.suppression_code = int(TrackedPose.SUPPRESS_INVALID_TARGET)
        msg.selected = False
        msg.valid = False
        msg.publishable = False
        msg.camera_visible = False
        msg.lidar_support = False
        msg.facing01 = 0.0
        msg.facing_robot = False
        msg.predicted = False
        msg.reacquire_active = False
        msg.contradictory_candidate = False
        msg.crossover_active = False
        msg.staleness_sec = float(max(self.target_timeout_sec, now - self._last_tracks_rx_sec if self._last_tracks_rx_sec > 0.0 else 0.0))
        return msg

    def _is_lock_candidate(self, track: Optional[TrackedPose]) -> bool:
        if track is None:
            return False
        if int(track.track_id) <= 0:
            return False
        if int(track.track_state) == int(TrackedPose.TRACK_DELETED):
            return False
        if not math.isfinite(float(track.pose.position.x)) or not math.isfinite(float(track.pose.position.y)):
            return False
        return bool(track.valid or track.selected or track.publishable or track.reacquire_active)

    def _should_hold_current_lock(self, current_locked: Optional[TrackedPose]) -> bool:
        if current_locked is None:
            return False
        if int(current_locked.track_id) <= 0:
            return False
        if int(current_locked.lock_state) == int(TrackedPose.LOCK_REACQUIRE):
            return False
        if not bool(current_locked.valid):
            return False

        staleness_sec = max(0.0, float(current_locked.staleness_sec))
        if bool(current_locked.publishable):
            return staleness_sec <= max(self.current_lock_stale_hold_sec, self.publishable_timeout_sec)
        if not bool(current_locked.camera_visible) and not bool(current_locked.lidar_support):
            return False
        return staleness_sec <= self.current_lock_stale_hold_sec

    def _is_same_target_as_locked(self, candidate: Optional[TrackedPose]) -> bool:
        if candidate is None or self._selected_track is None:
            return False
        if self._selected_track_id <= 0:
            return False
        if not bool(self._selected_track.valid):
            return False
        if int(self._selected_track.lock_state) == int(TrackedPose.LOCK_REACQUIRE):
            return False

        locked_staleness = max(0.0, float(self._selected_track.staleness_sec))
        candidate_staleness = max(0.0, float(candidate.staleness_sec))
        if locked_staleness > self.same_target_staleness_gate_sec:
            return False
        if candidate_staleness > self.same_target_staleness_gate_sec:
            return False

        dx = float(candidate.pose.position.x) - float(self._selected_track.pose.position.x)
        dy = float(candidate.pose.position.y) - float(self._selected_track.pose.position.y)
        distance = math.hypot(dx, dy)
        if distance > self.same_target_position_gate_m:
            return False

        if bool(self._selected_track.camera_visible) and bool(candidate.camera_visible):
            facing_delta = abs(float(candidate.facing01) - float(self._selected_track.facing01))
            if facing_delta > 0.35:
                return False

        return True

    def _score_track(self, track: TrackedPose) -> float:
        score = 0.0
        score += self.lock_score_w_track * self._clamp01(float(track.track_score))
        score += self.lock_score_w_confidence * self._clamp01(float(track.combined_confidence))
        score += self.lock_score_w_identity * self._clamp01(float(track.identity_confidence))
        score += self.lock_score_w_motion * self._clamp01(float(track.motion_confidence))
        score += self.lock_score_w_freshness * self._freshness_confidence(float(track.staleness_sec))
        score += self.lock_score_w_facing * self._clamp01(float(track.facing01))
        if int(track.track_id) == self._source_track_id:
            score += self.lock_score_w_history
        if bool(track.selected):
            score += self.lock_score_selected_bonus
        if bool(track.publishable):
            score += self.lock_score_publishable_bonus
        if bool(track.contradictory_candidate) or self._contradictory_candidate():
            score -= self.lock_score_contradiction_penalty
        if bool(track.crossover_active):
            score -= self.lock_score_crossover_penalty
        if float(track.staleness_sec) > self.publishable_timeout_sec:
            score -= self.lock_score_stale_penalty
        if not bool(track.camera_visible) and not bool(track.lidar_support):
            score -= self.lock_score_visibility_penalty
        return self._clamp01(score)

    def _tier_from_score(self, track: TrackedPose, score: float) -> LockTier:
        if score >= self.lock_score_strong_min and bool(track.publishable) and not bool(track.contradictory_candidate):
            return LockTier.LOCK_STRONG
        if score >= self.lock_score_weak_min and bool(track.valid):
            return LockTier.LOCK_WEAK
        if score >= self.lock_score_unsure_min and bool(track.valid):
            return LockTier.LOCK_UNSURE
        return LockTier.LOCK_LOST

    def _tier_for_track(self, track: Optional[TrackedPose]) -> LockTier:
        if track is None:
            return LockTier.LOCK_LOST
        return self._tier_from_score(track, self._score_track(track))

    def _lock_state_from_tier(self, tier: LockTier) -> int:
        if tier == LockTier.LOCK_STRONG:
            return TrackedPose.LOCK_LOCKED
        if tier == LockTier.LOCK_WEAK:
            return TrackedPose.LOCK_TEMP_OCCLUDED
        if tier == LockTier.LOCK_UNSURE:
            return TrackedPose.LOCK_REACQUIRE
        return TrackedPose.LOCK_NONE

    def _contradictory_candidate(self) -> bool:
        if self._last_debug is None:
            return False
        return bool(self._last_debug.contradictory_candidate)

    def _geometry_confidence(self, track: TrackedPose) -> float:
        if bool(track.lidar_support) and bool(track.camera_visible):
            return 1.0
        if bool(track.camera_visible):
            return 0.75
        if bool(track.lidar_support):
            return 0.60
        return 0.25 if bool(track.predicted) else 0.1

    def _visibility_confidence(self, track: TrackedPose) -> float:
        if bool(track.camera_visible) and not bool(track.predicted):
            return 1.0
        if bool(track.camera_visible):
            return 0.7
        if bool(track.predicted):
            return 0.25
        return 0.0

    def _freshness_confidence(self, staleness_sec: float) -> float:
        if staleness_sec <= 0.0:
            return 1.0
        return self._clamp01(1.0 - (staleness_sec / self.target_timeout_sec))

    def _combined_confidence(self, track: TrackedPose) -> float:
        terms = [
            float(track.lock_confidence),
            float(track.identity_confidence),
            float(track.motion_confidence),
            float(track.detection_confidence),
            float(track.geometry_confidence),
            float(track.visibility_confidence),
            float(track.freshness_confidence),
        ]
        penalty = 0.20 if self._contradictory_candidate() else 0.0
        return self._clamp01((sum(terms) / len(terms)) - penalty)

    @staticmethod
    def _clamp01(value: float) -> float:
        if not math.isfinite(value):
            return 0.0
        return max(0.0, min(1.0, float(value)))


def main(args=None):
    rclpy.init(args=args)
    node = TargetLockManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
