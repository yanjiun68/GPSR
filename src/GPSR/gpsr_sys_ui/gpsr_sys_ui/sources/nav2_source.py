from __future__ import annotations

import math
import time
from typing import Dict, Iterable, List, Optional, Set

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from nav2_msgs.msg import CollisionMonitorState

from ..event_model import MonitorEvent
from ..topic_types import TWIST_TYPE, TWIST_STAMPED_TYPE, resolve_velocity_type


def _goal_status_label(value: int) -> str:
    mapping = {
        GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
        GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
        GoalStatus.STATUS_EXECUTING: 'EXECUTING',
        GoalStatus.STATUS_CANCELING: 'CANCELING',
        GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
        GoalStatus.STATUS_CANCELED: 'CANCELED',
        GoalStatus.STATUS_ABORTED: 'ABORTED',
    }
    return mapping.get(value, str(value))


def _collision_action_label(value: int) -> str:
    for name in ('DO_NOTHING', 'STOP', 'SLOWDOWN', 'LIMIT', 'APPROACH'):
        if getattr(CollisionMonitorState, name, object()) == value:
            return name
    return str(value)


class Nav2Source:
    def __init__(self, node, cache, events, *, collision_state_topic: str,
                 cmd_pre_topic: str, cmd_post_topic: str, odom_topic: str,
                 navigate_status_topic: str, persistent_sec: float) -> None:
        self.node = node
        self.cache = cache
        self.events = events
        self.persistent_sec = float(persistent_sec)
        self._collision_stop_since: Optional[float] = None
        self._persistent_emitted = False
        self._velocity_topics: Dict[str, dict] = {}
        for role, topic in (('pre', cmd_pre_topic), ('post', cmd_post_topic)):
            entry = self._velocity_topics.setdefault(topic, {'roles': set(), 'subscription': None, 'type': None, 'warning': ''})
            entry['roles'].add(role)

        self.collision_sub = node.create_subscription(CollisionMonitorState, collision_state_topic, self._collision, 20)
        self.odom_sub = node.create_subscription(Odometry, odom_topic, self._odom, 20)
        self.status_sub = node.create_subscription(GoalStatusArray, navigate_status_topic, self._status, 20)
        self.discovery_timer = node.create_timer(0.5, self._discover_velocity_topics)
        self.health_timer = node.create_timer(0.5, self._evaluate)

        self.cache.set('nav2', {
            'navigate_status': 'UNKNOWN',
            'collision_action': '-',
            'collision_polygon': '-',
            'pre_linear': 0.0,
            'pre_angular': 0.0,
            'post_linear': 0.0,
            'post_angular': 0.0,
            'odom_linear': 0.0,
            'odom_angular': 0.0,
            'classification': '-',
        })

    def _discover_velocity_topics(self) -> None:
        try:
            graph = dict(self.node.get_topic_names_and_types())
        except Exception as exc:
            self.events.emit(MonitorEvent('WARN', 'NAV2', 'CMD_VEL_GRAPH_LOOKUP_FAILED', str(exc)))
            return

        for topic, entry in self._velocity_topics.items():
            if entry['subscription'] is not None:
                continue
            types = list(graph.get(topic, []))
            if not types:
                continue
            resolved = resolve_velocity_type(types)
            if resolved is None:
                code = 'CMD_VEL_MULTIPLE_TYPES' if len(set(types)) > 1 else 'CMD_VEL_UNSUPPORTED_TYPE'
                signature = f'{code}:{",".join(types)}'
                if entry['warning'] != signature:
                    self.events.emit(MonitorEvent(
                        'WARN', 'NAV2', code,
                        f'{topic} advertises {types}; no ambiguous subscription created',
                        component=topic,
                        details={'topic': topic, 'types': types},
                    ))
                    entry['warning'] = signature
                continue

            msg_type = Twist if resolved == TWIST_TYPE else TwistStamped
            entry['subscription'] = self.node.create_subscription(
                msg_type,
                topic,
                lambda msg, t=topic: self._velocity(t, msg),
                20,
            )
            entry['type'] = resolved
            entry['warning'] = ''
            self.node.get_logger().info(f'Monitoring {topic} as {resolved}')

    def _velocity(self, topic: str, msg) -> None:
        twist = msg.twist if isinstance(msg, TwistStamped) else msg
        linear = float(twist.linear.x)
        angular = float(twist.angular.z)
        entry = self._velocity_topics.get(topic, {})
        updates = {}
        for role in entry.get('roles', set()):
            updates[f'{role}_linear'] = linear
            updates[f'{role}_angular'] = angular
            updates[f'{role}_topic'] = topic
            updates[f'{role}_type'] = entry.get('type')
        self.cache.update_section('nav2', updates)

    def _odom(self, msg: Odometry) -> None:
        self.cache.update_section('nav2', {
            'odom_linear': float(msg.twist.twist.linear.x),
            'odom_angular': float(msg.twist.twist.angular.z),
            'odom_stamp_sec': time.time(),
        })

    def _status(self, msg: GoalStatusArray) -> None:
        status = 'UNKNOWN'
        if msg.status_list:
            status = _goal_status_label(msg.status_list[-1].status)
        self.cache.update_section('nav2', {'navigate_status': status})
        if status == 'ABORTED':
            self.events.emit(MonitorEvent('ERROR', 'NAV2', 'NAVIGATION_ABORTED', 'NavigateToPose action aborted'))

    def _collision(self, msg: CollisionMonitorState) -> None:
        action = _collision_action_label(int(msg.action_type))
        polygon = str(msg.polygon_name or '-')
        now = time.monotonic()
        self.cache.update_section('nav2', {
            'collision_action': action,
            'collision_polygon': polygon,
            'collision_stamp_sec': time.time(),
        })
        if action == 'STOP':
            if self._collision_stop_since is None:
                self._collision_stop_since = now
                self._persistent_emitted = False
                self.events.emit(MonitorEvent('INFO', 'NAV2', 'COLLISION_STOP', f'{polygon} / STOP', component='/collision_monitor'))
        else:
            self._collision_stop_since = None
            self._persistent_emitted = False

    def _evaluate(self) -> None:
        nav = self.cache.get('nav2', {}) or {}
        now = time.monotonic()
        classification = '-'
        if self._collision_stop_since is not None:
            duration = now - self._collision_stop_since
            self.cache.update_section('nav2', {'collision_stop_duration_sec': round(duration, 2)})
            if duration >= self.persistent_sec:
                classification = 'COLLISION_STOP_PERSISTENT'
                if not self._persistent_emitted:
                    self.events.emit(MonitorEvent(
                        'WARN', 'NAV2', classification,
                        f"{nav.get('collision_polygon', '-')} STOP active {duration:.1f} s",
                        component='/collision_monitor',
                        details={'duration_sec': duration},
                    ))
                    self._persistent_emitted = True
        pre = abs(float(nav.get('pre_linear', 0.0) or 0.0))
        post = abs(float(nav.get('post_linear', 0.0) or 0.0))
        odom = abs(float(nav.get('odom_linear', 0.0) or 0.0))
        if classification == '-' and pre > 0.05 and post < 0.01:
            classification = 'VELOCITY_BLOCKED_AFTER_CONTROLLER'
        elif classification == '-' and pre > 0.05 and post > 0.05 and odom < 0.01:
            classification = 'COMMAND_PRESENT_BASE_NOT_MOVING'
        self.cache.update_section('nav2', {'classification': classification})
