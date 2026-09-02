from __future__ import annotations

from typing import Dict
from lifecycle_msgs.msg import State, TransitionEvent
from lifecycle_msgs.srv import GetState

from ..event_model import MonitorEvent


LABELS = {
    State.PRIMARY_STATE_UNKNOWN: 'unknown',
    State.PRIMARY_STATE_UNCONFIGURED: 'unconfigured',
    State.PRIMARY_STATE_INACTIVE: 'inactive',
    State.PRIMARY_STATE_ACTIVE: 'active',
    State.PRIMARY_STATE_FINALIZED: 'finalized',
}


class LifecycleSource:
    def __init__(self, node, cache, events, node_names, rate_hz: float) -> None:
        self.node = node
        self.cache = cache
        self.events = events
        self.node_names = list(dict.fromkeys(node_names))
        self.clients: Dict[str, object] = {}
        self.pending: Dict[str, bool] = {}
        self.rows: Dict[str, dict] = {}
        self.transition_subs = []
        for name in self.node_names:
            self.clients[name] = node.create_client(GetState, f'{name.rstrip("/")}/get_state')
            topic = f'{name.rstrip("/")}/transition_event'
            self.transition_subs.append(node.create_subscription(TransitionEvent, topic, lambda msg, n=name: self._transition(n, msg), 20))
        self.timer = node.create_timer(1.0 / max(0.1, rate_hz), self.poll)

    def _expected(self, name: str) -> str:
        gpsr = self.cache.get('gpsr', {}) or {}
        skill = str(gpsr.get('current_skill', gpsr.get('skill', ''))).upper()
        if name in {'/controller_server', '/planner_server', '/bt_navigator', '/local_costmap/local_costmap', '/global_costmap/global_costmap'}:
            return 'active'
        if name == '/gpsr_object_detection_node':
            return 'active' if skill in {'FIND_OBJECT', 'COUNT_OBJECT'} else 'inactive'
        if name == '/gpsr_person_detection_node':
            return 'active' if skill in {'FIND_PERSON', 'FOLLOW_PERSON'} else 'inactive'
        if name == '/collision_monitor':
            return 'active'
        return ''

    def poll(self) -> None:
        for name, client in self.clients.items():
            if self.pending.get(name):
                continue
            if not client.service_is_ready():
                self.rows[name] = {'name': name, 'health': 'WARN', 'actual': 'unavailable', 'expected': self._expected(name), 'message': 'GetState service unavailable'}
                continue
            future = client.call_async(GetState.Request())
            self.pending[name] = True
            future.add_done_callback(lambda f, n=name: self._done(n, f))
        self.cache.set('lifecycle', list(self.rows.values()))

    def _done(self, name: str, future) -> None:
        self.pending[name] = False
        try:
            response = future.result()
            actual = LABELS.get(response.current_state.id, response.current_state.label or 'unknown')
        except Exception as exc:
            self.rows[name] = {'name': name, 'health': 'WARN', 'actual': 'error', 'expected': self._expected(name), 'message': str(exc)}
            return
        expected = self._expected(name)
        health = 'OK' if not expected or actual == expected else 'ERROR'
        message = '' if health == 'OK' else f'{actual.upper()}, expected {expected.upper()}'
        self.rows[name] = {'name': name, 'health': health, 'actual': actual, 'expected': expected, 'state': actual, 'message': message}
        if health == 'ERROR':
            self.events.emit(MonitorEvent('ERROR', 'LIFECYCLE', 'LIFECYCLE_STATE_MISMATCH', message, node=name))
        self.cache.set('lifecycle', list(self.rows.values()))

    def _transition(self, name: str, msg: TransitionEvent) -> None:
        self.events.emit(MonitorEvent(
            'INFO', 'LIFECYCLE', 'LIFECYCLE_TRANSITION',
            f'{msg.start_state.label} -> {msg.goal_state.label} via {msg.transition.label}',
            node=name,
            details={'transition_id': int(msg.transition.id)},
        ))
