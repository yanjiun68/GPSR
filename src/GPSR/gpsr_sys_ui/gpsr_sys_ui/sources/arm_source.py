from __future__ import annotations

import math
import time
from sensor_msgs.msg import JointState

from ..event_model import MonitorEvent


class ArmSource:
    def __init__(self, node, cache, events, topics, joints, stale_sec: float) -> None:
        self.node = node
        self.cache = cache
        self.events = events
        self.joints = list(joints)
        self.stale_sec = float(stale_sec)
        self.last_update = 0.0
        self.active_topic = ''
        self.subs = [node.create_subscription(JointState, topic, lambda msg, t=topic: self._on_msg(t, msg), 30) for topic in topics]
        self.timer = node.create_timer(0.5, self._health)

    def _on_msg(self, topic: str, msg: JointState) -> None:
        index = {name: i for i, name in enumerate(msg.name)}
        if not any(name in index for name in self.joints):
            return
        rows = []
        for name in self.joints:
            if name not in index:
                continue
            i = index[name]
            pos = msg.position[i] if i < len(msg.position) else float('nan')
            vel = msg.velocity[i] if i < len(msg.velocity) else float('nan')
            effort = msg.effort[i] if i < len(msg.effort) else float('nan')
            rows.append({
                'joint': name,
                'name': name,
                'angle_deg': math.degrees(pos) if math.isfinite(pos) else None,
                'velocity': vel if math.isfinite(vel) else None,
                'effort': effort if math.isfinite(effort) else None,
            })
        self.last_update = time.monotonic()
        self.active_topic = topic
        self.cache.set('arm', {'topic': topic, 'joints': rows, 'feedback_age_sec': 0.0, 'health': 'OK'})

    def _health(self) -> None:
        if not self.last_update:
            self.cache.set('arm', {'topic': self.active_topic, 'joints': [], 'feedback_age_sec': None, 'health': 'UNKNOWN'})
            return
        age = time.monotonic() - self.last_update
        arm = self.cache.get('arm', {}) or {}
        arm['feedback_age_sec'] = round(age, 3)
        if age > self.stale_sec:
            arm['health'] = 'WARN'
            self.events.emit(MonitorEvent('WARN', 'ARM', 'ARM_FEEDBACK_STALE', f'joint feedback age {age:.2f}s', component=self.active_topic))
        else:
            arm['health'] = 'OK'
        self.cache.set('arm', arm)
