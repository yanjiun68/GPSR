from __future__ import annotations

import math
import time
from sensor_msgs.msg import BatteryState

from ..event_model import MonitorEvent


class EnergySource:
    """Event-only battery observer. There is intentionally no dedicated Energy TUI page."""

    def __init__(self, node, cache, events, topic: str, stale_sec: float, warn_percent: float, critical_percent: float) -> None:
        self.cache = cache
        self.events = events
        self.stale_sec = float(stale_sec)
        self.warn = float(warn_percent)
        self.critical = float(critical_percent)
        self.last_update = 0.0
        self.state = 'UNKNOWN'
        self.sub = node.create_subscription(BatteryState, topic, self._on_msg, 10)
        self.timer = node.create_timer(1.0, self._health)

    def _on_msg(self, msg: BatteryState) -> None:
        self.last_update = time.monotonic()
        raw_pct = float(msg.percentage)
        pct = raw_pct * 100.0 if math.isfinite(raw_pct) and 0.0 <= raw_pct <= 1.0 else None
        voltage = float(msg.voltage) if math.isfinite(float(msg.voltage)) else None
        current = float(msg.current) if math.isfinite(float(msg.current)) else None
        self.cache.set('energy', {'percentage': pct, 'voltage': voltage, 'current': current, 'feedback_age_sec': 0.0})
        if pct is None:
            return
        new_state = 'OK'
        if pct <= self.critical:
            new_state = 'CRITICAL'
        elif pct <= self.warn:
            new_state = 'WARN'
        if new_state != self.state:
            if new_state == 'CRITICAL':
                self.events.emit(MonitorEvent('ERROR', 'ENERGY', 'BATTERY_CRITICAL', f'battery {pct:.1f}%'))
            elif new_state == 'WARN':
                self.events.emit(MonitorEvent('WARN', 'ENERGY', 'BATTERY_LOW', f'battery {pct:.1f}%'))
            elif self.state in {'WARN', 'CRITICAL'}:
                self.events.emit(MonitorEvent('INFO', 'ENERGY', 'BATTERY_LEVEL_RECOVERED', f'battery {pct:.1f}%'))
            self.state = new_state

    def _health(self) -> None:
        if not self.last_update:
            return
        age = time.monotonic() - self.last_update
        energy = self.cache.get('energy', {}) or {}
        energy['feedback_age_sec'] = round(age, 3)
        self.cache.set('energy', energy)
        if age > self.stale_sec:
            self.events.emit(MonitorEvent('WARN', 'ENERGY', 'ENERGY_SENSOR_STALE', f'battery_state age {age:.1f}s'))
