from __future__ import annotations

import json
from std_msgs.msg import String

from ..event_model import MonitorEvent


class ContextSource:
    """Special function: Retain only the verified phase-supervisor status hook."""

    def __init__(self, node, cache, events, supervisor_topic: str) -> None:
        """Special function: Subscribe to structured supervisor status without guessing GPSR data."""
        self.cache = cache
        self.events = events
        self.supervisor_sub = node.create_subscription(String, supervisor_topic, self._supervisor, 10)

    def _decode(self, msg: String, subsystem: str):
        try:
            value = json.loads(msg.data)
        except json.JSONDecodeError:
            self.events.emit(MonitorEvent('WARN', subsystem, 'INVALID_STATUS_JSON', 'status topic contained invalid JSON'))
            return None
        return value if isinstance(value, dict) else None

    def _supervisor(self, msg: String) -> None:
        """Special function: Cache a validated supervisor JSON object."""
        value = self._decode(msg, 'SUPERVISOR')
        if value is not None:
            self.cache.set('supervisor', value)
