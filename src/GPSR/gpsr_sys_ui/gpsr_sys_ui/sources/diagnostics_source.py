from __future__ import annotations

import time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from ..event_model import MonitorEvent


class DiagnosticsSource:
    def __init__(self, node, cache, events, topic: str) -> None:
        self.cache = cache
        self.events = events
        self.rows = {}
        self.sub = node.create_subscription(DiagnosticArray, topic, self._on_msg, 20)

    def _on_msg(self, msg: DiagnosticArray) -> None:
        now = time.time()
        for status in msg.status:
            severity = {
                DiagnosticStatus.OK: 'OK',
                DiagnosticStatus.WARN: 'WARN',
                DiagnosticStatus.ERROR: 'ERROR',
                DiagnosticStatus.STALE: 'STALE',
            }.get(status.level, 'UNKNOWN')
            self.rows[status.name] = {
                'name': status.name,
                'hardware_id': status.hardware_id,
                'health': severity,
                'message': status.message,
                'last_update': now,
                'values': {kv.key: kv.value for kv in status.values},
            }
            if severity in {'WARN', 'ERROR', 'STALE'}:
                self.events.emit(MonitorEvent(
                    'ERROR' if severity == 'ERROR' else 'WARN',
                    'DIAGNOSTICS',
                    f'DIAGNOSTIC_{severity}',
                    status.message or status.name,
                    component=status.name,
                    details={'hardware_id': status.hardware_id},
                ))
        self.cache.set('diagnostics', list(self.rows.values()))
