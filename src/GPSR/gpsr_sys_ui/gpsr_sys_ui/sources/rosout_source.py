from __future__ import annotations

import re
from rcl_interfaces.msg import Log

from ..event_model import MonitorEvent


PATTERNS = [
    (re.compile(r'No transition matching.*current state active|transition.*not valid.*active', re.I), 'NAV2_DUPLICATE_LIFECYCLE_TRANSITION', 'NAV2', 'ERROR'),
    (re.compile(r'Failed to bring up all requested nodes|bringup.*failed', re.I), 'NAV2_BRINGUP_FAILED', 'NAV2', 'ERROR'),
    (re.compile(r'Failed to make progress|FAILED_TO_MAKE_PROGRESS', re.I), 'NAV2_FAILED_TO_MAKE_PROGRESS', 'NAV2', 'ERROR'),
    (re.compile(r'No valid control|NO_VALID_CONTROL', re.I), 'NAV2_NO_VALID_CONTROL', 'NAV2', 'ERROR'),
    (re.compile(r'No valid path|NO_VALID_PATH', re.I), 'NAV2_NO_VALID_PATH', 'NAV2', 'ERROR'),
    (re.compile(r'hardware error|Hardware Error Status', re.I), 'DYNAMIXEL_HARDWARE_ERROR', 'ARM', 'ERROR'),
]


class RosoutSource:
    def __init__(self, node, cache, events, topic: str) -> None:
        self.events = events
        self.sub = node.create_subscription(Log, topic, self._on_msg, 100)

    def _on_msg(self, msg: Log) -> None:
        text = msg.msg or ''
        for regex, code, subsystem, severity in PATTERNS:
            if regex.search(text):
                self.events.emit(MonitorEvent(severity, subsystem, code, text, node=msg.name))
                break
