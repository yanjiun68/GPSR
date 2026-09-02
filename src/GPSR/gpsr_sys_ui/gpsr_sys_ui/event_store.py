from __future__ import annotations

import threading
import time
from collections import deque
from typing import Callable, Deque, Dict, List, Optional, Tuple

from .event_model import MonitorEvent


class EventStore:
    def __init__(self, max_events: int = 10000, dedupe_window_sec: float = 2.0,
                 on_event: Optional[Callable[[MonitorEvent], None]] = None) -> None:
        self._events: Deque[MonitorEvent] = deque(maxlen=max(1, int(max_events)))
        self._dedupe_window = max(0.0, float(dedupe_window_sec))
        self._last_by_fingerprint: Dict[Tuple[str, str, str, str, str], MonitorEvent] = {}
        self._lock = threading.RLock()
        self._on_event = on_event

    def emit(self, event: MonitorEvent) -> MonitorEvent:
        now = event.timestamp or time.time()
        event.timestamp = now
        fp = event.fingerprint()
        with self._lock:
            previous = self._last_by_fingerprint.get(fp)
            if previous and (now - previous.last_seen) <= self._dedupe_window:
                previous.last_seen = now
                previous.timestamp = now
                previous.repeat_count += 1
                previous.details.update(event.details)
                emitted = previous
            else:
                event.first_seen = event.first_seen or now
                event.last_seen = now
                self._events.append(event)
                self._last_by_fingerprint[fp] = event
                emitted = event
        if self._on_event:
            self._on_event(emitted)
        return emitted

    def snapshot(self) -> List[dict]:
        with self._lock:
            return [event.to_dict() for event in self._events]
