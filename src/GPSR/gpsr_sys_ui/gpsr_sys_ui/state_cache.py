from __future__ import annotations

import copy
import threading
from typing import Any, Dict


class StateCache:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._state: Dict[str, Any] = {}

    def set(self, key: str, value: Any) -> None:
        with self._lock:
            self._state[key] = value

    def update_section(self, section: str, values: Dict[str, Any]) -> None:
        with self._lock:
            current = self._state.setdefault(section, {})
            if not isinstance(current, dict):
                current = {}
                self._state[section] = current
            current.update(values)

    def set_nested(self, section: str, key: str, value: Any) -> None:
        with self._lock:
            current = self._state.setdefault(section, {})
            if not isinstance(current, dict):
                current = {}
                self._state[section] = current
            current[key] = value

    def get(self, key: str, default: Any = None) -> Any:
        with self._lock:
            return copy.deepcopy(self._state.get(key, default))

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._state)
