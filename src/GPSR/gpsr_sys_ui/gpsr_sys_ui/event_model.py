from __future__ import annotations

from dataclasses import asdict, dataclass, field
from time import time
from typing import Any, Dict, Optional


@dataclass
class MonitorEvent:
    severity: str
    subsystem: str
    event_code: str
    message: str
    component: str = ""
    node: str = ""
    timestamp: float = field(default_factory=time)
    first_seen: float = 0.0
    last_seen: float = 0.0
    repeat_count: int = 1
    details: Dict[str, Any] = field(default_factory=dict)
    gpsr_command: str = ""
    gpsr_skill: str = ""
    yasmin_state: str = ""
    incident_id: str = ""
    parent_event_id: str = ""
    root_cause_event_id: str = ""

    def __post_init__(self) -> None:
        self.severity = str(self.severity or "INFO").upper()
        self.subsystem = str(self.subsystem or "DIAGNOSTICS").upper()
        if not self.first_seen:
            self.first_seen = self.timestamp
        if not self.last_seen:
            self.last_seen = self.timestamp

    def fingerprint(self) -> tuple[str, str, str, str, str]:
        return (self.severity, self.subsystem, self.event_code, self.node, self.message)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)
