#!/usr/bin/env python3
"""Colorized, aligned terminal frontend for gpsr_sys_ui.

This module is intentionally presentation-only.  It consumes the monitor core's
normalized JSON state/event topics and never inspects Nav2, hardware, /proc, or
other subsystem interfaces directly.
"""

from __future__ import annotations

import curses
import fnmatch
import json
import math
import threading
import textwrap
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .presentation import (
    PAGE_TABS,
    Column as SharedColumn,
    fit_columns as shared_fit_columns,
    pad as shared_pad,
    queue_rows as shared_queue_rows,
    text_value as shared_text_value,
    truncate as shared_truncate,
    wrap_text as shared_wrap_text,
)

STATE_TOPIC = "/gpsr_system_monitor/state_json"
EVENT_TOPIC = "/gpsr_system_monitor/event_json"


# ---------------------------------------------------------------------------
# Theme
# ---------------------------------------------------------------------------

PAIR_DEFAULT = 1
PAIR_MUTED = 2
PAIR_ACCENT = 3
PAIR_OK = 4
PAIR_WARN = 5
PAIR_ERROR = 6
PAIR_FATAL = 7
PAIR_INFO = 8
PAIR_SELECTED = 9
PAIR_BORDER = 10
PAIR_VALUE = 11


def _init_theme() -> None:
    """Initialize a restrained terminal palette with a basic-color fallback."""
    curses.start_color()
    try:
        curses.use_default_colors()
        bg = -1
    except curses.error:
        bg = curses.COLOR_BLACK

    if curses.COLORS >= 256:
        colors = {
            PAIR_DEFAULT: (252, bg),    # soft white
            PAIR_MUTED: (245, bg),      # secondary text
            PAIR_ACCENT: (39, bg),      # cyan-blue navigation/accent
            PAIR_OK: (40, bg),          # green
            PAIR_WARN: (220, bg),       # amber
            PAIR_ERROR: (203, bg),      # soft red
            PAIR_FATAL: (196, bg),      # strong red
            PAIR_INFO: (81, bg),        # light cyan
            PAIR_SELECTED: (16, 81),    # dark text on cyan selection
            PAIR_BORDER: (240, bg),     # subtle separators
            PAIR_VALUE: (255, bg),      # bright measured values
        }
    else:
        colors = {
            PAIR_DEFAULT: (curses.COLOR_WHITE, bg),
            PAIR_MUTED: (curses.COLOR_WHITE, bg),
            PAIR_ACCENT: (curses.COLOR_CYAN, bg),
            PAIR_OK: (curses.COLOR_GREEN, bg),
            PAIR_WARN: (curses.COLOR_YELLOW, bg),
            PAIR_ERROR: (curses.COLOR_RED, bg),
            PAIR_FATAL: (curses.COLOR_RED, bg),
            PAIR_INFO: (curses.COLOR_CYAN, bg),
            PAIR_SELECTED: (curses.COLOR_BLACK, curses.COLOR_CYAN),
            PAIR_BORDER: (curses.COLOR_WHITE, bg),
            PAIR_VALUE: (curses.COLOR_WHITE, bg),
        }

    for pair_id, (fg, pair_bg) in colors.items():
        curses.init_pair(pair_id, fg, pair_bg)


def _attr(pair: int, *, bold: bool = False, dim: bool = False) -> int:
    value = curses.color_pair(pair)
    if bold:
        value |= curses.A_BOLD
    if dim:
        value |= curses.A_DIM
    return value


def _severity_attr(level: str) -> int:
    level = str(level or "").upper()
    if level == "FATAL":
        return _attr(PAIR_FATAL, bold=True)
    if level == "ERROR":
        return _attr(PAIR_ERROR, bold=True)
    if level == "WARN":
        return _attr(PAIR_WARN, bold=True)
    if level == "INFO":
        return _attr(PAIR_INFO)
    return _attr(PAIR_MUTED)


def _health_attr(value: str) -> int:
    value = str(value or "").upper()
    if value in {"OK", "HEALTHY", "ACTIVE", "RUNNING", "READY", "ONLINE", "SUCCEEDED", "EXECUTING", "ACCEPTED", "DONE", "COMPLETED", "LISTENING", "SPEAKING"}:
        return _attr(PAIR_OK, bold=True)
    if value in {"WARN", "WARNING", "BLOCKED", "STALE", "INACTIVE", "RECOVERING", "RETRYING", "SKIPPED", "CANCELED", "UNKNOWN", "PENDING", "IN QUEUE", "STOP", "SLOWDOWN", "LIMIT", "APPROACH"}:
        return _attr(PAIR_WARN, bold=True)
    if "PERSISTENT" in value or "BLOCKED" in value or "NOT_MOVING" in value:
        return _attr(PAIR_WARN, bold=True)
    if value in {"ERROR", "FAILED", "FAIL", "MISSING", "OFFLINE", "FATAL", "ABORTED"}:
        return _attr(PAIR_ERROR, bold=True)
    return _attr(PAIR_MUTED)


# ---------------------------------------------------------------------------
# Alignment helpers
# ---------------------------------------------------------------------------


def _text(value: Any, default: str = "-") -> str:
    if value is None:
        return default
    if isinstance(value, float) and not math.isfinite(value):
        return default
    result = str(value)
    return result if result else default


def _truncate(text: Any, width: int) -> str:
    value = _text(text, "")
    if width <= 0:
        return ""
    if len(value) <= width:
        return value
    if width == 1:
        return "…"
    return value[: width - 1] + "…"


def _pad(text: Any, width: int, align: str = "left") -> str:
    value = _truncate(text, width)
    if align == "right":
        return value.rjust(width)
    if align == "center":
        return value.center(width)
    return value.ljust(width)


def _wrap_text(value: Any, width: int) -> List[str]:
    """Special function: Wrap long command/transcript text without losing words."""
    if width <= 0:
        return []
    text = _text(value, "")
    if not text:
        return ["-"]
    return textwrap.wrap(text, width=width, replace_whitespace=True, drop_whitespace=True) or ["-"]


def _queue_rows(queue: Any) -> List[Dict[str, Any]]:
    """Special function: Produce stable queue table rows from normalized GPSR state."""
    rows: List[Dict[str, Any]] = []
    for position, raw in enumerate(_as_rows(queue)):
        index = int(raw.get("index", position) or 0)
        attempt = int(raw.get("attempt", 0) or 0)
        retry_limit = int(raw.get("max_retries", 0) or 0)
        rows.append({
            "number": f"{'>' if raw.get('current') else ' '} {index + 1}",
            "domain": raw.get("domain", "OTHER"),
            "skill": raw.get("skill", raw.get("action_type", "-")),
            "target": raw.get("target", "-"),
            "state": raw.get("state", "UNKNOWN"),
            "attempt": f"{attempt}/{retry_limit}",
            "current": bool(raw.get("current", False)),
        })
    return rows


@dataclass(frozen=True)
class Column:
    key: str
    title: str
    width: int
    min_width: int = 6
    align: str = "left"
    grow: int = 0


def _fit_columns(columns: Sequence[Column], available: int) -> List[Column]:
    """Fit a column contract to the terminal while preserving alignment."""
    if not columns:
        return []
    separators = 2 * (len(columns) - 1)
    usable = max(1, available - separators)
    widths = [c.width for c in columns]

    total = sum(widths)
    if total > usable:
        overflow = total - usable
        shrink_order = sorted(
            range(len(columns)),
            key=lambda i: (columns[i].grow, widths[i] - columns[i].min_width),
            reverse=True,
        )
        while overflow > 0:
            changed = False
            for i in shrink_order:
                if widths[i] > columns[i].min_width:
                    widths[i] -= 1
                    overflow -= 1
                    changed = True
                    if overflow <= 0:
                        break
            if not changed:
                break
    elif total < usable:
        extra = usable - total
        growers = [i for i, c in enumerate(columns) if c.grow > 0]
        if not growers:
            growers = [len(columns) - 1]
        weights = sum(max(1, columns[i].grow) for i in growers)
        for pos, i in enumerate(growers):
            if pos == len(growers) - 1:
                add = extra
            else:
                add = (extra * max(1, columns[i].grow)) // weights
            widths[i] += add
            extra -= add
            weights -= max(1, columns[i].grow)

    return [
        Column(c.key, c.title, widths[i], c.min_width, c.align, c.grow)
        for i, c in enumerate(columns)
    ]


# Section: Bind runtime rendering to the same hardware-free helpers exercised by tests.
Column = SharedColumn
_fit_columns = shared_fit_columns
_pad = shared_pad
_queue_rows = shared_queue_rows
_text = shared_text_value
_truncate = shared_truncate
_wrap_text = shared_wrap_text


# ---------------------------------------------------------------------------
# State compatibility helpers
# ---------------------------------------------------------------------------


def _dig(data: Any, *paths: str, default: Any = None) -> Any:
    """Return the first matching dotted path from a dict-like state snapshot."""
    if not isinstance(data, Mapping):
        return default
    for path in paths:
        cur: Any = data
        ok = True
        for part in path.split("."):
            if isinstance(cur, Mapping) and part in cur:
                cur = cur[part]
            else:
                ok = False
                break
        if ok:
            return cur
    return default


def _as_rows(value: Any) -> List[Dict[str, Any]]:
    if isinstance(value, list):
        return [x for x in value if isinstance(x, Mapping)]
    if isinstance(value, Mapping):
        rows: List[Dict[str, Any]] = []
        for key, item in value.items():
            if isinstance(item, Mapping):
                row = dict(item)
                row.setdefault("name", key)
                rows.append(row)
        return rows
    return []


class MonitorTuiNode(Node):
    """ROS subscriber node feeding a presentation-only snapshot cache."""

    def __init__(self) -> None:
        super().__init__("gpsr_sys_ui_tui")
        self.declare_parameter("state_topic", STATE_TOPIC)
        self.declare_parameter("event_topic", EVENT_TOPIC)
        state_topic = self.get_parameter("state_topic").value
        event_topic = self.get_parameter("event_topic").value
        self._lock = threading.Lock()
        self.state: Dict[str, Any] = {}
        self.events: deque[Dict[str, Any]] = deque(maxlen=10000)
        self.last_state_time = 0.0
        self.create_subscription(String, state_topic, self._on_state, 10)
        self.create_subscription(String, event_topic, self._on_event, 50)

    def _on_state(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        if not isinstance(payload, Mapping):
            return
        with self._lock:
            self.state = dict(payload)
            self.last_state_time = time.monotonic()

    def _on_event(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        if not isinstance(payload, Mapping):
            return
        with self._lock:
            self.events.append(dict(payload))

    def snapshot(self) -> Tuple[Dict[str, Any], List[Dict[str, Any]], float]:
        with self._lock:
            return dict(self.state), list(self.events), self.last_state_time


# ---------------------------------------------------------------------------
# Curses application
# ---------------------------------------------------------------------------


class MonitorTui:
    # Section: Keep the page contract visible here for operators and package identity tests.
    TABS = (
        "Overview",
        "GPSR",
        "Voice",
        "Nodes",
        "Lifecycle",
        "Nav2",
        "Arm",
        "Events",
        "System Telemetry",
    )

    SUBSYSTEMS = ("", "GPSR", "NAV2", "ARM", "VISION", "VOICE", "LIFECYCLE", "SYSTEM", "ENERGY", "DIAGNOSTICS")
    SEVERITY_RANK = {"DEBUG": 0, "INFO": 1, "WARN": 2, "ERROR": 3, "FATAL": 4}

    def __init__(self, node: MonitorTuiNode) -> None:
        self.node = node
        self.tab = 0
        self.paused = False
        self.frozen: Optional[Tuple[Dict[str, Any], List[Dict[str, Any]], float]] = None
        self.min_severity = "DEBUG"
        self.subsystem_index = 0
        self.search_text = ""
        self.node_filter = ""
        self.event_offset = 0
        self.status_message = ""

    # ----- drawing primitives ------------------------------------------------

    @staticmethod
    def _safe_add(win: "curses._CursesWindow", y: int, x: int, text: str, attr: int = 0, max_width: Optional[int] = None) -> None:
        h, w = win.getmaxyx()
        if y < 0 or y >= h or x < 0 or x >= w:
            return
        width = w - x - 1 if max_width is None else min(max_width, w - x - 1)
        if width <= 0:
            return
        try:
            win.addnstr(y, x, text, width, attr)
        except curses.error:
            pass

    def _separator(self, win: "curses._CursesWindow", y: int, char: str = "─") -> int:
        _, w = win.getmaxyx()
        self._safe_add(win, y, 0, char * max(0, w - 1), _attr(PAIR_BORDER))
        return y + 1

    def _section(self, win: "curses._CursesWindow", y: int, title: str, suffix: str = "") -> int:
        _, w = win.getmaxyx()
        label = f" {title.upper()} "
        self._safe_add(win, y, 0, label, _attr(PAIR_ACCENT, bold=True))
        if suffix:
            self._safe_add(win, y, len(label) + 1, suffix, _attr(PAIR_MUTED), max_width=max(0, w - len(label) - 2))
        return y + 1

    def _metric(self, win: "curses._CursesWindow", y: int, label: str, value: Any, unit: str = "", status: str = "") -> int:
        _, w = win.getmaxyx()
        label_w = min(26, max(16, w // 4))
        value_text = _text(value)
        if unit and value_text != "-":
            value_text = f"{value_text} {unit}"
        self._safe_add(win, y, 2, _pad(label, label_w), _attr(PAIR_MUTED))
        self._safe_add(win, y, 2 + label_w + 1, _pad(value_text, 18), _attr(PAIR_VALUE, bold=True))
        if status:
            self._safe_add(win, y, 2 + label_w + 21, _truncate(status, max(0, w - label_w - 25)), _health_attr(status))
        return y + 1

    def _draw_table(
        self,
        win: "curses._CursesWindow",
        y: int,
        rows: Sequence[Mapping[str, Any]],
        columns: Sequence[Column],
        *,
        row_style=None,
        max_rows: Optional[int] = None,
    ) -> int:
        h, w = win.getmaxyx()
        fitted = _fit_columns(columns, max(10, w - 2))
        x = 1
        for idx, col in enumerate(fitted):
            self._safe_add(win, y, x, _pad(col.title, col.width, col.align), _attr(PAIR_MUTED, bold=True), col.width)
            x += col.width + (2 if idx < len(fitted) - 1 else 0)
        y += 1
        self._safe_add(win, y, 1, "─" * max(0, min(w - 3, sum(c.width for c in fitted) + 2 * (len(fitted) - 1))), _attr(PAIR_BORDER))
        y += 1

        room = max(0, h - y - 2)
        if max_rows is not None:
            room = min(room, max_rows)
        for row in rows[:room]:
            x = 1
            base_attr = _attr(PAIR_DEFAULT)
            if row_style is not None:
                base_attr = row_style(row)
            for idx, col in enumerate(fitted):
                value = row.get(col.key, "-")
                attr = base_attr
                if col.key in {"health", "state", "actual", "expected", "status"}:
                    attr = _health_attr(_text(value))
                elif col.key in {"severity", "level"}:
                    attr = _severity_attr(_text(value))
                self._safe_add(win, y, x, _pad(value, col.width, col.align), attr, col.width)
                x += col.width + (2 if idx < len(fitted) - 1 else 0)
            y += 1
        return y

    # ----- chrome ------------------------------------------------------------

    def _draw_header(self, win: "curses._CursesWindow", state_age: float) -> int:
        _, w = win.getmaxyx()
        live = "PAUSED" if self.paused else ("LIVE" if state_age < 2.5 else "STALE")
        live_attr = _attr(PAIR_WARN, bold=True) if live in {"PAUSED", "STALE"} else _attr(PAIR_OK, bold=True)

        self._safe_add(win, 0, 1, "GPSR SYS UI", _attr(PAIR_VALUE, bold=True))
        self._safe_add(win, 0, 23, live, live_attr)

        x = 1
        y = 1
        for idx, title in enumerate(self.TABS):
            token = f" {idx + 1}:{title} "
            if x + len(token) >= w - 2:
                break
            attr = _attr(PAIR_SELECTED, bold=True) if idx == self.tab else _attr(PAIR_MUTED)
            self._safe_add(win, y, x, token, attr)
            x += len(token) + 1
        return self._separator(win, 2)

    def _draw_footer(self, win: "curses._CursesWindow") -> None:
        h, w = win.getmaxyx()
        y = h - 1
        keys = "1–9 pages  [ ] switch  A all  W warn+  E errors  C subsystem  / search  N node  X clear  P pause  q quit"
        self._safe_add(win, y, 0, _truncate(keys, w - 1), _attr(PAIR_MUTED))
        if self.status_message and h >= 2:
            self._safe_add(win, h - 2, 0, _truncate(self.status_message, w - 1), _attr(PAIR_INFO))

    # ----- page renderers ----------------------------------------------------

    def _overview(self, win, y: int, state: Mapping[str, Any], events: Sequence[Mapping[str, Any]]) -> None:
        context = _dig(state, "gpsr", default={}) or {}
        y = self._section(win, y, "Mission context")
        y = self._metric(win, y, "Command", _dig(context, "command_id", default="-"))
        y = self._metric(win, y, "Current skill", _dig(context, "current_skill", default="-"))
        y = self._metric(win, y, "Scheduler state", _dig(context, "scheduler_state", default="-"))
        y = self._metric(win, y, "Queue state", _dig(context, "queue_state", default="-"), status=_text(_dig(context, "queue_state", default=""), ""))
        completed_count = _dig(context, "completed_count", default=0)
        queue_length = _dig(context, "queue_length", default=0)
        y = self._metric(win, y, "Queue progress", f"{completed_count}/{queue_length} completed")
        retry = _dig(context, "retry_attempt", default=None)
        retry_limit = _dig(context, "retry_limit", default=None)
        if retry is not None or retry_limit is not None:
            y = self._metric(win, y, "Retry", f"{_text(retry, '0')}/{_text(retry_limit, '-')}")

        # Section: Keep the accepted task sequence visible on the default monitor page.
        task_rows = _queue_rows(_dig(context, "queue", default=[]))
        counts = _dig(context, "task_status_counts", default={}) or {}
        count_text = "  ".join(
            f"{label}:{int(counts.get(label, 0) or 0)}"
            for label in ("IN QUEUE", "RUNNING", "COMPLETED", "ABORTED")
        )
        y = self._section(win, y + 1, "ActionQueue tasks", count_text)
        y = self._draw_table(
            win,
            y,
            task_rows,
            [
                Column("number", "#", 5, 3),
                Column("skill", "TASK", 30, 12, grow=2),
                Column("target", "TARGET", 28, 10, grow=2),
                Column("state", "STATUS", 12, 8),
            ],
            max_rows=8,
            row_style=lambda row: _attr(PAIR_SELECTED, bold=True) if row.get("current") else _attr(PAIR_DEFAULT),
        )

        y += 1
        y = self._section(win, y, "Voice")
        voice = _dig(state, "voice", default={}) or {}
        stt = _dig(voice, "stt", default={}) or {}
        tts = _dig(voice, "tts", default={}) or {}
        y = self._metric(win, y, "STT", _dig(stt, "state", default="MISSING"), status=_text(_dig(stt, "health", default="MISSING")))
        y = self._metric(win, y, "TTS", _dig(tts, "state", default="MISSING"), status=_text(_dig(tts, "health", default="MISSING")))

        y += 1
        y = self._section(win, y, "System health", "Detailed host telemetry is on page 9")
        components = _as_rows(_dig(state, "components", default=[]))
        counts: Dict[str, int] = {}
        for comp in components:
            health = _text(comp.get("health", "UNKNOWN")).upper()
            counts[health] = counts.get(health, 0) + 1
        y = self._metric(win, y, "Observed components", len(components) if components else _dig(state, "node_count", default="-"))
        summary = "  ".join(f"{k}:{v}" for k, v in sorted(counts.items())) if counts else "-"
        y = self._metric(win, y, "Health summary", summary)

        y += 1
        y = self._section(win, y, "Active / recent failures")
        bad = [e for e in events if _text(e.get("severity", e.get("level", "INFO"))).upper() in {"WARN", "ERROR", "FATAL"}]
        rows = []
        for e in reversed(bad[-12:]):
            rows.append({
                "time": e.get("timestamp", e.get("time", "-")),
                "severity": e.get("severity", e.get("level", "INFO")),
                "subsystem": e.get("subsystem", "-"),
                "event": e.get("event_code", e.get("event", "-")),
                "message": e.get("message", "-"),
            })
        self._draw_table(win, y, rows, [
            Column("time", "TIME", 12, 8),
            Column("severity", "LEVEL", 7, 5),
            Column("subsystem", "SYSTEM", 11, 7),
            Column("event", "EVENT", 30, 14, grow=1),
            Column("message", "MESSAGE", 36, 14, grow=3),
        ])

    def _gpsr(self, win, y: int, state: Mapping[str, Any]) -> None:
        """Special function: Render command provenance and the complete action queue."""
        gpsr = _dig(state, "gpsr", default={}) or {}
        y = self._section(win, y, "GPSR command")
        y = self._metric(win, y, "Command ID", _dig(gpsr, "command_id", default="-"))
        y = self._metric(win, y, "Scheduler", _dig(gpsr, "scheduler_state", default="-"))
        y = self._metric(win, y, "Queue", _dig(gpsr, "queue_state", default="-"), status=_text(_dig(gpsr, "queue_state", default=""), ""))
        completed_count = _dig(gpsr, "completed_count", default=0)
        queue_length = _dig(gpsr, "queue_length", default=0)
        y = self._metric(win, y, "Queue progress", f"{completed_count}/{queue_length} completed")
        y = self._metric(win, y, "Current domain", _dig(gpsr, "current_domain", default="-"))
        y = self._metric(win, y, "Current skill", _dig(gpsr, "current_skill", default="-"))
        y = self._metric(win, y, "Retry", f"{_text(_dig(gpsr, 'retry_attempt', default=0))}/{_text(_dig(gpsr, 'retry_limit', default=0))}")

        # Section: Wrap both scheduler-provided command forms without parsing either string.
        _, width = win.getmaxyx()
        content_width = max(12, width - 4)
        y = self._section(win, y + 1, "Original command")
        for line in _wrap_text(_dig(gpsr, "raw_command", default=""), content_width):
            self._safe_add(win, y, 2, line, _attr(PAIR_VALUE), content_width)
            y += 1
        y = self._section(win, y + 1, "Normalized command")
        for line in _wrap_text(_dig(gpsr, "normalized_command", default=""), content_width):
            self._safe_add(win, y, 2, line, _attr(PAIR_VALUE), content_width)
            y += 1

        y = self._section(win, y + 1, "Action queue", "current action is marked with >")
        self._draw_table(
            win,
            y,
            _queue_rows(_dig(gpsr, "queue", default=[])),
            [
                Column("number", "#", 5, 3),
                Column("domain", "DOMAIN", 14, 8),
                Column("skill", "SKILL", 28, 12, grow=2),
                Column("target", "TARGET", 28, 10, grow=2),
                Column("state", "STATUS", 12, 8),
                Column("attempt", "ATTEMPT", 9, 7, align="right"),
            ],
            row_style=lambda row: _attr(PAIR_SELECTED, bold=True) if row.get("current") else _attr(PAIR_DEFAULT),
        )

    def _voice(self, win, y: int, state: Mapping[str, Any]) -> None:
        """Special function: Render STT and TTS health/activity as independent components."""
        voice = _dig(state, "voice", default={}) or {}
        stt = _dig(voice, "stt", default={}) or {}
        tts = _dig(voice, "tts", default={}) or {}
        transcript = _dig(voice, "transcript", default={}) or {}
        y = self._section(win, y, "Speech to text", "/voice_stt")
        y = self._metric(win, y, "Presence", "PRESENT" if stt.get("present") else "MISSING", status=_text(stt.get("health", "MISSING")))
        y = self._metric(win, y, "Activity", stt.get("state", "MISSING"), status=_text(stt.get("state", "MISSING")))
        y = self._metric(win, y, "Diagnostic", stt.get("message", "-"))
        y = self._section(win, y + 1, "Text to speech", "/voice_tts")
        y = self._metric(win, y, "Presence", "PRESENT" if tts.get("present") else "MISSING", status=_text(tts.get("health", "MISSING")))
        y = self._metric(win, y, "Activity", tts.get("state", "MISSING"), status=_text(tts.get("state", "MISSING")))
        y = self._metric(win, y, "Diagnostic", tts.get("message", "-"))

        # Section: Display the actual transcript and age without treating old text as stale health.
        age = transcript.get("age_sec")
        age_text = f"{float(age):.1f}s" if isinstance(age, (int, float)) else "-"
        y = self._section(win, y + 1, "Latest STT transcript", f"received {age_text} ago")
        _, width = win.getmaxyx()
        content_width = max(12, width - 4)
        for line in _wrap_text(transcript.get("text", ""), content_width):
            self._safe_add(win, y, 2, line, _attr(PAIR_VALUE, bold=True), content_width)
            y += 1

    def _nodes(self, win, y: int, state: Mapping[str, Any]) -> None:
        y = self._section(win, y, "ROS nodes", "graph-discovered and expected components")
        rows = _as_rows(_dig(state, "nodes", "components", default=[]))
        normalized = []
        for r in rows:
            normalized.append({
                "name": r.get("node_name", r.get("name", r.get("component_id", "-"))),
                "health": r.get("health", "UNKNOWN"),
                "state": r.get("lifecycle_state", r.get("state", "-")),
                "age": r.get("age_sec", r.get("last_update_age_sec", "-")),
                "message": r.get("message", ""),
            })
        self._draw_table(win, y, normalized, [
            Column("name", "NODE", 42, 18, grow=2),
            Column("health", "HEALTH", 10, 8),
            Column("state", "STATE", 14, 9),
            Column("age", "AGE(s)", 8, 6, align="right"),
            Column("message", "MESSAGE", 42, 14, grow=3),
        ])

    def _lifecycle(self, win, y: int, state: Mapping[str, Any]) -> None:
        y = self._section(win, y, "Lifecycle", "actual state compared with context-aware expected state")
        rows = _as_rows(_dig(state, "lifecycle", "lifecycle_nodes", default=[]))
        normalized = []
        for r in rows:
            normalized.append({
                "name": r.get("node_name", r.get("name", "-")),
                "actual": r.get("actual", r.get("lifecycle_state", r.get("state", "-"))),
                "expected": r.get("expected", r.get("expected_lifecycle_state", "-")),
                "health": r.get("health", "UNKNOWN"),
                "message": r.get("message", ""),
            })
        self._draw_table(win, y, normalized, [
            Column("name", "NODE", 42, 18, grow=2),
            Column("actual", "ACTUAL", 14, 9),
            Column("expected", "EXPECTED", 14, 9),
            Column("health", "HEALTH", 10, 8),
            Column("message", "MESSAGE", 38, 12, grow=3),
        ])

    def _nav2(self, win, y: int, state: Mapping[str, Any]) -> None:
        nav = _dig(state, "nav2", "navigation", default={}) or {}
        y = self._section(win, y, "Nav2", "controller → collision monitor → base / odometry")
        y = self._metric(win, y, "NavigateToPose", _dig(nav, "navigate_status", "status", default="-"), status=_text(_dig(nav, "navigate_status", "status", default=""), ""))
        y = self._metric(win, y, "Collision action", _dig(nav, "collision_action", "collision.action", default="-"), status=_text(_dig(nav, "collision_action", "collision.action", default=""), ""))
        y = self._metric(win, y, "Collision polygon", _dig(nav, "collision_polygon", "collision.polygon", default="-"))
        y += 1
        y = self._section(win, y, "Velocity chain")
        y = self._metric(win, y, "Pre-collision linear", _dig(nav, "cmd_vel_pre_linear", "pre.linear", default="-"), "m/s")
        y = self._metric(win, y, "Post-collision linear", _dig(nav, "cmd_vel_post_linear", "post.linear", default="-"), "m/s")
        y = self._metric(win, y, "Odometry linear", _dig(nav, "odom_linear", "odom.linear", default="-"), "m/s")
        y = self._metric(win, y, "Classification", _dig(nav, "classification", default="-"), status=_text(_dig(nav, "classification", default=""), ""))
        y += 1
        y = self._section(win, y, "Navigation components")
        comps = _as_rows(_dig(nav, "components", default=[]))
        self._draw_table(win, y, comps, [
            Column("name", "COMPONENT", 36, 16, grow=1),
            Column("health", "HEALTH", 10, 8),
            Column("state", "STATE", 16, 10),
            Column("message", "MESSAGE", 48, 14, grow=3),
        ])

    def _arm(self, win, y: int, state: Mapping[str, Any]) -> None:
        arm = _dig(state, "arm", default={}) or {}
        y = self._section(win, y, "RX200 arm", "actual joint feedback")
        y = self._metric(win, y, "Feedback age", _dig(arm, "feedback_age_sec", default="-"), "s", _text(_dig(arm, "health", default=""), ""))
        y += 1
        rows = _as_rows(_dig(arm, "joints", "joint_states", default=[]))
        normalized = []
        for r in rows:
            normalized.append({
                "joint": r.get("joint", r.get("name", "-")),
                "angle": r.get("angle_deg", r.get("position_deg", r.get("position", "-"))),
                "velocity": r.get("velocity", "-"),
                "effort": r.get("effort", "-"),
                "health": r.get("health", "OK"),
            })
        self._draw_table(win, y, normalized, [
            Column("joint", "JOINT", 22, 12, grow=1),
            Column("angle", "ANGLE(deg)", 14, 10, align="right"),
            Column("velocity", "VELOCITY(rad/s)", 18, 12, align="right"),
            Column("effort", "EFFORT", 12, 8, align="right"),
            Column("health", "HEALTH", 10, 8),
        ])

    def _event_filter(self, event: Mapping[str, Any]) -> bool:
        sev = _text(event.get("severity", event.get("level", "INFO"))).upper()
        if self.SEVERITY_RANK.get(sev, 1) < self.SEVERITY_RANK[self.min_severity]:
            return False
        subsystem = self.SUBSYSTEMS[self.subsystem_index]
        if subsystem and _text(event.get("subsystem", "")).upper() != subsystem:
            return False
        if self.node_filter:
            node = _text(event.get("node", event.get("node_name", "")), "")
            if not fnmatch.fnmatch(node, self.node_filter):
                return False
        if self.search_text:
            blob = " ".join(_text(v, "") for v in event.values()).lower()
            if self.search_text.lower() not in blob:
                return False
        return True

    def _events(self, win, y: int, events: Sequence[Mapping[str, Any]]) -> None:
        subsystem = self.SUBSYSTEMS[self.subsystem_index] or "ALL"
        filter_line = f"severity={self.min_severity}+   subsystem={subsystem}   node={self.node_filter or 'ALL'}   search={self.search_text or '-'}"
        y = self._section(win, y, "Event history", filter_line)
        filtered = [e for e in events if self._event_filter(e)]
        y = self._metric(win, y, "Visible / received", f"{len(filtered)} / {len(events)}")
        rows = []
        for e in reversed(filtered):
            rows.append({
                "time": e.get("timestamp", e.get("time", "-")),
                "severity": e.get("severity", e.get("level", "INFO")),
                "subsystem": e.get("subsystem", "-"),
                "node": e.get("node", e.get("node_name", "-")),
                "event": e.get("event_code", e.get("event", "-")),
                "message": (f"{e.get('message', '-')}{' x' + str(e.get('repeat_count')) if int(e.get('repeat_count', 1) or 1) > 1 else ''}"),
            })
        if self.event_offset:
            rows = rows[self.event_offset :]
        self._draw_table(win, y + 1, rows, [
            Column("time", "TIME", 12, 8),
            Column("severity", "LEVEL", 7, 5),
            Column("subsystem", "SYSTEM", 11, 7),
            Column("node", "NODE", 28, 12, grow=1),
            Column("event", "EVENT", 30, 14, grow=1),
            Column("message", "MESSAGE", 44, 14, grow=4),
        ])

    def _system(self, win, y: int, state: Mapping[str, Any]) -> None:
        sys = _dig(state, "system", "telemetry", default={}) or {}
        y = self._section(win, y, "System telemetry", "Linux /proc + /sys + nvidia-smi")

        y = self._section(win, y + 1, "CPU")
        y = self._metric(win, y, "Utilization", _dig(sys, "system_cpu_percent", "cpu_percent", default="-"), "%")
        y = self._metric(win, y, "Busiest core", _dig(sys, "system_cpu_core_max_percent", "cpu_core_max_percent", default="-"), "%")
        y = self._metric(win, y, "Cores >90%", _dig(sys, "system_cpu_cores_over_90", "cpu_cores_over_90", default="-"))
        load = "/".join(_text(_dig(sys, p, default="-")) for p in ("system_load_1m", "system_load_5m", "system_load_15m"))
        y = self._metric(win, y, "Load 1/5/15m", load)
        y = self._metric(win, y, "CPU temperature", _dig(sys, "system_cpu_temp_c", "cpu_temp_c", default="-"), "°C")

        y = self._section(win, y + 1, "Memory")
        y = self._metric(win, y, "RAM used", _dig(sys, "system_ram_percent", "ram_percent", default="-"), "%")
        y = self._metric(win, y, "RAM available", _dig(sys, "system_ram_available_gb", "ram_available_gb", default="-"), "GB")
        y = self._metric(win, y, "Swap used", _dig(sys, "system_swap_percent", "swap_percent", default="-"), "%")

        y = self._section(win, y + 1, "Disk")
        y = self._metric(win, y, "Disk used", _dig(sys, "system_disk_percent", "disk_percent", default="-"), "%")
        y = self._metric(win, y, "Disk available", _dig(sys, "system_disk_available_gb", "disk_available_gb", default="-"), "GB")

        y = self._section(win, y + 1, "NVIDIA GPU")
        y = self._metric(win, y, "GPU utilization", _dig(sys, "gpu_percent", "gpu_utilization", default="-"), "%")
        used = _dig(sys, "gpu_memory_used_mb", "vram_used_mb", default="-")
        total = _dig(sys, "gpu_memory_total_mb", "vram_total_mb", default="-")
        y = self._metric(win, y, "VRAM used / total", f"{_text(used)} / {_text(total)}", "MB")
        y = self._metric(win, y, "GPU temperature", _dig(sys, "gpu_temperature_c", "gpu_temp_c", default="-"), "°C")
        self._metric(win, y, "GPU power", _dig(sys, "gpu_power_w", default="-"), "W")

    # ----- input -------------------------------------------------------------

    def _prompt(self, win: "curses._CursesWindow", label: str, initial: str = "") -> Optional[str]:
        h, w = win.getmaxyx()
        curses.echo()
        curses.curs_set(1)
        try:
            prompt = f"{label}: "
            self._safe_add(win, h - 2, 0, " " * max(0, w - 1), _attr(PAIR_DEFAULT))
            self._safe_add(win, h - 2, 0, prompt, _attr(PAIR_ACCENT, bold=True))
            if initial:
                self._safe_add(win, h - 2, len(prompt), initial, _attr(PAIR_VALUE))
            win.move(h - 2, min(w - 2, len(prompt) + len(initial)))
            raw = win.getstr(h - 2, min(w - 2, len(prompt) + len(initial)), max(1, w - len(prompt) - len(initial) - 2))
            suffix = raw.decode("utf-8", errors="replace")
            return initial + suffix
        except curses.error:
            return None
        finally:
            curses.noecho()
            curses.curs_set(0)

    def handle_key(self, win, key: int) -> bool:
        if ord("1") <= key <= ord("9"):
            self.tab = key - ord("1")
            self.event_offset = 0
        elif key in (ord("["), curses.KEY_LEFT):
            self.tab = (self.tab - 1) % len(self.TABS)
            self.event_offset = 0
        elif key in (ord("]"), curses.KEY_RIGHT):
            self.tab = (self.tab + 1) % len(self.TABS)
            self.event_offset = 0
        elif key in (ord("a"), ord("A")):
            self.min_severity = "DEBUG"
        elif key in (ord("w"), ord("W")):
            self.min_severity = "WARN"
        elif key in (ord("e"), ord("E")):
            self.min_severity = "ERROR"
        elif key in (ord("c"), ord("C")):
            self.subsystem_index = (self.subsystem_index + 1) % len(self.SUBSYSTEMS)
        elif key == ord("/"):
            value = self._prompt(win, "Search")
            if value is not None:
                self.search_text = value.strip()
        elif key in (ord("n"), ord("N")):
            value = self._prompt(win, "Node glob", self.node_filter)
            if value is not None:
                self.node_filter = value.strip()
        elif key in (ord("x"), ord("X")):
            self.min_severity = "DEBUG"
            self.subsystem_index = 0
            self.search_text = ""
            self.node_filter = ""
            self.event_offset = 0
        elif key in (ord("p"), ord("P")):
            self.paused = not self.paused
            if self.paused:
                self.frozen = self.node.snapshot()
            else:
                self.frozen = None
        elif key == curses.KEY_UP and self.tab == 7:
            self.event_offset += 1
        elif key == curses.KEY_DOWN and self.tab == 7:
            self.event_offset = max(0, self.event_offset - 1)
        elif key in (ord("q"), ord("Q")):
            return False
        return True

    def draw(self, win: "curses._CursesWindow") -> None:
        win.erase()
        state, events, last_state = self.frozen if self.paused and self.frozen else self.node.snapshot()
        age = time.monotonic() - last_state if last_state else float("inf")
        y = self._draw_header(win, age)
        if self.tab == 0:
            self._overview(win, y, state, events)
        elif self.tab == 1:
            self._gpsr(win, y, state)
        elif self.tab == 2:
            self._voice(win, y, state)
        elif self.tab == 3:
            self._nodes(win, y, state)
        elif self.tab == 4:
            self._lifecycle(win, y, state)
        elif self.tab == 5:
            self._nav2(win, y, state)
        elif self.tab == 6:
            self._arm(win, y, state)
        elif self.tab == 7:
            self._events(win, y, events)
        elif self.tab == 8:
            self._system(win, y, state)
        self._draw_footer(win)
        win.refresh()

    def run(self, win: "curses._CursesWindow") -> None:
        _init_theme()
        curses.curs_set(0)
        win.keypad(True)
        win.timeout(200)
        running = True
        while running and rclpy.ok():
            self.draw(win)
            key = win.getch()
            if key != -1:
                running = self.handle_key(win, key)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MonitorTuiNode()
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()
    try:
        curses.wrapper(MonitorTui(node).run)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
