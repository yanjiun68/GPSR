"""Section: Hardware-free terminal presentation contracts and helpers."""

from __future__ import annotations

import json
import math
import textwrap
from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Sequence


# Section: Stable page order shared by runtime code, documentation, and tests.
PAGE_TABS = (
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


def text_value(value: Any, default: str = "-") -> str:
    """Special function: Normalize arbitrary display values into finite text."""
    if value is None:
        return default
    if isinstance(value, float) and not math.isfinite(value):
        return default
    result = str(value)
    return result if result else default


def truncate(value: Any, width: int) -> str:
    """Special function: Truncate a display value with a single ellipsis."""
    text = text_value(value, "")
    if width <= 0:
        return ""
    if len(text) <= width:
        return text
    if width == 1:
        return "…"
    return text[: width - 1] + "…"


def pad(value: Any, width: int, align: str = "left") -> str:
    """Special function: Align a safely truncated table cell."""
    text = truncate(value, width)
    if align == "right":
        return text.rjust(width)
    if align == "center":
        return text.center(width)
    return text.ljust(width)


def wrap_text(value: Any, width: int) -> List[str]:
    """Special function: Wrap long command/transcript text without losing words."""
    if width <= 0:
        return []
    text = text_value(value, "")
    if not text:
        return ["-"]
    return textwrap.wrap(text, width=width, replace_whitespace=True, drop_whitespace=True) or ["-"]


def format_parameters(value: Any) -> str:
    """Special function: Render structured action parameters as stable compact text."""
    if not isinstance(value, Mapping) or not value:
        return "-"

    # Section: JSON encoding keeps nested selectors, filters, booleans, and lists
    # unambiguous while ordinary single-token strings stay easy to scan.
    rendered: List[str] = []
    for key in sorted(value, key=lambda item: str(item)):
        parameter = value[key]
        if isinstance(parameter, str) and parameter and not any(
            character.isspace() for character in parameter
        ):
            encoded = parameter
        else:
            try:
                encoded = json.dumps(
                    parameter,
                    ensure_ascii=False,
                    sort_keys=True,
                    separators=(",", ":"),
                )
            except (TypeError, ValueError):
                encoded = str(parameter)
        rendered.append(f"{key}={encoded}")
    return " ".join(rendered) or "-"


@dataclass(frozen=True)
class Column:
    """Section: Declarative terminal table column contract."""

    key: str
    title: str
    width: int
    min_width: int = 6
    align: str = "left"
    grow: int = 0


def fit_columns(columns: Sequence[Column], available: int) -> List[Column]:
    """Special function: Fit a column contract while preserving minimum widths."""
    if not columns:
        return []
    separators = 2 * (len(columns) - 1)
    usable = max(1, available - separators)
    widths = [column.width for column in columns]
    total = sum(widths)
    if total > usable:
        overflow = total - usable
        shrink_order = sorted(
            range(len(columns)),
            key=lambda index: (columns[index].grow, widths[index] - columns[index].min_width),
            reverse=True,
        )
        while overflow > 0:
            changed = False
            for index in shrink_order:
                if widths[index] > columns[index].min_width:
                    widths[index] -= 1
                    overflow -= 1
                    changed = True
                    if overflow <= 0:
                        break
            if not changed:
                break
        # Section: Very narrow terminals may require going below preferred minima;
        # retaining one character per column is safer than writing past the window.
        while overflow > 0:
            changed = False
            for index in shrink_order:
                if widths[index] > 1:
                    widths[index] -= 1
                    overflow -= 1
                    changed = True
                    if overflow <= 0:
                        break
            if not changed:
                break
    elif total < usable:
        extra = usable - total
        growers = [index for index, column in enumerate(columns) if column.grow > 0] or [len(columns) - 1]
        weights = sum(max(1, columns[index].grow) for index in growers)
        for position, index in enumerate(growers):
            add = extra if position == len(growers) - 1 else (extra * max(1, columns[index].grow)) // weights
            widths[index] += add
            extra -= add
            weights -= max(1, columns[index].grow)
    return [
        Column(column.key, column.title, widths[index], column.min_width, column.align, column.grow)
        for index, column in enumerate(columns)
    ]


def queue_rows(queue: Any) -> List[Dict[str, Any]]:
    """Special function: Produce stable queue table rows from normalized GPSR state."""
    source = queue if isinstance(queue, list) else []
    rows: List[Dict[str, Any]] = []
    for position, raw in enumerate(source):
        if not isinstance(raw, Mapping):
            continue
        try:
            index = int(raw.get("index", position) or 0)
        except (TypeError, ValueError, OverflowError):
            index = position
        try:
            attempt = int(raw.get("attempt", 0) or 0)
        except (TypeError, ValueError, OverflowError):
            attempt = 0
        try:
            retry_limit = int(raw.get("max_retries", 0) or 0)
        except (TypeError, ValueError, OverflowError):
            retry_limit = 0
        rows.append({
            "number": f"{'>' if raw.get('current') else ' '} {index + 1}",
            "domain": raw.get("domain", "OTHER"),
            "skill": raw.get("skill", raw.get("action_type", "-")),
            "target": raw.get("target", "-"),
            "params": format_parameters(
                raw.get("parameters", raw.get("arguments", {}))
            ),
            # Section: Readiness and runtime status are intentionally separate columns.
            "preflight": raw.get("preflight_state", "UNKNOWN"),
            "state": raw.get("task_status", raw.get("state", "IN QUEUE")),
            "attempt": f"{attempt}/{retry_limit}",
            "current": bool(raw.get("current", False)),
        })
    return rows


def validation_rows(queue: Any) -> List[Dict[str, Any]]:
    """Special function: Produce diagnostic rows from normalized preflight fields."""
    source = queue if isinstance(queue, list) else []
    rows: List[Dict[str, Any]] = []
    for position, raw in enumerate(source):
        if not isinstance(raw, Mapping):
            continue
        try:
            index = int(raw.get("index", position) or 0)
        except (TypeError, ValueError, OverflowError):
            index = position
        blocked_by = raw.get("blocked_by", [])
        dependencies = ", ".join(str(item) for item in blocked_by) if isinstance(blocked_by, list) else text_value(blocked_by, "")
        resource = text_value(raw.get("preflight_resource"), "")
        rows.append({
            "number": str(index + 1),
            "skill": raw.get("skill", raw.get("action_type", "-")),
            "readiness": raw.get("preflight_state", "UNKNOWN"),
            "reason": raw.get("preflight_reason", "-"),
            "resource": resource or dependencies or "-",
            "message": raw.get("preflight_message", "-"),
        })
    return rows
