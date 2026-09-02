"""Section: Hardware-free terminal presentation contracts and helpers."""

from __future__ import annotations

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
