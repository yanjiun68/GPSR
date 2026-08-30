"""Section: YASMIN blackboard compatibility helpers."""

# Section: Blackboard compatibility access helpers.

from typing import Any


def bb_get(blackboard: Any, key: str, default: Any = None) -> Any:
    """Special function: Read a blackboard key with a default value."""
    return blackboard[key] if blackboard is not None and key in blackboard else default
