from __future__ import annotations

from typing import Iterable, Optional

TWIST_TYPE = 'geometry_msgs/msg/Twist'
TWIST_STAMPED_TYPE = 'geometry_msgs/msg/TwistStamped'
SUPPORTED_VELOCITY_TYPES = {TWIST_TYPE, TWIST_STAMPED_TYPE}


def resolve_velocity_type(type_names: Iterable[str]) -> Optional[str]:
    """Return one supported type only when the graph is unambiguous.

    A topic advertising zero types, multiple types, or one unsupported type returns
    None. The caller decides whether that means wait, warn, or reject.
    """
    unique = list(dict.fromkeys(str(x) for x in type_names))
    if len(unique) != 1:
        return None
    return unique[0] if unique[0] in SUPPORTED_VELOCITY_TYPES else None
