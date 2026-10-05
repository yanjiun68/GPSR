"""Pure costmap coordinate and occupancy helpers for deterministic tests."""

# Section: Standard-library imports used by pure geometry helpers.
from dataclasses import dataclass
import math
from typing import Optional, Tuple


# Section: Nav2 cost value conventions.
NO_INFORMATION = 255
DEFAULT_LETHAL_COST = 253


# Section: Minimal immutable costmap representation independent of ROS messages.
@dataclass(frozen=True)
class CostmapSnapshot:
    """Store the fields required to evaluate one Nav2 costmap cell."""

    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: Tuple[int, ...]


# Section: Costmap construction from a Nav2 Costmap message.
def snapshot_from_message(message) -> CostmapSnapshot:
    """Convert a ROS costmap message into a pure, testable snapshot."""

    origin = message.metadata.origin
    quaternion = origin.orientation
    yaw = math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )
    return CostmapSnapshot(
        resolution=float(message.metadata.resolution),
        width=int(message.metadata.size_x),
        height=int(message.metadata.size_y),
        origin_x=float(origin.position.x),
        origin_y=float(origin.position.y),
        origin_yaw=yaw,
        data=tuple(int(value) for value in message.data),
    )


# Section: Structural validation before indexing the costmap data array.
def is_valid_costmap(snapshot: CostmapSnapshot) -> bool:
    """Return whether the costmap has usable finite geometry and complete data."""

    return (
        math.isfinite(snapshot.resolution)
        and snapshot.resolution > 0.0
        and math.isfinite(snapshot.origin_x)
        and math.isfinite(snapshot.origin_y)
        and math.isfinite(snapshot.origin_yaw)
        and snapshot.width > 0
        and snapshot.height > 0
        and len(snapshot.data) == snapshot.width * snapshot.height
    )


# Section: World-coordinate conversion that supports rotated map origins.
def world_to_map(
    snapshot: CostmapSnapshot, x: float, y: float
) -> Optional[Tuple[int, int]]:
    """Return a costmap cell for a world point, or None when outside the map."""

    if not is_valid_costmap(snapshot) or not (math.isfinite(x) and math.isfinite(y)):
        return None
    dx = x - snapshot.origin_x
    dy = y - snapshot.origin_y
    cosine = math.cos(snapshot.origin_yaw)
    sine = math.sin(snapshot.origin_yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    cell_x = math.floor(local_x / snapshot.resolution)
    cell_y = math.floor(local_y / snapshot.resolution)
    if cell_x < 0 or cell_y < 0 or cell_x >= snapshot.width or cell_y >= snapshot.height:
        return None
    return int(cell_x), int(cell_y)


# Section: Row-major costmap array lookup.
def cost_at(snapshot: CostmapSnapshot, cell_x: int, cell_y: int) -> int:
    """Return the unsigned Nav2 cost at a validated map cell."""

    if not is_valid_costmap(snapshot):
        raise ValueError('costmap metadata or data are invalid')
    if cell_x < 0 or cell_y < 0 or cell_x >= snapshot.width or cell_y >= snapshot.height:
        raise IndexError('costmap cell is outside the map')
    return snapshot.data[cell_y * snapshot.width + cell_x]


# Section: Threshold-based cost blocking policy.
def is_blocked_cost(cost: int, lethal_cost: int, unknown_is_blocked: bool) -> bool:
    """Return whether a cell cost is unsafe under the configured policy."""

    if cost == NO_INFORMATION:
        return unknown_is_blocked
    return cost >= lethal_cost


# Section: Single-point map and occupancy evaluation.
def point_cost_status(
    snapshot: CostmapSnapshot,
    x: float,
    y: float,
    lethal_cost: int = DEFAULT_LETHAL_COST,
    unknown_is_blocked: bool = True,
) -> Tuple[bool, bool, Optional[int]]:
    """Return inside-map, blocked, and raw cost values for a world point."""

    cell = world_to_map(snapshot, x, y)
    if cell is None:
        return False, False, None
    cost = cost_at(snapshot, *cell)
    return True, is_blocked_cost(cost, lethal_cost, unknown_is_blocked), cost
