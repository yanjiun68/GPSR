"""Unit tests for pure costmap helpers; no ROS graph is required."""

# Section: System under test imports.
from nav2_diagnostic_reporter.costmap_utils import (
    CostmapSnapshot,
    NO_INFORMATION,
    cost_at,
    is_blocked_cost,
    point_cost_status,
    world_to_map,
)


# Section: Reusable deterministic 3-by-2 test costmap fixture.
def make_costmap(data=(0, 1, 2, 3, 253, 255)):
    """Create a simple axis-aligned map with one lethal and one unknown cell."""

    return CostmapSnapshot(1.0, 3, 2, 10.0, 20.0, 0.0, tuple(data))


# Section: World-to-grid boundary behavior tests.
def test_world_to_map_returns_row_major_cell_coordinates():
    """Ensure world positions use floor conversion and map origin offsets."""

    assert world_to_map(make_costmap(), 10.2, 21.9) == (0, 1)
    assert world_to_map(make_costmap(), 12.999, 20.001) == (2, 0)


def test_world_to_map_rejects_outside_points():
    """Ensure cells outside all map edges cannot be indexed."""

    assert world_to_map(make_costmap(), 9.99, 20.0) is None
    assert world_to_map(make_costmap(), 13.0, 20.0) is None
    assert world_to_map(make_costmap(), 10.0, 22.0) is None


# Section: Cost classification policy tests.
def test_unknown_cost_honors_unknown_policy_before_lethal_threshold():
    """Ensure unknown cells are configurable even though their raw value is high."""

    assert is_blocked_cost(NO_INFORMATION, 253, True)
    assert not is_blocked_cost(NO_INFORMATION, 253, False)


def test_point_cost_status_reports_inside_blocked_and_raw_cost():
    """Ensure costmap preflight can distinguish safe, lethal, and outside points."""

    assert point_cost_status(make_costmap(), 10.1, 20.1, 253, True) == (True, False, 0)
    assert point_cost_status(make_costmap(), 11.1, 21.1, 253, True) == (True, True, 253)
    assert point_cost_status(make_costmap(), 50.0, 50.0, 253, True) == (False, False, None)


def test_cost_at_uses_row_major_order():
    """Ensure direct array access matches Nav2 Costmap's documented data order."""

    assert cost_at(make_costmap(), 1, 1) == 253
