"""Pure semantic-map parsing, validation, classification, and hysteresis logic."""

# Standard-library section: immutable models make configuration behavior deterministic.
from dataclasses import dataclass
from math import hypot, isfinite
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence, Tuple

# Third-party section: YAML is used only at the configuration boundary.
import yaml


# Type section: polygons contain metric map-frame x/y coordinates.
Point = Tuple[float, float]
Polygon = Tuple[Point, ...]
SemanticSignature = Tuple[str, str]


# Error section: all startup validation failures use one explicit exception type.
class ConfigError(ValueError):
    """Report a semantic-map configuration that cannot be used safely."""


# Model section: represent configured waypoints without coupling classification to Nav2.
@dataclass(frozen=True)
class Waypoint:
    """Store a named metric waypoint and its semantic room membership."""

    name: str
    x: float
    y: float
    room: str


# Model section: represent a nested semantic sublocation.
@dataclass(frozen=True)
class LocationRegion:
    """Store a named sublocation polygon and optional navigation waypoint reference."""

    name: str
    polygon: Polygon
    waypoint: str = ""


# Model section: represent a top-level room and its nested locations.
@dataclass(frozen=True)
class RoomRegion:
    """Store a room polygon, default waypoint, and nested semantic regions."""

    name: str
    polygon: Polygon
    default_waypoint: str
    locations: Tuple[LocationRegion, ...]


# Model section: hold a fully validated semantic map.
@dataclass(frozen=True)
class SemanticMap:
    """Store validated frames, semantic regions, and package-local waypoint data."""

    map_frame: str
    base_frame: str
    rooms: Tuple[RoomRegion, ...]
    waypoints: Tuple[Waypoint, ...]


# Result section: return instantaneous geometric classification independently of debounce.
@dataclass(frozen=True)
class SemanticMatch:
    """Describe the semantic regions containing one reliable metric pose."""

    room: str = ""
    location: str = ""
    nearest_waypoint: str = ""


# Result section: expose the stable status model consumed by the ROS node.
@dataclass(frozen=True)
class SemanticState:
    """Describe a confirmed or uncertainty-qualified semantic state."""

    status: str
    room: str = ""
    location: str = ""
    nearest_waypoint: str = ""


# YAML section: reject duplicate mapping keys before ordinary parsing can overwrite them.
class _UniqueKeyLoader(yaml.SafeLoader):
    """Load YAML while preserving the invariant that semantic names are unique."""


# YAML helper: construct mappings while detecting duplicate keys at every nesting level.
def _construct_unique_mapping(
    loader: _UniqueKeyLoader,
    node: yaml.MappingNode,
    deep: bool = False,
) -> Dict[Any, Any]:
    """Build one YAML mapping and fail on a duplicate source key."""

    # Validation section: flatten YAML merge keys before checking the effective mapping.
    loader.flatten_mapping(node)
    mapping: Dict[Any, Any] = {}
    # Validation section: inspect each key before assigning its value.
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise ConfigError(f"Duplicate YAML key: {key!r}")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


# YAML registration section: apply duplicate detection to the loader's default map tag.
_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


# Geometry helper: calculate a signed orientation/cross-product value.
def _cross(a: Point, b: Point, c: Point) -> float:
    """Return the signed twice-area of triangle a-b-c."""

    return ((b[0] - a[0]) * (c[1] - a[1])) - (
        (b[1] - a[1]) * (c[0] - a[0])
    )


# Geometry helper: test whether a point lies on a finite segment, including endpoints.
def _point_on_segment(point: Point, start: Point, end: Point) -> bool:
    """Return true when point lies on the closed start/end segment."""

    epsilon = 1.0e-9
    if abs(_cross(start, end, point)) > epsilon:
        return False
    return (
        min(start[0], end[0]) - epsilon
        <= point[0]
        <= max(start[0], end[0]) + epsilon
        and min(start[1], end[1]) - epsilon
        <= point[1]
        <= max(start[1], end[1]) + epsilon
    )


# Geometry helper: test closed-segment contact for overlap and self-intersection checks.
def _segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    """Return true if the two closed line segments intersect or touch."""

    epsilon = 1.0e-9
    ab_c = _cross(a, b, c)
    ab_d = _cross(a, b, d)
    cd_a = _cross(c, d, a)
    cd_b = _cross(c, d, b)
    if ((ab_c > epsilon and ab_d < -epsilon) or
            (ab_c < -epsilon and ab_d > epsilon)) and (
        (cd_a > epsilon and cd_b < -epsilon) or
        (cd_a < -epsilon and cd_b > epsilon)
    ):
        return True
    return (
        (abs(ab_c) <= epsilon and _point_on_segment(c, a, b))
        or (abs(ab_d) <= epsilon and _point_on_segment(d, a, b))
        or (abs(cd_a) <= epsilon and _point_on_segment(a, c, d))
        or (abs(cd_b) <= epsilon and _point_on_segment(b, c, d))
    )


# Geometry API: apply a ray-casting point-in-polygon test with boundary inclusion.
def point_in_polygon(point: Point, polygon: Polygon) -> bool:
    """Return true when point is inside or exactly on a polygon boundary."""

    # Boundary section: make edge behavior explicit before ray casting.
    for index, start in enumerate(polygon):
        end = polygon[(index + 1) % len(polygon)]
        if _point_on_segment(point, start, end):
            return True

    # Interior section: toggle containment for every edge crossing to the right.
    inside = False
    point_x, point_y = point
    for index, start in enumerate(polygon):
        end = polygon[(index + 1) % len(polygon)]
        if (start[1] > point_y) == (end[1] > point_y):
            continue
        intersection_x = start[0] + (
            (point_y - start[1]) * (end[0] - start[0]) /
            (end[1] - start[1])
        )
        if intersection_x > point_x:
            inside = not inside
    return inside


# Geometry helper: yield all closed polygon edges in declaration order.
def _edges(polygon: Polygon) -> Iterable[Tuple[Point, Point]]:
    """Yield every polygon edge, including the closing last-to-first edge."""

    for index, start in enumerate(polygon):
        yield start, polygon[(index + 1) % len(polygon)]


# Geometry helper: reject contact between top-level polygons to avoid boundary ambiguity.
def _polygons_contact(first: Polygon, second: Polygon) -> bool:
    """Return true for overlap, containment, or boundary contact."""

    # Intersection section: detect crossing and shared boundary segments.
    for first_start, first_end in _edges(first):
        for second_start, second_end in _edges(second):
            if _segments_intersect(
                first_start,
                first_end,
                second_start,
                second_end,
            ):
                return True
    # Containment section: detect one polygon fully inside the other.
    return point_in_polygon(first[0], second) or point_in_polygon(second[0], first)


# Parsing helper: require a non-empty mapping at a named schema path.
def _mapping(value: Any, path: str, allow_empty: bool = False) -> Mapping[str, Any]:
    """Return a validated mapping or raise a path-specific configuration error."""

    if not isinstance(value, Mapping) or (not value and not allow_empty):
        qualifier = "mapping" if allow_empty else "non-empty mapping"
        raise ConfigError(f"{path} must be a {qualifier}.")
    return value


# Parsing helper: require a finite float and reject booleans as accidental coordinates.
def _number(value: Any, path: str) -> float:
    """Return one finite metric number from configuration."""

    if isinstance(value, bool):
        raise ConfigError(f"{path} must be a finite number.")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{path} must be a finite number.") from exc
    if not isfinite(result):
        raise ConfigError(f"{path} must be a finite number.")
    return result


# Parsing helper: validate polygon shape, area, repeated points, and self-intersections.
def _polygon(value: Any, path: str) -> Polygon:
    """Parse and validate one simple polygon with at least three metric vertices."""

    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ConfigError(f"{path} must be a coordinate list.")
    if len(value) < 3:
        raise ConfigError(f"{path} must contain at least three points.")

    # Coordinate section: convert every [x, y] pair to a finite immutable point.
    points = []
    for index, coordinate in enumerate(value):
        coordinate_path = f"{path}[{index}]"
        if (
            not isinstance(coordinate, Sequence)
            or isinstance(coordinate, (str, bytes))
            or len(coordinate) != 2
        ):
            raise ConfigError(f"{coordinate_path} must be exactly [x, y].")
        points.append(
            (
                _number(coordinate[0], f"{coordinate_path}[0]"),
                _number(coordinate[1], f"{coordinate_path}[1]"),
            )
        )
    polygon = tuple(points)
    if len(set(polygon)) != len(polygon):
        raise ConfigError(f"{path} must not repeat vertices.")

    # Area section: reject collinear or otherwise zero-area polygons.
    twice_area = sum(
        (start[0] * end[1]) - (end[0] * start[1])
        for start, end in _edges(polygon)
    )
    if abs(twice_area) <= 1.0e-9:
        raise ConfigError(f"{path} must enclose a non-zero area.")

    # Simplicity section: reject non-adjacent edge intersections.
    edge_list = list(_edges(polygon))
    edge_count = len(edge_list)
    for first_index, first_edge in enumerate(edge_list):
        for second_index in range(first_index + 1, edge_count):
            if second_index in {
                first_index,
                first_index + 1,
            } or (first_index == 0 and second_index == edge_count - 1):
                continue
            if _segments_intersect(
                first_edge[0],
                first_edge[1],
                edge_list[second_index][0],
                edge_list[second_index][1],
            ):
                raise ConfigError(f"{path} must be a simple non-self-intersecting polygon.")
    return polygon


# Parsing API: convert one in-memory YAML document into a fully validated semantic map.
def parse_semantic_map(data: Any) -> SemanticMap:
    """Parse and validate the complete semantic-map schema."""

    root = _mapping(data, "semantic map")
    frames = _mapping(root.get("frames"), "frames")
    map_frame = str(frames.get("map", "")).strip()
    base_frame = str(frames.get("base", "")).strip()
    if not map_frame or not base_frame:
        raise ConfigError("frames.map and frames.base must be non-empty strings.")

    # Waypoint section: load package-local points before checking semantic references.
    waypoint_data = _mapping(root.get("waypoints", {}), "waypoints", allow_empty=True)
    waypoint_names = set()
    waypoints = []
    for raw_name, raw_waypoint in waypoint_data.items():
        name = str(raw_name).strip()
        if not name or name in waypoint_names:
            raise ConfigError(f"Duplicate or empty waypoint name: {name!r}")
        waypoint = _mapping(raw_waypoint, f"waypoints.{name}")
        room = str(waypoint.get("room", "")).strip()
        if not room:
            raise ConfigError(f"waypoints.{name}.room must be non-empty.")
        waypoint_names.add(name)
        waypoints.append(
            Waypoint(
                name=name,
                x=_number(waypoint.get("x"), f"waypoints.{name}.x"),
                y=_number(waypoint.get("y"), f"waypoints.{name}.y"),
                room=room,
            )
        )

    # Room section: parse globally unique semantic names and hierarchical polygons.
    room_data = _mapping(root.get("rooms"), "rooms")
    semantic_names = set()
    rooms = []
    for raw_room_name, raw_room in room_data.items():
        room_name = str(raw_room_name).strip()
        if not room_name or room_name in semantic_names:
            raise ConfigError(f"Duplicate or empty semantic name: {room_name!r}")
        semantic_names.add(room_name)
        room = _mapping(raw_room, f"rooms.{room_name}")
        room_polygon = _polygon(room.get("polygon"), f"rooms.{room_name}.polygon")
        default_waypoint = str(room.get("default_waypoint", "")).strip()

        # Sublocation section: require every location to live inside its declared parent room.
        raw_locations = _mapping(
            room.get("locations", {}),
            f"rooms.{room_name}.locations",
            allow_empty=True,
        )
        locations = []
        for raw_location_name, raw_location in raw_locations.items():
            location_name = str(raw_location_name).strip()
            if not location_name or location_name in semantic_names:
                raise ConfigError(
                    f"Duplicate or empty semantic name: {location_name!r}"
                )
            semantic_names.add(location_name)
            location = _mapping(
                raw_location,
                f"rooms.{room_name}.locations.{location_name}",
            )
            location_polygon = _polygon(
                location.get("polygon"),
                f"rooms.{room_name}.locations.{location_name}.polygon",
            )
            if not all(point_in_polygon(point, room_polygon) for point in location_polygon):
                raise ConfigError(
                    f"Location {location_name!r} must be contained by room {room_name!r}."
                )
            locations.append(
                LocationRegion(
                    name=location_name,
                    polygon=location_polygon,
                    waypoint=str(location.get("waypoint", "")).strip(),
                )
            )

        # Sublocation ambiguity section: overlapping or touching siblings are rejected.
        for first_index, first in enumerate(locations):
            for second in locations[first_index + 1:]:
                if _polygons_contact(first.polygon, second.polygon):
                    raise ConfigError(
                        f"Locations {first.name!r} and {second.name!r} overlap or touch."
                    )
        rooms.append(
            RoomRegion(
                name=room_name,
                polygon=room_polygon,
                default_waypoint=default_waypoint,
                locations=tuple(locations),
            )
        )

    # Room ambiguity section: reject overlap and contact rather than guessing by proximity.
    for first_index, first in enumerate(rooms):
        for second in rooms[first_index + 1:]:
            if _polygons_contact(first.polygon, second.polygon):
                raise ConfigError(
                    f"Rooms {first.name!r} and {second.name!r} overlap or touch."
                )

    # Reference section: require every referenced waypoint to exist in its owning room.
    waypoint_by_name = {waypoint.name: waypoint for waypoint in waypoints}
    room_names = {room.name for room in rooms}
    for waypoint in waypoints:
        if waypoint.room not in room_names:
            raise ConfigError(
                f"Waypoint {waypoint.name!r} references unknown room {waypoint.room!r}."
            )
    for room in rooms:
        references = [room.default_waypoint]
        references.extend(location.waypoint for location in room.locations)
        for reference in (name for name in references if name):
            if reference not in waypoint_by_name:
                raise ConfigError(
                    f"Room {room.name!r} references unknown waypoint {reference!r}."
                )
            if waypoint_by_name[reference].room != room.name:
                raise ConfigError(
                    f"Waypoint {reference!r} does not belong to room {room.name!r}."
                )
    return SemanticMap(
        map_frame=map_frame,
        base_frame=base_frame,
        rooms=tuple(rooms),
        waypoints=tuple(waypoints),
    )


# Loading API: read a semantic YAML file with duplicate-key-aware parsing.
def load_semantic_map(path: str) -> SemanticMap:
    """Load and validate a semantic map from one filesystem path."""

    semantic_path = Path(path).expanduser()
    if not semantic_path.is_file():
        raise ConfigError(f"Semantic map file does not exist: {semantic_path}")
    try:
        with semantic_path.open("r", encoding="utf-8") as stream:
            data = yaml.load(stream, Loader=_UniqueKeyLoader)
    except yaml.YAMLError as exc:
        raise ConfigError(f"Failed to parse semantic YAML {semantic_path}: {exc}") from exc
    return parse_semantic_map(data)


# Classification API: resolve metric points without retaining runtime state.
class SemanticClassifier:
    """Classify pose coordinates using room and nested-location polygons only."""

    def __init__(self, semantic_map: SemanticMap) -> None:
        """Store the already validated immutable semantic map."""

        self.semantic_map = semantic_map

    # Classification method: room polygons are the sole primary semantic classifier.
    def classify(self, x: float, y: float) -> SemanticMatch:
        """Resolve one reliable metric point to room, location, and nearest waypoint."""

        point = (float(x), float(y))
        room = next(
            (
                region
                for region in self.semantic_map.rooms
                if point_in_polygon(point, region.polygon)
            ),
            None,
        )
        if room is None:
            return SemanticMatch()
        location = next(
            (
                region.name
                for region in room.locations
                if point_in_polygon(point, region.polygon)
            ),
            "",
        )
        room_waypoints = tuple(
            waypoint
            for waypoint in self.semantic_map.waypoints
            if waypoint.room == room.name
        )
        nearest = min(
            room_waypoints,
            key=lambda waypoint: hypot(waypoint.x - point[0], waypoint.y - point[1]),
            default=None,
        )
        return SemanticMatch(
            room=room.name,
            location=location,
            nearest_waypoint=nearest.name if nearest is not None else "",
        )


# Hysteresis API: retain confirmed state while requiring stable transition candidates.
class SemanticStateTracker:
    """Debounce semantic signature changes and gate updates from uncertain poses."""

    def __init__(self, stable_duration_sec: float) -> None:
        """Configure the non-negative time required before committing a transition."""

        duration = float(stable_duration_sec)
        if not isfinite(duration) or duration < 0.0:
            raise ValueError("stable_duration_sec must be a finite non-negative value.")
        self.stable_duration_sec = duration
        self._confirmed = SemanticMatch()
        self._initialized = False
        self._candidate_signature: Optional[SemanticSignature] = None
        self._candidate_since = 0.0

    # State helper: translate the confirmed match to the public KNOWN/UNKNOWN model.
    def _confirmed_state(self, status_override: str = "") -> SemanticState:
        """Return the current confirmed match with a computed or overridden status."""

        status = status_override or (
            "KNOWN" if self._confirmed.room else "UNKNOWN"
        )
        return SemanticState(
            status=status,
            room=self._confirmed.room,
            location=self._confirmed.location,
            nearest_waypoint=self._confirmed.nearest_waypoint,
        )

    # Update method: accept reliable stable candidates and preserve state on uncertainty.
    def update(
        self,
        match: SemanticMatch,
        now_sec: float,
        localization_reliable: bool = True,
    ) -> SemanticState:
        """Apply uncertainty gating and time-based hysteresis to one classification."""

        now = float(now_sec)
        if not isfinite(now):
            raise ValueError("now_sec must be finite.")

        # Uncertainty section: never commit a candidate using an unreliable pose.
        if not localization_reliable:
            self._candidate_signature = None
            return self._confirmed_state("LOCALIZATION_UNCERTAIN")

        # Initialization section: publish the first reliable classification immediately.
        if not self._initialized:
            self._confirmed = match
            self._initialized = True
            self._candidate_signature = None
            return self._confirmed_state()

        confirmed_signature = (
            self._confirmed.room,
            self._confirmed.location,
        )
        match_signature = (match.room, match.location)

        # Stable section: update secondary waypoint data without delaying it.
        if match_signature == confirmed_signature:
            self._confirmed = match
            self._candidate_signature = None
            return self._confirmed_state()

        # Candidate section: begin timing each newly observed semantic signature.
        if self._candidate_signature != match_signature:
            self._candidate_signature = match_signature
            self._candidate_since = now
            if self.stable_duration_sec > 0.0:
                return self._confirmed_state()

        # Debounce section: preserve the confirmed state until the duration elapses.
        if now - self._candidate_since < self.stable_duration_sec:
            return self._confirmed_state()

        # Commit section: accept a stable room/location pair and clear candidate state.
        self._confirmed = match
        self._candidate_signature = None
        return self._confirmed_state()
