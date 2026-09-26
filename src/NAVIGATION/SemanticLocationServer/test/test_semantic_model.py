"""Unit tests for semantic-map validation, classification, and stable transitions."""

# Standard-library section: deepcopy lets individual tests alter one valid fixture safely.
from copy import deepcopy

# Test dependency section: pytest validates explicit configuration failures.
import pytest

# Package section: test pure domain behavior independently of a ROS graph.
from SemanticLocationServer.semantic_map import (
    ConfigError,
    SemanticClassifier,
    SemanticMatch,
    SemanticStateTracker,
    load_semantic_map,
    parse_semantic_map,
)


# Fixture helper: return a complete two-room semantic map used across deterministic tests.
def _valid_config():
    """Build a valid in-memory map with one nested kitchen location and three waypoints."""

    return {
        "frames": {"map": "map", "base": "base_footprint"},
        "waypoints": {
            "kitchen_center": {
                "room": "kitchen",
                "x": 2.5,
                "y": 2.5,
            },
            "kitchen_table_observe": {
                "room": "kitchen",
                "x": 1.5,
                "y": 1.5,
            },
            "living_center": {
                "room": "living_room",
                "x": 7.0,
                "y": 2.0,
            },
        },
        "rooms": {
            "kitchen": {
                "polygon": [[0, 0], [4, 0], [4, 4], [0, 4]],
                "default_waypoint": "kitchen_center",
                "locations": {
                    "kitchen_table": {
                        "polygon": [[1, 1], [2, 1], [2, 2], [1, 2]],
                        "waypoint": "kitchen_table_observe",
                    }
                },
            },
            "living_room": {
                "polygon": [[5, 0], [9, 0], [9, 4], [5, 4]],
                "default_waypoint": "living_center",
                "locations": {},
            },
        },
    }


# Classifier helper: construct a classifier from the canonical valid fixture.
def _classifier():
    """Return a classifier backed by a freshly parsed valid semantic map."""

    return SemanticClassifier(parse_semantic_map(_valid_config()))


# Required test: a point inside the kitchen reports a known kitchen room.
def test_inside_kitchen():
    """Resolve a kitchen point that is outside its nested table polygon."""

    result = _classifier().classify(3.0, 3.0)
    state = SemanticStateTracker(1.0).update(result, 0.0)
    assert result.room == "kitchen"
    assert state.status == "KNOWN"


# Required test: a point inside the second room resolves independently of waypoint distance.
def test_inside_living_room():
    """Resolve a point contained by the living-room polygon."""

    result = _classifier().classify(7.0, 2.0)
    assert result.room == "living_room"


# Required test: nested polygon containment reports both hierarchy levels.
def test_inside_sublocation():
    """Resolve a point inside kitchen and its kitchen-table sublocation."""

    result = _classifier().classify(1.5, 1.5)
    assert result.room == "kitchen"
    assert result.location == "kitchen_table"


# Required test: no polygon containment remains semantically unknown.
def test_outside_all_polygons():
    """Keep room and location empty when the point is outside all configured geometry."""

    result = _classifier().classify(4.5, 2.0)
    state = SemanticStateTracker(1.0).update(result, 0.0)
    assert result.room == ""
    assert result.location == ""
    assert result.nearest_waypoint == ""
    assert state.status == "UNKNOWN"


# Required test: malformed coordinates fail configuration loading loudly.
def test_malformed_polygon():
    """Reject a room polygon with fewer than three points."""

    config = deepcopy(_valid_config())
    config["rooms"]["kitchen"]["polygon"] = [[0, 0], [1, 1]]
    with pytest.raises(ConfigError, match="at least three"):
        parse_semantic_map(config)


# Required test: duplicate names across hierarchy levels are not accepted.
def test_duplicate_semantic_name():
    """Reject a sublocation that reuses an existing room name."""

    config = deepcopy(_valid_config())
    location = config["rooms"]["kitchen"]["locations"].pop("kitchen_table")
    config["rooms"]["kitchen"]["locations"]["living_room"] = location
    with pytest.raises(ConfigError, match="Duplicate"):
        parse_semantic_map(config)


# Validation test: duplicate source-level YAML keys are detected before overwrite.
def test_duplicate_yaml_key(tmp_path):
    """Reject duplicate room keys that a normal YAML loader would silently overwrite."""

    path = tmp_path / "duplicate.yaml"
    path.write_text(
        "frames: {map: map, base: base_footprint}\n"
        "rooms:\n"
        "  kitchen: {polygon: [[0, 0], [1, 0], [0, 1]]}\n"
        "  kitchen: {polygon: [[2, 2], [3, 2], [2, 3]]}\n",
        encoding="utf-8",
    )
    with pytest.raises(ConfigError, match="Duplicate YAML key"):
        load_semantic_map(str(path))


# Required test: nearest waypoint is secondary and cannot change room classification.
def test_nearest_waypoint_same_room():
    """Choose only among waypoints belonging to the polygon-confirmed room."""

    result = _classifier().classify(3.9, 3.9)
    assert result.room == "kitchen"
    assert result.nearest_waypoint == "kitchen_center"


# Required test: one boundary-adjacent candidate sample cannot flicker the stable room.
def test_boundary_flicker():
    """Preserve kitchen when living-room candidate duration is below debounce."""

    classifier = _classifier()
    tracker = SemanticStateTracker(stable_duration_sec=1.0)
    assert tracker.update(classifier.classify(3.9, 2.0), 0.0).room == "kitchen"
    assert tracker.update(classifier.classify(5.1, 2.0), 0.2).room == "kitchen"
    assert tracker.update(classifier.classify(3.9, 2.0), 0.4).room == "kitchen"


# Required test: a continuously observed new region commits after the debounce duration.
def test_stable_transition():
    """Commit living room only after its candidate remains stable for one second."""

    classifier = _classifier()
    tracker = SemanticStateTracker(stable_duration_sec=1.0)
    tracker.update(classifier.classify(3.0, 3.0), 0.0)
    assert tracker.update(classifier.classify(7.0, 2.0), 1.0).room == "kitchen"
    assert tracker.update(classifier.classify(7.0, 2.0), 2.0).room == "living_room"


# Required test: unreliable localization reports uncertainty and cannot commit a new room.
def test_uncertain_localization():
    """Retain the last confirmed kitchen while flagging an unreliable living-room pose."""

    tracker = SemanticStateTracker(stable_duration_sec=1.0)
    tracker.update(SemanticMatch(room="kitchen"), 0.0)
    result = tracker.update(
        SemanticMatch(room="living_room"),
        2.0,
        localization_reliable=False,
    )
    assert result.status == "LOCALIZATION_UNCERTAIN"
    assert result.room == "kitchen"


# Validation test: overlapping top-level room geometry fails instead of choosing nearest.
def test_overlapping_rooms_rejected():
    """Reject two room polygons whose interiors overlap."""

    config = deepcopy(_valid_config())
    config["rooms"]["living_room"]["polygon"] = [
        [3, 1],
        [6, 1],
        [6, 3],
        [3, 3],
    ]
    with pytest.raises(ConfigError, match="overlap or touch"):
        parse_semantic_map(config)


# Validation test: a referenced waypoint must exist and belong to the parent room.
def test_unknown_waypoint_reference_rejected():
    """Reject a default waypoint reference that cannot be resolved."""

    config = deepcopy(_valid_config())
    config["rooms"]["kitchen"]["default_waypoint"] = "missing"
    with pytest.raises(ConfigError, match="unknown waypoint"):
        parse_semantic_map(config)
