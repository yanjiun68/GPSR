# SemanticLocationServer

The requested mixed-case name cannot generate ROS interfaces under Humble, so the actual ROS package is `semantic_location_server`; the standalone source directory remains `NAVIGATION/SemanticLocationServer`. It answers “where is the robot semantically?” by consuming an existing map-frame pose from TF or `/amcl_pose`. It does not replace or control AMCL, SLAM, Nav2, or waypoint navigation.

For a complete operator procedure covering arena mapping, AMCL verification, room polygons, GPSR waypoints, physical validation, competition-day startup, and configuration freeze, follow [docs/PRE_COMPETITION_SETUP.md](docs/PRE_COMPETITION_SETUP.md).

## Architecture

```text
existing AMCL/TF map → base_footprint
                  ↓
            metric x, y, yaw
                  ↓
       configured polygon containment
                  ↓
       room → optional nested location
                  ↓
      debounce + covariance uncertainty gate
                  ↓
topic /semantic_location/current
service /semantic_location/get_current
```

Room classification is based only on polygon containment. The nearest waypoint is calculated after the room is known and never turns an outside pose into a known room.

## Interactive visualizer

Open [docs/semantic_location_visualizer.html](docs/semantic_location_visualizer.html) in a browser to explore the classification pipeline without running ROS. Click or drag the simulated robot through the example polygons, change AMCL variance and debounce duration, replay the unit-test scenarios, inspect the simulated service response, or edit the JSON mirror of the semantic YAML configuration.

## Semantic map format

The installed [semantic_locations.yaml](config/semantic_locations.yaml) is illustrative. It is deliberately not copied from competition coordinates and must be replaced or overridden before physical use.

```yaml
frames:
  map: map
  base: base_footprint

waypoints:
  kitchen_center:
    room: kitchen
    x: 2.0
    y: 2.0

rooms:
  kitchen:
    polygon:
      - [0.0, 0.0]
      - [4.0, 0.0]
      - [4.0, 4.0]
      - [0.0, 4.0]
    default_waypoint: kitchen_center
    locations:
      kitchen_table:
        polygon:
          - [1.0, 1.0]
          - [2.0, 1.0]
          - [2.0, 2.0]
          - [1.0, 2.0]
        waypoint: kitchen_table_observe
```

Polygon points are `[x, y]` coordinates in `frames.map`. Polygons require at least three finite points, non-zero area, no repeated vertices, and no self-intersection. Room polygons, and sibling sublocation polygons, may not overlap or touch because shared boundaries would make exact-boundary classification ambiguous. Leave a measured doorway/gap or define boundaries with adequate separation.

Semantic names are globally unique. Every waypoint reference must exist, and the waypoint’s `room` must match the owning room. A sublocation polygon must remain inside its parent room.

## Status and boundary behavior

- `KNOWN`: the last stable reliable pose is inside a room polygon.
- `UNKNOWN`: the last stable reliable pose is outside every room polygon. No nearest-room fallback is applied.
- `LOCALIZATION_UNCERTAIN`: AMCL planar variance exceeds `max_xy_variance`; the last confirmed room is retained and no transition is committed.

The first reliable classification is published immediately. Later changes of room or sublocation must remain unchanged for `stable_duration_sec`. Uncertain samples reset the pending transition so uncertainty time cannot satisfy the debounce.

TF does not provide AMCL covariance, so covariance checking is unavailable with `pose_source:=tf`. Select `pose_source:=amcl` to enable it.

## Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `semantic_map_file` | installed example YAML | Semantic geometry file |
| `map_frame` | `map` | Existing global localization frame |
| `base_frame` | `base_footprint` | Existing robot base frame |
| `pose_source` | `tf` | `tf` or `amcl` |
| `amcl_topic` | `/amcl_pose` | AMCL pose topic |
| `update_rate_hz` | `5.0` | Classification frequency |
| `stable_duration_sec` | `1.0` | Transition debounce time |
| `use_localization_covariance` | `true` | Gate AMCL transitions by variance |
| `max_xy_variance` | `0.5` | Maximum accepted x/y covariance diagonal |
| `pose_timeout_sec` | `1.0` | Maximum AMCL sample age |
| `publish_debug_markers` | `false` | Publish configured polygon outlines |

The `map_frame` and `base_frame` parameters must match the YAML `frames` values. Invalid configuration stops node startup loudly.

## ROS interfaces

- `/semantic_location/current` (`semantic_location_server/msg/SemanticLocation`), reliable and transient-local.
- `/semantic_location/get_current` (`semantic_location_server/srv/GetSemanticLocation`).
- `/semantic_location/debug_markers` (`visualization_msgs/msg/MarkerArray`), available when marker publication is enabled.

The semantic message contains status, room, sublocation, nearest waypoint, metric pose diagnostics, and optional covariance diagnostics.

## Build and test

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select semantic_location_server --symlink-install
source install/setup.bash
colcon test --packages-select semantic_location_server
colcon test-result --verbose
```

## Launch

Start the existing localization stack first. Then launch this package separately with a verified map file:

```bash
source /opt/ros/humble/setup.bash
source "$PUTRABOT_WORKSPACE_ROOT/install/setup.bash"
ros2 launch semantic_location_server semantic_location_server.launch.py \
  semantic_map_file:=/absolute/path/to/verified_semantic_locations.yaml \
  pose_source:=tf \
  map_frame:=map \
  base_frame:=base_footprint
```

To use AMCL covariance explicitly:

```bash
ros2 launch semantic_location_server semantic_location_server.launch.py \
  semantic_map_file:=/absolute/path/to/verified_semantic_locations.yaml \
  pose_source:=amcl \
  amcl_topic:=/amcl_pose \
  use_localization_covariance:=true \
  max_xy_variance:=0.5
```

This launch file starts no Nav2, AMCL, SLAM, map server, robot-state publisher, camera, or `my_robot_nav` node.

## Verification and RViz

```bash
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic echo /amcl_pose --once
ros2 topic echo /semantic_location/current
ros2 service call /semantic_location/get_current semantic_location_server/srv/GetSemanticLocation "{}"
```

Enable markers with `publish_debug_markers:=true`. In RViz, set the fixed frame to `map`, add a `MarkerArray` display, and select `/semantic_location/debug_markers`.

Physically test the sequence `living_room → doorway/outside → kitchen → kitchen_table → kitchen → living_room`. Confirm that the doorway remains `UNKNOWN` if it is outside all polygons, brief boundary movement does not flicker, stable transitions occur after the debounce, and excessive AMCL covariance produces `LOCALIZATION_UNCERTAIN`.

## Consumer integration

No GPSR or navigation consumer is modified by this package. A future `navigation_skill_sm.py`, `find_object_skill_sm.py`, `find_person_skill_sm.py`, or `SearchCoordinator` can call `/semantic_location/get_current` before deciding whether navigation to a target room is necessary:

```text
target_room == current.room and current.status == KNOWN
    ├── yes: begin room-local behavior
    └── no: request existing named-pose navigation
```

The existing `my_robot_nav` stack remains external. Launch it using the command appropriate for the robot, then run this package’s launch command in a separate terminal; do not include or duplicate its localization infrastructure here.
