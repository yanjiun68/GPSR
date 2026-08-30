# Pre-Competition Arena Setup Guide

This is the operator procedure for preparing a new RoboCup arena in the workspace selected by `PUTRABOT_WORKSPACE_ROOT`. Complete it before running GPSR tasks in a new arena.

The required preparation flow is:

```text
create occupancy map → verify AMCL/TF → define room polygons
        → create navigation and observation waypoints
        → associate semantic regions and waypoints
        → validate the standalone semantic server
        → validate transitions and GPSR search readiness
        → freeze the competition configuration
```

## 1. Prerequisites and package boundaries

Required components:

- ROS 2 Humble.
- The existing `my_robot_nav` Nav2, SLAM, and AMCL stack.
- A completed occupancy map for the current arena.
- RViz2.
- The existing `waypoints/gpsr_waypoint.yaml` waypoint workflow.
- A built `semantic_location_server` package.

The semantic server does **not** create the SLAM map, replace AMCL, replace Nav2, drive the robot, or execute room searches. It consumes the localization already produced by the navigation stack.

Build the relevant packages before setup:

```bash
cd "$PUTRABOT_WORKSPACE_ROOT"
source /opt/ros/humble/setup.bash
colcon build --packages-select my_robot_nav semantic_location_server --symlink-install
source install/setup.bash
```

Use a separate terminal for each long-running launch. In every new terminal run:

```bash
cd "$PUTRABOT_WORKSPACE_ROOT"
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 2. Produce the arena occupancy map

The repository's real mapping entry point is `my_robot_nav/launch/mapping.launch.py`. It includes `real_robot.launch.py`, starts the robot hardware/sensors, starts `slam_toolbox` in mapping mode using `/scan_filtered`, and opens RViz. Do not start a second copy of the base or lidar drivers unless the launch is intentionally changed by the robot maintainer.

### 2.1 Start mapping

Terminal 1:

```bash
ros2 launch my_robot_nav mapping.launch.py \
  use_sim_time:=false \
  scan_topic:=/scan_filtered
```

Verify that the map and scan exist:

```bash
ros2 topic echo /map --once
ros2 topic echo /scan_filtered --once
```

If joystick driving is the approved operator method, start the repository's existing teleoperation launch in Terminal 2:

```bash
ros2 launch my_robot_nav joy_teleop.launch.py
```

Drive slowly through the complete navigable arena. Cover all sides of:

- living room;
- kitchen;
- dining room;
- bedroom;
- halls and corridors;
- entrances;
- doorways and narrow connections.

Revisit loop closures from more than one direction. Watch the live RViz map while driving; do not save while major wall duplication or unclosed loops remain.

### 2.2 Save the map

The workspace has the `nav2_map_server` `map_saver_cli` executable. While SLAM is still publishing `/map`, save the final candidate:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f "$PUTRABOT_WORKSPACE_ROOT/maps/competition_arena"
```

This should create:

```text
maps/competition_arena.yaml
maps/competition_arena.pgm
```

Confirm both files before stopping mapping:

```bash
ls -lh \
  "$PUTRABOT_WORKSPACE_ROOT/maps/competition_arena.yaml" \
  "$PUTRABOT_WORKSPACE_ROOT/maps/competition_arena.pgm"
```

Record `competition_arena.yaml` as the map used for all later steps. Stop the mapping launch only after the files are verified.

### 2.3 Restart in localization mode

The repository's static-map localization entry point includes the robot, map server, AMCL, Nav2, RViz, and `/nav/navigate_named_pose`. Disable its older point-based semantic navigator so it is not confused with the standalone polygon server:

```bash
ros2 launch my_robot_nav static_map_nav.launch.py \
  map:="$PUTRABOT_WORKSPACE_ROOT/maps/competition_arena.yaml" \
  use_semantic_nav:=false \
  use_rviz:=true
```

In RViz, use **2D Pose Estimate** to seed AMCL at the robot's actual position and heading. The startup gate intentionally waits for an initial pose before activating navigation.

## 3. Verify map quality

Do not create semantic polygons until all these checks pass:

- [ ] Walls are represented at the correct positions.
- [ ] Room boundaries are distinguishable.
- [ ] Doorways are open and wide enough in the costmap.
- [ ] There are no major duplicated walls.
- [ ] There is no severe accumulated SLAM drift.
- [ ] Furniture relevant to collision avoidance is represented appropriately.
- [ ] The robot localizes reliably after an initial pose is supplied.
- [ ] Nav2 can plan and execute paths between rooms.

The semantic polygons, occupancy map, AMCL pose, and waypoints must all use the same `map` frame. A visually good polygon in a different coordinate frame is invalid.

## 4. Verify AMCL and TF before collecting coordinates

With `static_map_nav.launch.py` running and the initial pose set, verify the actual localization topic:

```bash
ros2 topic info /amcl_pose -v
ros2 topic echo /amcl_pose --once
```

Verify the map-to-base transform used throughout this workspace:

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

Before proceeding, record and confirm:

```yaml
localization:
  map_frame: map
  base_frame: base_footprint
  amcl_topic: /amcl_pose
```

If these values differ on competition hardware, update both the `frames` section of `semantic_locations.yaml` and the matching launch arguments. Do not collect points until TF is continuous and `/amcl_pose` is stable.

## 5. Collect room and waypoint coordinates

Use RViz with the occupancy map displayed. Move or navigate the robot to each point of interest and observe the map-frame pose with:

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

The output reports translation `(x, y, z)`, quaternion rotation, and roll/pitch/yaw. Record:

- `x` and `y` for polygon corners and waypoint positions;
- quaternion `x, y, z, w` for `gpsr_waypoint.yaml`;
- yaw for the operator worksheet and orientation review.

For room corners, RViz may also be used to inspect map coordinates visually. Semantic polygons should describe the usable semantic area, not blindly trace every wall. Keep them out of walls and unreachable space. Decide deliberately whether doorways are part of a room or an `UNKNOWN` gap.

Example geometry only—these are not arena coordinates:

```yaml
kitchen:
  polygon:
    - [2.1, 1.0]
    - [5.7, 1.0]
    - [5.7, 4.1]
    - [2.1, 4.1]
```

## 6. Define all semantic rooms

Edit the standalone package configuration:

```bash
nano "$PUTRABOT_WORKSPACE_ROOT/src/NAVIGATION/SemanticLocationServer/config/semantic_locations.yaml"
```

Define rooms in this practical order:

1. `living_room`
2. `kitchen`
3. `dining_room`
4. `bedroom`
5. hall or corridor, when semantically useful
6. entrance areas, when semantically useful

For each room record its unique name, polygon, and optional `default_waypoint`.

Polygon rules enforced by the current parser:

- at least three `[x, y]` points;
- finite map-frame coordinates;
- non-zero enclosed area;
- no repeated vertices;
- no self-intersection;
- no overlap or contact between room polygons;
- no overlap or contact between sibling sublocation polygons;
- every sublocation vertex must remain inside its parent room.

Because touching room polygons are rejected, leave a measured gap at shared boundaries. Such a gap reports `UNKNOWN`; use it intentionally around uncertain doorway boundaries and validate it physically.

## 7. Define sublocations intentionally

Possible sublocations include `sofa`, `dining_table`, `kitchen_table`, `counter`, `cabinet`, `shelf`, `sink`, `refrigerator`, and `entrance`. Not every furniture item needs a polygon.

Classify each candidate in the setup worksheet:

| Type | Representation | Use |
|---|---|---|
| AREA | nested polygon in `semantic_locations.yaml` | Robot can meaningfully be inside the area |
| NAVIGATION_TARGET | named pose in `gpsr_waypoint.yaml` | Robot needs to navigate to a destination |
| OBSERVATION_TARGET | named observation pose in `gpsr_waypoint.yaml` | Perception/search needs a safe camera view |

A table or sofa is often better represented by an observation waypoint than by a polygon because the robot normally stands beside it, not inside it. Use a nested polygon only when “robot is at this semantic area” has a useful and testable meaning.

## 8. Prepare repository-compatible waypoints

The existing named-pose server reads this file once at startup:

```text
$PUTRABOT_WORKSPACE_ROOT/waypoints/gpsr_waypoint.yaml
```

There is no repository tool that safely appends the GPSR list format. Record poses with TF and edit the YAML explicitly.

### 8.1 Capture and save a waypoint

1. Localize the robot and confirm `/amcl_pose`.
2. Move the robot to the required clear pose.
3. Run `ros2 run tf2_ros tf2_echo map base_footprint`.
4. Record translation and quaternion.
5. Assign a unique integer `id` and descriptive `name`.
6. Add the pose to `gpsr_waypoint.yaml` using its actual schema.

Example only:

```yaml
waypoints:
  - id: 20
    frame_id: map
    name: kitchen_table_observe_1
    pose:
      position:
        x: 2.40
        y: 1.65
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.7071
        w: 0.7071
```

The named-pose server accepts the explicit `name`, the numeric `id` as text, and an alias such as `waypoint_20` for ID 20. Use the descriptive name for GPSR.

### 8.2 Required waypoint categories

- Room entry: `kitchen_entry`, `living_room_entry`.
- Room center: `kitchen_center`, `living_room_center`.
- Semantic target: `sofa`, `dining_table`.
- Observation pose: `kitchen_table_observe_1`, `kitchen_counter_observe_1`.

### 8.3 Reload and command a waypoint

The server does not watch the YAML for changes. After editing, stop and restart `static_map_nav.launch.py` so `/nav/navigate_named_pose` reloads the file. Then verify the action exists:

```bash
ros2 action info /nav/navigate_named_pose
```

Command a real configured waypoint, for example `kitchen_center`:

```bash
ros2 action send_goal \
  /nav/navigate_named_pose \
  receptionist_interfaces/action/NavigateNamedPose \
  "{target_name: kitchen_center}" \
  --feedback
```

Confirm action success, physical arrival, orientation, collision clearance, and sensor view. A server-side name match is not evidence that the physical pose is safe.

## 9. Waypoint placement rules

### Room waypoint

Place the robot clearly inside the room, away from the doorway, with enough clearance for localization and path completion.

### Furniture waypoint

Place it where the robot can approach safely, will not collide with the furniture, and can manipulate or perceive the target if the task requires it.

### Observation waypoint

Optimize camera field of view, object and human visibility, depth-camera range, lighting, and manipulator reach where applicable. Test the view using the real perception pipeline.

Never put an observation pose inside an obstacle, inside a table, too close to a wall, or directly in a narrow doorway.

## 10. Associate semantic regions with waypoints

The two concepts remain distinct:

```text
semantic polygon → area containing the robot
navigation waypoint → destination the robot can reach
```

`gpsr_waypoint.yaml` is consumed by the existing navigation action. The standalone `semantic_locations.yaml` contains a smaller waypoint mapping used only for same-room nearest-waypoint reporting and reference validation. When a name appears in both files, keep its position and room association consistent manually.

Example relationship:

```yaml
rooms:
  kitchen:
    polygon: [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]]
    default_waypoint: kitchen_center
    locations:
      kitchen_table:
        polygon: [[1.0, 1.0], [2.0, 1.0], [2.0, 2.0], [1.0, 2.0]]
        waypoint: kitchen_table_observe_1
```

The semantic server never infers a room from the nearest waypoint.

## 11. Prepare observation waypoints for GPSR

For `FIND_OBJECT` and `FIND_PERSON`, prepare an ordered room-search worksheet. The current semantic server does not execute or parse this search list; it is input for a future `SearchCoordinator` consumer.

```yaml
search_locations:
  kitchen:
    - kitchen_table_observe_1
    - kitchen_counter_observe_1
    - refrigerator_observe_1
  living_room:
    - sofa_observe_1
    - coffee_table_observe_1
```

For every listed name:

1. add the actual pose to `waypoints/gpsr_waypoint.yaml`;
2. navigate to it with `/nav/navigate_named_pose`;
3. run the real object and person perception pipelines;
4. verify the expected search surfaces or people area are visible;
5. order poses so coverage is useful and travel is reasonable.

The intended future behavior is:

```text
room search → observation waypoint 1 → perception
            → observation waypoint 2 → perception → ...
```

Do not claim GPSR search readiness merely because the names exist in YAML.

## 12. Complete semantic configuration example

This example matches the implemented parser exactly. All coordinates remain illustrative and must be replaced with measured arena values.

```yaml
frames:
  map: map
  base: base_footprint

waypoints:
  kitchen_center:
    room: kitchen
    x: 2.0
    y: 2.0
  kitchen_table_observe_1:
    room: kitchen
    x: 1.5
    y: 1.5
  living_room_center:
    room: living_room
    x: 7.0
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
        waypoint: kitchen_table_observe_1

  living_room:
    polygon:
      - [5.0, 0.0]
      - [9.0, 0.0]
      - [9.0, 4.0]
      - [5.0, 4.0]
    default_waypoint: living_room_center
    locations: {}
```

Do not add the `search_locations` worksheet to this file expecting runtime search behavior; the current server ignores search coordination data.

## 13. Test the standalone server

Keep localization running. In another terminal launch the actual package using AMCL so covariance checks are active:

```bash
ros2 launch semantic_location_server semantic_location_server.launch.py \
  semantic_map_file:="$PUTRABOT_WORKSPACE_ROOT/src/NAVIGATION/SemanticLocationServer/config/semantic_locations.yaml" \
  pose_source:=amcl \
  amcl_topic:=/amcl_pose \
  map_frame:=map \
  base_frame:=base_footprint \
  stable_duration_sec:=1.0 \
  publish_debug_markers:=true
```

Verify the node, topic, and query service:

```bash
ros2 node list | grep semantic_location_server
ros2 topic echo /semantic_location/current
ros2 service call \
  /semantic_location/get_current \
  semantic_location_server/srv/GetSemanticLocation \
  "{}"
```

In RViz, set **Fixed Frame** to `map`, add a `MarkerArray` display, and select `/semantic_location/debug_markers`.

## 14. Validate rooms physically

Drive this sequence:

```text
living_room → doorway → kitchen → kitchen_table
            → kitchen → doorway → living_room
```

Expected output:

```text
living_room → stable boundary/intentional UNKNOWN → kitchen
            → kitchen / kitchen_table → kitchen
            → stable boundary/intentional UNKNOWN → living_room
```

Verify:

- correct room while stationary;
- correct nested location;
- no rapid room flickering;
- `UNKNOWN` only in intended uncovered geometry;
- doorway behavior is repeatable in both directions.

## 15. Tune hysteresis

The implemented starting value is:

```yaml
stable_duration_sec: 1.0
```

If the value is too low, the room may flicker at doorways. If it is too high, legitimate room transitions appear late. Test several crossings at normal robot speed and change only this launch argument first:

```bash
ros2 launch semantic_location_server semantic_location_server.launch.py \
  semantic_map_file:="$PUTRABOT_WORKSPACE_ROOT/src/NAVIGATION/SemanticLocationServer/config/semantic_locations.yaml" \
  pose_source:=amcl \
  stable_duration_sec:=1.5
```

Record the approved value in the competition startup notes.

## 16. Test AMCL accuracy before changing polygons

Validate `/amcl_pose` at a room center, near a wall, in each doorway, inside each semantic sublocation, and at every observation waypoint.

```text
semantic location is wrong
          ↓
check /amcl_pose and map→base_footprint first
          ↓
AMCL wrong? ── yes → fix localization/map/initial pose
          └── no  → fix semantic polygon/configuration
```

Do not enlarge or shift polygons merely to hide a localization error. With `pose_source:=amcl`, variance above `max_xy_variance` reports `LOCALIZATION_UNCERTAIN` and does not commit a new room.

## 17. Validate every Nav2 waypoint

For each competition waypoint, send the named-pose action and fill this table. Leave cells unchecked until physically verified.

| Waypoint | Navigation | Orientation | Clearance | Camera view | Approved |
|---|---|---|---|---|---|
| kitchen_entry | [ ] | [ ] | [ ] | [ ] | [ ] |
| kitchen_center | [ ] | [ ] | [ ] | [ ] | [ ] |
| living_room_entry | [ ] | [ ] | [ ] | [ ] | [ ] |
| living_room_center | [ ] | [ ] | [ ] | [ ] | [ ] |
| kitchen_table_observe_1 | [ ] | [ ] | [ ] | [ ] | [ ] |
| kitchen_counter_observe_1 | [ ] | [ ] | [ ] | [ ] | [ ] |
| sofa_observe_1 | [ ] | [ ] | [ ] | [ ] | [ ] |
| coffee_table_observe_1 | [ ] | [ ] | [ ] | [ ] | [ ] |

Add rows for every final waypoint. Confirm Nav2 reports success only after the robot is in a useful physical pose.

## 18. Validate GPSR readiness

Complete these checks before enabling full GPSR operation:

- [ ] Navigate from bedroom to kitchen.
- [ ] Navigate from kitchen to living room.
- [ ] Target room is reported correctly before `FIND_OBJECT` search.
- [ ] Every object observation waypoint name exists and navigates successfully.
- [ ] Object perception covers the required surfaces at each pose.
- [ ] Target room is reported correctly before `FIND_PERSON` search.
- [ ] Every person observation waypoint name exists and navigates successfully.
- [ ] Person perception covers the required human areas at each pose.
- [ ] Current room is correct while stationary.
- [ ] Current room updates after a stable transition.

Consumer integration is not yet automatic: `FIND_OBJECT`, `FIND_PERSON`, and `SearchCoordinator` must be updated in a future task to query `/semantic_location/get_current` and consume an ordered search list. This setup validates the data and ROS interface needed by that integration without claiming it already exists.

## 19. Competition-day startup

### Terminal 1: robot, map, AMCL, Nav2, RViz, and named poses

```bash
cd "$PUTRABOT_WORKSPACE_ROOT"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_nav static_map_nav.launch.py \
  map:="$PUTRABOT_WORKSPACE_ROOT/maps/competition_arena.yaml" \
  use_semantic_nav:=false \
  use_rviz:=true
```

Set the initial pose with RViz **2D Pose Estimate**, then verify:

```bash
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_footprint
```

### Terminal 2: polygon semantic server

```bash
cd "$PUTRABOT_WORKSPACE_ROOT"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch semantic_location_server semantic_location_server.launch.py \
  semantic_map_file:="$PUTRABOT_WORKSPACE_ROOT/src/NAVIGATION/SemanticLocationServer/config/semantic_locations.yaml" \
  pose_source:=amcl \
  stable_duration_sec:=1.0
```

Verify the loaded file and current room:

```bash
ros2 param get /semantic_location_server semantic_map_file
ros2 service call \
  /semantic_location/get_current \
  semantic_location_server/srv/GetSemanticLocation \
  "{}"
```

### Test one approved navigation waypoint

```bash
ros2 action send_goal \
  /nav/navigate_named_pose \
  receptionist_interfaces/action/NavigateNamedPose \
  "{target_name: kitchen_center}" \
  --feedback
```

### Terminal 3: GPSR stack, after the checks above

```bash
cd "$PUTRABOT_WORKSPACE_ROOT"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch gpsr_task_scheduler gpsr_scheduler.launch.py
```

Until GPSR consumer integration is added, “semantic configuration loaded” means the standalone server reports the approved map and the operator has verified waypoint/search worksheets; the scheduler does not yet consume semantic-location queries automatically.

## 20. Handle arena changes

Use this decision table after any layout update:

| Change | Rebuild occupancy map? | Change polygon? | Change waypoint? |
|---|---|---|---|
| Furniture moves slightly | Usually no | Usually no | Maybe |
| Room geometry changes | Possibly | Yes | Inspect all affected poses |
| A waypoint becomes blocked | No | Usually no | Yes, create a safe replacement |
| A table moves | Usually no | Only if its semantic area changed | Observation pose likely changes |
| Doorway width changes | Possibly | Re-test boundary | Re-test entry poses |

Always re-run AMCL, Nav2, semantic-transition, and perception-view checks for affected areas. Arena-specific guarantees cannot be inferred from an old map.

## 21. Freeze the final configuration

Before competition, inspect all pending workspace changes:

```bash
cd "$PUTRABOT_WORKSPACE_ROOT"
git status --short
git diff -- \
  maps \
  waypoints/gpsr_waypoint.yaml \
  src/NAVIGATION/SemanticLocationServer/config/semantic_locations.yaml
```

Preserve together:

- `competition_arena.yaml` and its image file;
- `semantic_locations.yaml`;
- `gpsr_waypoint.yaml`;
- navigation parameters used during approval;
- the ordered observation-waypoint worksheet or future search configuration.

After human review, create a competition-specific branch, commit, and tag according to the team's normal Git workflow. Do not make the freeze from an unreviewed dirty tree, and do not allow an automated tool to commit or tag the configuration without operator approval.

## 22. Final pre-competition checklist

- [ ] Final occupancy map generated
- [ ] Final map YAML and image backed up together
- [ ] Map quality checklist completed
- [ ] AMCL localization validated
- [ ] `map` to `base_footprint` TF validated
- [ ] `/amcl_pose` topic and covariance validated
- [ ] All room polygons configured
- [ ] Polygon overlap/contact validation passed
- [ ] Intended `UNKNOWN` areas checked
- [ ] All sublocation polygons checked physically
- [ ] Room-center waypoints tested
- [ ] Semantic-target waypoints tested
- [ ] Object observation poses tested
- [ ] Person observation poses tested
- [ ] Doorway transitions tested in both directions
- [ ] Hysteresis tuned at normal robot speed
- [ ] `semantic_location_server` launch tested
- [ ] Semantic output visually verified in RViz
- [ ] Query service returns the correct current state
- [ ] Navigation to every required waypoint tested
- [ ] GPSR room-search waypoint worksheet validated
- [ ] GPSR consumer-integration limitation acknowledged
- [ ] Final map, semantic, waypoint, and search configurations reviewed
- [ ] Final configurations backed up and version-controlled
