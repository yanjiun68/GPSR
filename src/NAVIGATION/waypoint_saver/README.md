# Waypoint Saver

This standalone ROS 2 package saves the robot's current TF pose under a name in
an explicitly requested waypoint YAML file. It has no dependency on
`my_robot_nav`, `robot_state_publisher`, or `joint_state_publisher`.

```text
ROS service request ──> waypoint_saver_server ──read──> TF
                               │
                               └──atomic write──> request.yaml_path
```

## Build and run

```bash
cd "$PUTRABOT_WORKSPACE_ROOT"
source /opt/ros/humble/setup.bash
colcon build --packages-select waypoint_saver
source install/setup.bash
ros2 launch waypoint_saver waypoint_saver.launch.py
```

The node reads `map -> base_footprint` by default. Override either frame without
changing any robot launch file:

```bash
ros2 run waypoint_saver waypoint_saver_server.py --ros-args \
  -p target_frame:=map \
  -p robot_frame:=base_link \
  -p max_transform_age_sec:=1.0
```

The default freshness policy accepts transforms whose age is less than or equal
to `1.0` second. It rejects stale, future-dated, and zero/uninitialized dynamic
TF timestamps before changing the waypoint file.

## Save the current pose

The YAML path must be absolute. A missing file is created with a `waypoints`
list. Existing files must already use that list schema.

```bash
ros2 service call \
  /waypoint_saver/save_current \
  waypoint_saver/srv/SaveWaypoint \
  "{name: kitchen, yaml_path: $PUTRABOT_WORKSPACE_ROOT/waypoints/gpsr_waypoint.yaml, id: -1, overwrite: false}"
```

Use `id: -1` to assign the next numeric ID. Use `overwrite: true` only when the
matching name or ID should intentionally be replaced.

Set `PUTRABOT_WORKSPACE_ROOT` to the absolute workspace path when the package is
not running from the standard colcon `install/<package>/share/<package>` layout.

## Isolation guarantee

The node only creates a ROS service and TF subscriptions. It does not publish
`/robot_description`, `/joint_states`, or TF, and it does not include or alter
navigation or hardware launch files. The navigation launch integration starts
this isolated service as an independent node and can disable it with
`start_waypoint_saver:=false`.
