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
cd /home/yanjiun68/workspace/WS1/PutrabotUPM-project
source /opt/ros/humble/setup.bash
colcon build --packages-select waypoint_saver
source install/setup.bash
ros2 run waypoint_saver waypoint_saver_server.py
```

The node reads `map -> base_footprint` by default. Override either frame without
changing any robot launch file:

```bash
ros2 run waypoint_saver waypoint_saver_server.py --ros-args \
  -p target_frame:=map \
  -p robot_frame:=base_link
```

## Save the current pose

The YAML path must be absolute. A missing file is created with a `waypoints`
list. Existing files must already use that list schema.

```bash
ros2 service call \
  /waypoint_saver/save_current \
  waypoint_saver/srv/SaveWaypoint \
  "{name: kitchen, yaml_path: /absolute/path/waypoint.yaml, id: -1, overwrite: false}"
```

Use `id: -1` to assign the next numeric ID. Use `overwrite: true` only when the
matching name or ID should intentionally be replaced.

## Isolation guarantee

The node only creates a ROS service and TF subscriptions. It does not publish
`/robot_description`, `/joint_states`, or TF, and it does not include or alter
any `my_robot_nav` launch file.
