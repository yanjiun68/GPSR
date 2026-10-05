# Nav2 Diagnostic Reporter

`nav2_diagnostic_reporter` is a standalone ROS 2 Humble package that exposes a diagnostic `nav2_msgs/action/NavigateToPose` proxy. It does not launch, reconfigure, or replace Nav2.

## Goal flow

```text
Client
  │ /navigate_to_pose_diagnostic
  ▼
Validate pose → transform goal to map → get map → base_footprint TF
  → verify global costmap/start/goal cells → ComputePathToPose
  → /navigate_to_pose → Nav2 result
                     │
                     └── abort: repeat preflight, then report proven cause
```

The proxy action (`navigate_to_pose_diagnostic`) and actual Nav2 action (`navigate_to_pose`) have different default names, preventing recursive forwarding.

## Defaults for this workspace

The supplied YAML matches `my_robot_nav`: `map`, `base_footprint`, `GridBased`, `/global_costmap/costmap_raw`, `/compute_path_to_pose`, and `/navigate_to_pose`. Adjust the YAML if the active Nav2 deployment uses different names or frames.

## Build and launch

```bash
cd /home/yanjiun68/workspace/WS1/PutrabotUPM-project
source /opt/ros/humble/setup.bash
colcon build --packages-select nav2_diagnostic_reporter
source install/setup.bash
ros2 launch nav2_diagnostic_reporter gpsr_static_map.launch.py
```

This integrated launch reuses `my_robot_nav/static_map_nav.launch.py`, includes `diagnostic_reporter.launch.py`, and enables diagnostic speech by default. `stt_node.py` performs speech-to-text and cannot speak; `/nav2_diagnostics/user_message` is instead bridged to the existing `/voice/tts/say` TTS service. Set `enable_diagnostic_speech:=false` to disable this behavior. Set `start_voice_tts:=true` only when another launch is not already providing that service.

To launch the reporter alone, use `ros2 launch nav2_diagnostic_reporter diagnostic_reporter.launch.py`. Then send goals to the proxy rather than directly to Nav2:

```bash
ros2 action send_goal --feedback \
  /navigate_to_pose_diagnostic nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

## Diagnostic outputs

- `/nav2_diagnostics/error_code` (`std_msgs/msg/String`): stable error code.
- `/nav2_diagnostics/user_message` (`std_msgs/msg/String`): non-technical message, with cooldown suppression.
- `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`): code plus technical evidence such as frame, cost, and action status.

The complete code catalogue is defined in `nav2_diagnostic_reporter/error_codes.py`. Humble's `NavigateToPose` and `ComputePathToPose` results do not carry rich Nav2 failure codes; this package therefore never parses logs. If navigation aborts but planner preflight still succeeds, it reports `EXECUTION_FAILED` rather than inventing a cause.

## Cancellation and concurrent goals

The configured policy is preemption: a newly accepted proxy goal cancels the currently forwarded Nav2 goal, waits up to `cancel_timeout_sec` during the previous execution loop, and then starts its own preflight. A cancellation requested by the original client is forwarded to Nav2 as well.

The node uses a `MultiThreadedExecutor` and a `ReentrantCallbackGroup`. This is necessary because the proxy action execution waits for action-client futures while Nav2 action, TF, costmap, and cancellation callbacks must still run.

## Test

```bash
cd /home/yanjiun68/workspace/WS1/PutrabotUPM-project
source /opt/ros/humble/setup.bash
source install/setup.bash
pytest -p no:cacheprovider src/nav2_diagnostic_reporter/test
```
