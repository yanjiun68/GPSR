# gpsr_sys_ui

`gpsr_sys_ui` is a low-overhead ROS 2 diagnostic backend plus a nine-page terminal UI for the Putrabot UPM GPSR robot stack.

**Maintainer / author:** Putrabot UPM  
**Email:** `putrabotupm@gmail.com`

The monitor is intentionally **interface-first**: it consumes existing ROS outputs and standard ROS introspection instead of reimplementing Nav2, Interbotix, lifecycle, perception, or GPSR behavior.

## Pages

```text
1 Overview
2 GPSR
3 Voice
4 Nodes
5 Lifecycle
6 Nav2
7 Arm
8 Events
9 System Telemetry
```

There is **no Energy page**. `/battery_state` may still be observed at low rate to emit ENERGY events such as `BATTERY_LOW`, `BATTERY_CRITICAL`, or `ENERGY_SENSOR_STALE`; those appear only on the Events page.

## What is monitored

- ROS graph node inventory, missing required nodes, duplicate node names.
- Lifecycle `GetState` plus transition events.
- `/diagnostics` structured diagnostics.
- `/rosout` as fallback evidence for known error patterns.
- Nav2 NavigateToPose status.
- Nav2 Collision Monitor state (`STOP`, polygon name, persistent stop).
- `cmd_vel` before/after Collision Monitor and odometry correlation.
- Automatic ROS-graph lookup of `Twist` vs `TwistStamped` before creating velocity subscriptions.
- RX200 joint feedback from existing `sensor_msgs/JointState` topics.
- GPSR/YASMIN and phase-supervisor optional JSON status hooks.
- GPSR original/normalized commands and full structured action queue state.
- Independent Voice STT/TTS presence, diagnostic health/activity, and actual STT transcript.
- CPU/RAM/load/disk/thermal telemetry directly from `/proc`, `/sys`, and `statvfs`.
- NVIDIA GPU utilization/VRAM/temperature/power through structured `nvidia-smi` query output.
- Normalized, deduplicated event history and filters.

## Backward-compatible monitor topics

The package is named `gpsr_sys_ui`, but these already-established monitor API topics are intentionally preserved:

```text
/gpsr_system_monitor/state_json
/gpsr_system_monitor/event_json
```

This prevents a package rename from silently breaking an existing frontend or logger.

## Build

Place the complete directory at:

```text
<workspace>/src/gpsr_sys_ui
```

Then:

```bash
cd ~/workspace/WS1/PutrabotUPM-project
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build \
  --packages-select gpsr_sys_ui \
  --symlink-install

source install/setup.bash
```

Verify:

```bash
ros2 pkg prefix gpsr_sys_ui
ros2 pkg list | grep gpsr_sys_ui
```

## Run

Terminal 1 — diagnostic backend:

```bash
ros2 launch gpsr_sys_ui monitor.launch.py
```

Terminal 2 — colored terminal UI:

```bash
ros2 run gpsr_sys_ui monitor_tui
```

The TUI is deliberately a separate process. Closing the display does not stop `monitor_core`.

## TUI keys

```text
1..9       page selection
[  ]       previous / next page
A          all severities
W          WARN and above
E          ERROR and above
C          cycle subsystem filter
/          text search
N          node glob filter
X          clear event filters
P          pause / unpause displayed snapshot
q          quit TUI
```

The terminal uses semantic colors:

```text
cyan/blue   active tab and section headings
green       OK / ACTIVE / RUNNING / DONE / LISTENING / SPEAKING
amber       WARN / BLOCKED / STALE / RETRYING / SKIPPED / CANCELED
red         ERROR / FAILED / MISSING / ABORTED
white       primary values
gray        labels, table headers, secondary information
```

## Nav2 velocity type safety

The monitor never creates `Twist` and `TwistStamped` subscriptions simultaneously on one topic.

It performs:

```text
ROS graph lookup
    ↓
actual advertised type
    ↓
create exactly one matching subscription
```

If the topic does not exist yet, the monitor remains alive and retries. If the graph reports an unsupported or multiple type condition, it emits a warning and does not create an ambiguous subscription.

## Collision Monitor prerequisite

If you want the active Collision Monitor polygon/action displayed, configure Nav2 Collision Monitor to publish its state topic, for example:

```yaml
collision_monitor:
  ros__parameters:
    state_topic: collision_monitor_state
```

Then ensure `config/monitor.yaml` points to the resulting topic.

## System telemetry implementation

No `psutil` dependency is used.

```text
/proc/stat       CPU total + per-core utilization
/proc/meminfo    RAM + swap
/proc/loadavg    load average
/sys/...         thermal sensors
os.statvfs()     disk usage
nvidia-smi       GPU/VRAM/temperature/power
```

## GPSR and Voice data sources

The UI never parses natural-language commands or reconstructs queue state from logs. It consumes these structured owner interfaces:

```text
/gpsr/task_plan                    std_msgs/msg/String JSON (scheduler owner)
/gpsr/state                        std_msgs/msg/String JSON (scheduler owner)
/gpsr/action_queue_status_json     std_msgs/msg/String JSON (executor owner)
/voice/stt/transcript              std_msgs/msg/String (voice STT owner)
/diagnostics                       diagnostic_msgs/msg/DiagnosticArray
/putrabot_bringup/managed_process_status_json
```

The executor status publisher is event-driven and transient-local. It reports queue start, action selection, retry, action completion, and queue completion. The existing `ExecuteActionQueue` feedback fields receive the same cursor/action/status values.

```text
Scheduler plan/state ─┐
                      ├─> monitor_core cache ─> state_json ─> nine-page TUI
Executor queue status ┤
Voice transcript/diag ┘
```

STT and TTS are ordinary ROS nodes, not lifecycle nodes. Their verified graph names are `/voice_stt` and `/voice_tts`. Voice diagnostics distinguish `LISTENING`, `PROCESSING` (when an owner reports it), `SPEAKING`, and `IDLE`. Transcript age is display metadata only and never creates a stale-health failure.

No structured voice interface currently exposes speech confidence or the last TTS utterance, so the UI deliberately does not invent those values. If GPSR or voice nodes start after the monitor, graph polling and transient-local queue status allow the pages to recover without restarting the UI.

Queue display states are normalized centrally to `PENDING`, `RUNNING`, `DONE`, `SKIPPED`, `RETRYING`, `FAILED`, `CANCELED`, or `UNKNOWN`; every item also retains its producer-supplied raw state. The Events page emits command, queue, action, retry, failure, completion, and Voice presence transitions without changing scheduler/executor behavior.

## Interface audit

After sourcing the robot workspace:

```bash
bash src/gpsr_sys_ui/tools/interface_audit.sh \
  | tee /tmp/gpsr_sys_ui_interfaces.txt
```

The helper is read-only. It lists graph/interfaces and queries lifecycle state; it does not transition lifecycle nodes or run recovery actions.

## Tests

Hardware-free tests cover:

- event deduplication;
- event filtering-compatible data semantics;
- `/proc/stat` CPU parsing/delta calculation;
- `/proc/meminfo` parsing;
- Linux-resource collection using fake `/proc` + `/sys` roots;
- Nav2 velocity topic type resolution, including ambiguous/unsupported cases.
- Voice STT/TTS independent presence, diagnostics, activity, transcript, and late startup.
- GPSR command provenance, queue mapping, domains, targets, retries, skipped/failed/canceled states.
- Terminal wrapping, truncation, column fitting, page order, and current-row markers.
- Executor queue-status snapshots and observer-failure isolation.

Run:

```bash
python3 -m pytest -q test
```
