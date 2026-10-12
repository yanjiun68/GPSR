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
- GPSR original/normalized commands, pre-execution skill readiness, and full structured action queue state.
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
green       OK / ACTIVE / READY / RUNNING / DONE / LISTENING / SPEAKING
amber       WARN / PARTIAL / BLOCKED / UNKNOWN / STALE / RETRYING / SKIPPED / CANCELED
red         ERROR / FAILED / UNAVAILABLE / MISSING / ABORTED
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
/gpsr/action_queue_validation_json std_msgs/msg/String JSON (preflight owner)
/gpsr/action_queue_status_json     std_msgs/msg/String JSON (executor owner)
/voice/stt/transcript              std_msgs/msg/String (voice STT owner)
/diagnostics                       diagnostic_msgs/msg/DiagnosticArray
/putrabot_bringup/managed_process_status_json
```

The task plan represents scheduler intent. Pre-execution validation represents runtime capability/readiness before physical execution. Queue status represents actual executor runtime state. These dimensions remain separate: for example, an action can be `UNAVAILABLE` during preflight while its executor state is still `PENDING`, or can be `READY` during preflight and later fail during execution.

The scheduler plan, validation, and executor status publishers may be event-driven and transient-local. They publish without checking for UI subscribers and retain their latest snapshots, so robot execution runs normally when `monitor_core` and the TUI are stopped. The monitor uses reliable transient-local QoS for the optional validation source. If `/gpsr/action_queue_validation_json` does not exist, the monitor still starts and the GPSR page reports validation as not available without treating that absence as a robot failure.

The Overview page shows a compact preflight result and readiness counts alongside the accepted ActionQueue. The GPSR page shows detailed preflight readiness, unavailable skills, dependency-blocked actions, and the complete runtime sequence with domain, action parameters, preflight state, execution status, and retry details. Parameters are retained as structured monitor-state data and rendered as compact `key=value` text, including nested selectors and filters. Operator-facing task statuses are `IN QUEUE`, `RUNNING`, `RETRYING`, `COMPLETED`, `ABORTED`, and `SKIPPED`. Preflight states are independently normalized to `READY`, `UNAVAILABLE`, `BLOCKED`, or `UNKNOWN`.

```text
Scheduler plan/state ────────────┐
                                 │
Pre-execution validation ────────┼─> monitor_core cache
                                 │          ↓
Executor queue status ───────────┤      state_json
                                 │          ↓
Voice transcript/diagnostics ────┘      nine-page TUI
```

The arrows are one-way ROS topic subscriptions. `gpsr_sys_ui` is an observer only: it does not call services, transition lifecycle nodes, start skill servers, or alter scheduler/executor behavior. Neither the scheduler nor the executor imports, launches, calls, or waits for `gpsr_sys_ui`.

STT and TTS are ordinary ROS nodes, not lifecycle nodes. Their verified graph names are `/voice_stt` and `/voice_tts`. Voice diagnostics distinguish `LISTENING`, `PROCESSING` (when an owner reports it), `SPEAKING`, and `IDLE`. Transcript age is display metadata only and never creates a stale-health failure.

No structured voice interface currently exposes speech confidence or the last TTS utterance, so the UI deliberately does not invent those values. If GPSR or voice nodes start after the monitor, graph polling and transient-local queue status allow the pages to recover without restarting the UI.

Queue display states are normalized centrally to `PENDING`, `RUNNING`, `DONE`, `SKIPPED`, `RETRYING`, `FAILED`, `CANCELED`, or `UNKNOWN`; every item also retains its producer-supplied raw state. Validation values are normalized separately from structured owner data and retain their raw state. The Events page emits command, preflight summary, unavailable/blocked skill, queue, action, retry, failure, completion, and Voice presence transitions without changing scheduler/executor behavior.

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
- GPSR preflight normalization, step-ID/index merging, stale-command protection, and event deduplication.
- Independent preflight readiness and executor runtime status presentation.
- Terminal wrapping, truncation, column fitting, page order, and current-row markers.
- Executor queue-status snapshots and observer-failure isolation.

Run:

```bash
python3 -m pytest -q src/GPSR/gpsr_sys_ui/test
```
