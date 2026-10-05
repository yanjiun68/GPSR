# robot_state_cache

`robot_state_cache` provides three reusable behaviors:

```text
fixed-size circular history
        +
latest-value sample-and-hold
        +
freshness tracking
```

The header-only `CircularCache<T>` is ROS-independent. The included String node is a reference ROS 2 adapter and a directly usable cache for text/JSON state. Other robot subsystems can instantiate the same template for ROI, pose, navigation, or manipulation types without sharing a global singleton.

## Data flow

```text
Producer (arbitrary rate)
          │
          ▼
     validator hook
          │ accepted only
          ▼
fixed preallocated ring ──► oldest-to-newest history
          │
          ▼
 latest accepted value
          │ read only
          ▼
 fixed-rate ROS timer
          │
          ├──► held value
          └──► freshness diagnostics
```

Timer publication never inserts a cache entry and never increments sequence. Sequence numbers start at `1`, advance only for accepted producer updates, and remain monotonic across `clear()` within one cache lifetime.

## Memory and overwrite behavior

Construction allocates a vector with exactly `cache_capacity` slots. Normal updates assign into the current slot and wrap the write index; the vector is never resized.

```text
capacity = 5

A B C D E
→ B C D E F
→ C D E F G
```

`get_history()` always returns logical order from oldest to newest even though physical slot order wraps.

## Freshness states

```text
EMPTY ──first accepted update──► VALID ──stale threshold──► STALE
  ▲                                 ▲                         │
  │ clear                           └────new update────────────┤
  │                                                           ▼
  └────────────────────────────── clear ◄────────────────── EXPIRED
                                      new update ─────────────► VALID
```

- `EMPTY`: no accepted input exists; no invented default value is published.
- `VALID`: `age < stale_timeout_sec`.
- `STALE`: `stale_timeout_sec <= age < expire_timeout_sec`.
- `EXPIRED`: `age >= expire_timeout_sec`.

STALE and EXPIRED preserve the held value and history for diagnostics. Output policy is separate: VALID always publishes, STALE follows `publish_stale_data`, and EXPIRED follows `publish_expired_data` (false by default).

## Public C++ API

`robot_state_cache::CircularCache<T>` provides:

- `update(value[, timestamp])`
- `get_latest()` and `get_latest_valid(...)`
- `get_history()` (oldest → newest)
- `get_history_count()`, `size()`, `capacity()`, and `empty()`
- `status(...)`, `age(...)`, and atomic `snapshot(...)`
- `get_by_sequence(...)`
- `clear()` without changing capacity

All operations are mutex-protected. Returned entries/history/snapshots are copies, so the internal mutex is released before downstream processing, ROS publication, or logging.

## ROS topics and QoS

- `/robot_state_cache/input`: `std_msgs/msg/String`, reliable volatile, depth 10.
- `/robot_state_cache/held`: `std_msgs/msg/String`, reliable transient-local, depth 1.
- `/robot_state_cache/status`: `diagnostic_msgs/msg/DiagnosticArray`, reliable transient-local, depth 1.

The held and status topics are state-oriented so a late consumer receives the newest publication. Input and held topic names must differ, preventing a self-feedback loop. Diagnostics expose `state`, `has_value`, `is_currently_valid`, `sequence`, `age_sec`, `history_count`, `capacity`, and `accepted_ros_time_sec`.

## Configuration and operation

```yaml
robot_state_cache:
  ros__parameters:
    cache_capacity: 10
    publish_rate_hz: 20.0
    stale_timeout_sec: 0.5
    expire_timeout_sec: 2.0
    publish_stale_data: true
    publish_expired_data: false
```

Run the example adapter:

```bash
ros2 launch robot_state_cache cache.launch.py
ros2 topic pub --once /robot_state_cache/input std_msgs/msg/String "{data: ROI_A}"
ros2 topic echo /robot_state_cache/held
ros2 topic echo /robot_state_cache/status
```

A 2 Hz producer and 20 Hz timer produce repeated held output between updates:

```text
input:   ROI_A                    ROI_B
held:    A A A A A A A A A A     B B B B B B B B B B
sequence 1 1 1 1 1 1 1 1 1 1     2 2 2 2 2 2 2 2 2 2
```
