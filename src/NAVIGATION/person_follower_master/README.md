# person_follower_master

Industrial-grade single-master person-following for this ROS 2 Humble workspace.

## What it does

Locks onto one specific person ("master") via OSNet Re-ID appearance features
and follows them while ignoring other people in the scene. CPU-only — no GPU
required.

## Pipeline

```
RealSense RGB+Depth ──► vision_node (YOLO11n + depth back-projection)
                            │ /cam_detections (with bbox)
                            ▼
                      reid_enricher_node (OSNet-x0_25, CPU)
                            │ /cam_detections_reid (+ 512-d embedding)
                            ▼
                      master_tracker_node
                        (Kalman + Hungarian assoc on motion+appearance,
                         master enrollment + Re-ID re-acquisition)
                            │ /master_target (+ /tracks, /master_markers)
                            ▼
                      follow_controller_node (PD + lidar safety)
                            │ /cmd_vel
                            ▼
                      cmd_vel_to_targets ──► Arduino base
```

## Master enrollment

By default: stand 0.8–2.0 m in front of the robot, centered (±15° bearing),
for 3 s. The robot logs `MASTER ENROLLED: track #N` and starts following.

Manual overrides:
```bash
ros2 service call /enroll_master std_srvs/srv/Trigger '{}'
ros2 service call /reset_master  std_srvs/srv/Trigger '{}'
```

## Loss behaviour

| Loss age      | Action                                          |
| ------------- | ----------------------------------------------- |
| < 2 s         | hold position, no motion                        |
| 2 – 10 s      | scan-rotate (±0.3 rad/s, flipping every 3 s)    |
| ≥ 10 s        | full stop + alarm log every 2 s                 |

Re-acquisition happens any time a CONFIRMED track matches the master gallery
with cosine ≥ 0.72 and lies within 3 m of the last-known position.

## Build

```bash
# Inside the iaig_nav conda env
pip install torchreid scipy
# Fallback if torchreid on PyPI is broken on your system:
#   pip install git+https://github.com/KaiyangZhou/deep-person-reid.git

cd ~/下载/ros2_ws/ros2_ws
source scripts/deploy/setup_env.sh
colcon build --symlink-install --packages-select vision person_follower_master
source install/setup.bash
```

## Run

```bash
bash src/person_follower_master/scripts/deploy/clean_launch_follow.sh
# common overrides
bash src/person_follower_master/scripts/deploy/clean_launch_follow.sh rviz:=false
bash src/person_follower_master/scripts/deploy/clean_launch_follow.sh camera:=false  # camera already launched elsewhere
```

## Follow-then-return-home (industrial mode)

For the full "follow the person, then retrace the path home" behaviour,
use the composite launch instead of the bare follow launch:

```bash
source scripts/deploy/setup_env.sh
ros2 launch person_follower_master person_follow_return.launch.py
```

This wires up (see [launch/person_follow_return.launch.py](launch/person_follow_return.launch.py)):

- drivers + RealSense + scan_self_filter (`/scan` → `/scan_filtered`)
- the stock follow stack (vision + reid + master_tracker + follow_controller),
  with `follow_controller` retargeted to `/cmd_vel_follow`
- `breadcrumb_teach_repeat` (my_robot_nav) recording the path during
  TEACHING and replaying it during RETURNING, routed to `/cmd_vel_return_raw`
- `return_safety_wrapper` — lidar-driven slow/stop/abort between breadcrumb
  and the mux
- `cmd_vel_mux` — single writer to `/cmd_vel`, arbitrating FOLLOW vs RETURN
- `session_manager` — lifecycle state machine with operator services

### Operator interface

The stack is hands-free by default: **stand in front of the robot for ~3 s to
enroll as master, and TEACHING auto-starts**. You just walk where you want
the robot to go, then step aside — it auto-returns. No terminal needed after
the initial launch.

```bash
# --- Hands-free (default flow) ---
# 1. Launch the stack (one time).
# 2. Stand 0.8-2.0 m in front, centered, 3 s.
#    -> master enrolled -> session_manager auto-fires start -> TEACHING.
# 3. Walk where you want the robot. It follows + records breadcrumbs.
# 4. When you're at the destination: turn to face the robot, stand 0.8-2.0 m
#    in front, centered, HOLD STILL for 5 s.
#    -> still-gesture fires -> RETURNING.
# 5. Robot drives back along the taught path with lidar-gated avoidance
#    (STOP / EVADE / ABORT as needed) and lands in ARRIVED when home.
#
# If you walk off without the still-gesture, the robot does NOT auto-return
# (a lost-master auto-trigger would let follow_controller's scan-rotate
# poison the taught path). Use /return_home/go_home to force it.

# --- Manual overrides (any time) ---
ros2 service call /return_home/start   std_srvs/srv/Trigger {}   # force TEACHING (no-op if already teaching)
ros2 service call /return_home/go_home std_srvs/srv/Trigger {}   # go home now (don't wait for lost timeout)
ros2 service call /return_home/stop    std_srvs/srv/Trigger {}   # E-stop, latch STOPPED
ros2 service call /return_home/reset   std_srvs/srv/Trigger {}   # clear STOPPED -> IDLE
```

Disable auto-start by setting `auto_start_on_master_enroll: false` in
[config/session_params.yaml](config/session_params.yaml) — then
`/return_home/start` becomes the only way to arm TEACHING.

> **Important:** the "home" pose is whatever pose the robot is in when
> TEACHING begins (whether that was the auto-trigger or a manual `start`).
> Park the robot where you want it to return to *before* enrolling as master.

### Status topics

```bash
ros2 topic echo /session_manager/state            # IDLE / TEACHING / RETURNING / ARRIVED / STOPPED
ros2 topic echo /session_manager/mode             # what the mux is following
ros2 topic echo /session_manager/diagnostics      # full health snapshot @ 5 Hz
ros2 topic echo /cmd_vel_mux/active_source        # which source drove /cmd_vel this tick
ros2 topic echo /return_safety/status             # CLEAR / SLOW / STOP / EVADE / EMERGENCY / ABORT
ros2 topic echo /return_safety/blocked_sec        # how long we've been blocked
ros2 topic echo /breadcrumb_path                  # taught path (RViz)
ros2 topic echo /breadcrumb_repeat_path           # remaining path to home (RViz)
```

### State machine

```
        master enrolled (auto)
        OR /return_home/start
   IDLE ──────────────────────────► TEACHING
     ▲                                  │
     │             master stands still facing robot >= 5 s
     │             OR /return_home/go_home
     │                                  │
     │                                  ▼
     │   arrival (within 0.25 m/0.35 rad of home,
     │   cmd=0 held 0.8 s)       RETURNING
     │                                  │
  ARRIVED ◄────────────────────────────┤   safety ABORT / timeout / stop
     │                                  ▼
     │                              STOPPED
     │       /return_home/reset         │
     └──────────────────────────────────┘
```

### Safety guarantees

| Failure mode                       | Behaviour                                 |
| ---------------------------------- | ----------------------------------------- |
| Any source (follow/return) silent > 0.25 s | mux publishes 0                   |
| `session_manager` dies / mode silent > 3 s | mux publishes 0                   |
| Master LOST continuously > 1.8 s during TEACHING | mux swallows `/cmd_vel_follow` (zero) — prevents follow_controller scan-rotate from being recorded into the breadcrumb path |
| `/scan_filtered` silent > 0.5 s during RETURN | safety wrapper publishes 0     |
| Obstacle < 0.25 m (any direction)  | full freeze (EMERGENCY)                   |
| Obstacle in forward cone < 0.35 m  | linear = 0, angular passes through (STOP) |
| STOP held > 3 s, one side ≥ 0.45 m clear | lateral strafe + forward creep (EVADE) |
| STOP held > 3 s, both sides blocked | stay STOP → eventual ABORT               |
| EVADE strafe > 0.60 m or > 8 s without clear | revert to STOP                   |
| Blocked > 60 s continuously (STOP/EMERGENCY/EVADE) | safety wrapper emits ABORT → STOPPED |
| RETURNING > max(60 s, 3 × teach)   | session_manager latches STOPPED           |

The stack prefers stop-and-wait first (giving people a chance to move) and
only escalates to EVADE after 3 s of continuous block. EVADE is a small
lateral slip past a static obstacle; once the forward cone clears (≥ 0.55 m,
with hysteresis above the 0.35 m stop band), breadcrumb's scan-matching
pure-pursuit re-acquires the taught path on its own. Set
`enable_evade: false` in `session_params.yaml` to revert to pure stop-and-wait.

### Tuning (see [config/session_params.yaml](config/session_params.yaml))

| Parameter                               | Default  | Purpose                                       |
| --------------------------------------- | -------- | --------------------------------------------- |
| `auto_go_home_on_still_sec`             | 5.0      | hold-still-facing-robot gesture to trigger RETURNING; 0 to disable |
| `cmd_vel_mux.master_lost_grace_sec`     | 1.8      | continuous master-lost time before mux swallows follow cmd (< 2.0 s scan-rotate threshold) |
| `arrival_tol_m` / `arrival_tol_yaw`     | 0.25 / 0.35 | how close to home counts as "arrived"      |
| `repeat_timeout_ratio`                  | 3.0      | budget = max(floor, ratio × teach_duration)   |
| `slow_start_range` / `stop_range`       | 1.20 / 0.35 | lidar slow-down band for RETURNING         |
| `emergency_range`                       | 0.25     | 360° freeze threshold                         |
| `abort_timeout_sec`                     | 60.0     | how long to stay blocked before giving up     |
| `enable_evade`                          | true     | allow lateral-strafe bypass of static obstacles |
| `evade_after_sec`                       | 3.0      | stop-and-wait grace before attempting EVADE   |
| `evade_speed` / `evade_forward_creep`   | 0.12 / 0.04 | strafe speed + forward creep during EVADE  |
| `evade_side_clearance_m`                | 0.45     | min side clearance required to pick that side |
| `evade_clear_margin_m`                  | 0.55     | forward clearance (hysteresis) to exit EVADE  |
| `evade_max_distance_m` / `evade_max_duration_sec` | 0.60 / 8.0 | EVADE budget before reverting to STOP |
| breadcrumb `record_distance` / `record_yaw` | 0.30 / 0.35 | breadcrumb spacing                     |
| breadcrumb `max_linear_speed`           | 0.18     | repeat-phase top speed (safety wrapper also scales) |

## Verification checklist

1. `ros2 topic hz /cam_detections` → ~10 Hz
2. `ros2 topic hz /cam_detections_reid` → ~10 Hz
3. `ros2 topic hz /master_target` → 20 Hz
4. Enrollment: one person at ~1.2 m, centered, 3 s → `state=FOLLOWING`
5. Multi-person: 3 visible → 3 CONFIRMED tracks, exactly one is_master
6. Occlusion: master behind box 1 s → track_id unchanged after reappear
7. Swap test: two people cross → track_id sticks with master
8. Long loss: out-of-view > 10 s → stop + alarm

## Tuning quick reference

| Symptom                        | Adjust                                                         |
| ------------------------------ | -------------------------------------------------------------- |
| ID switches too readily        | raise `relock_min_sim` 0.75→0.80, `relock_margin` 0.10→0.15    |
| Fails to recover after occlusion | lower `recovery_max_sim_gate` 0.72→0.68                      |
| Oscillates around target       | lower `kp_v` 0.60→0.40, widen `d_deadzone` 0.15→0.20           |
| Slow to turn                   | raise `kp_w` 1.20→1.6 (with `w_max` 0.60→0.80)                 |

## Notes

- IAIG-Nav's `safety_wrapper` and `collision_monitor` are **not** in this
  launch. This stack publishes `/cmd_vel` directly; the controller does its
  own lidar front-cone cutoff and acceleration clamping.
- Do not run this stack at the same time as `clean_launch_real.sh` — both
  write `/cmd_vel`.
- The dead-code files `src/vision/scripts/person_follower.py` and
  `person_follower_nav2.py` are not used by this stack (and no longer
  installed as executables).
