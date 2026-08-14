# Person Following Config README

This document explains the parameters currently exposed in the `person_following` YAML configs under `config/profiles/<profile>/`.

It is intended to be both:

- a parameter reference
- a competition tuning and debugging guide

It documents the union of parameters currently present in the profile YAML files, not every parameter declared in code. Where a profile omits a key, the node falls back to its built-in default.

## Overview

Profiles live here:

- `config/profiles/competition_default/`
- `config/profiles/indoor_safe/`

The launch files load them as follows:

- `person_perception.launch.py`
  - `vision.yaml`
  - `lidar_candidates.yaml`
  - `tracker.yaml`
- `person_follow_nav2.launch.py`
  - `vision.yaml`
  - `lidar_candidates.yaml`
  - `tracker.yaml`
  - `target_lock.yaml`
  - `behavior.yaml`
  - `mapped_follow.yaml`
  - `unmapped_follow.yaml`
  - `safety.yaml`
  - `server.yaml`
  - `follow_ready_voice.yaml`
  - `boundary_zone_checker.yaml`

Important distinction:

- YAML-exposed params are the settings you actively tune here.
- Nodes also declare additional parameters in code for topics, frames, model paths, and launch wiring defaults.
- Some of those code-declared params are overridden directly by launch files and should not be treated as normal tuning knobs.

## Subsystem Ownership Map

- `vision_node` -> RGB-D detection quality, depth quality, facing cues
- `follow_person` -> LiDAR person-like candidate extraction
- `person_track_manager` -> multi-target association, ID stability, prediction, ReID fusion
- `target_lock_manager` -> selected-person consistency and publishability
- `follow_behavior_manager` -> follow/hold/search motion intent decisions
- `nav_follow_interface` -> mapped follow goal generation through Nav2
- `local_follow_interface` -> unmapped/reactive follow and bypass behavior
- `safety_supervisor` -> inhibit logic and stop reasons
- `person_follow_server` -> external action/service-facing session defaults
- `follow_ready_voice_server` -> spoken ready announcement behavior
- `boundary_zone_checker` -> zone transition gating and RViz override hooks

## Data Flow

```text
vision_node -> /cam_detections
follow_person -> /people_candidates
        \/
person_track_manager -> /person_tracks_state
        \/
target_lock_manager -> /person_target_locked
        \/
follow_behavior_manager -> /person_follow/motion_intent
        \/
nav_follow_interface or local_follow_interface -> /cmd_vel_follow
        \/
safety_supervisor gates execution and publishes /person_follow/safety_status
```

## Impact Levels

- `HIGH` -> directly affects tracking stability, lock quality, safety timing, or robot motion aggressiveness
- `MEDIUM` -> affects smoothness, responsiveness, or secondary filtering behavior
- `LOW` -> visualization, debug output, or wiring compatibility

## Gate Types

- Position gate
  - Euclidean or axis-aligned spatial tolerance.
  - If too small, valid matches are rejected.
  - If too large, nearby people or clutter can merge.
- Mahalanobis gate
  - Covariance-aware association gate used by the tracker.
  - Lower is stricter and usually safer in clean scenes.
  - Higher is more tolerant but increases wrong associations in crowds.
- Appearance/ReID gate
  - Uses visual similarity to keep identity stable.
  - Too strict blocks legitimate reacquisition after pose or lighting changes.
  - Too loose allows cross-person identity contamination.
- Score/selection gate
  - Final publish/switch thresholds for tracker and lock outputs.
  - Lower makes the system more willing to keep a target.
  - Higher makes it more conservative and stable, but easier to lose.

## Do Not Touch Casually

- Topic names unless you are deliberately rewiring the pipeline
- Frame IDs unless the TF tree has been changed consistently across the robot
- Launch-wired model/topic/action parameters unless you know the launch graph
- `nav_follow_interface.yaml`, which is only a deprecated compatibility shim

## Quick Tuning (Emergency)

- ID unstable:
  - slightly increase `maha_gate_thresh` only if the tracker is fragmenting
  - slightly reduce `appearance_gate` only if reacquisition is failing
  - raise `selected_match_min_score` or `lock_score_strong_min` if wrong-person switching is the problem
- Target lost too easily:
  - increase `max_occluded_frames`
  - increase `target_lock_lost_timeout_sec`
  - increase `target_timeout_sec` or `publishable_timeout_sec` carefully
- Robot too aggressive:
  - increase `follow_distance_m` or `follow_distance_default_m`
  - reduce `nominal_speed_mps`, `max_linear_speed_mps`, or `angular_speed_max_rad_s`
- Robot too hesitant:
  - reduce `follow_confidence_min`
  - reduce `goal_resend_sec` or raise `goal_update_hz`
  - reduce `slowdown_distance_m` only if obstacle behavior is clearly over-conservative

## Recommended Tuning Order

1. `vision.yaml`: make detections and depth stable first
2. `lidar_candidates.yaml`: make clusters clean and repeatable
3. `tracker.yaml`: stabilize ID and association
4. `target_lock.yaml`: stabilize the selected person output
5. `behavior.yaml`: tune follow/search/hold feel
6. `mapped_follow.yaml` or `unmapped_follow.yaml`: tune motion execution
7. `safety.yaml`: verify inhibits and timeout behavior

## Common Symptoms -> Related Params

- ID keeps switching between nearby people
  - `maha_gate_thresh`
  - `appearance_gate`
  - `track_position_gate_m`
  - `selected_match_min_score`
  - `lock_score_switch_margin`
- Target is lost during brief occlusion
  - `max_occluded_frames`
  - `temp_occluded_hold_sec`
  - `target_lock_lost_timeout_sec`
  - `target_timeout_sec`
  - `publishable_timeout_sec`
- Lock jumps after crossing trajectories
  - `crossover_confirm_frames`
  - `crossover_ambiguity_margin`
  - `contradiction_threshold`
  - `lock_score_crossover_penalty`
- Robot follows too aggressively
  - `follow_distance_m`
  - `nominal_speed_mps`
  - `max_linear_speed_mps`
  - `distance_kp`
  - `angular_speed_max_rad_s`
- Robot stops too early or too often
  - `stop_distance_m`
  - `critical_stop_distance_m`
  - `slowdown_distance_m`
  - `depth_stop_distance_m`
  - `obstacle_fault_timeout_sec`
- Mapped mode updates feel slow
  - `goal_update_hz`
  - `goal_resend_sec`
  - `min_goal_step_m`
  - `search_goal_resend_sec`
- Unmapped mode bypasses too often or not enough
  - `bypass_trigger_sec`
  - `bypass_duration_sec`
  - `wait_before_bypass_sec`
  - `side_clearance_m`
  - `corridor_half_width_m`

## Competition Tips

- Prefer stability over responsiveness in crowded scenes.
- Make conservative changes to tracker and lock thresholds close to match time.
- Increase tolerance for brief occlusion only as much as needed; too much tolerance makes wrong-person recovery harder.
- Reduce motion aggressiveness before increasing perception risk.
- Avoid retuning multiple subsystems in the same run unless the failure source is already clear.

## Rosbag Tuning Workflow

1. Record a bag for one failure pattern at a time: static target, lateral motion, short occlusion, crowd crossing, narrow corridor.
2. Replay with a fixed baseline profile.
3. Change one parameter group only: vision, lidar, tracker, lock, behavior, mapped, unmapped, or safety.
4. Compare:
   - track continuity
   - target lock continuity
   - switch behavior
   - stop/follow smoothness
5. Promote only changes that improve the intended failure mode without destabilizing another one.

## Real-Time Debug Tips

- Monitor `/person_tracks_state` first.
  - If IDs are already unstable here, debug `vision.yaml`, `lidar_candidates.yaml`, and `tracker.yaml`.
- Monitor `/person_target_locked` next.
  - If tracks are stable but the selected target is unstable, debug `target_lock.yaml`.
- If track and lock are stable but the robot behaves badly, debug:
  - `behavior.yaml`
  - `mapped_follow.yaml`
  - `unmapped_follow.yaml`
  - `safety.yaml`

## Profile Notes

- `competition_default` is generally more responsive and more permissive.
- `indoor_safe` is generally more conservative and slower.
- Most profile files have identical key sets, except `unmapped_follow.yaml`.
- `indoor_safe/unmapped_follow.yaml` omits 11 keys present in `competition_default`:
  - `unmapped_backend`
  - `goal_update_hz`
  - `goal_resend_sec`
  - `goal_min_step_m`
  - `goal_smoothing_alpha`
  - `max_local_goal_distance_m`
  - `goal_cancel_cooldown_sec`
  - `local_goal_frame`
  - `publish_debug_markers`
  - `debug_markers_topic`
  - `debug_status_topic`
- When those keys are missing, `local_follow_interface.py` uses its built-in defaults.

---

## `vision.yaml`

Node: `vision_node.py`  
Role: RGB-D detection, depth projection, optional face/facing features, GUI/debug overlays  
Launch wiring: loaded by both perception and full-stack launch files; launch usually overrides image topics, frame IDs, model path, provider list, and timing debug flag

### Detection and Facing

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `yolo_conf_thresh` | `0.55` | `0.8` | HIGH | Person detector confidence cutoff. Typical `0.55-0.8`. Lower finds more people but increases false positives; higher is cleaner but misses distant/partial people. Danger: too low creates ghost tracks, too high starves tracker. |
| `min_depth_valid_ratio` | `0.1` | `0.35` | HIGH | Minimum valid depth fraction inside the ROI. Typical `0.1-0.35`. Lower tolerates sparse depth; higher demands cleaner geometry. Danger: too high drops real targets with weak depth, too low accepts noisy range estimates. |
| `retinaface_enabled` | `true` | `true` | MEDIUM | Enables face/facing refinement. Keep on if GPU/CPU budget allows. Turning off reduces facing quality and some identity cues. |
| `retinaface_conf_thresh` | `0.7` | `0.7` | MEDIUM | Face detection confidence threshold. Typical `0.6-0.8`. Lower catches weak faces but adds junk; higher is cleaner but misses side views. |
| `retinaface_nms_iou` | `0.4` | `0.4` | MEDIUM | NMS overlap threshold for face proposals. Lower is stricter; higher keeps more overlapping boxes. Typical `0.3-0.5`. |
| `retinaface_every_n_frames` | `6` | `5` | MEDIUM | Face detection cadence. Lower updates facing more often but costs compute; higher reduces CPU/GPU load but makes facing cues stale. Danger: too high delays facing flips. |
| `facing_yaw_thresh_deg` | `25.0` | `25.0` | MEDIUM | Yaw limit for treating target as facing the robot. Lower is stricter; higher is more permissive. Typical `20-35`. |
| `facing_pitch_thresh_deg` | `20.0` | `20.0` | MEDIUM | Pitch limit for facing classification. Lower rejects more head tilt; higher tolerates posture changes. Typical `15-25`. |

### GUI, Sync, and Visualization

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `enable_gui` | `true` | `true` | LOW | Opens on-screen debug window. Leave on for lab tuning; disable on headless or competition deployments if display issues matter. |
| `publish_markers` | `true` | `true` | LOW | Publishes RViz markers. Safe to disable if bandwidth or CPU is tight. |
| `debug_mode` | `false` | `false` | LOW | Extra debug behavior/logging. Keep off unless diagnosing internals. |
| `locked_target_topic` | `/person_target_locked` | `/person_target_locked` | LOW | GUI lock-assist topic. Wiring param; do not change casually. |
| `gui_lock_match_gate_m` | `0.45` | `0.35` | MEDIUM | Spatial match radius for GUI-assisted target lock. Lower prevents accidental picks; higher tolerates more cursor/latency error. Typical `0.3-0.5`. |
| `gui_lock_timeout_sec` | `0.65` | `0.5` | MEDIUM | Time window to keep GUI lock candidate valid. Lower is stricter; higher tolerates delays. |
| `sync_queue` | `10` | `10` | MEDIUM | RGB/depth sync queue depth. Increase only if messages are dropping under jitter. Too high can increase stale pairings. |
| `sync_slop_sec` | `0.06` | `0.06` | MEDIUM | Approximate time sync tolerance. Typical `0.03-0.08`. Lower gives tighter pairing; higher tolerates sensor skew but risks mismatched RGB-depth. |
| `marker_scale` | `0.18` | `0.18` | LOW | RViz marker scale only. |
| `marker_lifetime_sec` | `0.5` | `0.5` | LOW | Marker persistence only. |
| `tf_timeout_sec` | `0.1` | `0.1` | MEDIUM | TF lookup timeout for projection. Too low causes dropped measurements under TF jitter; too high can hide TF lag. |
| `robot_radius_m` | `0.2655` | `0.2655` | MEDIUM | Used when rejecting detections too close to robot body. Change only if robot footprint changes. |
| `runtime_timing_debug` | `false` | `false` | LOW | Timing instrumentation only. |

### Depth ROI and Embedding Quality

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `depth_roi_scale_w` | `0.4` | `0.4` | MEDIUM | Width fraction of bbox sampled for depth. Lower reduces background contamination; higher uses more pixels. Typical `0.3-0.5`. |
| `depth_roi_scale_h` | `0.5` | `0.5` | MEDIUM | Height fraction of bbox sampled for depth. Similar tradeoff as width scale. |
| `retinaface_input_w` | `640` | `640` | LOW | Model input size. Keep aligned with model expectations. |
| `retinaface_input_h` | `640` | `640` | LOW | Model input size. Keep aligned with model expectations. |
| `retinaface_out_scores` | `''` | `''` | LOW | Optional ONNX output name override. Leave empty unless model export differs. |
| `retinaface_out_boxes` | `''` | `''` | LOW | Optional ONNX output name override. |
| `retinaface_out_landmarks` | `''` | `''` | LOW | Optional ONNX output name override. |
| `torso_region_height_ratio` | `0.22` | `0.22` | MEDIUM | Region split for body appearance features. Change only if crop composition is clearly wrong. |
| `upper_body_region_height_ratio` | `0.18` | `0.18` | MEDIUM | Region split for appearance cues. |
| `lower_body_region_height_ratio` | `0.18` | `0.18` | MEDIUM | Region split for appearance cues. |
| `hist_visible_ratio_min` | `0.6` | `0.6` | MEDIUM | Minimum visible fraction before histogram updates count. Lower updates with partial views; higher demands cleaner full-body visibility. |
| `face_embed_min_area_px` | `1024.0` | `1024.0` | MEDIUM | Minimum face area before embedding update. Too low admits blurry faces; too high blocks most updates. |
| `face_embed_max_yaw_deg` | `35.0` | `35.0` | MEDIUM | Yaw limit for face embedding updates. Higher tolerates more off-angle faces but reduces embedding quality. |
| `face_embed_max_pitch_deg` | `25.0` | `25.0` | MEDIUM | Pitch limit for face embedding updates. |
| `identity_quality_min_for_update` | `0.6` | `0.6` | MEDIUM | Minimum quality before identity features update. Lower adapts faster; higher is safer against contamination. |
| `crop_blur_max_laplacian` | `45.0` | `45.0` | MEDIUM | Blur gate for crop updates. Lower rejects more blurry crops; higher allows weaker imagery. |
| `crop_min_bbox_area_px` | `6000.0` | `6000.0` | MEDIUM | Minimum body box area for high-quality crop updates. Too low admits tiny noisy crops; too high blocks distant subjects. |
| `crop_min_visible_body_ratio` | `0.55` | `0.55` | MEDIUM | Minimum visible body ratio for crop updates. Lower tolerates occlusion; higher prefers clean full views. |
| `crop_aspect_ratio_min` | `0.25` | `0.25` | MEDIUM | Lower aspect bound for valid body crops. |
| `crop_aspect_ratio_max` | `0.8` | `0.8` | MEDIUM | Upper aspect bound for valid body crops. Too wide may accept merged detections. |

---

## `lidar_candidates.yaml`

Node: `follow_person.py`  
Role: extracts person-like LiDAR clusters before fusion  
Launch wiring: loaded by both launch files; launch usually overrides `scan_topic`, `base_frame`, and marker publication flag

### Spatial Crop and Clustering

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `min_range_m` | `0.2` | `0.2` | MEDIUM | Ignore very close returns. Raise only if the scanner sees robot structure. |
| `max_range_m` | `6.0` | `6.0` | MEDIUM | Ignore very distant returns. Lower gives cleaner close-follow candidates; higher sees farther but noisier shapes. |
| `min_x_m` | `0.2` | `0.0` | HIGH | Front crop minimum in robot frame. Competition profile ignores points almost on the robot. Too low can keep robot-edge clutter. |
| `max_x_m` | `3.5` | `6.0` | HIGH | Front crop maximum. Lower reduces distant clutter; higher sees farther candidates. Danger: too high in crowds creates extra ghost candidates. |
| `min_y_m` | `-1.5` | `-3.0` | HIGH | Left-right crop minimum. Narrower corridor reduces side clutter; wider corridor admits side people. |
| `max_y_m` | `1.5` | `3.0` | HIGH | Left-right crop maximum. Same tradeoff as `min_y_m`. |
| `cluster_dist_m` | `0.16` | `0.14` | HIGH | Point clustering tolerance. Typical `0.12-0.18`. Lower splits bodies apart; higher merges adjacent objects. Danger: too high merges legs and clutter. |
| `min_cluster_points` | `5` | `4` | HIGH | Minimum points to keep a cluster. Lower sees weaker targets; higher removes noise. Too high misses distant people. |
| `max_cluster_points` | `80` | `80` | MEDIUM | Maximum cluster size. Prevents large walls/furniture from being treated as people. |
| `min_cluster_size_m` | `0.05` | `0.05` | MEDIUM | Minimum physical cluster size. Lower keeps tiny noise; higher may reject distant legs. |
| `max_cluster_size_m` | `0.6` | `0.6` | HIGH | Maximum physical cluster size for a person-like object. Too high accepts carts or walls; too low rejects wide stances. |

### Candidate Aging and Person-Likeness

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `lidar_candidate_max_age_sec` | `0.8` | `0.6` | MEDIUM | Maximum age for a candidate before it is considered stale. Higher smooths jitter; lower is cleaner but more fragmentary. |
| `lidar_candidate_min_persistence_frames` | `3` | `4` | HIGH | How many frames a cluster must persist before it counts strongly. Lower is responsive; higher is safer indoors. |
| `lidar_width_person_min_m` | `0.1` | `0.1` | HIGH | Minimum accepted cluster width for person-likeness. Too low admits poles/noise. |
| `lidar_width_person_max_m` | `0.6` | `0.75` | HIGH | Maximum accepted cluster width. Too high accepts furniture; too low rejects bags or wider stances. |
| `lidar_width_stability_max_std` | `0.14` | `0.1` | HIGH | Allowed width variation over time. Lower requires more stable person-like shape; higher tolerates noisy scans. |
| `lidar_candidate_assoc_gate_m` | `0.3` | `0.25` | MEDIUM | Association distance used while aging candidate clusters. Lower reduces merges; higher tolerates motion. |
| `lidar_person_likeness_min` | `0.6` | `0.5` | HIGH | Minimum heuristic score to treat cluster as person-like. Lower increases recall; higher is cleaner. Danger: too low floods fusion with clutter. |

### Visualization and Wiring

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `publish_markers` | `true` | `true` | LOW | RViz markers only. |
| `tf_timeout_sec` | `0.1` | `0.1` | MEDIUM | TF wait for cluster projection. Too low drops candidates under TF jitter. |
| `marker_scale_m` | `0.2` | `0.2` | LOW | Marker size only. |
| `marker_lifetime_sec` | `0.1` | `0.5` | LOW | Marker persistence only. |

---

## `tracker.yaml`

Node: `person_track_manager.py`  
Role: fuses camera and LiDAR, manages track lifecycle, preserves identity, predicts selected target, and publishes track state  
Launch wiring: loaded by both launch files; launch usually overrides topics, frames, ReID model path/providers, and timing debug flag

### Fusion and Base Association

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `auto_select_nearest_on_start` | `true` | `true` | HIGH | Auto-picks nearest initial target. Good for demo/competition flow; disable only if selection is always external. |
| `crowd_mode` | `true` | `true` | HIGH | Enables crowd-aware weighting logic. Keep on in multi-person environments. |
| `cam_lidar_gate_m` | `0.28` | `0.24` | HIGH | Camera/LiDAR fusion gate. Typical `0.2-0.35`. Lower prevents false fusion; higher tolerates calibration noise. Danger: too high merges nearby people. |
| `track_position_gate_m` | `0.48` | `0.38` | HIGH | Base spatial association gate for maintaining tracks. Typical `0.35-0.6`. Too low fragments tracks; too high causes ID swaps. |
| `allow_camera_only_updates` | `true` | `true` | HIGH | Allows track maintenance when only camera sees target. Usually keep on. Turning off makes tracking brittle when LiDAR drops out. |
| `allow_lidar_only_updates` | `false` | `false` | HIGH | Allows generic LiDAR-only updates. Off is safer for identity consistency. Turning on increases clutter-driven mistakes. |
| `allow_lidar_only_for_selected` | `true` | `true` | HIGH | Lets the selected target bridge briefly through LiDAR-only evidence. Usually valuable for short camera dropouts. |
| `selected_lidar_bridge_sec` | `1.0` | `0.9` | HIGH | How long selected target may bridge through LiDAR-only support. Too high risks wrong-person bridging; too low loses target quickly. |
| `bridge_start_delay_sec` | `0.2` | `0.2` | MEDIUM | Delay before LiDAR bridge logic engages. Prevents instant switching into bridge mode on brief camera gaps. |
| `selected_track_stale_max_sec` | `1.1` | `0.5` | HIGH | Maximum staleness allowed before selected-track handling gives up. Higher helps occlusion; lower is stricter. |
| `selected_lidar_gate_m` | `0.35` | `0.24` | HIGH | LiDAR bridge distance gate for the selected target. Typical `0.2-0.4`. Too high grabs the wrong person; too low misses the right one. |
| `selected_lidar_bearing_gate_deg` | `28.0` | `18.0` | HIGH | Bearing gate for selected-target LiDAR bridge. Lower is stricter in crowds; higher tolerates angle changes. |
| `selected_lidar_max_speed_mps` | `1.4` | `1.2` | HIGH | Rejects implausibly fast bridge candidates. Raise only if target motion is legitimately faster. |
| `selected_lidar_confirm_hits` | `2` | `3` | HIGH | Number of consistent LiDAR hits needed before selected bridge is trusted. Higher is safer but slower. |

### Association Gates and Scoring

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `appearance_gate` | `0.13` | `0.11` | HIGH | Appearance match acceptance gate. Lower is stricter. Typical current use is `0.1-0.15`. Danger: too low blocks reacquisition, too high allows identity bleed. |
| `association_gate_mode` | `mahalanobis` | `mahalanobis` | HIGH | Core association strategy. Keep as `mahalanobis` unless code changes justify another mode. |
| `maha_gate_thresh` | `9.21` | `8.5` | HIGH | Main Mahalanobis association gate. Typical `8-10`. Lower is conservative; higher accepts more uncertain matches. Danger: too high causes wrong associations in crowds. |
| `occluded_maha_gate_thresh` | `7.0` | `6.5` | HIGH | Tighter Mahalanobis gate during occlusion handling. Typical `6-8`. Lower is safer; higher helps reacquisition through uncertainty. |
| `enable_target_lock_publish` | `false` | `true` | MEDIUM | Publishes internal target lock info from tracker. Usually leave as profile author intended. More debug visibility when true. |
| `crowd_cam_lidar_gate_m` | `0.5` | `0.5` | HIGH | Wider fusion gate used in crowd logic. Keep only as wide as needed for calibration error; too wide merges neighbors. |
| `lidar_only_recovery_mode` | `true` | `true` | HIGH | Enables special recovery behavior when camera evidence is gone but LiDAR may still support continuity. Usually keep on for short camera dropouts. |
| `recovery_camera_lost_sec` | `1.0` | `1.0` | HIGH | Time after camera loss before recovery logic becomes relevant. Lower reacts sooner; higher waits for camera to return first. |
| `selected_lidar_min_x_m` | `0.1` | `0.1` | MEDIUM | Minimum forward distance for selected-target LiDAR bridging. Avoids locking onto near-robot clutter. |
| `candidate_same_gate_m` | `0.12` | `0.12` | MEDIUM | Distance threshold for treating candidates as the same. Smaller reduces merges; larger consolidates duplicates. |
| `selected_lidar_ambiguity_ratio` | `1.25` | `1.2` | HIGH | Required score margin between best and second-best LiDAR bridge candidate. Higher is safer but less tolerant. |
| `bridge_score_w_dist` | `0.6` | `0.65` | MEDIUM | Weight for distance in bridge candidate scoring. Higher favors spatial proximity. |
| `bridge_score_w_bearing` | `0.3` | `0.25` | MEDIUM | Weight for bearing consistency in bridge scoring. |
| `bridge_score_w_speed` | `0.1` | `0.1` | MEDIUM | Weight for speed plausibility in bridge scoring. |
| `min_cam_confidence` | `0.45` | `0.5` | HIGH | Minimum camera detection confidence to update tracker. Lower increases recall; higher reduces noisy updates. |
| `occluded_app_gate` | `0.35` | `0.35` | HIGH | Appearance tolerance during occlusion handling. Larger is more forgiving. Danger: too high can relock the wrong person after crossover. |

### Lifecycle and Score Dynamics

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `max_msg_age_sec` | `0.5` | `0.45` | MEDIUM | Maximum sensor message age accepted by tracker. Lower protects against stale data; higher tolerates comms jitter. |
| `min_hits_confirm` | `2` | `1` | HIGH | Hits needed before a tentative track becomes confirmed. Lower is responsive; higher is cleaner. |
| `max_missed_frames` | `15` | `20` | HIGH | Missed updates allowed before degrading track state. Higher tolerates dropout; lower prunes faster. |
| `max_occluded_frames` | `45` | `45` | HIGH | Frames allowed in occluded state. One of the main occlusion survival knobs. Too high prolongs stale identities. |
| `max_lost_frames` | `20` | `20` | HIGH | Frames allowed in lost state before deletion. Higher preserves recovery window; lower cleans dead tracks sooner. |
| `confirm_score_thresh` | `0.5` | `0.5` | MEDIUM | Score needed to confirm a track. Lower confirms faster; higher requires more evidence. |
| `delete_score_thresh` | `0.2` | `0.2` | MEDIUM | Score below which a track is deleted. Lower keeps weak tracks longer; higher deletes aggressively. |
| `score_hit_gain` | `0.12` | `0.12` | MEDIUM | Score increase per good update. Higher confirms tracks faster. |
| `score_miss_penalty` | `0.04` | `0.04` | MEDIUM | Score decrease per miss. Higher prunes more aggressively. |

### Selected Target Persistence and Reacquisition

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `target_lock_lost_timeout_sec` | `2.4` | `2.0` | HIGH | How long tracker keeps selected target context after lock loss. Increase for brief occlusions; too high risks stale identity persistence. |
| `temp_occluded_hold_sec` | `1.0` | `0.9` | HIGH | Temporary occlusion hold time. Increase if target disappears briefly behind people or poles. |
| `selected_track_internal_stale_sec` | `2.0` | `2.0` | MEDIUM | Internal stale limit for selected track reasoning. |
| `selected_hard_hold_sec` | `1.2` | `1.1` | HIGH | Short post-selection no-switch protection window. Too low allows instant jumping; too high prevents necessary corrections. |
| `selected_reacquire_window_sec` | `2.4` | `1.2` | HIGH | Time window for reacquiring the previous selected target. Higher helps continuity; lower is safer in crowds. |
| `selected_no_switch_during_hard_hold` | `true` | `true` | HIGH | Enforces no-switch policy during hard hold. Keep true for stable selection. |
| `switch_confirm_frames` | `10` | `12` | HIGH | Frames required before switching selected target. Higher means fewer switches, slower corrections. |
| `reacquire_confirm_frames` | `3` | `9` | HIGH | Evidence needed to reacquire selected target. Higher is safer but much slower. |
| `front_occluder_range_margin_m` | `0.4` | `0.4` | MEDIUM | Margin used when reasoning about front occluders. Larger is more tolerant of partial blockage. |
| `bbox_size_change_gate` | `0.65` | `0.5` | HIGH | Allowed bbox size change for matching selected target. Lower rejects abrupt size changes; higher tolerates depth jumps. |
| `selected_bearing_gate_deg` | `48.0` | `40.0` | HIGH | Bearing gate for selected-target matching. Lower reduces side-switching; higher tolerates motion. |
| `selected_match_min_score` | `0.62` | `0.72` | HIGH | Minimum selected-target match score to keep same person. One of the first switch-stability knobs. |
| `selected_match_clean_update_score` | `0.72` | `0.88` | HIGH | Score level considered a clean selected-target update. Higher is more conservative. |
| `selected_match_ambiguous_margin` | `0.24` | `0.28` | HIGH | Margin needed over competing candidate before selected match is trusted. Higher is safer in crowds. |
| `selected_center_corridor_px` | `170.0` | `180.0` | MEDIUM | Preferred image-center corridor for selected target continuity. Larger corridor is more permissive. |
| `selected_history_min_count` | `4` | `4` | MEDIUM | Required history support for some selected-target decisions. Higher favors mature tracks. |
| `selected_bbox_overlap_min_iou` | `0.16` | `0.24` | HIGH | Minimum bbox overlap for selected-target consistency checks. Higher is stricter and safer. |

### Relock, Crossover, and Contradiction Handling

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `relock_cooldown_sec` | `0.6` | `1.8` | HIGH | Cooldown after relock before another relock. Higher reduces oscillation; too high delays recovery. |
| `post_relock_protection_sec` | `0.9` | `1.8` | HIGH | Extra protection window after relock. Larger is safer against instant switch-back. |
| `post_relock_switch_margin` | `0.28` | `0.32` | HIGH | Extra score margin needed to switch soon after relock. Higher makes relock stickier. |
| `post_relock_min_score` | `0.66` | `0.78` | HIGH | Minimum score for post-relock confidence. Higher is more conservative. |
| `crossover_enter_range_delta_m` | `0.45` | `0.45` | MEDIUM | Range similarity threshold for entering crossover logic. |
| `crossover_enter_bearing_delta_deg` | `10.0` | `10.0` | MEDIUM | Bearing similarity threshold for crossover detection. |
| `crossover_enter_center_delta_px` | `50.0` | `50.0` | MEDIUM | Image-center similarity threshold for crossover detection. |
| `crossover_enter_bbox_ratio_delta` | `0.2` | `0.2` | MEDIUM | Bbox scale similarity threshold for crossover detection. |
| `crossover_exit_stable_sec` | `0.8` | `0.8` | HIGH | Stable time required before exiting crossover logic. |
| `crossover_confirm_frames` | `8` | `8` | HIGH | Frames needed to confirm crossover state. Increase if crossover detection is too jumpy. |
| `crossover_ambiguity_margin` | `0.22` | `0.22` | HIGH | Margin used when deciding if a crossover is ambiguous. Lower triggers ambiguity more often. |
| `crossover_min_relock_score` | `0.8` | `0.8` | HIGH | Minimum score required to relock during crossover recovery. |
| `single_strong_cue_min_score` | `0.76` | `0.82` | HIGH | Score threshold allowing one dominant cue to drive decisions. Higher is safer but less responsive. |
| `contradiction_threshold` | `0.4` | `0.4` | HIGH | When evidence is considered contradictory. Lower flags contradiction sooner. |
| `contradictory_candidate_margin` | `0.22` | `0.24` | HIGH | Margin used when calling another candidate contradictory. Higher requires a clearer conflict. |

### Appearance Memory and Gallery

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `color_hist_alpha` | `0.25` | `0.25` | MEDIUM | EMA rate for color histogram update. Lower preserves history; higher adapts faster but risks contamination. |
| `stable_anchor_min_confidence` | `0.8` | `0.8` | MEDIUM | Confidence needed before track becomes a stable appearance anchor. |
| `stable_anchor_max_age_sec` | `8.0` | `8.0` | MEDIUM | Max age for using anchor appearance. Longer keeps older identity memory. |
| `identity_freeze_after_relock_sec` | `1.0` | `1.0` | HIGH | Temporarily freezes identity updates after relock. Helps avoid immediate appearance contamination. |
| `weak_appearance_penalty` | `0.18` | `0.18` | MEDIUM | Penalty when appearance evidence is weak. Higher makes tracker trust appearance less. |
| `lidar_only_selected_penalty` | `0.22` | `0.22` | MEDIUM | Penalty when selected target is maintained mostly by LiDAR-only evidence. |
| `gallery_max_size` | `6` | `6` | MEDIUM | Max stored appearance exemplars per track. Higher increases memory diversity but slows/loosens identity matching. |
| `gallery_min_diversity_distance` | `0.1` | `0.1` | MEDIUM | Minimum separation between gallery entries. Lower stores near-duplicates; higher stores only varied views. |
| `reid_input_w` | `128` | `128` | LOW | Model input size. Keep aligned with model expectations. |
| `reid_input_h` | `256` | `256` | LOW | Model input size. Keep aligned with model expectations. |
| `embedding_ema_alpha` | `0.3` | `0.3` | MEDIUM | EMA update rate for embeddings. Lower is stable; higher adapts faster. |
| `embedding_gallery_size` | `10` | `10` | MEDIUM | Internal embedding gallery length. |
| `embedding_update_min_conf` | `0.7` | `0.7` | MEDIUM | Minimum confidence before embedding update. |
| `embedding_update_max_assoc_cost` | `0.45` | `0.4` | HIGH | Maximum association cost still allowed to update embedding. Lower is safer; higher risks contaminating identity memory. |
| `reid_min_bbox_area_px` | `2500.0` | `2500.0` | MEDIUM | Minimum bbox area for ReID update. Too low accepts tiny poor crops. |
| `reid_update_max_abs_yaw_deg` | `45.0` | `40.0` | MEDIUM | Yaw limit for updating ReID. Lower is cleaner; higher adapts to angled views. |
| `reid_update_max_abs_pitch_deg` | `30.0` | `28.0` | MEDIUM | Pitch limit for updating ReID. |

### Prediction and Motion Model

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `predicted_target_max_age_sec` | `0.45` | `0.3` | HIGH | Max age of predicted selected-target state for control/continuity logic. Higher smooths gaps; too high uses stale prediction. |
| `predicted_target_temp_lost_max_age_sec` | `0.4` | `0.18` | HIGH | Prediction age limit while temporarily lost. Lower is safer, higher is more tolerant. |
| `predicted_target_max_covariance` | `0.45` | `0.35` | HIGH | Rejects predictions that become too uncertain. Lower is conservative. |
| `predicted_target_max_speed_m_s` | `1.4` | `1.1` | HIGH | Rejects implausibly fast predicted target motion. Higher tolerates sprint-like motion, lower is safer indoors. |
| `predicted_target_max_horizon_sec` | `0.35` | `0.28` | HIGH | Prediction horizon. Larger bridges gaps longer but increases drift risk. |
| `process_noise_pos` | `0.1` | `0.1` | MEDIUM | Process noise for position. Increase if motion model is too stiff; decrease if prediction jitters. |
| `process_noise_vel` | `0.5` | `0.5` | MEDIUM | Process noise for velocity. Higher tolerates erratic motion but widens uncertainty. |
| `meas_noise_lidar` | `0.08` | `0.08` | MEDIUM | LiDAR measurement noise assumption. Lower makes LiDAR dominate more. |
| `meas_noise_camera` | `0.2` | `0.2` | MEDIUM | Camera measurement noise assumption. Lower trusts camera more. |
| `imm_transition_stay_prob` | `0.9` | `0.9` | MEDIUM | Probability IMM stays in same motion mode. Higher gives smoother mode persistence. |
| `imm_turn_rate_rad_s` | `0.9` | `0.8` | MEDIUM | Expected turn-rate model. |
| `imm_stop_tau_sec` | `0.6` | `0.6` | MEDIUM | Stop model time constant. |
| `imm_q_vel_turn_scale` | `1.4` | `1.4` | MEDIUM | Process noise scaling in turn mode. |
| `imm_q_vel_stop_scale` | `0.3` | `0.3` | MEDIUM | Process noise scaling in stop mode. |
| `imm_init_mu_cv` | `0.6` | `0.6` | LOW | Initial constant-velocity mode probability. |
| `imm_init_mu_turn` | `0.25` | `0.25` | LOW | Initial turning mode probability. |
| `imm_init_mu_stop` | `0.15` | `0.15` | LOW | Initial stop mode probability. |

### Matching Weights and Debug

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `w_pos` | `0.55` | `0.55` | HIGH | Base association weight for position. Raising it makes tracker more geometry-driven. |
| `w_app` | `0.35` | `0.35` | HIGH | Base association weight for appearance. Raising it helps identity in clean vision, but hurts under poor lighting. |
| `w_motion` | `0.1` | `0.1` | MEDIUM | Base association weight for motion consistency. |
| `crowd_w_pos` | `0.35` | `0.38` | HIGH | Position weight in crowd mode. Lowering relative to appearance helps distinguish nearby people visually. |
| `crowd_w_app` | `0.55` | `0.52` | HIGH | Appearance weight in crowd mode. Too high can overtrust poor ReID crops. |
| `crowd_w_motion` | `0.1` | `0.1` | MEDIUM | Motion weight in crowd mode. |
| `invalid_cost` | `1000000.0` | `1000000.0` | LOW | Sentinel cost for impossible associations. Do not tune casually. |
| `publish_debug_markers` | `true` | `true` | LOW | RViz markers only. |
| `track_marker_scale_m` | `0.2` | `0.2` | LOW | Marker size only. |
| `target_marker_scale_m` | `0.24` | `0.24` | LOW | Marker size only. |
| `marker_lifetime_sec` | `0.25` | `0.25` | LOW | Marker persistence only. |
| `update_period_sec` | `0.02` | `0.02` | MEDIUM | Tracker loop period. Lower means faster updates but more CPU load. |
| `camera_meas_noise_min` | `0.18` | `0.18` | MEDIUM | Lower bound for adaptive camera noise. |
| `camera_meas_noise_max` | `0.7` | `0.7` | MEDIUM | Upper bound for adaptive camera noise. |
| `depth_quality_weight` | `0.55` | `0.55` | MEDIUM | Weight of depth quality in adaptive camera trust. |
| `bbox_area_weight` | `0.25` | `0.25` | MEDIUM | Weight of bbox area in adaptive camera trust. |
| `runtime_timing_debug` | `false` | `false` | LOW | Timing instrumentation only. |

---

## `target_lock.yaml`

Node: `target_lock_manager.py`  
Role: converts tracker state into a final selected target stream with publishability and stickiness rules  
Launch wiring: loaded by full-stack launch; launch usually overrides frames and target topics

### Lock Persistence and Publishability

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `target_timeout_sec` | `0.65` | `1.0` | HIGH | Max age before target lock is considered stale. Lower is responsive; higher keeps target alive through short gaps. Danger: too high hides stale lock output. |
| `publishable_timeout_sec` | `0.32` | `0.55` | HIGH | Age limit for target to stay publishable. Lower is stricter; higher smooths brief dropouts. |
| `combined_publishable_min` | `0.32` | `0.5` | HIGH | Minimum combined lock score to publish. Higher is safer and more conservative. |
| `relock_stability_cycles` | `3` | `6` | HIGH | Number of stable cycles required before relock is accepted. Higher reduces oscillation. |
| `relock_cooldown_sec` | `0.25` | `1.5` | HIGH | Cooldown between relocks. Larger strongly suppresses jumping. |
| `current_lock_stale_hold_sec` | `0.45` | `0.75` | HIGH | Extra hold time for current lock while evidence is slightly stale. Helps continuity, but too high keeps bad locks alive. |
| `same_target_position_gate_m` | `0.35` | `0.4` | HIGH | Position gate for deciding whether a candidate is still the same selected person. |
| `same_target_staleness_gate_sec` | `0.55` | `1.0` | HIGH | Time gate for treating target as continuous. Higher is more tolerant to delays. |
| `publish_timer_period_sec` | `0.1` | `0.01` | MEDIUM | Publish decision loop period. Lower is more responsive but higher CPU and noisier updates. |

### Lock Score Composition

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `lock_score_w_track` | `0.22` | `0.2` | HIGH | Weight for tracker quality in lock score. |
| `lock_score_w_confidence` | `0.25` | `0.28` | HIGH | Weight for confidence in lock score. Higher makes lock more detection-confidence-driven. |
| `lock_score_w_identity` | `0.16` | `0.18` | HIGH | Weight for identity consistency. Raise if wrong-person switches are the main issue. |
| `lock_score_w_motion` | `0.08` | `0.08` | MEDIUM | Weight for motion consistency. |
| `lock_score_w_freshness` | `0.14` | `0.14` | HIGH | Weight for freshness. Higher punishes stale candidates more. |
| `lock_score_w_facing` | `0.0` | `0.04` | MEDIUM | Weight for facing-the-robot cue. Use cautiously; facing is helpful but can be intermittent. |
| `lock_score_w_history` | `0.1` | `0.12` | MEDIUM | Weight for continuity/history support. |
| `lock_score_selected_bonus` | `0.12` | `0.14` | HIGH | Bonus for keeping current selected target. Higher improves stickiness. |
| `lock_score_publishable_bonus` | `0.05` | `0.05` | MEDIUM | Bonus for already-publishable candidates. |

### Penalties and Decision Thresholds

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `lock_score_contradiction_penalty` | `0.25` | `0.28` | HIGH | Penalty for contradictory evidence. Larger reduces risky switches. |
| `lock_score_crossover_penalty` | `0.16` | `0.18` | HIGH | Penalty during crossover scenarios. Increase if lock jumps during path crossing. |
| `lock_score_stale_penalty` | `0.18` | `0.2` | HIGH | Penalty for stale data. Higher prevents stale locks from surviving. |
| `lock_score_visibility_penalty` | `0.05` | `0.12` | MEDIUM | Penalty for poor visibility. Higher favors only cleanly visible targets. |
| `lock_score_switch_margin` | `0.12` | `0.12` | HIGH | Required score advantage before switching lock. One of the key anti-jump knobs. |
| `lock_score_retain_margin` | `0.04` | `0.05` | HIGH | Margin favoring current lock retention. Higher makes the lock stickier. |
| `lock_score_strong_min` | `0.62` | `0.76` | HIGH | Threshold for strong lock quality. Higher makes lock output more conservative. |
| `lock_score_weak_min` | `0.45` | `0.58` | HIGH | Threshold for weak-but-usable lock quality. |
| `lock_score_unsure_min` | `0.28` | `0.4` | HIGH | Threshold below which candidates are essentially not lock-worthy. |
| `target_lock_debug_topic` | `/person_target_lock_debug` | `/person_target_lock_debug` | LOW | Debug topic wiring. Do not change casually. |

---

## `behavior.yaml`

Node: `follow_behavior_manager.py`  
Role: turns target state and safety state into high-level follow/search/hold motion intent  
Launch wiring: loaded by full-stack launch; launch usually overrides target/facing/enable topics and frames

### Core Follow State

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `follow_distance_m` | `0.8` | `0.85` | HIGH | Desired follow distance. Larger is safer and calmer; smaller is more aggressive. Typical `0.8-1.0`. |
| `lock_confirm_sec` | `0.15` | `0.35` | HIGH | Time required before target lock is treated as confirmed for behavior state. Lower responds faster; higher is safer against flapping. |
| `target_soft_timeout_sec` | `1.2` | `1.0` | HIGH | Age where behavior starts to distrust target but has not fully abandoned it. |
| `target_hard_timeout_sec` | `2.0` | `1.8` | HIGH | Age where behavior gives up and transitions harder toward stop/search logic. |
| `search_timeout_sec` | `8.0` | `6.0` | HIGH | Maximum search duration before behavior gives up. Longer is more persistent but can waste match time. |
| `follow_confidence_min` | `0.6` | `0.78` | HIGH | Confidence needed for normal follow behavior. Lower is more permissive; higher is more conservative. |
| `cautious_confidence_min` | `0.42` | `0.6` | HIGH | Lower confidence threshold that still allows cautious following. |
| `stop_distance_m` | `0.45` | `0.5` | HIGH | Distance where behavior prefers to stop instead of continue approaching. |
| `hold_distance_m` | `1.0` | `1.0` | HIGH | Distance used for hold behavior logic around the target. |

### Motion Intent Shape

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `nominal_speed_mps` | `0.55` | `0.4` | HIGH | Nominal commanded speed hint in stable follow. Lower is calmer and safer. |
| `cautious_speed_mps` | `0.3` | `0.22` | HIGH | Reduced speed hint used in cautious follow states. |
| `face_hold_sec` | `10.0` | `10.0` | MEDIUM | Time to keep face/facing-related hold behavior. |
| `translate_staleness_sec` | `0.6` | `0.45` | HIGH | How stale target data can be before translation is suppressed. Lower prevents chasing stale poses. |
| `rotate_bearing_deg` | `25.0` | `22.0` | MEDIUM | Bearing threshold for rotate/search emphasis. |
| `search_rotate_speed_rad_s` | `0.45` | `0.35` | MEDIUM | Rotate speed during search. Lower is calmer and safer; higher searches faster but looks aggressive. |
| `behavior_status_topic` | `/person_follow/behavior_status` | `/person_follow/behavior_status` | LOW | Wiring topic. Do not change casually. |
| `target_distance_topic` | `/person_follow/target_distance_m` | `/person_follow/target_distance_m` | LOW | Publishes the current planar distance from robot to selected target. |

---

## `mapped_follow.yaml`

Node: `nav_follow_interface.py`  
Role: converts target and motion intent into Nav2 goals for mapped following  
Launch wiring: loaded by full-stack launch; launch usually overrides frames, Nav2 action name, `cmd_vel_raw_topic`, and follow topics

### Goal Generation

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `default_follow_mode` | `mapped` | `mapped` | HIGH | Startup mode expectation. Keep aligned with `safety.yaml`, `server.yaml`, and `unmapped_follow.yaml` unless mode switching is intentional. |
| `goal_update_hz` | `2.0` | `1.5` | HIGH | Goal refresh rate. Higher updates path faster but can churn Nav2 more. Typical `1.5-3.0`. |
| `min_goal_step_m` | `0.3` | `0.25` | HIGH | Minimum target motion before resending a new goal. Lower is more responsive; higher reduces goal spam. |
| `goal_resend_sec` | `2.5` | `2.0` | HIGH | Periodic goal resend timeout. Lower refreshes Nav2 more aggressively. |
| `max_goal_distance_m` | `2.5` | `2.0` | HIGH | Clamp on how far ahead the follow goal may be sent. Lower is more conservative and less aggressive. |

### Goal Churn and Rotation

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `goal_resend_cooldown_sec` | `1.25` | `1.0` | MEDIUM | Minimum time between resends. Lower reacts faster; higher prevents excessive churn. |
| `goal_cancel_cooldown_sec` | `1.0` | `0.8` | MEDIUM | Minimum time between cancellations. Lower is more reactive but can thrash. |
| `yaw_deadband_deg` | `7.0` | `6.0` | MEDIUM | Small-angle tolerance before commanding more rotation emphasis. |
| `angular_speed_min_rad_s` | `0.12` | `0.12` | MEDIUM | Minimum angular command used during search/rotation shaping. |
| `angular_speed_max_rad_s` | `0.55` | `0.45` | HIGH | Upper angular speed limit. Lower is safer; higher rotates faster but can feel abrupt. |
| `search_goal_resend_sec` | `3.5` | `3.0` | MEDIUM | Goal refresh during search-like states. |
| `nav_available_topic` | `/person_follow/nav2_available` | `/person_follow/nav2_available` | LOW | Wiring topic. |
| `tf_ok_topic` | `/person_follow/tf_ok` | `/person_follow/tf_ok` | LOW | Wiring topic. |

---

## `nav_follow_interface.yaml`

Node: `nav_follow_interface.py`  
Role: deprecated compatibility shim  
Launch wiring: still resolvable by profile path, but full tuning lives in `mapped_follow.yaml`

This file intentionally contains no active parameters.

- Keep it empty unless you are intentionally restoring backward compatibility behavior.
- Do not place new mapped-follow tuning here.

---

## `unmapped_follow.yaml`

Node: `local_follow_interface.py`  
Role: local/reactive following when using unmapped mode, including obstacle-aware slowing and bypass behavior  
Launch wiring: loaded by full-stack launch; launch usually overrides topics, frames, `cmd_vel_raw_topic`, scan/depth topics, and Nav2 action name

### Profile Mismatch Warning

`indoor_safe/unmapped_follow.yaml` does not expose the same schema as `competition_default/unmapped_follow.yaml`.

Missing in `indoor_safe`:

- `unmapped_backend`
- `goal_update_hz`
- `goal_resend_sec`
- `goal_min_step_m`
- `goal_smoothing_alpha`
- `max_local_goal_distance_m`
- `goal_cancel_cooldown_sec`
- `local_goal_frame`
- `publish_debug_markers`
- `debug_markers_topic`
- `debug_status_topic`

When those keys are absent, `local_follow_interface.py` uses its built-in defaults.

### Mode, Goal, and Control Basics

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `default_follow_mode` | `mapped` | `mapped` | HIGH | Startup mode expectation. Keep aligned with mapped/safety/server defaults unless deliberate. |
| `unmapped_backend` | `mppi` | `<MISSING>` | LOW | Backend selector. Missing in `indoor_safe`, so code default applies. Do not change casually unless backend support changes. |
| `control_hz` | `10.0` | `10.0` | HIGH | Reactive control loop rate. Higher improves responsiveness but increases load and noise sensitivity. |
| `goal_update_hz` | `4.0` | `<MISSING>` | HIGH | Local goal refresh rate. Missing in `indoor_safe`; node default is used. |
| `goal_resend_sec` | `2.0` | `<MISSING>` | HIGH | Timeout before resending local goal. Missing in `indoor_safe`; node default is used. |
| `goal_min_step_m` | `0.15` | `<MISSING>` | HIGH | Minimum target motion before refreshing local goal. Lower is more responsive, higher smoother. |
| `goal_smoothing_alpha` | `0.35` | `<MISSING>` | HIGH | Smoothing on local goal motion. Lower is smoother but laggier; higher reacts faster. |
| `max_local_goal_distance_m` | `1.6` | `<MISSING>` | HIGH | Clamp on local goal distance. Larger lets the robot chase harder; smaller keeps it close and conservative. |
| `follow_distance_default_m` | `0.8` | `0.85` | HIGH | Desired local follow spacing. |
| `target_stale_timeout_sec` | `1.2` | `1.0` | HIGH | Stale limit before local controller stops trusting target pose. |
| `goal_cancel_cooldown_sec` | `0.4` | `<MISSING>` | MEDIUM | Minimum spacing between local goal cancels. Missing in `indoor_safe`; node default applies. |

### Speed and Controller Gains

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `max_linear_speed_mps` | `0.45` | `0.32` | HIGH | Upper local forward speed. Lower is safer and more stable indoors. |
| `max_angular_speed_rad_s` | `0.75` | `0.6` | HIGH | Upper local rotation speed. Lower reduces overshoot and jerk. |
| `min_linear_speed_mps` | `0.05` | `0.04` | MEDIUM | Minimum commanded forward speed once moving. Too high can make fine approach jerky. |
| `distance_kp` | `0.65` | `0.55` | HIGH | Distance controller proportional gain. Too high causes aggressive surging; too low feels sluggish. Typical `0.5-0.7`. |
| `yaw_kp` | `1.2` | `1.1` | HIGH | Heading controller proportional gain. Too high causes oscillation; too low turns lazily. Typical `1.0-1.3`. |
| `local_goal_frame` | `odom` | `<MISSING>` | LOW | Local goal frame. Missing in `indoor_safe`; code default is used. Do not change casually unless TF design changed. |

### Local Sensing and Corridor Logic

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `scan_timeout_sec` | `0.6` | `0.5` | HIGH | Stale limit for LiDAR scan data in local mode. Lower is safer; higher tolerates temporary lag. |
| `depth_timeout_sec` | `0.8` | `0.7` | HIGH | Stale limit for depth obstacle data. |
| `corridor_half_width_m` | `0.45` | `0.4` | HIGH | Half-width of preferred free corridor in front of robot. Lower is stricter; higher accepts wider passage assumptions. |
| `corridor_lookahead_m` | `1.4` | `1.2` | HIGH | Lookahead distance for corridor checks. Larger anticipates obstacles sooner. |
| `critical_stop_distance_m` | `0.42` | `0.45` | HIGH | Hard stop threshold for local obstacle logic. |
| `slowdown_distance_m` | `0.9` | `0.95` | HIGH | Distance where local mode starts slowing. Higher is safer but can feel hesitant. |
| `side_clearance_m` | `0.55` | `0.6` | HIGH | Required side clearance. Higher avoids brushing obstacles but blocks more often. |
| `depth_stop_distance_m` | `0.6` | `0.65` | HIGH | Depth-based stop threshold. |
| `depth_critical_distance_m` | `0.38` | `0.42` | HIGH | Depth-based emergency threshold. |
| `depth_center_ratio` | `0.35` | `0.35` | MEDIUM | Fraction of center image region used for depth obstacle checks. |
| `depth_min_valid_pixels` | `150` | `180` | MEDIUM | Minimum valid depth pixels before center-depth reading is trusted. Higher is safer but more likely to disable depth assistance. |

### Bypass Behavior and Debug

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `bypass_trigger_sec` | `1.2` | `1.4` | HIGH | How long blockage must persist before bypass begins. Lower bypasses sooner; higher waits longer. |
| `bypass_duration_sec` | `1.4` | `1.2` | HIGH | Planned duration of bypass maneuver. Larger gives more escape motion but can overshoot. |
| `bypass_forward_speed_mps` | `0.12` | `0.1` | MEDIUM | Forward speed during bypass. |
| `bypass_turn_speed_rad_s` | `0.45` | `0.38` | MEDIUM | Turn speed during bypass. |
| `wait_before_bypass_sec` | `0.6` | `0.7` | MEDIUM | Pause before committing to bypass. |
| `publish_debug_markers` | `true` | `<MISSING>` | LOW | Debug markers; missing in `indoor_safe`, so node default applies. |
| `debug_markers_topic` | `/person_follow/unmapped_debug_markers` | `<MISSING>` | LOW | Debug topic wiring. |
| `debug_status_topic` | `/person_follow/local_debug` | `<MISSING>` | LOW | Debug topic wiring. |
| `obstacle_inhibit_topic` | `/person_follow/obstacle_inhibit` | `/person_follow/obstacle_inhibit` | LOW | Wiring topic. |
| `local_sensing_ok_topic` | `/person_follow/local_sensing_ok` | `/person_follow/local_sensing_ok` | LOW | Wiring topic. |

---

## `safety.yaml`

Node: `safety_supervisor.py`  
Role: global inhibit logic for stale tracking, TF failure, nav availability, and local obstacle sensing issues  
Launch wiring: loaded by full-stack launch; launch usually overrides follow topics, track topics, stop reason topic, frames, and backend status topics

### Safety Timeouts

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `default_follow_mode` | `mapped` | `mapped` | HIGH | Startup mode expectation. Keep consistent with other mode defaults unless intentional. |
| `tracker_timeout_sec` | `1.0` | `0.9` | HIGH | Max time tracker state may go stale before safety reacts. Lower is safer; higher tolerates pipeline latency. |
| `target_soft_timeout_sec` | `1.2` | `1.0` | HIGH | Time to begin soft inhibit behavior for stale target. |
| `target_hard_timeout_sec` | `2.5` | `2.0` | HIGH | Time to escalate to hard inhibit for stale target. |
| `tf_timeout_sec` | `0.75` | `0.6` | HIGH | TF health timeout. Lower reacts faster to transform issues. |
| `nav_timeout_sec` | `1.5` | `1.2` | HIGH | Nav backend availability timeout. Lower is stricter on Nav2 health. |
| `obstacle_fault_timeout_sec` | `3.0` | `2.5` | HIGH | How long obstacle inhibit may persist before treated as fault-like. |
| `local_sensing_timeout_sec` | `1.0` | `0.8` | HIGH | Local sensing health timeout in unmapped mode. |

### Wiring

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `behavior_status_topic` | `/person_follow/behavior_status` | `/person_follow/behavior_status` | LOW | Wiring topic. |
| `nav_available_topic` | `/person_follow/nav2_available` | `/person_follow/nav2_available` | LOW | Wiring topic. |
| `tf_ok_topic` | `/person_follow/tf_ok` | `/person_follow/tf_ok` | LOW | Wiring topic. |
| `obstacle_inhibit_topic` | `/person_follow/obstacle_inhibit` | `/person_follow/obstacle_inhibit` | LOW | Wiring topic. |
| `local_sensing_ok_topic` | `/person_follow/local_sensing_ok` | `/person_follow/local_sensing_ok` | LOW | Wiring topic. |

---

## `server.yaml`

Node: `person_follow_server.py`  
Role: exposes action/service entry points and default session-level settings  
Launch wiring: loaded by full-stack launch; launch usually overrides target/session/follow topics

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `tracks_state_timeout_sec` | `0.5` | `0.5` | HIGH | Max age of tracker state for server feedback validity. Too low causes feedback dropouts; too high reports stale state. |
| `default_follow_distance_m` | `0.8` | `0.85` | HIGH | Default requested follow distance when a session starts. Keep aligned with `behavior.yaml` and `unmapped_follow.yaml` expectations. |
| `default_face_hold_sec` | `10.0` | `10.0` | MEDIUM | Default face-hold request duration for a session. |
| `default_follow_mode` | `mapped` | `mapped` | HIGH | Default session mode. Keep aligned with mapped/unmapped/safety defaults. |
| `feedback_hz` | `10.0` | `10.0` | MEDIUM | Feedback publication rate for clients. |
| `behavior_status_topic` | `/person_follow/behavior_status` | `/person_follow/behavior_status` | LOW | Wiring topic. |
| `select_topic` | `/person_target_select_id` | `/person_target_select_id` | LOW | Wiring topic for explicit target selection. |

---

## `follow_ready_voice.yaml`

Node: `follow_ready_voice_server.py`  
Role: announces readiness to follow after a session is started and safety conditions allow it  
Launch wiring: loaded by full-stack launch; launch may override voice backend binaries/model paths outside the YAML

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `default_text` | `I am able to follow you` | `I am able to follow you` | LOW | Default spoken phrase. |
| `default_timeout_sec` | `4.0` | `4.0` | MEDIUM | Max announcement action duration. Lower fails faster; higher tolerates slower voice backends. |
| `action_name` | `/person_follow/announce_ready` | `/person_follow/announce_ready` | LOW | Action wiring. |
| `wait_poll_sec` | `0.1` | `0.1` | LOW | Poll period while waiting for prerequisites. |
| `feedback_heartbeat_sec` | `0.5` | `0.5` | LOW | Feedback heartbeat interval for the action. |

---

## `boundary_zone_checker.yaml`

Node: `boundary_zone_checker.py`  
Role: boundary transition logic and RViz override support  
Launch wiring: full-stack launch loads the profile YAML and separately passes `boundary_config_file`; there is also a top-level legacy `config/boundary_zone_checker.yaml` used as a zone template/default file

| Param | Competition | Indoor | Impact | Meaning, tuning, range, danger |
| --- | --- | --- | --- | --- |
| `boundary_enable_rviz_override` | `false` | `false` | LOW | Allows operator RViz click input to override target boundary zone selection. Keep off in normal runs. |
| `boundary_rviz_target_zone` | `''` | `''` | LOW | Zone name targeted by RViz override. Only meaningful when override is enabled. |

Note:

- The actual zone geometry is not stored in the profile files above.
- Zone definitions come from the separate boundary config file path passed by launch.
- The top-level `config/boundary_zone_checker.yaml` is a template/default geometry file, not a normal tuning profile.

## Safe Tuning Rules

- Tune `HIGH` impact parameters before touching `MEDIUM` or `LOW` ones.
- Change one subsystem at a time.
- Prefer fixing perception and tracking before compensating with behavior or control hacks.
- Keep `default_follow_mode` aligned across `mapped_follow.yaml`, `unmapped_follow.yaml`, `safety.yaml`, and `server.yaml` unless deliberate mode switching is required.
- Revalidate with rosbag or a repeatable scenario after each tuning batch.
