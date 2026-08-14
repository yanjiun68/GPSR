—————————————————————————INTRODUCTION——————————————————————————————————
Yes — I read the `person_following` package itself in detail.

The real source is here:

* `person_following/package.xml`
* `person_following/CMakeLists.txt`
* `person_following/launch/`
* `person_following/person_following/nodes/`
* `person_following/person_following/utils/`
* `person_following/scripts/`
* `person_following/models/`

The important thing is:

**the real implementation is in `person_following/person_following/nodes/`**
The files in `scripts/` are mostly thin wrappers so ROS can run them as executables.

---

# 1. What this package is for

This package is a full ROS 2 **person-following stack**.

It is not just “detect a person and move.”

It does all of this:

1. detect people from RGB-D camera
2. extract person-like candidates from LiDAR
3. fuse camera + LiDAR
4. track multiple people over time
5. lock one selected target
6. estimate whether that target is facing the robot
7. decide behavior: follow / cautious / hold / search / stop
8. send movement commands through Nav2 or raw `cmd_vel`
9. supervise safety and expose action/service API

So architecturally, this package is:

**perception + fusion + tracking + target lock + behavior + execution + safety + control interface**

---

# 2. Folder structure and what each part means

## `package.xml`

This declares the ROS package metadata.

Main points:

* package name: `person_following`
* description says it is a ROS 2 person following stack with tracking, lock, behavior, nav, safety
* maintainer is set
* license is still `TODO`, so that part is incomplete

Dependencies declared include:

* `rclpy`
* `ament_index_python`
* `std_msgs`
* `geometry_msgs`
* `sensor_msgs`
* `visualization_msgs`
* `builtin_interfaces`
* `nav2_msgs`
* `tf2_ros`
* `rcl_interfaces`
* `cv_bridge`
* `putrabot_interfaces`
* `launch_ros`
* `message_filters`
* `std_srvs`

That tells us this package depends on:

* ROS Python nodes
* TF
* camera image handling
* visualization
* Nav2 action interface
* custom messages from `putrabot_interfaces`

Important detail:
this package **does not define its own messages**.
It relies on `putrabot_interfaces` for messages like tracked targets, behavior status, safety status, motion intent, and lock debug.

---

## `CMakeLists.txt`

This builds and installs the Python package.

What it does:

* enables `ament_cmake` and `ament_cmake_python`
* installs the Python package `person_following`
* installs executable scripts from `scripts/`
* installs `launch/` and `models/` into the package share directory

So this package is a normal ROS 2 Python package with launch files and bundled models.

---

## `models/`

This folder contains the ML models used by the package:

* `yolo11n.pt` → YOLO person detector
* `RetinaFace_int.onnx` → face / head pose related detection
* `osnet_x0_5.onnx` → ReID embedding model
* `export_osnet_onnx.py` → helper for exporting model
* `.data` side file for ONNX model

So the stack supports:

* person detection
* optional face/facing cue
* ReID identity features

---

## `launch/`

There are 2 launch files:

### `person_perception.launch.py`

This is a lighter launch for:

* `vision_node`
* `follow_person`
* `person_track_manager`

So this launch is more like:
**perception + fusion only**

### `person_follow_nav2.launch.py`

This is the full stack launch.

It starts:

* `vision_node`
* `follow_person`
* `person_track_manager`
* `target_lock_manager`
* `follow_behavior_manager`
* `nav_follow_interface`
* `safety_supervisor`
* `person_follow_server`

So this is the real full person-following pipeline.

---

## `scripts/`

These are wrapper executables.

Example pattern:

* script file imports `main` from the real node module and runs it

So these are mostly here so ROS can launch them easily.

---

## `person_following/nodes/`

This is the real source code.

Files:

* `vision_node.py`
* `follow_person.py`
* `person_track_manager.py`
* `target_lock_manager.py`
* `follow_behavior_manager.py`
* `nav_follow_interface.py`
* `safety_supervisor.py`
* `person_follow_server.py`
* `person_follow_client.py`
* `person_follow_nav2_goal.py`

---

## `person_following/utils/runtime_config.py`

This is the backend/runtime selection helper.

It decides:

* which ONNX providers to use
* whether YOLO uses CPU or CUDA

So this package is designed to run on different hardware.

---

# 3. Launch flow of the full package

The full launch file is `person_follow_nav2.launch.py`.

Its structure is very important because it shows the intended architecture.

## Step 1: perception nodes

It starts:

### `vision_node`

Inputs:

* RGB topic
* aligned depth topic
* camera info topic

Outputs:

* `/cam_detections`
* `/cam_detections_markers`

### `lidar_people_candidates`

Inputs:

* `/scan`

Outputs:

* `/people_candidates`
* `/people_candidates_markers`

So after this stage the system has:

* camera-based person detections in 3D
* LiDAR person-like candidates

---

## Step 2: fusion/tracking node

### `person_track_manager`

Inputs:

* `/cam_detections`
* `/people_candidates`
* RGB topic

Outputs:

* `/person_tracks`
* `/person_tracks_poses`
* `/person_tracks_state`
* `/person_target_locked`
* `/person_target_locked_marker`
* `/person_track_debug`
* `/person_target_lock_debug`
* target facing topic

But in the full launch, one important parameter is set:

* `enable_target_lock_publish = False`

That means in this launch, the tracker still computes the selected target state internally, but the stabilized lock output is handled by `target_lock_manager`.

---

## Step 3: lock stabilization

### `target_lock_manager`

Inputs:

* `/person_tracks_state`
* `/person_target_lock_debug`

Outputs:

* `/person_target_locked`
* `/person_target_facing_robot`

This node republishes a stable selected target and debounces fast ID switching.

---

## Step 4: behavior decision

### `follow_behavior_manager`

Inputs:

* `/person_target_locked`
* facing topic
* follow enable topic
* safety status topic

Outputs:

* behavior status
* motion intent

This node decides:

* follow
* cautious follow
* hold
* temporary occluded
* reacquire
* local search
* safe stop
* aborted

---

## Step 5: motion execution

### `nav_follow_interface`

Inputs:

* selected target
* motion intent
* safety status
* follow enable

Outputs:

* raw `cmd_vel`
* Nav2 action goals
* nav availability
* TF health

This is the bridge between high-level person-follow logic and actual robot motion.

---

## Step 6: safety layer

### `safety_supervisor`

Inputs:

* follow enable
* tracks state
* selected target
* behavior status
* nav available
* TF OK
* obstacle inhibit

Outputs:

* safety status
* stop reason

This is the watchdog.

---

## Step 7: control API

### `person_follow_server`

This is the action/service interface for:

* starting follow
* selecting a person
* clearing selection
* stopping
* querying status
* updating some follow parameters

---

# 4. Detailed explanation of each real node

## A. `vision_node.py`

This is the camera perception frontend.

### Main purpose

Turn RGB-D frames into person detections in robot coordinates, with extra identity and facing information.

### Main parameters

It declares many parameters, including:

* `rgb_topic`
* `depth_topic`
* `camera_info_topic`
* `camera_frame`
* `global_frame`
* `yolo_model_path`
* `yolo_conf_thresh`
* `yolo_device`
* `enable_gui`
* `publish_markers`
* `locked_target_topic`
* depth ROI parameters
* `retinaface_enabled`
* `retinaface_onnx_path`
* `retinaface_providers`
* facing thresholds
* crop/appearance quality thresholds

So this node is highly tunable.

### What it subscribes to

* camera info
* locked target topic
* internally synced RGB + depth via message filters

### What it publishes

* `/cam_detections`
* `/cam_detections_markers`

### Internal logic

The pipeline is roughly:

1. synchronize RGB and depth
2. run YOLO person detection
3. for each person box, compute a robust depth from a center ROI
4. convert 2D detection + depth into 3D point
5. transform that point into tracking frame
6. optionally run RetinaFace on upper/head area
7. estimate facing cue from head pose thresholds
8. extract body appearance cues:

   * torso / upper / lower histograms
   * quality metrics
9. package everything into camera detection messages

### Why it matters

This node gives the tracker much more than just 2D boxes:

* 3D position
* depth quality
* facing estimate
* appearance features
* crop quality info

So this is a strong perception frontend.

---

## B. `follow_person.py`

This file contains the LiDAR candidate extractor.

Class:

* `LidarPeopleCandidates`

### Main purpose

Convert laser scan clusters into person-like candidates.

### Parameters

It declares things like:

* `scan_topic`
* `base_frame`
* min/max range
* x/y ROI
* clustering distance
* min/max cluster points
* min/max cluster size
* persistence settings
* width constraints
* candidate association gate
* person-likeness threshold

### Subscribes to

* scan topic

### Publishes

* `/people_candidates`
* `/people_candidates_markers`

### Internal logic

The flow is:

1. read laser scan
2. convert ranges to XY points
3. transform to base frame
4. filter by ROI
5. cluster adjacent scan points
6. reject clusters with bad size/point count
7. maintain short-lived candidate tracks
8. score each cluster for person-likeness
9. publish centroids as person candidates

### Meaning

This node is not full tracking.
It is a **LiDAR candidate generator** to support fusion and continuity.

---

## C. `person_track_manager.py`

This is the most important file in the package.

It is the main intelligence center.

### What it contains

Classes:

* `TrackState`
* `TargetLockState`
* `FusedMeasurement`
* `Track`
* `ReIDExtractor`
* `PersonTrackManager`

### Main purpose

Fuse camera + LiDAR observations into tracks, manage identity, and protect the selected target.

### Inputs

* camera detections
* LiDAR candidates
* RGB frames
* selected target ID command (`/person_target_select_id`)

### Outputs

* `/person_tracks`
* `/person_tracks_poses`
* `/person_tracks_state`
* `/person_target_locked`
* `/person_target_locked_marker`
* `/person_track_debug`
* `/person_target_lock_debug`
* facing topic

### Huge parameter set

This node has by far the most parameters. They cover:

#### Fusion and gating

* `cam_lidar_gate_m`
* `track_position_gate_m`
* association gate mode
* Mahalanobis thresholds

#### Lidar-only logic

* `allow_lidar_only_updates`
* `allow_lidar_only_for_selected`
* selected LiDAR bridge timing / gates / ambiguity thresholds

#### Track lifecycle

* min confirm hits
* missed frames
* occluded frames
* lost frames
* confirm / delete thresholds

#### Selected-target protection

* hard hold time
* reacquire window
* no-switch during hard hold
* switch confirm frames
* relock cooldown
* contradictory candidate checks
* crossover handling
* post-relock protection

#### Identity and ReID

* reid model path
* reid providers
* gallery size
* bbox area threshold
* yaw/pitch limits for update
* embedding EMA alpha

#### Motion model

* process noise
* measurement noise
* IMM stay probability
* turn rate
* stop time constant
* initial IMM mode weights

#### Cost weights

* position / appearance / motion weights
* separate crowd-mode weights

So this file is extremely configurable.

### What it does internally

The main cycle is roughly:

1. collect recent camera detections and LiDAR candidates
2. build fused measurements:

   * camera+LiDAR
   * camera-only
   * sometimes lidar-only
3. for selected target, optionally create a temporary LiDAR bridge measurement
4. predict all current tracks forward in time
5. associate measurements to tracks using:

   * position
   * appearance
   * motion consistency
   * crowd-mode weights
6. update matched tracks
7. age unmatched tracks into occluded/lost/deleted
8. create new tentative tracks from unmatched measurements
9. update selected-target lock state
10. auto-select nearest if configured
11. publish outputs and debug markers

### Identity logic

This file is not only geometry-based.

It uses:

* body ReID embedding
* embedding gallery
* color histogram similarity
* face/facing-related cues from camera
* motion consistency

So it is designed to reduce ID switching in crowds.

### Selected-target protection

This is the special strength of the file.

It tries to answer:
“Should I keep trusting the current selected person, or switch?”

That logic includes:

* hard hold window
* relock cooldown
* reacquire confirmation
* contradictory candidate rejection
* crossover state handling
* ambiguity margins
* post-relock protection

That is much better than a plain nearest-person tracker.

### Motion model

The code uses an IMM-like design rather than one simple CV model.
That helps because humans:

* stop
* turn
* slow down
* move unpredictably

### Best summary of this file

`person_track_manager.py` is:

**fusion engine + multi-target tracker + identity memory + selected-target lock controller**

---

## D. `target_lock_manager.py`

This file is the selected-target stabilizer.

### Purpose

Take the raw selected track from track state and republish a cleaner stable target.

### Parameters

* track state topic
* target debug topic
* target topic
* target facing topic
* target timeout
* publishable timeout
* minimum combined confidence
* relock stability cycles
* relock cooldown

### Inputs

* `/person_tracks_state`
* `/person_target_lock_debug`

### Outputs

* `/person_target_locked`
* `/person_target_facing_robot`

### Internal logic

1. cache latest tracks by ID
2. find which track is currently marked selected
3. debounce relock:

   * new selected ID must stay stable for some cycles
   * cooldown avoids rapid toggling
4. build final selected target:

   * geometry confidence
   * visibility confidence
   * freshness confidence
   * combined confidence
   * publishable flag
5. publish facing bool from selected target’s `facing_robot`

### Meaning

This node is a **stability filter** between tracker and behavior layer.

---

## E. `follow_behavior_manager.py`

This is the behavior state machine.

### Purpose

Decide what the robot should do next based on target and safety state.

### Parameters

Important ones include:

* target/facing/follow/safety topics
* `follow_distance_m`
* `face_hold_sec`
* `lock_confirm_sec`
* soft/hard target timeout
* confidence thresholds
* rotate bearing threshold
* stop distance
* hold distance
* nominal / cautious speed
* search rotate speed

### Inputs

* selected target
* facing bool
* follow enable
* safety status

### Outputs

* behavior status
* motion intent

### States

It defines states such as:

* IDLE
* WAIT_FOR_TARGET
* LOCKING_TARGET
* FOLLOWING
* FOLLOWING_CAUTIOUS
* HOLD_LOCKED
* TEMP_OCCLUDED
* REACQUIRE
* LOCAL_SEARCH
* SAFE_STOP
* ABORTED

### Internal logic

In plain words:

* follow disabled → `IDLE`
* no fresh target → `WAIT_FOR_TARGET`
* target just appeared → `LOCKING_TARGET`
* good fresh target → `FOLLOWING`
* weaker target → `FOLLOWING_CAUTIOUS`
* target too close → `HOLD_LOCKED`
* target stale but recoverable → `TEMP_OCCLUDED` or `REACQUIRE`
* gone longer → `LOCAL_SEARCH`
* safety problem → `SAFE_STOP`
* stop condition satisfied → `ABORTED`

### Motion intents it outputs

* HOLD
* ROTATE
* FOLLOW_POSE
* SEARCH

So this file is the policy brain of the stack.

---

## F. `nav_follow_interface.py`

This file executes behavior.

### Purpose

Convert motion intent into robot commands.

### Parameters

* target topic
* motion intent topic
* safety topic
* follow enable topic
* nav availability topic
* TF OK topic
* frames
* Nav2 action name
* raw cmd_vel topic
* goal update rate
* min goal step
* resend timing
* cancel cooldown
* max goal distance
* yaw deadband
* angular speed min/max
* search resend timing

### Inputs

* selected target
* motion intent
* safety status
* follow enable

### Outputs

* raw `cmd_vel`
* nav availability bool
* tf ok bool
* Nav2 goals through action client

### Behavior

* `HOLD` → cancel Nav2, publish zero twist
* `ROTATE` → cancel Nav2, rotate in place
* `SEARCH` → go to last known map position or rotate
* `FOLLOW_POSE` → compute a standoff target behind the person and send `NavigateToPose`

So it mixes:

* local rotation control
* Nav2 global goal following

That is a sensible hybrid design.

---

## G. `safety_supervisor.py`

This is the watchdog node.

### Purpose

Check whether the full person-follow stack is healthy enough to continue.

### Parameters

* follow enable topic
* tracks state topic
* target topic
* behavior status topic
* safety status topic
* stop reason topic
* nav available topic
* TF ok topic
* obstacle inhibit topic
* timeout settings

### Inputs

* follow enabled
* tracks state
* selected target
* behavior state
* nav availability
* TF health
* obstacle inhibit

### Outputs

* safety status
* stop reason

### What it checks

* tracker freshness
* selected target freshness
* TF validity
* Nav2 availability
* obstacle block too long
* aborted behavior state

So this file is the safety/health governor.

---

## H. `person_follow_server.py`

This is the user/control interface.

### Purpose

Provide action/service API to control person following.

### Parameters

* tracks state topic
* target topic
* facing topic
* behavior status topic
* safety status topic
* select topic
* follow enable topic
* session active topic
* feedback rate
* default follow params

### Inputs

* tracks state
* target
* facing
* behavior status
* safety status

### Outputs

* select target command
* follow enable
* session active

### Role

It exposes:

* start following
* stop following
* select track
* clear track
* get status
* update follow parameters

So this is the API layer for higher-level task logic or a CLI.

---

## I. `person_follow_client.py`

This is just a terminal helper client for the server.
It is not part of the core intelligence.

---

## J. `person_follow_nav2_goal.py`

This file is very small and mainly acts as an entry/wrapper rather than a full independent logic file.

---

## K. `runtime_config.py`

This utility file handles runtime backend selection.

### What it solves

* parse ONNX provider parameter strings
* resolve available ONNX runtime providers
* resolve YOLO device choice

### Why it matters

This lets the package run on:

* CUDA system
* CPU-only system

without hardcoding one backend.

---

# 5. End-to-end topic flow

The cleanest way to understand the package is by topic flow.

## Perception

`vision_node`
→ `/cam_detections`

`follow_person`
→ `/people_candidates`

## Fusion / tracking

`person_track_manager`
→ `/person_tracks_state`
→ debug topics

## Stable selected target

`target_lock_manager`
→ `/person_target_locked`
→ `/person_target_facing_robot`

## Behavior

`follow_behavior_manager`
→ behavior status
→ motion intent

## Execution

`nav_follow_interface`
→ raw `cmd_vel`
→ Nav2 goals

## Safety

`safety_supervisor`
→ safety status
→ stop reason

## Control

`person_follow_server`
→ follow enable
→ session active
→ select target command

That is the actual logic chain.

---

# 6. What is good about this package

The strongest points are:

### It is layered well

Perception, tracking, behavior, execution, safety, and control are separated into different nodes.

### The tracker is advanced

`person_track_manager.py` is not a toy tracker.
It includes:

* camera/LiDAR fusion
* ReID
* appearance memory
* crowd logic
* selected-target-first protection
* bridge/reacquire handling

### It supports real deployment concerns

* debug markers
* GUI option
* CPU/GPU runtime selection
* launch arguments for tuning
* safety watchdog

### It is built for one-person following, not generic MOT only

That is important.
The code prioritizes “do not switch away from my chosen person.”

---

# 7. Weak points or things to be aware of

A few practical issues stand out:

### `package.xml` license is unfinished

Still `TODO`.

### `person_track_manager.py` is very large

It is powerful, but it is also a very big “brain file,” so maintenance is harder.

### A lot of behavior depends on good tuning

Because there are many thresholds, the real-world result depends heavily on:

* camera-LiDAR gate
* selected-lidar bridge settings
* association weights
* follow/hold/search thresholds
* ReID update conditions

So this package is strong, but tuning quality matters a lot.

---

# 8. Best plain-English summary

If I explain each important file in one sentence:

* `vision_node.py`
  “Find people in RGB-D and give me 3D position, facing, and appearance cues.”

* `follow_person.py`
  “Find person-like blobs from LiDAR.”

* `person_track_manager.py`
  “Fuse everything, track everyone, and protect the selected target.”

* `target_lock_manager.py`
  “Publish one stable selected target.”

* `follow_behavior_manager.py`
  “Decide whether to follow, hold, rotate, search, or stop.”

* `nav_follow_interface.py`
  “Turn that decision into actual robot motion.”

* `safety_supervisor.py`
  “Stop the system from continuing when the stack is unhealthy.”

* `person_follow_server.py`
  “Give other nodes or users a clean way to control the whole stack.”

---

# 9. Final answer

So yes — the `person_following` package is a **complete person-following subsystem**, not just a detector.

Its structure is:

* `package.xml` and `CMakeLists.txt` define it as a ROS 2 Python package
* `launch/` wires either perception-only or the full follow stack
* `models/` holds YOLO, RetinaFace, and ReID models
* `nodes/` contains the real logic:

  * perception
  * LiDAR candidate extraction
  * fusion/tracking
  * target locking
  * behavior
  * execution
  * safety
  * control API
* `utils/runtime_config.py` handles CPU/GPU backend selection
* `scripts/` are mostly wrapper executables

The single most important file is still:

**`person_track_manager.py`**

because that is where the real target-follow intelligence lives.

I can break down `person_track_manager.py` line by line next.

————————————————————————————vision_node.py——————————————————————————————————————————
Yes. I read the real `vision_node.py` in `person_following/person_following/nodes/vision_node.py`.

This node is the **camera perception frontend** of your person-following system.

Its job is:

**RGB image + aligned depth + camera info + TF**
→ detect person with YOLO
→ estimate 3D position
→ optionally estimate face/head pose with RetinaFace
→ extract appearance cues
→ publish `CamDetections` for the tracker

So this node is not just “camera detect person”.
It is more like:

**person detector + depth projector + optional facing estimator + appearance feature extractor**

---

# 1. What this node subscribes and publishes

## Subscribes

It uses these inputs:

* RGB image
* depth image aligned to RGB
* camera info
* locked target topic

In code that is:

* `rgb_topic`
* `depth_topic`
* `camera_info_topic`
* `locked_target_topic`

Default topics are RealSense style topics:

* `/camera/camera/color/image_raw`
* `/camera/camera/aligned_depth_to_color/image_raw`
* `/camera/camera/color/camera_info`

## Publishes

It publishes:

* `/cam_detections`
* `/cam_detections_markers`

So downstream nodes like `person_track_manager.py` use `/cam_detections` as the camera measurement source.

---

# 2. Main purpose of the file

This file answers these questions for every frame:

1. Is there a person in the RGB image?
2. Where is that person in 3D relative to the robot?
3. Is the person likely facing the robot?
4. Is this crop good enough for identity update?
5. What body-color / appearance info can I give the tracker?

That is the full purpose.

---

# 3. Big structure of the file

The file has 3 main parts:

## A. helper functions

* `_resolve_model_path`
* `_nms_xyxy`

## B. `RetinaFaceOnnx` class

This handles optional face detection and landmark decoding from ONNX.

## C. `VisionNode` class

This is the real ROS node.

---

# 4. `_resolve_model_path()`

This function makes model paths safer.

It handles cases like:

* absolute path
* relative path
* path beginning with `models/`
* old path strings containing `NAVIGATION/person_following/models/`

So if launch passes `models/RetinaFace_int.onnx`, this helper converts it to the correct installed package path.

Why it matters:
when ROS installs packages, model files are usually found under package share, not your original source folder.

---

# 5. `_nms_xyxy()`

This is a manual Non-Maximum Suppression implementation.

Purpose:
if RetinaFace returns overlapping face boxes, NMS keeps the best one and removes duplicates.

Input:

* boxes in `xyxy`
* scores
* IoU threshold

Output:

* indices of kept boxes

So this is standard cleanup after face detection.

---

# 6. `RetinaFaceOnnx` class

This class runs the RetinaFace ONNX model.

It is optional.

If `onnxruntime` is missing or model load fails, RetinaFace gets disabled and the node still works with YOLO only.

## What this class does

### In `__init__`

It:

* stores ONNX path
* stores thresholds
* creates an ONNX runtime session
* checks input shape
* records input width and height

So after this, the class is ready to run face inference.

---

## `_prior_boxes()`

This creates **anchor priors** for RetinaFace.

This is important because RetinaFace ONNX output is usually not final boxes directly.
It predicts offsets relative to priors.

So this function generates the reference anchors.

---

## `_decode_boxes()`

This converts RetinaFace box offsets into actual bounding boxes.

It takes:

* loc outputs
* priors

and converts them from:

* center/width/height style
  into:
* `x1, y1, x2, y2`

---

## `_decode_landmarks()`

This converts landmark offsets into actual face landmark positions.

It gives 5 landmarks:

* left eye
* right eye
* nose
* left mouth
* right mouth

---

## `_select_outputs()`

This tries to find which ONNX outputs are:

* boxes
* scores
* landmarks

It supports:

* name-based mapping
* shape-based guessing

So the code is flexible if the ONNX export names are slightly different.

---

## `detect()`

This is the main RetinaFace inference function.

Flow:

1. resize image to ONNX input size
2. subtract RetinaFace mean values
3. convert to `1x3xHxW`
4. run ONNX inference
5. decode boxes and landmarks
6. filter by confidence threshold
7. run NMS
8. scale detections back to original image size

It returns:

* face boxes
* face scores
* 5-point landmarks

So `RetinaFaceOnnx` is only a face/head helper used by `VisionNode`.

---

# 7. `VisionNode.__init__()`

This is where the real node is created.

It does a lot, so I will break it down.

---

## 7.1 Parameter declaration

The node declares many parameters.

These are grouped logically:

### Camera topics

* `rgb_topic`
* `depth_topic`
* `camera_info_topic`

### Frames

* `camera_frame`
* `global_frame`

Important:

* `camera_frame` default is `camera_color_optical_frame`
* `global_frame` default is `base_footprint`

That means 3D projection is first in optical frame, then transformed into robot base frame.

---

### YOLO settings

* `yolo_model_path`
* `yolo_conf_thresh`
* `yolo_device`

So YOLO is the required person detector.

---

### GUI/debug settings

* `debug_mode`
* `enable_gui`
* `publish_markers`
* `locked_target_topic`
* GUI lock gating settings

This means the node can:

* show OpenCV debug window
* show RViz markers
* highlight currently locked target in GUI

---

### Sync settings

* `sync_queue`
* `sync_slop_sec`

These control RGB-depth synchronization.

---

### Marker settings

* marker scale
* marker lifetime

---

### Depth settings

* `tf_timeout_sec`
* `robot_radius_m`
* `depth_roi_scale_w`
* `depth_roi_scale_h`
* `min_depth_valid_ratio`

These are very important for robust depth estimation.

---

### RetinaFace settings

* enable/disable
* ONNX path
* input size
* confidence threshold
* NMS IoU
* output names
* providers
* runtime timing debug

---

### Facing thresholds

* `facing_yaw_thresh_deg`
* `facing_pitch_thresh_deg`

These decide when a person counts as facing the robot.

---

### Appearance / identity quality settings

* torso / upper / lower region height ratios
* histogram visible ratio threshold
* face embed area and pose thresholds
* identity quality threshold
* blur threshold
* bbox area threshold
* visible body ratio threshold
* aspect ratio thresholds

So this node is also deciding whether a crop is good enough to help identity tracking.

---

## 7.2 Reading parameters

After declaring them, the code reads and stores them into class members.

That is normal ROS style.

Notable details:

* `_param_bool()` is used so strings like `"true"` or `"false"` also work safely
* provider/device requests are resolved later

---

## 7.3 Runtime backend resolution

This part is from:

* `resolve_yolo_device()`
* `resolve_onnx_providers()`

So the node decides:

* YOLO on CUDA or CPU
* RetinaFace ONNX on CUDA or CPU

This is useful because your laptop may sometimes have GPU available and sometimes not.

---

## 7.4 YOLO model loading

If not in debug mode:

* YOLO must exist
* model is loaded from resolved path

If ultralytics is missing, the node throws a runtime error.

So YOLO is treated as required.

---

## 7.5 ROS utilities setup

It creates:

* `CvBridge`
* camera info storage
* previous center storage for velocity estimate
* locked target storage

---

## 7.6 RetinaFace loading

If RetinaFace is enabled and backend is available:

* it loads `RetinaFaceOnnx`

If loading fails:

* RetinaFace is disabled
* node continues running

So face estimation is optional, not mandatory.

---

## 7.7 Subscribers

It subscribes to:

* `CameraInfo`
* locked target topic

Then it creates synchronized subscribers for:

* RGB image
* depth image

using `ApproximateTimeSynchronizer`.

This is very important:
the node processes RGB and depth together as matched pairs.

---

## 7.8 Publishers

It creates:

* `/cam_detections`
* `/cam_detections_markers`

---

## 7.9 TF listener

It creates a TF buffer and listener.

This is required because camera 3D points must be transformed into `global_frame`.

---

## 7.10 Heartbeat timer

Once per second, `heartbeat_cb()` logs whether:

* camera info is present
* YOLO is loaded
* RetinaFace is loaded

So heartbeat is mainly for runtime debugging.

---

# 8. `heartbeat_cb()`

This just prints a status line every second.

It helps you know quickly if the node is alive and whether:

* intrinsics exist
* YOLO is loaded
* RetinaFace is loaded

---

# 9. `info_cb()`

This stores the latest `CameraInfo`.

It is very important because camera intrinsics are needed for 3D projection:

* `fx`
* `fy`
* `cx`
* `cy`

Without camera info, the node cannot back-project person pixels into 3D.

---

# 10. `locked_target_cb()`

This stores the latest selected target from tracking.

It updates:

* `latest_locked_target`
* receive time
* `latched_locked_target`

The latch is cleared if the track is invalid or unselected.

Why this exists:
the GUI can draw the currently locked detection differently from others.

So this callback is only for display/association in the vision window, not for the main detection logic.

---

# 11. `synced_cb()` — the main function

This is the most important function in the whole node.

Every synced RGB-depth pair comes here.

I will explain it step by step.

---

## Step 1: create output message

It creates a `CamDetections` message and sets its stamp from RGB stamp.

So the output detections are tied to the RGB frame time.

---

## Step 2: debug mode shortcut

If `debug_mode=True`, it publishes one fake detection at origin.

This is only for testing the rest of the pipeline when camera perception is not ready.

So in debug mode:

* no real YOLO
* no depth
* no real perception

---

## Step 3: ensure prerequisites exist

If camera info is missing or YOLO model is missing:

* return immediately

So the node refuses to process until it has:

* intrinsics
* detector

---

## Step 4: convert ROS images to OpenCV

It uses `cv_bridge`:

* RGB → `bgr8`
* depth → passthrough

If conversion fails, it warns and returns.

---

## Step 5: read intrinsics

It extracts:

* `fx`
* `fy`
* `cx`
* `cy`

from `self.cam_info.k`

These are used for pinhole projection.

---

## Step 6: run YOLO

It calls `run_yolo(rgb_img)`.

That function:

* runs YOLO model
* uses `imgsz=640`
* uses configured confidence threshold
* uses resolved device
* keeps only class `0` which is `person`

So this node only outputs person detections.

---

## Step 7: if no detections

If YOLO finds nothing:

* GUI shows raw frame
* empty detection message is published
* empty markers are published

So downstream tracker still receives a valid message, just with zero detections.

---

## Step 8: prepare TF timestamp

It uses the RGB message stamp for TF lookup.

This is good, because the camera point should be transformed at the same time the image was taken.

---

## Step 9: prepare camera matrix for head pose

If RetinaFace is enabled, it builds:

`camK = [[fx,0,cx],[0,fy,cy],[0,0,1]]`

This is later used by `solvePnP` for head pose estimation.

---

## Step 10: loop over every person detection

For every YOLO person box `(xmin, ymin, xmax, ymax, conf)`:

### 10a. compute image center

It computes:

* `u`
* `v`

as box center.

This is the pixel used for back-projection.

---

### 10b. check pixel validity

If `(u,v)` is outside the depth image bounds:

* skip detection

---

### 10c. compute robust depth

It calls `depth_median_in_bbox()`.

This is a very important design choice.

Instead of using one single depth pixel, it:

* takes a smaller ROI around the bbox center
* converts all valid values to meters
* uses the median depth
* returns valid-depth ratio

This makes depth much more stable.

It also flags:

* `weak_depth = depth_valid_ratio < min_depth_valid_ratio`

So detections with poor depth are marked weaker.

---

### 10d. back-project to camera 3D

Using pinhole equations:

* `Xc = (u - cx) * Z / fx`
* `Yc = (v - cy) * Z / fy`
* `Zc = Z`

This gives the person center in camera optical coordinates.

---

### 10e. transform to robot frame

It calls `transform_point_camera_to_global(Xc, Yc, Zc, stamp)`.

This uses TF to transform from:

* `camera_frame`
  to
* `global_frame`

Usually:

* camera optical frame
  to
* `base_footprint`

So the tracker gets robot-relative person position.

---

### 10f. create `CamDetection`

It fills many fields:

Basic geometry:

* `x`, `y`, `z`
* `confidence`
* bbox coordinates
* `depth_median`
* `depth_valid_ratio`
* `center_u`, `center_v`

Then it initializes identity-related fields:

* body reid flags
* face embedding flags
* detection quality
* identity quality
* continuity flags
* image-center velocity

So every detection is richer than a plain bbox.

---

### 10g. compute useful distances

It computes:

* planar distance from robot center
* clearance distance from robot body edge
* 3D distance

These are mostly logged.

---

### 10h. store debug data in `embedding`

The code puts `[u, v, Z]` into `det.embedding`.

This is a lightweight debug payload.

So here `embedding` is not an actual learned embedding.

---

### 10i. optional face/head pose

If RetinaFace exists:

* it calls `_estimate_facing_from_person_bbox()`

This returns:

* yaw
* pitch
* roll
* facing01

Then it appends those to `det.embedding` and also stores them in explicit fields:

* `head_yaw_deg`
* `head_pitch_deg`
* `head_roll_deg`
* `facing01`

So this is the main facing estimate.

Important:
`facing01` becomes `1.0` only if:

* `|yaw| <= facing_yaw_thresh_deg`
* `|pitch| <= facing_pitch_thresh_deg`

By default that means roughly:

* yaw within 25 deg
* pitch within 20 deg

So the person must look fairly straight at the camera.

---

### 10j. set continuity flag

`det.continuity_only = weak_depth`

This means detections with weak depth are treated more cautiously later.

---

### 10k. extract body histograms

It extracts 3 HSV histograms:

* torso region
* upper-body region
* lower-body region

using `_extract_region_hist()`.

Each region has:

* histogram values
* valid flag
* confidence

These are appearance cues for the tracker.

---

### 10l. compute detection quality and identity quality

Detection quality:

* depends on YOLO confidence and depth valid ratio

Identity quality:

* computed by `_identity_quality()`
* depends on:

  * blur
  * bbox area
  * visible ratio
  * bbox aspect ratio

This is very important:
the node does not blindly trust every crop for identity update.

If identity quality is high enough and depth is not weak:

* body ReID is marked valid
* histogram validity/confidence is trusted more

If identity quality is poor:

* histogram confidence is reduced

So this node is filtering whether a crop is “good identity evidence” or just “detection only”.

---

### 10m. estimate image-center velocity

It calls `_estimate_center_velocity()`.

This compares current center `(u,v)` with nearest previous center and estimates:

* `vel_u`
* `vel_v`
* confidence

This is a simple image-space motion cue.

It is not perfect tracking, but it gives the tracker an extra continuity hint.

---

### 10n. add detection to output

The filled `CamDetection` is appended to `dets_msg`.

---

### 10o. GUI drawing

If GUI is enabled:

* draws bbox
* draws center point
* colors locked one red, others green
* prints confidence and depth

Locked/not-locked is decided by `_is_locked_detection()` using distance in robot frame to the selected track.

So the GUI is not just pretty display; it tries to show which visual detection corresponds to the locked target.

---

## Step 11: after loop

After all detections:

* GUI frame is shown
* centers are stored for next velocity estimate
* `/cam_detections` is published
* markers are published

That is the end of one full frame cycle.

---

# 12. `_estimate_facing_from_person_bbox()`

This function estimates if the person is facing the robot.

Flow:

1. take person bbox
2. crop upper part of the body, roughly head/upper-body region
3. run RetinaFace on that crop
4. take best detected face
5. convert landmarks from crop coords to full image coords
6. call `_head_pose_from_5pt()`
7. compare yaw and pitch against thresholds
8. return yaw, pitch, roll, facing flag

So this is a **best-effort face-based facing detector**.

If anything fails:

* return zeros
* facing = 0

That means facing cue is conservative.

---

# 13. `_head_pose_from_5pt()`

This is the math behind head pose.

It uses:

* 5 detected face landmarks
* a small canonical 3D face model
* `cv2.solvePnP()`

Then converts the rotation matrix to approximate:

* pitch
* yaw
* roll

So this is not deep-learned head pose estimation.
It is a classical geometric pose estimate from 5 face points.

This is a smart lightweight choice.

---

# 14. `_extract_region_hist()`

This computes an HSV histogram for a body region.

Inputs:

* RGB image
* bbox
* start ratio
* height ratio

Process:

1. crop a portion of the person bbox
2. compute visible ratio
3. reject if visible ratio too small
4. convert ROI to HSV
5. compute a `12 x 8` histogram over H and S
6. normalize histogram

Return:

* histogram list
* valid flag
* confidence

Why it matters:
the tracker later compares these histograms to reduce ID switching.

---

# 15. `_identity_quality()`

This decides how trustworthy a crop is for identity update.

It uses:

* bbox area
* aspect ratio
* crop blur using Laplacian variance
* visible ratio from depth
* whether aspect ratio looks person-like

Then it combines them into one score.

So this function asks:

“Is this crop sharp enough, large enough, visible enough, and person-shaped enough to be useful for identity?”

That is a very practical idea.

---

# 16. `_estimate_center_velocity()` and `_store_centers_for_velocity()`

These are simple frame-to-frame motion cues.

It stores previous detection centers and time.

Then for each new center:

* finds nearest previous center
* estimates pixel velocity
* gives confidence based on how close the match is

This is not a full tracker, but it gives:

* rough image motion direction
* continuity hint

That can help the later tracker.

---

# 17. `_current_locked_target()` and `_is_locked_detection()`

These are mainly for GUI visualization.

`_current_locked_target()`:

* returns the latest latched target if valid

`_is_locked_detection()`:

* compares a camera detection’s robot-frame `(x,y)` to the locked target pose
* if within `gui_lock_match_gate_m`, it counts as locked

So GUI red box = “this detection probably matches the current selected track.”

---

# 18. `run_yolo()`

This is the YOLO helper.

It:

* calls `self.model.predict(...)`
* uses fixed `imgsz=640`
* uses configured confidence threshold
* uses resolved device
* filters results to class `0` only

So only `person` detections survive.

This is the main detector of the node.

---

# 19. `depth_median_in_bbox()`

This is one of the most important helper functions.

It does not use the whole bbox.

It:

1. clamps bbox inside image
2. defines a smaller center ROI using `depth_roi_scale_w` and `depth_roi_scale_h`
3. collects all valid depth values
4. converts them to meters
5. returns:

   * median depth
   * valid ratio

Why this is good:

* person bbox edges often include background
* single depth pixel is noisy
* center median is much more stable

This function is a major reason your 3D position can be stable.

---

# 20. `depth_to_meters()`

This handles depth encoding.

It supports:

* `16UC1` / `mono16` → interpreted as millimeters, converted to meters
* `32FC1` → already meters

So it is compatible with common RealSense aligned depth formats.

---

# 21. `transform_point_camera_to_global()`

This is the camera-to-robot transform step.

It:

1. looks up TF from `camera_frame` to `global_frame`
2. extracts translation and quaternion
3. builds rotation matrix
4. transforms the 3D camera point into robot/global frame

So this is how a pixel becomes a robot-frame detection.

This is critical because `person_track_manager.py` expects positions in the same frame as LiDAR candidates.

---

# 22. `quaternion_to_rotation_matrix()`

This converts quaternion to a 3x3 rotation matrix.

It is a straightforward math helper for the transform.

---

# 23. `make_markers()`

This publishes RViz markers for detections.

Each detection is shown as:

* a sphere
* in `global_frame`
* at detection `(x,y,z)`

Markers are green and have short lifetime, so old markers disappear quickly.

So this is useful for RViz debugging.

---

# 24. What this node really contributes to the whole package

This node provides the tracker with much richer info than just YOLO boxes.

Each `CamDetection` carries:

* robot-frame 3D position
* bbox coordinates
* YOLO confidence
* depth quality
* head pose / facing cue
* torso/upper/lower appearance histograms
* detection quality
* identity quality
* image-space motion cue

So the tracker can do much better identity association.

That is why this node is important.

---

# 25. Strengths of this implementation

The good parts are:

## Good depth handling

Using median depth inside a center ROI is a strong design.

## Good frame handling

It uses camera optical frame and then TF to robot frame, which is correct.

## Optional face/facing cue

RetinaFace is optional, so the system can still run without it.

## Identity-aware design

This node already computes appearance quality and body histograms before the tracker stage.

## Useful debugging

It has:

* GUI
* locked target overlay
* RViz markers
* heartbeat
* runtime backend logs

---

# 26. Weak points / practical limitations

A few things to be aware of:

## 1. No body ReID model actually computed here

Fields like `body_reid_embedding` exist, but in this node they are left empty.

So despite the field names, this node mainly provides:

* histograms
* quality scores
* facing info

not a real learned body embedding.

## 2. Center-velocity estimate is simple

It just matches to nearest previous center in image space.
In crowds, this can be noisy.

## 3. Facing depends on visible face

If face is not visible, turned away, too small, blurred, or occluded:

* RetinaFace fails
* facing becomes 0

So facing cue is useful, but not always available.

## 4. Many logs every frame

This node logs camera depth and distances per detection.
That can become noisy and may slow debugging output if frame rate is high.

---

# 27. Plain-English summary

The easiest way to think about `vision_node.py` is:

It watches the RGB-D camera, finds people with YOLO, gets their depth, converts them into 3D robot-frame positions, optionally checks whether their face is looking toward the robot, extracts body color features, and sends all of that to the tracker.

So in one sentence:

**`vision_node.py` turns raw camera data into rich person observations for tracking and target locking.**

---

# 28. Super short flow chart

The internal flow is basically:

RGB + Depth + CameraInfo
→ synchronize
→ YOLO person boxes
→ center depth median
→ 2D pixel to 3D camera point
→ TF to `base_footprint`
→ optional RetinaFace head pose
→ body hist + quality scores
→ publish `CamDetections`

---

If you want, the next best step is I explain **each important function in `vision_node.py` one by one with “input → process → output” format**, or I can explain **how `vision_node.py` connects to `person_track_manager.py`**.



————————————————————————————follow_person.py———————————————————————————————————————————
Yes — `follow_person.py` is much simpler than `vision_node.py`, but it is still important.

This file is the **LiDAR candidate detector** for your person-following stack.

Its job is **not** to identify a person exactly.
Its job is:

**LaserScan**
→ find human-sized blobs in front of the robot
→ keep only stable ones
→ publish them as `/people_candidates`

So this node gives the tracker a second sensor stream besides the camera.

---

# 1. What this file is for

In your full pipeline:

* `vision_node.py` gives camera-based person detections
* `follow_person.py` gives LiDAR-based person-like candidates
* `person_track_manager.py` fuses both

So `follow_person.py` is basically answering:

**“From LiDAR only, where are the likely person-sized objects?”**

It helps when:

* camera depth is weak
* lighting is bad
* brief occlusion happens
* you want extra spatial stability

But by itself, this node is **not enough** to know who the correct person is.
It only gives candidate positions.

---

# 2. Structure of the file

This file has 3 main parts:

### `LidarCandidateTrack`

A small dataclass for short-term candidate memory.

### `LidarPeopleCandidates`

The ROS node.

### `main()`

Normal ROS startup entry.

---

# 3. `LidarCandidateTrack` dataclass

This is a lightweight memory object for each LiDAR candidate.

It stores:

* `track_id`
  candidate ID

* `centroid`
  current 2D position `[x, y]`

* `velocity`
  estimated 2D velocity

* `age_sec`
  how long this candidate has existed

* `last_seen_sec`
  last update time

* `persistence`
  how many updates it survived

* `widths`
  recent observed cluster widths

This tells you something important:

`follow_person.py` is **not purely frame-by-frame**.
It has a tiny short-term persistence tracker.

That is good, because raw LiDAR clustering alone is often noisy.

---

# 4. `LidarPeopleCandidates.__init__()`

This is where the node is configured.

The node name is:

* `lidar_people_candidates`

---

## 4.1 Parameters

The file declares several groups of parameters.

## A. Input and frame

* `scan_topic`
* `base_frame`

Usually:

* input is `/scan`
* output frame is `base_footprint`

So no matter what frame the LiDAR publishes in, this node wants to express candidates in robot base frame.

---

## B. ROI filtering

These parameters limit where candidates are allowed:

* `min_range_m`
* `max_range_m`
* `min_x_m`
* `max_x_m`
* `min_y_m`
* `max_y_m`

Default logic:

* ignore very near points
* ignore very far points
* keep only points in front of the robot
* keep only points inside a left-right corridor

This is important.

By default:

* `min_x = 0.0`
* `max_x = 6.0`
* `min_y = -3.0`
* `max_y = 3.0`

So the node only looks **in front of the robot**, not behind it.

That makes sense for person following, because the robot usually only cares about forward targets.

---

## C. Clustering parameters

* `cluster_dist_m`
* `min_cluster_points`
* `max_cluster_points`

These control how scan points get grouped into one blob.

Default:

* neighbor distance threshold = `0.18 m`

So if two adjacent scan points are closer than 18 cm, they stay in the same cluster.

---

## D. Cluster physical size

* `min_cluster_size_m`
* `max_cluster_size_m`

These reject blobs that are too small or too large.

Default:

* too tiny → probably noise
* too big → probably wall, table, or large obstacle

This is a very important filter.

---

## E. TF

* `tf_timeout_sec`

Used when transforming from scan frame into base frame.

---

## F. Marker settings

* `publish_markers`
* `marker_scale_m`
* `marker_lifetime_sec`

These are only for RViz display.

---

## G. Candidate persistence / person-likeness

* `lidar_candidate_max_age_sec`
* `lidar_candidate_min_persistence_frames`
* `lidar_width_person_min_m`
* `lidar_width_person_max_m`
* `lidar_width_stability_max_std`
* `lidar_candidate_assoc_gate_m`
* `lidar_person_likeness_min`

These are the interesting ones.

They say:

* a candidate should survive for several frames
* its width should stay in a human-like range
* its width should not fluctuate too much
* it should match nearby old candidates to build persistence
* only candidates with enough “person-likeness” get published

So the node is not just clustering.
It is trying to publish only **stable human-like clusters**.

---

## 4.2 QoS

It uses sensor-style QoS:

* `BEST_EFFORT`

That is normal for LiDAR.

---

## 4.3 TF setup

It creates:

* `tf_buffer`
* `tf_listener`

This is needed because the scan may come in LiDAR frame, but you want candidates in `base_frame`.

---

## 4.4 Publishers and subscribers

### Subscriber

* `LaserScan` on `scan_topic`

### Publishers

* `/people_candidates` as `PoseArray`
* `/people_candidates_markers` as `MarkerArray`

It also initializes:

* `_candidate_tracks = {}`
* `_next_track_id = 1`

So this node keeps a small internal memory of candidates over time.

---

# 5. Main callback: `scan_cb()`

This is the core of the file.

Every LiDAR scan goes through this function.

The logic is very clean.

---

## Step 1: convert scan ranges to XY points

The code loops through every range value in `scan.ranges`.

For each valid range:

* `x = r * cos(angle)`
* `y = r * sin(angle)`

If the range is invalid or outside min/max range:

* it stores `None`

Important detail:
it keeps the **scan order**.

That matters because later clustering is based on neighboring scan points in sequence.

So the result is a list like:

* valid point
* valid point
* None
* valid point
* ...

The `None` entries help break clusters naturally.

---

## Step 2: lookup TF from scan frame to base frame

It tries:

* transform from `scan.header.frame_id` to `base_frame`
* at the scan timestamp

If that fails:

* fallback to latest available transform

If both fail:

* callback returns without publishing

This means the node depends on a working TF tree.

That is normal and necessary.

---

## Step 3: transform points into base frame and apply ROI

Every valid scan-frame point is transformed into base frame using:

* rotation matrix `R`
* translation vector `t`

Then it checks whether the transformed point lies inside:

* `min_x <= x <= max_x`
* `min_y <= y <= max_y`

If not:

* replace with `None`

So after this stage, `pts_base` is still scan-ordered, but only contains:

* points in front ROI
* in `base_frame`

This is very important because now clustering happens in the robot coordinate frame.

---

## Step 4: cluster points in scan order

It calls:

* `sequential_clusters(pts_base, self.cluster_dist)`

This is a fast scan-order clustering algorithm.

It works like this:

* if next point is near previous point, same cluster
* if gap is too large, start a new cluster
* if `None` occurs, cluster breaks immediately

This is much simpler than DBSCAN, but for 2D laser scan it is often good enough and very fast.

So the clusters reflect contiguous LiDAR blobs.

---

## Step 5: filter clusters and compute observations

For each cluster:

### 5a. point count filter

It rejects clusters with:

* too few points
* too many points

Why:

* too few = noisy speck
* too many = big surface or wall section

---

### 5b. compute bounding box size

It gets:

* `size_x = max(x) - min(x)`
* `size_y = max(y) - min(y)`
* `size = max(size_x, size_y)`

This is a rough width/size measure.

Then it rejects if:

* `size < min_cluster_size_m`
* `size > max_cluster_size_m`

So only medium-sized blobs survive.

This is the first human-shape approximation.

---

### 5c. compute centroid

It computes:

* `cx = mean(x)`
* `cy = mean(y)`

This centroid becomes the candidate position.

Then it creates an observation dict containing:

* `centroid`
* `width`
* `motion_hint`

Important:
`motion_hint` here is just `norm([cx, cy])`, so it is really distance from robot, not true motion.
And later it is not actually used much.

So the meaningful observation fields are mainly:

* centroid
* width

---

## Step 6: update short-term candidate tracks

It converts raw observations into stable persistent candidates by calling:

* `_update_candidate_tracks(observations, now_sec)`

This is the second most important part of the file after clustering.

Without this part, the node would be much noisier.

---

## Step 7: publish `PoseArray`

It creates a `PoseArray` in `base_frame`.

For each persistent candidate:

* `pose.position.x = cx`
* `pose.position.y = cy`
* `pose.position.z = person_likeness`
* `pose.orientation.w = 1.0`

Important detail:

### `z` is used to store `person_likeness`

This is a hack.

Since `PoseArray` has no custom confidence field, the code stores confidence in `position.z`.

So downstream code must know this convention.

That means:

* `x, y` = actual candidate location
* `z` = not height, but person-likeness score

It works, but it is not very clean semantically.

---

## Step 8: publish markers

If enabled, it calls:

* `make_markers(persistent, stamp)`

So RViz can show current candidates.

---

# 6. `sequential_clusters()`

This function is the clustering algorithm.

It takes:

* scan-ordered point list
* neighbor distance threshold

Logic:

* if current point is `None`, end current cluster
* if first valid point, start cluster
* if distance from previous point <= threshold, keep same cluster
* otherwise, close previous cluster and start new one

This is efficient because LiDAR points already come ordered by angle.

### Why it works

On a laser scan, points belonging to one object tend to appear next to each other in scan order.

### Limitation

If two people stand very close together, their clusters may merge into one.

So this method is fast and simple, but not perfect in crowds.

---

# 7. TF helper functions

## `lookup_tf()`

Looks up the transform.

First tries exact scan time.
If that fails, it tries latest transform.

This is a practical fallback.

---

## `tf_to_R_t()`

Converts ROS transform quaternion + translation into:

* rotation matrix `R`
* translation vector `t`

Even though it computes full 3D rotation, the node later only uses XY.

---

## `transform_2d()`

Applies:

* `pb = R @ p + t`

using point `[x, y, 0]`.

So each LiDAR point becomes a base-frame point.

---

# 8. `_update_candidate_tracks()`

This is the heart of the persistence logic.

Input:

* current frame observations
* current time

Output:

* filtered persistent candidates ready for publishing

Let’s go step by step.

---

## Step 1: match each new observation to existing candidate track

For every observation:

* compare to all current candidate tracks
* find nearest track within `lidar_candidate_assoc_gate_m`

If no nearby old track exists:

* create new `LidarCandidateTrack`

If a nearby track exists:

* update that track

So this is a nearest-neighbor association in 2D position.

---

## Step 2: new track creation

If no match:

* assign new `track_id`
* store centroid
* set persistence = 1
* initialize widths list with current width

So every new blob starts as a weak candidate.

---

## Step 3: matched track update

If matched:

* compute `dt`
* estimate velocity from centroid difference
* smooth velocity with `0.5 old + 0.5 new`
* update centroid
* update `last_seen_sec`
* increase `age_sec`
* increase `persistence`
* append new width
* keep last 6 widths only

This means the node stores a short width history for each candidate.

That width history is very important for person-likeness scoring.

---

## Step 4: delete stale tracks

If a track was not matched recently and:

* `now_sec - last_seen_sec > lidar_candidate_max_age_sec`

then it is deleted.

So old LiDAR candidates disappear quickly when scan evidence is gone.

---

## Step 5: compute person-likeness and keep only good persistent tracks

For every remaining candidate track:

* compute `likeness = _person_likeness(tr)`

Then require:

* `persistence >= lidar_candidate_min_persistence_frames`
* `likeness >= lidar_person_likeness_min`

Only then the candidate is published.

This is the main denoising rule.

So a cluster must be:

* stable enough across frames
* human-like enough in width and behavior

before it becomes a final output candidate.

---

# 9. `_person_likeness()`

This function turns a candidate track into a score between roughly 0 and 1.

It uses 4 parts.

---

## 9.1 Width score

It computes:

* mean width from recent widths

If width is inside:

* `lidar_width_person_min_m <= mean_width <= lidar_width_person_max_m`

then:

* `width_score = 1.0`

Else:

* `width_score = 0.25`

So width matters a lot.

This is trying to answer:
“Does the cluster look roughly person-sized?”

---

## 9.2 Stability score

It computes standard deviation of widths.

If width changes too much across recent frames:

* stability score goes down

Formula is roughly:

* high score when width std is small
* low score when width std is large

This is smart because real people usually produce somewhat stable LiDAR width over short time.

---

## 9.3 Persistence score

It increases with how many frames the candidate survived.

If it has already survived enough frames:

* persistence score saturates at 1.0

This helps reject one-frame noise.

---

## 9.4 Motion score

It computes:

* `1 - ||velocity|| / 2`

So faster-moving candidate tracks get lower score.

This is a bit conservative.

It is basically saying:

* very unstable or fast-jumping LiDAR candidates are less trustworthy

This is not true human motion modeling.
It is more like a stability penalty.

---

## 9.5 Final score

The final weighted sum is:

* `0.35 * width_score`
* `0.25 * stability_score`
* `0.25 * persistence_score`
* `0.15 * motion_score`

So width is the strongest feature, then stability and persistence.

This tells you the node believes:

**“person-likeness” = human width + stable width + lasting across frames + not too jumpy**

That is a reasonable heuristic for LiDAR-only candidates.

---

# 10. `make_markers()`

This builds RViz markers for the persistent candidates.

For each candidate:

* create a sphere in `base_frame`
* position it at centroid
* scale it using `marker_scale`
* color it blue-ish / green-ish depending on person-likeness

Interesting part:

* `g = 0.2 + 0.8 * likeness`
* `b = 1.0`

So higher-likeness candidates look more bright/strong in green-blue.

This is just for visualization, but useful for tuning.

---

# 11. What this node publishes exactly

## `/people_candidates`

Type:

* `geometry_msgs/PoseArray`

Each pose contains:

* `x` = candidate x in base frame
* `y` = candidate y in base frame
* `z` = person-likeness score

This is the actual output used by `person_track_manager.py`.

---

## `/people_candidates_markers`

Type:

* `visualization_msgs/MarkerArray`

This is only for RViz.

---

# 12. How this node fits into the full system

This node is not meant to work alone.

Its purpose in your full pipeline is:

### Camera side

`vision_node.py`
gives:

* person positions
* appearance cues
* facing cues

### LiDAR side

`follow_person.py`
gives:

* stable person-like blobs

### Fusion side

`person_track_manager.py`
combines both

So `follow_person.py` mainly strengthens:

* spatial continuity
* robustness to short camera loss
* selected-target LiDAR bridge logic later in the tracker

That is why this node matters.

---

# 13. Strengths of this implementation

What is good here:

## It is simple and fast

The algorithm is lightweight:

* scan conversion
* scan-order clustering
* simple persistence memory

That is good for real-time robotics.

## It uses TF correctly

Candidates are published in `base_frame`, which is exactly what fusion wants.

## It filters noise reasonably

It does not publish every raw LiDAR blob.
It requires:

* enough points
* reasonable size
* enough persistence
* decent width stability

## It is useful for fusion

This is exactly the kind of clean candidate source a camera-LiDAR fusion tracker wants.

---

# 14. Limitations of this file

There are also some limitations.

## 1. It is not a real person classifier

It does not truly know “this is a person.”

It only knows:

* this looks like a stable human-sized LiDAR blob

So in some scenes, table legs, narrow shelves, or chair clusters may still look person-like.

---

## 2. Scan-order clustering can merge nearby people

If two people stand too close:

* they may become one cluster

So crowded scenes are harder.

---

## 3. No leg-shape modeling

This node does not detect:

* two-leg pattern
* walking gait
* temporal body shape

It only uses:

* cluster size
* width stability
* persistence

So it is a heuristic detector, not a sophisticated LiDAR human detector.

---

## 4. `PoseArray.position.z` is overloaded

Using `z` to store confidence is practical, but semantically messy.

A custom message would be cleaner.

---

## 5. Motion score is simplistic

It penalizes large velocity norm, but that velocity is based only on centroid jumps between short candidate matches.

So it is more a stability term than true motion understanding.

---

# 15. Plain-English summary

The easiest way to understand `follow_person.py` is:

It watches the LiDAR scan, groups nearby scan points into blobs, removes blobs that are too small, too large, or too unstable, keeps blobs that persist for a few frames and look person-sized, and publishes their center positions as person candidates.

So in one sentence:

**`follow_person.py` turns raw LiDAR scan blobs into stable person-like candidate positions for the fusion tracker.**

---

# 16. Very short pipeline view

Inside this file, the logic is:

`/scan`
→ valid XY points
→ TF to base frame
→ ROI filter
→ sequential clustering
→ size filter
→ centroid extraction
→ short-term candidate tracking
→ person-likeness scoring
→ publish `/people_candidates`

---

# 17. Most important tuning parameters

If you tune this node, the most important ones are:

### `cluster_dist_m`

Controls whether one object becomes one cluster or gets split.

### `min_cluster_points` / `max_cluster_points`

Rejects sparse noise and very large objects.

### `min_cluster_size_m` / `max_cluster_size_m`

Very important for human-like size filtering.

### `lidar_candidate_assoc_gate_m`

Controls how easily the same candidate is matched across frames.

### `lidar_candidate_min_persistence_frames`

Higher value = more stable but slower to appear.

### `lidar_width_person_min_m` / `lidar_width_person_max_m`

These decide what “human-like width” means.

### `lidar_person_likeness_min`

This is the final strictness threshold.

---

# 18. My honest conclusion

`follow_person.py` is a good supporting node.

It is:

* lightweight
* clean
* useful for fusion

But it is **not the main intelligence** of the person-following package.

Its role is:

**give the main tracker stable LiDAR evidence of person-like objects**

That is exactly the correct role for LiDAR in your architecture.

The next best step is to explain **how `follow_person.py` and `vision_node.py` are fused inside `person_track_manager.py`**, because that is where the real power of the package comes from.


————————————————————————————————————————————————person_track_manager.py————————————————————————————————————————————————————————————————————————————
  Yes — `person_track_manager.py` is the **core brain** of the whole `person_following` package.

Among all the files, this one does the most important work.

It is not just a tracker. It is doing **four jobs at once**:

1. fuse camera detections and LiDAR candidates
2. maintain multi-person tracks over time
3. protect one selected target from ID switching
4. publish rich track state for the rest of the system

So the best way to understand it is:

**`vision_node.py` says “I see people from camera.”**
**`follow_person.py` says “I see person-like blobs from LiDAR.”**
**`person_track_manager.py` says “which observations belong to which person, and which one is my chosen person?”**

---

# 1. Main role of this file

This file receives:

* camera detections from `/cam_detections`
* LiDAR candidates from `/people_candidates`
* RGB image for optional ReID extraction
* manual selected target ID from `/person_target_select_id`

Then it produces:

* all live tracks
* selected target state
* debug information
* markers
* optional direct selected-target publish

So this file is the place where the raw perception becomes a real tracked person model.

---

# 2. Main classes in the file

There are 5 important class definitions before the main node.

## `TrackState`

This is the lifecycle of a normal track:

* `TENTATIVE`
* `CONFIRMED`
* `OCCLUDED`
* `LOST`
* `DELETED`

Meaning:

* `TENTATIVE`: new track, not trusted yet
* `CONFIRMED`: stable track
* `OCCLUDED`: recently missing but still kept alive
* `LOST`: mostly gone
* `DELETED`: remove it

---

## `TargetLockState`

This is a second, separate lifecycle only for the selected target:

* `NONE`
* `LOCKED`
* `TEMP_OCCLUDED`
* `TEMP_LOST`
* `REACQUIRE`

This separation is very important.

A track can still exist in the tracker, but the selected-target logic may say:

* still locked
* temporarily occluded
* now trying to reacquire

So this file separates:

* generic tracking state
* selected-person-following state

That is a strong design.

---

## `FusedMeasurement`

This is one observation after fusion logic.

A `FusedMeasurement` can come from:

* `camera_only`
* `cam_lidar`
* `lidar_only`

It stores much more than position.

Important fields include:

* `pos_xy` → final 2D measurement position
* `camera_xy` → camera-based XY
* `lidar_xy` → LiDAR-based XY if available
* `camera_confidence`
* `cam_lidar_error`
* `embedding`
* `depth_valid_ratio`
* `bbox_area_px`
* `range_m`
* `bearing_rad`
* `center_uv`
* `bbox_xyxy`
* `facing01`
* `head_rpy_deg`
* torso / upper / lower histograms
* face embedding
* identity quality
* detection quality
* `continuity_only`
* image-center velocity cues

So the measurement object is already a rich sensor package, not just `[x,y]`.

---

## `Track`

This is one tracked person.

It stores:

* `track_id`
* state vector `x = [px, py, vx, vy]`
* covariance `P`
* lifecycle state
* frame id
* last update time
* hits / misses / age
* last measurement source
* appearance embedding memory
* last facing result
* IMM states and probabilities
* track score
* appearance and motion confidence
* embedding gallery
* histogram references

This means one track contains both:

* motion memory
* identity memory

That is why this file is much stronger than a simple nearest-neighbor tracker.

---

## `ReIDExtractor`

This is an optional ONNX-based body embedding extractor.

If ONNX runtime and OpenCV are available, it:

* loads the ReID model
* crops the person image
* runs inference
* normalizes the feature vector

If not available, it safely returns no embedding.

So ReID is optional, not mandatory.

---

# 3. What the node subscribes and publishes

Inside `PersonTrackManager.__init__()`:

## Subscriptions

It subscribes to:

* `CamDetections` on `/cam_detections`
* `PoseArray` on `/people_candidates`
* RGB image on `rgb_topic`
* `Int32` on `/person_target_select_id`

So it is listening to both sensors plus selection command.

---

## Publishers

It publishes:

* `/person_tracks` → markers
* `/person_tracks_poses` → `PoseArray`
* `/person_tracks_state` → `TrackedPoseArray`
* `/person_target_locked` → selected target
* `/person_target_locked_marker`
* `/person_track_debug`
* `/person_target_lock_debug`
* target facing topic

These outputs are what the rest of the system uses.

---

# 4. Huge parameter system

This file has a very large number of parameters. That is because most of the behavior is tunable.

The parameters fall into several groups.

## A. Topics and frames

These define input/output topics and the main tracking frame.

Examples:

* camera detections topic
* LiDAR candidate topic
* RGB topic
* `track_frame`
* `target_facing_topic`

---

## B. Fusion / association gates

These decide how camera and LiDAR get combined, and how tracks get matched.

Examples:

* `cam_lidar_gate_m`
* `track_position_gate_m`
* `allow_camera_only_updates`
* `allow_lidar_only_updates`
* `selected_lidar_gate_m`
* `selected_lidar_bearing_gate_deg`
* `selected_lidar_max_speed_mps`
* `association_gate_mode`
* `maha_gate_thresh`

These parameters control how strict the tracker is.

---

## C. Track lifecycle

These decide how easy it is to confirm, occlude, lose, and delete a track.

Examples:

* `min_hits_confirm`
* `max_missed_frames`
* `max_occluded_frames`
* `max_lost_frames`
* `confirm_score_thresh`
* `delete_score_thresh`

---

## D. Selected-target protection

This is the most special part of the file.

Examples:

* `selected_hard_hold_sec`
* `selected_reacquire_window_sec`
* `selected_no_switch_during_hard_hold`
* `switch_confirm_frames`
* `reacquire_confirm_frames`
* `selected_match_min_score`
* `selected_match_clean_update_score`
* `selected_match_ambiguous_margin`
* `relock_cooldown_sec`
* `post_relock_protection_sec`
* `crossover_*`
* `contradiction_threshold`

These are what make the file not just a generic tracker.

They encode the logic:
**“do not switch away from the chosen person too easily.”**

---

## E. Motion model

These control the IMM / Kalman behavior.

Examples:

* process noise
* measurement noise
* IMM transition probabilities
* turn rate
* stop time constant

---

## F. Appearance and ReID

These control identity updates.

Examples:

* ReID model path
* ReID providers
* embedding EMA alpha
* gallery size
* minimum bbox area
* yaw and pitch limits for appearance update

These decide when appearance evidence is good enough to trust.

---

# 5. Internal memory the node keeps

Besides tracks, the node keeps a lot of selected-target memory.

Examples:

* `selected_track_id`
* `selected_last_seen_sec`
* `selected_last_clean_seen_sec`
* `target_lock_state`
* `target_reacquire_count`
* `selected_embedding_frozen`
* `selected_relock_cooldown_until_sec`
* selected range history
* selected bearing history
* selected bbox size history
* selected center history
* selected torso / upper / lower hist refs
* selected face/body embedding refs
* selected identity gallery
* stable anchor embedding
* crossover state

This tells you the design philosophy very clearly:

The file is trying to remember **what the selected person has looked like recently**, not only where they are.

---

# 6. The runtime loop: `timer_cb()`

This is the main tracking loop. It runs every 0.05 s, so about **20 Hz**.

The full logic is:

1. build fused measurements
2. update crossover state
3. predict all tracks
4. associate measurements to tracks
5. update matched tracks
6. age unmatched tracks
7. spawn new tracks
8. delete dead tracks
9. auto-select nearest if needed
10. publish outputs

That is the core cycle.

Now I explain each part.

---

# 7. `_build_fused_measurements()`

This is the fusion front-end.

It takes the latest camera detections and latest LiDAR candidates and turns them into a list of `FusedMeasurement`.

## First freshness check

It checks whether:

* camera message is fresh enough
* LiDAR message is fresh enough

using `max_msg_age_sec`.

So stale messages are ignored.

---

## LiDAR side

It reads the `PoseArray` candidates and converts them into a list of 2D points in `track_frame`.

---

## Camera stale case

If camera is stale, the file does something special.

It may try:

### selected-target LiDAR bridge

This is `_build_selected_lidar_bridge_measurement()`.

This is one of the smartest parts of the whole file.

It says:

“If camera is temporarily stale, but LiDAR sees a plausible point near the predicted selected target, create a temporary `lidar_only` measurement just for the selected target.”

This is not blind LiDAR use.

It checks:

* selected track exists
* selected track is still valid
* camera stale amount is inside a bridge window
* track is not too stale
* LiDAR candidate is close enough to predicted selected target
* bearing is close enough
* jump speed is not too high
* choice is not ambiguous
* same candidate persists for enough hits

Only then it accepts the bridge measurement.

That is exactly the kind of logic you need in crowds.

---

## Camera fresh case

If camera is fresh, the file processes each camera detection.

For each camera detection it builds a dict with:

* `cam_xy`
* confidence
* depth quality
* bbox size
* range
* bearing
* center UV
* bbox coordinates
* facing info
* head pose
* histograms
* face embedding
* body ReID confidence
* quality scores
* continuity flag
* image-center velocity
* embedding

If body embedding is not already present in the detection, it may try to extract it from the latest RGB image using the ReID model.

---

## Camera-LiDAR matching

Then it tries to match each camera detection to a LiDAR point.

If SciPy is available:

* Hungarian assignment

Else:

* greedy nearest pairing

Matching is only allowed if distance is within `cam_lidar_gate_m`.

---

## Fused measurement types

After matching:

### `cam_lidar`

If a camera detection matches a LiDAR point:

* final position = LiDAR point
* camera info still kept
* source = `cam_lidar`

This is important:
the code trusts LiDAR position more when both exist.

### `camera_only`

If no LiDAR match but camera-only updates are allowed:

* final position = camera position
* source = `camera_only`

### `lidar_only`

Only used under controlled situations:

* camera stale recovery
* non-crowd mode
* selected-target bridge
* or explicitly allowed modes

So `lidar_only` is treated more cautiously.

---

# 8. `_predict_track()`

This is the motion prediction step.

This file does not use only one constant velocity model.

It uses an **IMM-style predictor** with 3 models:

* constant velocity
* turning
* slow/stop

The function:

* mixes model probabilities
* predicts each model separately
* updates model probabilities
* merges them back into one state and covariance

That is much better than a simple Kalman CV-only tracker, because people:

* turn
* stop
* slow down
* move unpredictably

So this file’s motion modeling is actually fairly advanced.

---

# 9. `_update_track()`

This is the measurement update step after a track gets matched.

What it updates:

## A. Kalman/IMM state

It runs the measurement update for all IMM models and then merges them.

So position/velocity get corrected using the new measurement.

---

## B. ReID embedding

If measurement has an embedding:

* `last_embedding` is updated
* EMA embedding can be updated
* gallery can be updated

But appearance update is only allowed if conditions are good enough:

* association cost not too bad
* camera confidence high enough
* depth valid ratio high enough
* bbox area big enough
* head pose okay

So the file does **not** blindly learn from every frame.

That is very good.

---

## C. Histogram references

It blends torso / upper / lower histograms gradually using `_blend_hist_ref()`.

So color appearance becomes more stable over time.

---

## D. Face and facing

It updates:

* `last_facing01`
* `last_head_rpy_deg`

So each track also stores whether the last strong evidence suggested the person was facing the robot.

---

## E. Selected-target history

If this track is the selected one and history update is allowed, it updates selected history:

* range
* bearing
* bbox size
* center
* hist refs
* identity gallery
* stable anchor embedding

This history is later used for reacquire and ambiguity handling.

---

# 10. `_associate()`

This is the matching logic.

It is multi-stage.

## Stage 0: selected target first

It first calls `_associate_selected_first()`.

This means:
the chosen person is handled before all normal track associations.

That is a big design decision.

The selected person is more important than general MOT purity.

---

## Stage 1: strict association

For the remaining tracks and measurements, it does a strict association pass.

This only uses measurements that have:

* high enough camera confidence
* enough depth quality
* embedding available

So this pass prefers strong, identity-rich measurements.

---

## Stage 2: relaxed association

Then it does a second pass with looser conditions for whatever is left.

So the file tries:

* first, high-quality identity-aware assignment
* then, more relaxed geometric assignment

That is a good staged design.

---

# 11. `_associate_stage()`

This is the generic association engine used in Stage 1 and Stage 2.

For each track-measurement pair it checks:

* Euclidean distance gate
* Mahalanobis gate
* appearance gate

Then it builds cost from:

* position cost
* appearance cost
* motion cost

and runs either:

* Hungarian assignment
* or greedy fallback

So the file is not doing simple nearest neighbor only.

---

# 12. `_appearance_cost()`

This compares a measurement embedding against a track’s embedding memory.

It uses:

* EMA embedding
* gallery embeddings

and returns the best cosine-distance-based cost.

So appearance matching is based on:
**“how similar is this new embedding to what I know this track looks like?”**

---

# 13. Selected-target-first logic

This is the part that makes the file special.

## `_associate_selected_first()`

If the selected target is valid, it does not let normal association decide first.

Instead it:

1. gathers candidate measurements that pass hard gates
2. scores them with `_selected_match_score()`
3. compares best and second-best candidates
4. rejects front occluders
5. decides whether the match is:

   * clean
   * acceptable
   * ambiguous
   * rejected

### Hard gates include:

* position gate
* bearing gate
* image center corridor gate

So only measurements consistent with recent selected-person history are considered.

---

## `_selected_match_score()`

This score combines:

* appearance score
* range score
* bearing score
* bbox-size score
* motion score

Appearance score itself gets penalized if:

* embedding is missing
* confidence is weak
* measurement is lidar-only
* continuity-only flag is set

So this score tries to answer:
**“Does this measurement really look like my selected person?”**

---

## Clean vs acceptable

If the best match score is above `selected_match_clean_update_score`:

* treat as clean strong selected match

If above `selected_match_min_score`:

* treat as acceptable but weaker match

If too ambiguous:

* reject and go into hold/occlusion path

This is very practical.

---

# 14. Front occluder rejection

The function `_is_front_occluder()` is important.

It tries to detect cases where:

* something appears in front of the selected person
* at similar bearing / image area
* but closer in range

This prevents a nearby blocker from being mistaken as the selected person.

That is very useful in real crowds.

---

# 15. Selected no-match handling

If selected target has no acceptable match, `_handle_selected_no_match()` manages the state machine.

The path is roughly:

* `LOCKED`
  → `TEMP_OCCLUDED`
  → `TEMP_LOST`
  → `REACQUIRE`

During this time:

* appearance learning gets frozen
* hold windows can be used
* the track is not immediately abandoned

This is how the robot avoids losing the person instantly after one bad frame.

---

# 16. Reacquire logic

When selected target is missing but identity is still protected, the file enters reacquire mode.

## `_associate_selected_reacquire()`

This does a different kind of scoring.

It checks whether measurements fit selected-person history:

* range
* bearing
* bbox size
* center location
* body embedding
* histograms
* face embedding if available

Then `_score_reacquire_candidate()` combines:

* appearance
* range
* bearing
* bbox
* center
* identity consistency

It also computes whether cues are contradictory.

So reacquire is not “grab nearest person.”
It is much stricter.

---

## Cross-track reacquire rules

If the reacquire candidate belongs to a different track ID than the old selected one, the file becomes even stricter.

It checks things like:

* hard hold window
* reacquire window
* missing appearance
* contradictory cues
* weak appearance
* weak identity
* ambiguous best-vs-second score
* crossover activity

This is exactly the logic that reduces wrong-person relock in crowded scenes.

---

# 17. Crossover logic

The file has a dedicated `_update_crossover_state()`.

This tries to detect when another measurement is very close to the selected target in:

* range
* bearing
* image center
* bbox size

but is not actually the selected target’s own measurement.

If so, it marks `crossover_active = True`.

While crossover is active:

* relock rules become stricter
* single weak cue is not enough
* ambiguity tolerance decreases

So the code explicitly knows:
**“a dangerous crossing situation is happening.”**

That is advanced behavior.

---

# 18. History and identity memory

The selected target keeps several histories:

* range history
* bearing history
* bbox width history
* bbox height history
* image center history

It also keeps long-term identity references:

* selected body embedding ref
* stable anchor embedding
* identity gallery
* torso / upper / lower histogram refs
* face embedding ref

These are used for:

* selected hard gates
* reacquire gating
* reacquire scoring
* contradiction detection

So the selected target is remembered by both:

* where it has been
* what it has looked like

---

# 19. Contradiction handling

The code checks for contradictory identity evidence.

For example:

* one cue says strong match
* another cue says strong mismatch

If those cues disagree too much, the candidate can be rejected.

That is safer than trusting one single feature blindly.

This is especially useful when:

* ReID is weak
* color hist is misleading
* lighting changes
* face is partial

---

# 20. Track spawn and deletion

## Spawning

`_spawn_track()` creates a new track from a measurement.

Initial state:

* position from measurement
* zero velocity
* moderate covariance
* tentative state
* track score 0.2

If embedding exists, it stores it immediately.

---

## Deletion

Tracks are deleted when:

* too many misses
* score gets too low
* lost too long

So weak tracks naturally disappear.

---

# 21. Auto-select behavior

`_auto_select_if_needed()` will automatically select the nearest confirmed track if:

* no selected target currently exists
* `auto_select_nearest_on_start` is enabled

So the system can start following without manual ID selection.

---

# 22. Publish logic

`_publish_outputs()` publishes all outputs every cycle.

Important outputs:

## `/person_tracks_state`

This includes, for each track:

* track id
* pose
* lifecycle state
* track score
* appearance confidence
* motion confidence
* lock confidence
* identity confidence
* motion confidence
* staleness
* predicted flag
* reacquire flag
* contradictory candidate flag
* crossover flag
* selected flag
* publishable flag
* camera visible
* lidar support
* facing01
* facing_robot
* lock state
* suppression code

This is a very rich track state message.

---

## `/person_target_locked`

When direct target publishing is enabled, it publishes the selected target with:

* pose
* track score
* confidence fields
* predicted status
* publishable
* facing info
* suppression code

---

## Facing output

This file computes facing from the selected track’s `last_facing01`:

* `facing01 >= 0.5` → `facing_robot = True`

So inside this file, facing is tied to the actual vision-derived facing value.

---

# 23. Publishable / predicted logic

The selected target is not always published as fully valid.

`_selected_target_publishable()` checks:

* target really is selected
* not deleted
* not in reacquire state
* no contradiction active
* staleness within prediction limits

So even if the selected track still exists internally, it may be suppressed from publication if it is too uncertain.

This is a good safety layer.

---

# 24. Confidence calculation

The file computes several confidence values.

## Lock confidence

Combination of:

* track score
* identity confidence
* motion confidence
* freshness penalty
* penalties for predicted / contradictory / reacquire cases

## Identity confidence

Combination of:

* app confidence
* similarity to selected body embedding ref
* similarity to stable anchor embedding
* similarity to identity gallery

## Motion confidence

Based on motion quality and prediction uncertainty.

So the file is not just outputting one raw confidence number.
It separates the reasons for trust.

---

# 25. What makes this file strong

This file is strong because it is not doing only one thing.

It combines:

## A. Real sensor fusion

camera + LiDAR, with careful use of lidar-only cases

## B. Real motion tracking

IMM-style prediction, not just nearest point

## C. Real identity memory

embedding, histograms, face cues, gallery, stable anchor

## D. Selected-target protection

hard hold, relock cooldown, reacquire windows, contradiction checks

## E. Crowd-specific logic

crossover detection, ambiguous-margin logic, stricter relock rules

That is why this file is the real intelligence center of the package.

---

# 26. Main weakness of the file

Its biggest weakness is not logic quality.

Its biggest weakness is **size and complexity**.

At about 150kB / 3000+ lines with many responsibilities, it is hard to maintain.

It is currently acting as:

* fusion module
* tracker
* selected-target manager
* identity manager
* debug publisher

all in one file.

So technically it is powerful, but structurally it is a “god file”.

---

# 27. Plain-English summary

The easiest way to understand `person_track_manager.py` is:

It takes person observations from camera and LiDAR, builds and updates multiple tracked people, gives special protection to the currently followed person, tries hard not to switch to the wrong person during occlusion or crowd crossing, and publishes the final track and target state for the rest of the robot.

So in one sentence:

**`person_track_manager.py` is the fusion + tracking + selected-person-lock brain of your whole person-following system.**

---

# 28. Very short internal flow

The internal flow is:

camera detections + LiDAR candidates
→ fused measurements
→ predict tracks
→ selected-target-first association
→ normal association
→ update / miss / spawn / delete tracks
→ manage target lock state
→ publish tracks and selected target

---

# 29. Best mental model for this file

Think of it like this:

* `vision_node.py` gives “who might be there”
* `follow_person.py` gives “where blobs are”
* `person_track_manager.py` decides “which blob and camera box belong to which person”
* and then also decides “which one is my person, and should I keep trusting that ID?”

That second part is the most important.

Because in person-following, the real problem is often **not detection**.
It is **not switching to the wrong human**.

---

Next, I can break down `person_track_manager.py` by **section and function**, for example only the selected-target logic, only the fusion logic, or only the IMM tracking part.

DETAILS：
Yes — I’ll break `person_track_manager.py` down by **section and function**, so it is easier to understand.

This file is the **main brain** of your whole person-following stack.

A very good way to read it is:

**Input layer**

* `cam_cb`
* `lidar_cb`
* `rgb_cb`
* `select_cb`

**Main loop**

* `timer_cb`

**Fusion layer**

* `_build_fused_measurements`
* `_build_selected_lidar_bridge_measurement`

**Tracking layer**

* `_spawn_track`
* `_predict_track`
* `_update_track`

**Association layer**

* `_associate`
* `_associate_selected_first`
* `_associate_selected_reacquire`
* `_associate_stage`

**Selected-target protection layer**

* `_selected_*`
* `_handle_selected_no_match`
* `_enter_locked_*`
* `_update_crossover_state`

**Confidence / publish layer**

* `_selected_*confidence`
* `_make_tracks_state`
* `_make_selected_target_msg`
* `_publish_outputs`

---

# 1. File opening section: enums and data structures

---

## `TrackState`

This is the normal lifecycle of any track.

* `TENTATIVE`: new track, not trusted yet
* `CONFIRMED`: stable track
* `OCCLUDED`: recently missing, but still kept
* `LOST`: missing longer
* `DELETED`: remove from system

This is standard multi-object tracking logic.

---

## `TargetLockState`

This is only for the **selected** target.

* `NONE`
* `LOCKED`
* `TEMP_OCCLUDED`
* `TEMP_LOST`
* `REACQUIRE`

This is important because selected-target following needs its own logic, different from normal track survival.

A normal track can be alive, but selected-target logic may still say:

* still locked
* temporarily occluded
* currently reacquiring

---

## `FusedMeasurement`

This is the tracker’s input unit.

One `FusedMeasurement` may come from:

* camera only
* camera + LiDAR
* LiDAR only

Important fields:

### Geometry

* `pos_xy`
* `camera_xy`
* `lidar_xy`
* `range_m`
* `bearing_rad`

### Detection quality

* `camera_confidence`
* `cam_lidar_error`
* `depth_valid_ratio`
* `bbox_area_px`

### Appearance

* `embedding`
* torso/upper/lower histograms
* face embedding

### Facing

* `facing01`
* `head_rpy_deg`

### Extra continuity cues

* `continuity_only`
* image center velocity
* bbox
* center UV

So this is a very rich measurement object.

---

## `Track`

This is one tracked person.

Important state inside a track:

### Motion

* `x = [px, py, vx, vy]`
* `P` covariance

### Lifecycle

* `state`
* `hits`
* `misses`
* `age`
* `last_update_sec`

### Identity

* `last_embedding`
* `ema_embedding`
* embedding gallery
* torso/upper/lower histogram refs

### Facing

* `last_facing01`
* `last_head_rpy_deg`

### IMM

* `imm_xs`
* `imm_Ps`
* `imm_mu`

### Confidence

* `track_score`
* `appearance_confidence`
* `motion_confidence`

So each track is both:

* motion memory
* appearance memory

---

# 2. `ReIDExtractor`

This helper class loads and runs the ONNX body ReID model.

---

## `__init__`

Purpose:

* open the ONNX model
* configure runtime providers
* discover input name
* remember input size

So this is just model setup.

---

## `enabled`

Returns whether ONNX runtime session loaded correctly.

If False, the whole tracker still works, just without learned body embeddings.

---

## `_extract_from_crop`

Input:

* cropped BGR image of person

Process:

* resize
* BGR to RGB
* normalize
* `CHW` layout
* ONNX inference
* L2 normalize output

Output:

* 1D embedding vector

This is the actual ReID feature extraction.

---

## `extract_from_bbox`

Input:

* full RGB frame
* bbox coords

Process:

* crop bbox
* validate crop size
* call `_extract_from_crop`

Output:

* body embedding or `None`

---

## `extract`

Input:

* image + center point

This is a fallback crop strategy around a center point, used when exact bbox crop path is not preferred.

---

# 3. `PersonTrackManager.__init__()`

This is where almost all runtime configuration happens.

This section is huge, but its job is simple:

* declare parameters
* create subscribers/publishers
* load optional ReID model
* initialize all tracker memory
* start timers

---

## Main groups of parameters

### Sensor topics

* camera detections
* LiDAR candidates
* RGB image
* select target topic

### Frames

* `track_frame`

### Fusion

* camera-LiDAR gate
* lidar-only rules
* message freshness

### Track lifecycle

* confirm hits
* miss limits
* delete limits

### Selected target protection

* hard hold
* reacquire window
* switch confirmation
* post-relock protection
* contradiction thresholds
* ambiguity thresholds

### Motion model

* process noise
* measurement noise
* IMM settings

### Appearance / ReID

* providers
* bbox area thresholds
* yaw/pitch thresholds
* gallery size
* EMA alpha

So `__init__()` defines the personality of the tracker.

---

## Subscribers created

### `cam_cb`

Receives `CamDetections`

### `lidar_cb`

Receives `PoseArray`

### `rgb_cb`

Receives RGB image

### `select_cb`

Receives manual selected track ID

---

## Publishers created

These are the node outputs:

* tracks markers
* tracks pose array
* full tracks state
* selected target
* selected target marker
* debug string
* target lock debug
* facing bool

---

## Internal memory initialized

Very important fields include:

### General tracking

* `self.tracks`
* `self.next_track_id`

### Latest raw inputs

* latest cam detections
* latest lidar candidates
* latest RGB frame

### Selected target state

* `selected_track_id`
* `target_lock_state`
* `selected_last_seen_sec`
* `selected_last_clean_seen_sec`
* `selected_relock_cooldown_until_sec`

### Selected identity memory

* selected embedding ref
* stable anchor embedding
* selected hist refs
* selected face ref
* selected identity gallery

### Selected history

* range history
* bearing history
* bbox width/height history
* image center history

This memory is what makes selected-person following robust.

---

# 4. Input callbacks

These are simple, but important.

---

## `cam_cb(self, msg)`

Stores latest camera detections message.

It does not process immediately.
It only updates cached input for the main timer loop.

Why:
the node wants one synchronized tracker cycle, not random callback-driven updates.

---

## `lidar_cb(self, msg)`

Stores latest LiDAR candidate message.

Same idea: cache first, process later in `timer_cb()`.

---

## `rgb_cb(self, msg)`

Stores latest RGB image so the ReID extractor can crop person embeddings later if needed.

This is not the camera detection itself.
It is only the image buffer for ReID extraction.

---

## `select_cb(self, msg)`

This handles external person selection.

Input:

* `Int32` track ID

Process:

* if value <= 0, clear selected target
* else set `selected_track_id`
* reset relevant selected-target state
* update reasons/debug

So this allows manual selection from another node.

---

# 5. Heartbeat and logging helpers

---

## `heartbeat_cb`

Prints periodic status summary:

* number of tracks
* selected target info
* lock state
* whether ReID is enabled

This is mainly runtime observability.

---

## `_log_onnx_runtime`

Logs ONNX backend details.

Used when ReID backend loads.

---

## `_maybe_log_timing`

Small helper for timing logs if model timing/debug is enabled.

---

# 6. Main loop: `timer_cb()`

This is the most important function in the file.

It runs every timer period and performs a full tracker update.

The order matters a lot.

---

## Step 1: get current time

Used for freshness, age, prediction, and state transitions.

---

## Step 2: build fused measurements

Calls:

* `_build_fused_measurements(now_sec)`

This produces the list of usable sensor observations for this cycle.

---

## Step 3: update crossover state

Calls:

* `_update_crossover_state(now_sec, fused)`

This checks whether another person is dangerously close to the selected target in appearance/geometry space.

---

## Step 4: predict all tracks forward

Calls:

* `_predict_track(tr, dt)`

for each live track.

This moves tracks forward in time before matching new measurements.

---

## Step 5: associate measurements to tracks

Calls:

* `_associate(fused)`

This produces:

* matched pairs
* unmatched tracks
* unmatched measurements

---

## Step 6: update matched tracks

For each matched pair:

* `_update_track(...)`

This is where state, confidence, appearance, facing, and identity memory get refreshed.

---

## Step 7: handle unmatched tracks

Unmatched tracks get miss counts and may move from:

* confirmed → occluded → lost → deleted

Selected target logic also reacts here.

---

## Step 8: spawn new tracks

Unmatched measurements may create new tentative tracks:

* `_spawn_track(meas, now_sec)`

---

## Step 9: auto-select if needed

Calls:

* `_auto_select_if_needed(now_sec)`

Useful when user wants automatic nearest-person start.

---

## Step 10: publish outputs

Calls:

* `_publish_outputs(now_sec)`

This updates all downstream nodes.

So `timer_cb()` is the central coordinator.

---

# 7. Fusion layer

---

## `_build_fused_measurements(now_sec)`

This is where raw camera and LiDAR inputs become tracker-ready measurements.

It is one of the most important functions.

### Inputs

* latest camera detections
* latest LiDAR candidate PoseArray
* latest RGB frame for optional ReID

### Output

* list of `FusedMeasurement`

---

### Part A: freshness filtering

If camera or LiDAR messages are too old:

* ignore them

This prevents stale sensor data from poisoning the tracker.

---

### Part B: parse LiDAR candidates

LiDAR candidate positions are extracted from `PoseArray`.

Important:
in your design, `pose.position.z` carries person-likeness score from `follow_person.py`.

So LiDAR input includes position + lightweight confidence.

---

### Part C: parse camera detections

For each camera detection, it extracts:

* XY position
* confidence
* depth ratio
* bbox area
* bbox coords
* facing/head pose
* histograms
* quality scores
* embeddings if already present

If needed, it may call `_extract_reid_embedding()` using latest RGB frame.

So camera detections become richer fused candidates.

---

### Part D: camera-LiDAR matching

It tries to pair camera detections with LiDAR candidates within `cam_lidar_gate_m`.

If both exist:

* source becomes `cam_lidar`
* final position usually prefers LiDAR geometry
* camera attributes remain attached

If no LiDAR partner:

* can still create `camera_only` measurement if allowed

This is the basic fusion step.

---

### Part E: lidar-only handling

Plain lidar-only measurements are treated carefully.

Usually only allowed when:

* explicitly enabled
* non-crowd mode
* selected-target bridge case
* or special selected-target fallback conditions

So the file does not over-trust LiDAR-only candidates.

---

## `_extract_reid_embedding(...)`

This crops the person bbox from the latest RGB frame and runs the ReID extractor.

It is used when:

* camera detection did not already provide embedding
* bbox quality is sufficient
* ReID model is enabled

So this plugs learned appearance into the fusion pipeline.

---

## `_det_array(det, field_name)`

Small helper that safely converts message arrays into NumPy arrays.

Used for fields like:

* histogram arrays
* embeddings
* face embeddings

---

## `_build_selected_lidar_bridge_measurement(...)`

This is one of the smartest functions in the file.

It helps the selected person survive short camera dropouts.

### Purpose

When camera is stale or weak, try to build a temporary `lidar_only` measurement **only for the selected target**.

### Conditions checked

* selected target must exist
* selected track must still be valid
* current lock state must allow bridge
* LiDAR candidate must be near predicted selected position
* bearing must be close
* motion jump must not be too large
* ambiguity against other candidates must be low
* candidate may need repeated support across frames

If passed:

* return a `FusedMeasurement(source='lidar_only')`

If not:

* return `None`

This function is extremely useful in real-world crowded following.

---

## `_bridge_log_throttled`

Just prevents bridge-related logs from spamming every cycle.

---

# 8. Track creation and motion model

---

## `_spawn_track(meas, now_sec)`

Creates a new track from one unmatched measurement.

### Initializes

* state vector from measurement position
* zero velocity
* covariance
* tentative state
* scores/confidences
* appearance memory if available
* facing info if available
* IMM state

So this is the birth of a new person track.

---

## `_init_track_imm(tr)`

Initializes IMM model state arrays for the track.

This means each track internally carries several motion-model hypotheses.

---

## `_model_F_Q(model_idx, dt)`

Returns motion model matrices:

* state transition `F`
* process noise `Q`

for each IMM mode.

The modes roughly correspond to:

* normal moving
* turning
* stopping/slow

So human motion is modeled more realistically.

---

## `_merge_models(xs, Ps, mu)`

Merges multiple IMM state estimates into one combined state and covariance.

This gives the final public track state.

---

## `_predict_track(tr, dt)`

This is the prediction step before association.

### Process

* compute IMM mixing
* propagate each motion model
* update mode probabilities
* merge results back

### Purpose

Advance track state into the current time before matching measurements.

This is why the tracker can survive brief dropouts.

---

# 9. Track update

---

## `_update_track(...)`

This is the measurement update after a match.

It is a very important function.

### Motion update

* Kalman/IMM correction using measurement position

### Track lifecycle update

* increase hits
* reset misses
* promote tentative → confirmed when conditions met

### Source update

* remember whether this update came from camera, fused, or lidar-only

### Facing update

* store `last_facing01`
* store head roll/pitch/yaw if available

### Appearance update

* embeddings
* EMA embedding
* gallery entries
* histogram blending
* face embedding refs if appropriate

### Confidence update

* track score
* appearance confidence
* motion confidence

### Selected-target memory update

If this is the selected track:

* maybe update selected history
* maybe update selected identity gallery
* maybe refresh trusted evidence timestamps

So `_update_track()` is where a matched measurement turns into a stronger remembered identity.

---

# 10. Association layer

---

## `_associate(fused)`

This is the top-level association manager.

It does not directly match everything at once.

It runs in stages.

### Stage order

1. selected target first
2. selected reacquire if needed
3. strict normal association
4. relaxed normal association

This design is very intentional.

The selected person gets priority over generic MOT.

---

## `_associate_selected_first(...)`

This tries to match the selected track before anyone else.

### Process

* check selected track validity
* gather candidate measurements
* apply hard gates
* compute selected-match score
* compare best vs second-best
* reject front occluders
* decide whether match is:

  * clean
  * acceptable
  * ambiguous
  * reject

If accepted:

* selected track is matched first
* that measurement is removed from general pool

This is one of the biggest reasons your tracker avoids wrong-person switching.

---

## `_associate_selected_reacquire(...)`

Used when selected target is not normally matched and the system is in reacquire mode.

This is stricter than normal association.

It uses:

* selected target history
* appearance refs
* bbox history
* center history
* contradiction checks
* cross-track relock rules

So reacquire is not “closest measurement wins.”
It is “which measurement best fits the remembered selected person.”

---

## `_associate_stage(...)`

This is the generic engine for normal association.

It handles:

* candidate gating
* cost matrix building
* Hungarian assignment or fallback greedy matching

It uses the following costs:

* geometry cost
* appearance cost
* innovation/motion cost

So normal track association still benefits from both motion and identity.

---

# 11. Cost and gate helpers

---

## `_appearance_cost(tr, embedding)`

Compares measurement embedding to the track’s known identity memory.

Usually based on cosine distance against:

* EMA embedding
* embedding gallery

Lower cost = more similar appearance.

---

## `_measurement_noise_for(meas)`

Returns measurement noise based on source quality.

For example:

* camera+LiDAR may be lower noise
* camera-only weak depth may be higher noise
* lidar-only may be different noise

So measurement uncertainty is source-dependent.

---

## `_mahalanobis_distance(tr, meas)`

Computes statistical gating distance between predicted track state and measurement.

Useful for rejecting unlikely matches.

---

## `_innovation_cost(tr, meas)`

Extra motion consistency cost from innovation residual.

This helps reward measurements that fit expected movement.

---

# 12. Selected-target validity and protection helpers

These functions define whether the selected target still deserves protection.

---

## `_selected_track_is_valid(now_sec)`

Checks whether selected track exists and is not too stale/deleted.

---

## `_selected_identity_protected(now_sec)`

Returns whether selected identity protection window is still active.

This matters during occlusion and reacquire.

---

## `_selected_trusted_evidence_age(now_sec)`

Age since last strong trustworthy selected-target evidence.

This is more important than raw last seen time.

---

## `_selected_cross_track_reacquire_allowed(now_sec)`

Decides whether switching to a different track ID is even allowed during reacquire.

This is where the file becomes very conservative in crowds.

---

## `_selected_hard_gate_pass(tr, meas)`

Strict selected-target gate using:

* position
* bearing
* image center corridor
* maybe bbox consistency

Only measurements that pass this can compete strongly for selected target.

---

## `_selected_reacquire_gate_pass(meas)`

Gate used during reacquire phase.

Usually based on selected history and less immediate trust.

---

# 13. Selected-target scoring and occlusion logic

---

## `_selected_match_score(tr, meas)`

This is one of the key functions.

It computes a score for:
“how much does this measurement look like my selected person?”

Typical ingredients:

* appearance similarity
* range consistency
* bearing consistency
* bbox size consistency
* motion consistency
* penalties for weak depth / lidar-only / continuity-only

It returns:

* total score
* score breakdown dictionary

So this is the selected-target decision score.

---

## `_is_front_occluder(tr, meas)`

Checks whether a candidate is probably just a blocker in front of the selected person.

This prevents a closer wrong object/human from stealing the selected lock.

Very useful when somebody walks between robot and target.

---

## `_accept_selected_match(...)`

Commits a selected match:

* update selected timestamps
* record clean vs acceptable evidence
* change lock state if needed

---

## `_handle_selected_no_match(now_sec, hold=False)`

Runs when no acceptable selected match is found.

It drives state transitions such as:

* `LOCKED` → `TEMP_OCCLUDED`
* `TEMP_OCCLUDED` → `TEMP_LOST`
* `TEMP_LOST` → `REACQUIRE`

It also freezes or protects selected identity memory when appropriate.

---

## `_enter_locked_from_selected_match(...)`

Used when selected target gets a proper direct match.

Moves/keeps state in `LOCKED`.

---

## `_enter_locked_from_reacquire(...)`

Used when selected target is successfully reacquired.

Also starts post-relock protection windows.

---

## `_post_relock_protection_active(now_sec)`

Returns whether the tracker is in the special period right after reacquire where switching should be harder.

---

# 14. Crossover and selected history

---

## `_update_crossover_state(now_sec, fused)`

This checks whether a dangerous crossing situation is happening.

Typical condition:
another candidate is extremely close to the selected person in geometry or image space.

If yes:

* set crossover active
* later make reacquire and switching stricter

This is very important for crowded scenes.

---

## `_update_selected_history(meas)`

Stores selected-person history:

* range
* bearing
* bbox width/height
* image center

This history is used later for reacquire scoring and contradiction checks.

---

## `_selected_reason(tag)`

Stores human-readable reason/debug code for current selected-target decision.

---

## `_reset_selected_lock_state()`

Clears selected target lock mode and related counters.

---

## `_clear_selected_uncertainty_memory(clear_clean=False)`

Resets ambiguity/contradiction-related memory.

---

## `_clear_selected_histories()`

Clears selected range/bearing/bbox/center histories.

Usually used when selected person is fully reset.

---

## `_is_selected_self_measurement(...)`

Checks whether a measurement is essentially the selected track’s own current measurement rather than another dangerous nearby candidate.

Useful inside crossover logic.

---

# 15. Histogram and history helpers

---

## `_blend_hist_ref(...)`

Blends new histogram into stored histogram reference.

This smooths appearance memory over time.

---

## `_history_ready()`

Returns whether there is enough selected history to use history-based gates and scores reliably.

---

## `_hist_mean(...)`

Simple average helper.

---

## `_hist_angle_mean(...)`

Angle-aware average helper for bearing history.

Very important because angles wrap around.

---

## `_bbox_iou(a, b)`

Computes IoU between two boxes.

Used when comparing bbox consistency over time.

---

# 16. Publishability and suppression logic

---

## `_selected_target_publishable(now_sec, tr)`

Checks whether selected target is safe enough to publish outward.

A selected track may exist internally but still be too uncertain to expose as usable follow target.

---

## `_suppression_code_for_track(now_sec, tr)`

Returns an integer code describing why a track/target is suppressed.

Useful for debugging downstream behavior.

---

## `_camera_visible_for_track(now_sec, tr)`

Returns whether the track still has recent camera evidence.

This is different from just “track exists.”

---

## `_lidar_support_for_track(tr)`

Checks whether LiDAR contributed support to this track.

---

## `_decision_code_for_reason(tag)`

Maps human-readable selected-target reason tags into numeric debug codes.

---

## `_selected_prediction_allowed(now_sec, tr, max_age_sec)`

Checks whether predicted selected target can still be treated as usable for a short time even without new measurement.

---

## `_selected_is_predicted(now_sec, tr)`

Returns whether selected target is currently being maintained mainly by prediction.

---

# 17. Confidence functions

These functions are important because downstream behavior uses confidence.

---

## `_selected_lock_confidence(now_sec, tr)`

Overall confidence that:

* this is the selected person
* lock is still valid

It includes penalties for:

* staleness
* contradiction
* predicted-only state
* reacquire state

---

## `_selected_identity_confidence(tr)`

Confidence that this track’s identity matches the selected person’s remembered identity.

Uses:

* embedding similarity
* stable anchor similarity
* gallery similarity
* maybe histogram similarity

---

## `_selected_motion_confidence(now_sec, tr)`

Confidence from motion consistency and freshness.

---

## `_selected_contradictory_candidate_active(now_sec, tr)`

Checks whether a contradictory rival candidate is currently active near the selected target.

This is important because even if one candidate looks okay, contradiction may lower trust.

---

# 18. Generic math helpers

---

## `_track_state_to_msg`

Maps internal `TrackState` enum to message constant.

---

## `_lock_state_to_msg`

Maps internal `TargetLockState` enum to message constant.

---

## `_clamp01`

Clamps to `[0,1]`.

---

## `_cosine_similarity`

Cosine similarity helper for embeddings.

---

## `_hist_similarity`

Similarity helper for histograms.

---

# 19. Reacquire scoring and learning rules

---

## `_score_reacquire_candidate(meas)`

This is the main scoring function during selected-target reacquire.

It compares the measurement against stored selected identity/history.

Typical ingredients:

* body embedding similarity
* stable anchor similarity
* gallery similarity
* histogram similarity
* range consistency
* bearing consistency
* bbox consistency
* center consistency

This is one of the most important functions for not switching to wrong person after occlusion.

---

## `_find_track_for_measurement(meas, exclude_tid=None)`

Finds which current track best corresponds to a measurement.

Useful in reacquire logic.

---

## `_selected_learning_allowed(meas)`

Decides whether this measurement is good enough to update selected identity memory.

This prevents learning from:

* weak depth
* bad bbox
* poor head pose
* continuity-only measurement
* uncertain cases

Very important to avoid corrupting identity memory.

---

## `_update_selected_identity_gallery(embedding)`

Adds a new embedding to selected identity gallery, respecting max gallery size.

---

# 20. Auto-selection and publishing

---

## `_auto_select_if_needed(now_sec)`

If enabled and no target is selected, automatically choose nearest confirmed track.

This is useful for start-follow workflows.

---

## `_publish_outputs(now_sec)`

Top-level output publisher.

It calls all the message/marker builders and publishes:

* tracks
* tracks poses
* full tracks state
* selected target
* markers
* debug topics
* facing bool

This is the last step of each timer cycle.

---

# 21. Output builders

---

## `_track_facing01(tr)`

Returns selected track’s facing score.

---

## `_track_facing_robot(tr)`

Converts facing score to boolean.

Usually `facing01 >= 0.5`.

---

## `_make_target_lock_debug(now_sec)`

Builds the detailed target-lock debug message.

This is extremely useful for tuning selected-target logic.

---

## `_make_tracks_pose_array()`

Builds simple `PoseArray` of current visible tracks.

---

## `_make_tracks_state(now_sec)`

Builds the rich `TrackedPoseArray`.

This is one of the most important outputs because it includes:

* track id
* pose
* state
* confidence fields
* selected flag
* publishable flag
* predicted flag
* reacquire flag
* contradiction/crossover info
* camera visible
* LiDAR support
* facing info

This is what downstream nodes really consume.

---

## `_make_selected_target_msg(now_sec, tr)`

Builds the final selected target message.

Includes:

* pose
* confidence fields
* predicted/reacquire flags
* facing info
* publishability
* suppression code

---

## `_make_tracks_markers()`

RViz markers for all tracks.

---

## `_make_target_marker(now_sec)`

RViz marker for selected target.

---

## `_make_debug_markers()`

Extra debug markers, often for selected target, track states, or special status visualization.

---

# 22. Final helpers

---

## `_iter_visible_tracks()`

Returns tracks that are not deleted and are considered visible enough for output.

---

## `_track_color(tr)`

Chooses marker color based on track state / selected state.

---

## `_lifetime_msg(sec_f)`

Builds marker lifetime duration.

---

## `_now_sec`, `_stamp_to_sec`, `_time_msg_to_sec`

Time conversion helpers.

---

## `_angle_diff(a, b)`

Angle wrap-safe difference.

Very important for bearing comparisons.

---

# 23. Best way to understand the full flow

Here is the **full mental flow** of the file:

### Step A: read latest sensors

Camera detections, LiDAR candidates, RGB image, selected ID

### Step B: build usable measurements

`_build_fused_measurements`

### Step C: protect selected target first

`_associate_selected_first` or `_associate_selected_reacquire`

### Step D: do normal tracking

`_associate_stage`, `_predict_track`, `_update_track`, `_spawn_track`

### Step E: manage selected lock state

`_handle_selected_no_match`, `_enter_locked_*`, `_update_crossover_state`

### Step F: compute confidence and visibility

`_selected_*confidence`, `_selected_target_publishable`

### Step G: publish rich outputs

`_make_tracks_state`, `_make_selected_target_msg`, markers

That is the real architecture of `person_track_manager.py`.

---

# 24. The most important functions to master first

If you want to truly understand this file, focus on these 10 first:

1. `timer_cb`
2. `_build_fused_measurements`
3. `_build_selected_lidar_bridge_measurement`
4. `_predict_track`
5. `_update_track`
6. `_associate`
7. `_associate_selected_first`
8. `_associate_selected_reacquire`
9. `_selected_match_score`
10. `_score_reacquire_candidate`

If you understand these, you understand most of the file.

---

# 25. Super short plain-English summary

This file watches camera and LiDAR observations, turns them into tracked people, remembers what the chosen person looks like, becomes very careful when someone crosses in front or occlusion happens, and tries hard to relock the same person instead of switching to the wrong one.

So the single best sentence is:

**`person_track_manager.py` is the fusion, tracking, identity-memory, and selected-target-protection engine of your person-following system.**

I can next break down just the **selected-target logic path** from `_associate_selected_first()` to `_score_reacquire_candidate()` in even more detail.

——————————————————————————————————————————————target_lock_manager.py————————————————————————————————————————————————————————————————————————————
Yes — `target_lock_manager.py` is much smaller than `person_track_manager.py`, but it is still important.

This file is **not** the tracker.
It is a **selected-target stabilizer and republisher**.

Its job is:

* listen to the full tracked-person state from `person_track_manager.py`
* find which track is currently marked `selected=True`
* avoid unstable instant relock switching
* build one final `/person_target_locked` message
* publish `/person_target_facing_robot`

So the easiest way to think about it is:

**`person_track_manager.py` = decides who the selected person is**
**`target_lock_manager.py` = makes that selected output cleaner and more stable for downstream nodes**

---

# 1. What file it really is

There are 2 related files:

* `person_following/person_following/nodes/target_lock_manager.py` → real logic
* `person_following/scripts/target_lock_manager.py` → wrapper script

The real code is in:

* `nodes/target_lock_manager.py`

The script version is just for ROS execution.

---

# 2. What this node subscribes to

It subscribes to 2 topics:

## A. `tracks_state_topic`

Default:

* `/person_tracks_state`

Type:

* `TrackedPoseArray`

This comes from `person_track_manager.py`.

This is the main input.

It contains all tracks, including:

* track IDs
* pose
* selected flag
* confidence fields
* facing info
* predicted/reacquire flags
* publishable flag
* suppression code

---

## B. `target_lock_debug_topic`

Default:

* `/person_target_lock_debug`

Type:

* `TargetLockDebug`

This is debug information from the tracker.

In this file, it is mainly used for:

* `contradictory_candidate`

So this node uses tracker debug info to lower trust when the selected target is contradicted by another candidate.

---

# 3. What this node publishes

It publishes 2 things:

## A. `target_topic`

Default:

* `/person_target_locked`

Type:

* `TrackedPose`

This is the final selected target message for downstream nodes like:

* `follow_behavior_manager.py`
* `nav_follow_interface.py`

---

## B. `target_facing_topic`

Default:

* `/person_target_facing_robot`

Type:

* `std_msgs/Bool`

This is just a simple bool:

* `True` if selected target is valid and facing robot
* `False` otherwise

---

# 4. Main purpose of this file

This node does 3 main jobs:

## Job 1: cache all current tracks

It stores latest tracks in:

* `_tracks_by_id`

## Job 2: stabilize relock changes

If the selected target suddenly changes from one ID to another, it does **not always accept it instantly**.

It uses:

* `relock_stability_cycles`
* `relock_cooldown_sec`

to avoid rapid flip-flopping.

## Job 3: build a clean final target message

It copies the selected track and recomputes:

* geometry confidence
* visibility confidence
* freshness confidence
* combined confidence
* final `publishable`

So this file is basically:

**selected target filter + confidence wrapper + stable republisher**

---

# 5. Parameters in detail

The file declares these parameters:

## Topic names

* `tracks_state_topic`
* `target_lock_debug_topic`
* `target_topic`
* `target_facing_topic`

These are just topic routing.

---

## Frames

* `track_frame`
* `global_frame`
* `base_frame`

Important:
in the current code, only `track_frame` is actually used for the invalid fallback message.
`global_frame` and `base_frame` are declared but not really used in the logic.

So these two are more like unused placeholders right now.

---

## Timeout/confidence parameters

* `target_timeout_sec` default `1.0`
* `publishable_timeout_sec` default `0.70`
* `combined_publishable_min` default `0.40`

These are very important.

### `target_timeout_sec`

If the whole tracks input becomes too old, the node stops trusting it and publishes an invalid target.

### `publishable_timeout_sec`

Even if target is still “valid”, it may stop being publishable if it is too stale.

### `combined_publishable_min`

Minimum combined confidence for target to count as publishable.

---

## Relock stabilization parameters

* `relock_stability_cycles` default `1`
* `relock_cooldown_sec` default `0.3`

These control how it accepts changes in selected track ID.

---

# 6. Internal state variables

The node keeps these important internal states:

* `_last_tracks_rx_sec`
* `_tracks_by_id`
* `_selected_track`
* `_selected_track_id`
* `_candidate_track_id`
* `_candidate_cycles`
* `_last_debug`
* `_last_lock_change_sec`

What they mean:

## `_selected_track_id`

This is the currently accepted selected target ID inside this node.

## `_candidate_track_id`

This is a possible new selected track ID that is being tested before acceptance.

## `_candidate_cycles`

How many consecutive callbacks that candidate has remained selected.

## `_last_lock_change_sec`

When the last accepted lock switch happened.

So this node has a small internal “debounce memory” for lock changes.

---

# 7. `__init__()` logic

The constructor does these things:

1. create node named `target_lock_manager`
2. declare parameters
3. read parameters into member variables
4. initialize internal state
5. subscribe to track state and debug topics
6. create publishers
7. start timer at `0.1 sec` period
8. log startup message

Important detail:
the timer is 10 Hz.

So this node republishes target status at 10 Hz, not every incoming callback instantly.

That makes it more regular for downstream behavior.

---

# 8. `_now_sec()`

This is just a helper:

* current ROS clock time in seconds

Used for:

* staleness
* cooldown
* timeout checks

---

# 9. `_tracks_state_cb()` — the most important callback

This is the main input callback.

It receives the full `TrackedPoseArray`.

Let’s go step by step.

---

## Step 1: record latest receive time

```python
self._last_tracks_rx_sec = now
```

This lets the timer later decide if track data became stale.

---

## Step 2: cache tracks by ID

```python
self._tracks_by_id = {track.track_id: track ...}
```

Only tracks with `track_id > 0` are kept.

So this builds an internal dictionary of all current tracks.

---

## Step 3: find the selected track

It loops through `msg.tracks` and picks the first one where:

* `track.selected == True`

So this node trusts the upstream tracker’s selected flag.

It does **not** decide selection itself.

---

## Step 4: if no selected track exists

If no selected track is found:

* candidate state is cleared
* accepted selected state is cleared
* `_selected_track = None`
* `_selected_track_id = 0`

So the node fully resets its selected output.

This means downstream nodes will get invalid target afterward.

---

## Step 5: if selected ID is same as already accepted ID

If the newly received selected ID matches the node’s already accepted selected ID:

* update `_selected_track`
* clear candidate test state
* return

So stable selected target just keeps refreshing normally.

---

## Step 6: relock cooldown check

If a different selected ID appears but:

```python
(now - self._last_lock_change_sec) < relock_cooldown_sec
```

then the node ignores that change for now.

This prevents:

* very fast switch back-and-forth
* relock thrashing

So even if upstream tracker changes selected ID, this node can temporarily refuse to accept it.

---

## Step 7: candidate ID tracking

If new selected ID is different from current accepted ID and cooldown allows it:

### if new ID is not same as current candidate

* set `_candidate_track_id = selected_id`
* set `_candidate_cycles = 1`
* return

So this starts testing the new selected candidate.

### if new ID matches candidate again

* increment `_candidate_cycles`

If:

```python
_candidate_cycles >= relock_stability_cycles
```

then:

* accept new selected ID
* move it into `_selected_track_id`
* store `_selected_track`
* clear candidate state
* update `_last_lock_change_sec`

So this is the relock debounce mechanism.

---

# 10. Meaning of relock stabilization

This is one of the most important behaviors in this file.

The idea is:

**Do not instantly believe every selected-ID change from upstream.**

Instead:

* require stability for some cycles
* enforce cooldown between accepted switches

This is useful because in crowded scenes the upstream tracker may briefly toggle selected ID.

This file tries to smooth that.

---

# 11. `_debug_cb()`

This one is simple.

It just stores the latest debug message:

* `_last_debug = msg`

That debug message is later used in:

* `_contradictory_candidate()`

So this node uses only a small part of tracker debug.

---

# 12. `_timer_cb()` — the main publish loop

This function runs every 0.1 s.

It is the output stage of the node.

---

## Step 1: check if track state input is stale

If:

* no track state has ever arrived, or
* latest track state is older than `target_timeout_sec`

then:

* publish invalid target
* publish facing false
* return

So the node refuses to keep publishing stale target forever.

---

## Step 2: check whether accepted selected track exists

If:

* `_selected_track is None`, or
* `_selected_track_id <= 0`

then:

* publish invalid target
* publish facing false
* return

So no accepted selected target means output becomes invalid.

---

## Step 3: build final selected target

If everything is okay:

* call `_build_selected_target(now, selected)`

Then publish that `TrackedPose`.

---

## Step 4: publish facing bool

It publishes:

```python
facing.data = bool(msg.valid and msg.facing_robot)
```

So facing bool is only true if:

* target is valid
* target itself says `facing_robot = True`

This is important:
this file does **not** compute facing.
It only forwards the facing result already stored in the selected track.

---

# 13. `_build_selected_target()` — the core function

This is the main logic function of the file.

It takes one selected `TrackedPose` from upstream and builds the final public target message.

This function is basically:

* copy
* recompute confidence
* decide publishability
* sanitize invalid states

Let’s break it down.

---

## Step 1: create new `TrackedPose`

It copies many fields directly from the selected track:

* header
* track_id
* pose
* track_state
* track_score
* conf_app
* conf_motion
* lock_confidence
* identity_confidence
* motion_confidence
* staleness_sec
* predicted
* reacquire_active
* crossover_active
* facing01
* facing_robot
* lock_state
* suppression_code
* camera_visible
* lidar_support

And it forces:

* `selected = True`
* `valid = True` initially

So the output begins as a copy of the selected track.

---

## Step 2: contradictory candidate flag

It sets:

```python
msg.contradictory_candidate = self._contradictory_candidate()
```

That means the final target can inherit a contradiction warning from tracker debug.

This matters later for `publishable`.

---

## Step 3: compute derived confidence fields

This file recomputes:

* `detection_confidence`
* `geometry_confidence`
* `visibility_confidence`
* `freshness_confidence`
* `combined_confidence`

These are the core value-added outputs of this file.

---

# 14. Confidence functions in detail

---

## `_geometry_confidence(track)`

Logic:

### if both LiDAR and camera support exist

→ `1.0`

### if camera visible only

→ `0.75`

### if LiDAR support only

→ `0.60`

### if predicted only

→ `0.25`

### otherwise

→ `0.1`

Meaning:
this score reflects how geometrically grounded the target is.

Best case:

* camera + LiDAR together

Weak case:

* prediction only

---

## `_visibility_confidence(track)`

Logic:

### camera visible and not predicted

→ `1.0`

### camera visible but predicted

→ `0.7`

### predicted without camera visible

→ `0.25`

### otherwise

→ `0.0`

Meaning:
this score reflects how directly visible the target is right now.

---

## `_freshness_confidence(staleness_sec)`

Logic:

* if freshness is perfect or negative → `1.0`
* otherwise linearly decreases with staleness

Formula:

```python
1.0 - staleness_sec / target_timeout_sec
```

clamped to `[0, 1]`

So if staleness reaches `target_timeout_sec`, freshness confidence goes to 0.

---

## `detection_confidence`

This one is simply:

```python
msg.detection_confidence = clamp01(track_score)
```

So it maps tracker’s score into a normalized detection confidence.

---

## `_combined_confidence(track)`

This is the most important confidence output.

It averages these 7 terms:

* `lock_confidence`
* `identity_confidence`
* `motion_confidence`
* `detection_confidence`
* `geometry_confidence`
* `visibility_confidence`
* `freshness_confidence`

Then if contradiction is active:

* subtract penalty `0.20`

Finally clamp to `[0,1]`.

So combined confidence is a **broad trust score**, not just one raw detector score.

That is a nice design.

---

# 15. Publishable logic

After confidence is computed, the node sets:

```python
msg.publishable = (
    msg.valid
    and not msg.contradictory_candidate
    and msg.staleness_sec <= publishable_timeout_sec
    and msg.combined_confidence >= combined_publishable_min
)
```

So target is publishable only if:

1. target is valid
2. no contradictory candidate active
3. not too stale
4. combined confidence is high enough

This is one of the most important outcomes of this file.

Because downstream behavior often should trust only:

* valid
* publishable
  targets

---

# 16. Valid vs publishable

This distinction is important.

## `valid`

Means:

* there is a selected target object
* data is not fundamentally broken

## `publishable`

Means:

* safe enough to expose as usable follow target

So something can be:

* `valid=True`
* `publishable=False`

Example:

* selected target still exists
* but confidence is weak or stale

That is a very useful distinction.

---

# 17. Extra invalidation rules

After publishable is computed, the function still applies extra safety checks.

---

## Rule 1: very weak and too stale

If:

* `combined_confidence < 0.2`
* and `staleness_sec > target_timeout_sec`

then:

* `msg.valid = False`

So extremely stale and weak target becomes fully invalid, not just non-publishable.

---

## Rule 2: non-finite position

If pose x or y is not finite:

* `valid = False`
* `publishable = False`
* `suppression_code = SUPPRESS_INVALID_TARGET`

This prevents NaN/Inf pose from leaking downstream.

---

## Rule 3: if invalid, convert to standard invalid target msg

If `msg.valid == False`, the function returns:

* `_invalid_target_msg(now)`

So the output gets normalized into one consistent invalid message format.

---

# 18. `_publish_invalid()`

This helper publishes the invalid fallback state.

It does two things:

* publish `_invalid_target_msg(now)`
* publish facing false

So whenever the selected target is missing or input is stale, downstream nodes get a consistent “no valid target” output.

---

# 19. `_invalid_target_msg()`

This creates the standard invalid target message.

Important fields set:

* `track_id = 0`
* `frame_id = track_frame`
* orientation `w = 1.0`
* `track_state = TRACK_DELETED`
* `lock_state = LOCK_NONE`
* `suppression_code = SUPPRESS_INVALID_TARGET`
* `selected = False`
* `valid = False`
* `publishable = False`
* `camera_visible = False`
* `lidar_support = False`
* `facing01 = 0.0`
* `facing_robot = False`
* `predicted = False`
* `reacquire_active = False`
* `contradictory_candidate = False`
* `crossover_active = False`

It also sets `staleness_sec` based on how old the last tracks input is.

So this message is the standardized “no target” object.

That is good because downstream nodes can rely on one clear invalid format.

---

# 20. `_contradictory_candidate()`

This checks:

* if no debug message → False
* else return `last_debug.contradictory_candidate`

So contradiction flag comes from the tracker, not from this node.

This node simply uses it to lower trust.

---

# 21. `_clamp01()`

This just safely clamps values to `[0,1]`.

If value is NaN or Inf:

* returns 0

Used for all confidence values.

---

# 22. What this file does **not** do

This is very important.

`target_lock_manager.py` does **not** do these things:

* it does not track people
* it does not fuse camera and LiDAR
* it does not compute selected target itself
* it does not compute facing itself
* it does not do ReID
* it does not choose among multiple targets based on appearance

All of that is upstream in `person_track_manager.py`.

This file only:

* stabilizes selected ID changes
* recomputes confidence wrappers
* republishes final selected target

---

# 23. How it fits into the full package

The flow is:

## Upstream

`person_track_manager.py`
→ publishes `/person_tracks_state`
→ publishes `/person_target_lock_debug`

## This node

`target_lock_manager.py`
→ chooses stable selected track ID
→ recomputes target confidence
→ publishes `/person_target_locked`
→ publishes `/person_target_facing_robot`

## Downstream

`follow_behavior_manager.py`
uses selected target + facing
to decide:

* follow
* hold
* search
* stop

So this file is the bridge between:

* tracking brain
  and
* behavior brain

---

# 24. Why this node is useful

You may ask:
why not let `person_track_manager.py` publish `/person_target_locked` directly?

Answer:
because this extra node gives you:

## A. relock debounce

Prevents target ID from changing too fast.

## B. clean target wrapper

Creates one stable public target topic.

## C. simpler downstream interface

Behavior nodes only consume one target topic, not full tracker complexity.

## D. extra publishability filtering

Lets you suppress weak selected target even if tracker still holds it internally.

So architecturally, this node is a good separation layer.

---

# 25. Strengths of this implementation

What is good here:

## It is small and focused

This file does one job only.

## It adds output stability

The relock candidate / cooldown logic is useful.

## It separates internal tracking from public target output

That makes the system cleaner.

## It computes understandable confidence layers

* geometry
* visibility
* freshness
* combined

That is nice for debugging and behavior logic.

---

# 26. Limitations / weak points

A few things to know:

## 1. It trusts upstream selected flag

If upstream tracker chooses wrong selected person, this node cannot fix identity logic.
It can only delay switching a bit.

## 2. `relock_stability_cycles` default is only 1

So by default, relock debounce is weak.

It means:

* one stable callback is enough to accept new selected ID
* only cooldown is really protecting you by default

If you want more stability, increasing this value may help.

## 3. `global_frame` and `base_frame` are unused

They are declared but not used in logic.

## 4. It uses only one debug field

`TargetLockDebug` likely has more info, but this file currently only uses:

* contradictory candidate

So its use of debug info is minimal.

---

# 27. Plain-English summary

The easiest way to think about `target_lock_manager.py` is:

It listens to the tracker’s full list of tracks, watches which one is marked as the selected person, refuses to switch too quickly when the selected ID changes, repackages that selected person into one final clean target message with extra confidence scores, and publishes a simple facing boolean for downstream behavior.

So in one sentence:

**`target_lock_manager.py` is the stable selected-target output filter between tracking and follow behavior.`**

---

# 28. Super short logic flow

The file’s runtime logic is basically:

`/person_tracks_state`
→ find selected track
→ debounce lock changes
→ compute confidence wrapper
→ decide valid/publishable
→ publish `/person_target_locked`
→ publish `/person_target_facing_robot`

---

# 29. Best mental model

If `person_track_manager.py` is the **brain deciding “who is my person”**,
then `target_lock_manager.py` is the **gatekeeper deciding “is this selected target stable and trustworthy enough to hand to the behavior system?”**

If you want, next I can explain `follow_behavior_manager.py` in the same style, because that is the next node after `target_lock_manager.py`.


————————————————————————————————————————————————follow_behavior_manager.py——————————————————————————————————————————————————————————————————————————
Yes — next is `follow_behavior_manager.py`.

This file is the **behavior brain** after tracking.

A simple way to place it in the pipeline is:

* `vision_node.py` → sees people
* `follow_person.py` → sees LiDAR person-like blobs
* `person_track_manager.py` → tracks everyone and picks the selected person
* `target_lock_manager.py` → republishes one stable selected target
* **`follow_behavior_manager.py`** → decides what the robot should do about that target
* `nav_follow_interface.py` → turns that decision into actual motion

So this file is **not** about detection or tracking.
It is about **decision making**.

Its main job is:

**selected target + facing info + safety status + follow enable**
→ decide behavior state
→ output motion intent

The easiest summary is:

**`follow_behavior_manager.py` decides whether the robot should follow, hold, rotate, search, wait, stop, or abort.**

---

# 1. What this node subscribes to

It subscribes to four main inputs.

## `target_topic`

Default:

* `/person_target_locked`

Type:

* `TrackedPose`

This is the stabilized selected target from `target_lock_manager.py`.

This is the most important input.

From that target, this node reads things like:

* pose
* validity
* publishable
* staleness
* facing info
* confidence
* predicted flag
* reacquire flag

---

## `target_facing_topic`

Default:

* `/person_target_facing_robot`

Type:

* `std_msgs/Bool`

This is a simple boolean version of facing.

Even though the `TrackedPose` already has facing info, this separate bool makes the behavior logic easier.

---

## `follow_enable_topic`

Default:

* `/person_follow/enable`

Type:

* `std_msgs/Bool`

This is the master on/off switch.

If false, behavior becomes idle.

---

## `safety_topic`

Default:

* `/person_follow/safety_status`

Type:

* `SafetyStatus`

This comes from `safety_supervisor.py`.

This input tells behavior if the system is:

* healthy
* waiting
* inhibited
* faulted
* aborted

So behavior never decides in isolation.
It also respects safety state.

---

# 2. What this node publishes

It publishes two outputs.

## `behavior_status_topic`

Default:

* `/person_follow/behavior_status`

Type:

* `BehaviorStatus`

This tells the rest of the system:

* current behavior state
* reason code
* target availability details
* some confidence/state context

It is the detailed behavior report.

---

## `motion_intent_topic`

Default:

* `/person_follow/motion_intent`

Type:

* `MotionIntent`

This is the action command for the next stage.

Typical motion intents are:

* HOLD
* ROTATE
* FOLLOW_POSE
* SEARCH

So the behavior node does not drive the robot directly.
It publishes intention.

---

# 3. Main purpose of this file

This file answers the question:

**Given the current selected target and system safety, what should the robot do right now?**

Examples:

* no target yet → wait
* target just appeared → locking
* target is valid and in front → follow
* target is too close → hold
* target is stale for a short time → temporary occluded
* target disappeared longer → reacquire
* target gone even longer → local search
* safety blocked → safe stop
* target kept facing robot too long → aborted

That is the real job of this node.

---

# 4. High-level structure of the file

The file mainly contains:

* enum-like state definitions
* parameter setup
* subscriber callbacks
* timer loop
* `_decide_state()`
* `_build_motion_intent()`
* message builders

So compared to `person_track_manager.py`, this file is much smaller and cleaner.

It is a policy state machine.

---

# 5. Parameters in detail

This file has a lot of thresholds, because behavior decisions depend on timing and distance.

The parameters can be grouped like this.

---

## A. Topic parameters

These are just input/output routing:

* `target_topic`
* `target_facing_topic`
* `follow_enable_topic`
* `safety_topic`
* `behavior_status_topic`
* `motion_intent_topic`

---

## B. Behavior geometry thresholds

These decide what “too close,” “hold,” and “follow” mean.

### `follow_distance_m`

Desired standoff distance from person.

This is the main following distance.

### `hold_distance_m`

If person is closer than this, robot should hold instead of advancing.

### `stop_distance_m`

A stricter near-distance threshold.

This is the “too near” or immediate no-advance zone.

### `rotate_bearing_thresh_deg`

If target bearing is large enough, robot may prefer rotate behavior.

So distance and bearing thresholds shape the behavior.

---

## C. Lock and wait timing

### `lock_confirm_sec`

When target first appears, behavior may wait a short time in `LOCKING_TARGET` before full follow.

This avoids instantly reacting to one noisy frame.

---

## D. Occlusion and search timing

### `target_soft_timeout_sec`

Target slightly stale but maybe recoverable.

### `target_hard_timeout_sec`

Target much older; more serious loss.

### `reacquire_timeout_sec`

How long to stay in reacquire before moving on.

### `search_timeout_sec`

How long to stay in local search before safe stop.

These timings create a progressive loss policy instead of immediate failure.

---

## E. Confidence thresholds

### `min_publishable_confidence`

Below this, target is too weak for confident follow.

### `cautious_confidence_threshold`

Below this, follow may downgrade to cautious behavior.

So the node distinguishes:

* strong follow
* cautious follow
* hold / stop

---

## F. Speed-related parameters

These do not directly drive motors, but they are packed into behavior/motion intent.

### `nominal_speed_mps`

Normal follow mode.

### `cautious_speed_mps`

Slower speed when confidence is weaker.

### `search_rotate_speed_radps`

Used during local search / rotate behavior.

---

## G. Face-hold / abort parameters

### `face_hold_sec`

If the person is near and facing the robot for long enough, behavior can interpret that as a stop interaction condition.

This is a very interesting design.

It means:
if the person turns and faces the robot near the robot for long enough,
the robot may stop or abort follow intentionally.

That can make sense in human-robot interaction.

---

# 6. Internal state variables

The node keeps cached inputs and some timing memory.

Important ones include:

* latest selected target
* latest facing bool
* latest follow enable
* latest safety status
* time last valid target was seen
* time entered current state
* face-hold start time
* current behavior state

So it has memory of:

* what input currently says
* how long the current state has lasted
* how long face-hold condition has been active

That timing memory is what makes the behavior state machine work.

---

# 7. Input callbacks

These callbacks are simple caches, not heavy logic.

---

## `_target_cb(msg)`

Stores latest selected target.

Also updates “last target seen” timing when target is valid/fresh enough.

This is important for timeout-based behavior.

---

## `_target_facing_cb(msg)`

Stores whether target is currently facing robot.

Simple boolean cache.

---

## `_follow_enable_cb(msg)`

Stores whether following is enabled.

If disabled, behavior will go idle.

---

## `_safety_cb(msg)`

Stores latest safety status.

Behavior decisions later depend on this.

---

# 8. Main timer loop

This file usually uses a timer-based loop to evaluate behavior continuously.

The loop does roughly:

1. inspect latest target
2. inspect facing / safety / follow enable
3. decide current behavior state
4. build behavior status message
5. build motion intent message
6. publish both

So this node is a pure policy evaluator.

---

# 9. Behavior states

This is the most important conceptual part.

The file defines a set of named behavior states.

The exact enum/message names may be in `putrabot_interfaces`, but logically the states are:

## `IDLE`

Follow not enabled.

Robot should do nothing.

---

## `WAIT_FOR_TARGET`

Follow is enabled, but no usable target yet.

Robot is ready, but waiting.

---

## `LOCKING_TARGET`

A target has appeared, but behavior wants a short confirmation time before fully committing.

This avoids jumping too fast on noisy first detection.

---

## `FOLLOWING`

Normal confident follow state.

Selected target is valid, publishable, and confidence is strong.

---

## `FOLLOWING_CAUTIOUS`

Target exists, but confidence is weaker or conditions are less ideal.

Robot should still follow, but more cautiously.

---

## `HOLD_LOCKED`

Robot has the target, but should hold position.

Typical reasons:

* target too close
* target is near and facing robot
* confidence suggests holding rather than advancing

---

## `TEMP_OCCLUDED`

Target just became stale or temporarily not usable, but likely recoverable.

This is the first loss stage.

---

## `REACQUIRE`

The robot is actively trying to recover the target after a longer temporary loss.

More serious than `TEMP_OCCLUDED`.

---

## `LOCAL_SEARCH`

Target has been missing long enough that behavior escalates to search strategy.

Usually means rotate or move toward last known area.

---

## `SAFE_STOP`

Behavior cannot continue due to safety or target availability issues.

Robot should stop.

---

## `ABORTED`

A terminal behavior stop condition.

This can happen when:

* safety says hard stop
* face-hold condition signals end of interaction
* other stop condition is reached

This is more final than `SAFE_STOP`.

---

# 10. `_decide_state()` — the core logic

This is the heart of the file.

It takes current cached inputs and returns the next behavior state.

This is the single most important function here.

The logic is basically a priority tree.

---

## Case 1: follow disabled

If follow enable is false:

→ `IDLE`

This is the top priority.

No follow means no active behavior.

---

## Case 2: safety says stop/inhibit/fault

If safety is not okay enough for motion:

→ `SAFE_STOP`
or
→ `ABORTED`

depending on the exact safety status

This means behavior respects the watchdog.

---

## Case 3: no target yet

If follow is enabled but no valid usable target is present:

→ `WAIT_FOR_TARGET`

This is the standby state.

---

## Case 4: target just appeared

If a target has recently become valid, but lock confirmation time has not elapsed:

→ `LOCKING_TARGET`

This creates a short dwell before the robot fully commits to follow.

That helps reduce twitchy behavior.

---

## Case 5: target too close

If target distance is below hold threshold:

→ `HOLD_LOCKED`

Because following closer would be bad.

---

## Case 6: target is near and facing robot

If:

* target is facing robot
* target is within hold/interaction distance

→ `HOLD_LOCKED`

This makes sense socially:
if the person turns and faces the robot close by, robot should hold rather than keep pushing forward.

---

## Case 7: face-hold persists too long

If the face-hold condition continues longer than `face_hold_sec`:

→ `ABORTED`

This looks like a task-level interpretation:
“the human stopped, turned to the robot, and interaction ended.”

So behavior may stop following entirely.

---

## Case 8: target is good and strong

If target is:

* valid
* publishable
* fresh
* good enough confidence

→ `FOLLOWING`

This is the normal mode.

---

## Case 9: target is weaker but still usable

If target exists but:

* confidence lower
* maybe predicted
* maybe reacquire active
* maybe not ideal

→ `FOLLOWING_CAUTIOUS`

This means the robot still follows, but behavior marks it as reduced-confidence follow.

---

## Case 10: temporary target loss

If target was recently valid but is now stale only for a short time:

→ `TEMP_OCCLUDED`

This is the soft-loss stage.

---

## Case 11: longer target loss

If target has been missing longer but still within reacquire window:

→ `REACQUIRE`

Now the robot should actively try to recover.

---

## Case 12: even longer loss

If reacquire times out but search timeout not exhausted:

→ `LOCAL_SEARCH`

This is the last recovery stage before stop.

---

## Case 13: all recovery exhausted

If search also times out:

→ `SAFE_STOP`

So behavior does not keep wandering forever.

---

# 11. State transition philosophy

This file is designed around **progressive degradation**, not abrupt jumps.

The loss path is usually:

`FOLLOWING`
→ `TEMP_OCCLUDED`
→ `REACQUIRE`
→ `LOCAL_SEARCH`
→ `SAFE_STOP`

This is a good design because real-world tracking always has brief dropouts.

If you went directly:

* from follow
* to stop

the robot would feel too fragile.

---

# 12. Face-hold logic

This is one of the most interesting behavior ideas in the file.

The behavior manager monitors whether:

* the person is near
* and facing the robot

If that happens briefly:

* hold

If it persists longer than `face_hold_sec`:

* abort/finish

This means the behavior manager is interpreting the human’s body orientation as a social interaction signal.

That is a nice human-robot interaction touch.

In plain language:
“if the person stops and turns to face me, maybe they want me to stop.”

---

# 13. `_build_motion_intent()` — the output action logic

After deciding the behavior state, the node converts it into `MotionIntent`.

This is the second most important function.

The intent is a simpler action-level message for `nav_follow_interface.py`.

---

## If state is `IDLE`

Usually outputs neutral/hold style intent.

Robot should not move.

---

## If state is `WAIT_FOR_TARGET`

Usually no real movement, likely hold.

Robot waits.

---

## If state is `LOCKING_TARGET`

Usually may output rotate or hold depending on target bearing.

The robot may orient toward the target without fully following yet.

---

## If state is `FOLLOWING`

Outputs `FOLLOW_POSE`

This tells the next node:

* send normal follow goal toward standoff point

Intent also carries speed preference like nominal speed.

---

## If state is `FOLLOWING_CAUTIOUS`

Still outputs `FOLLOW_POSE`, but with cautious parameters.

Usually lower preferred speed or more conservative mode.

---

## If state is `HOLD_LOCKED`

Outputs `HOLD`

Robot should stay in place.

---

## If state is `TEMP_OCCLUDED`

Often outputs `ROTATE` or temporary hold, depending on design and target memory.

The goal is to reacquire without aggressive movement.

---

## If state is `REACQUIRE`

Usually outputs `ROTATE` or targeted recovery action.

This tells the next node to try recovering the target.

---

## If state is `LOCAL_SEARCH`

Outputs `SEARCH`

This means:

* rotate search, or
* navigate to last known target area

depending on the execution node.

---

## If state is `SAFE_STOP` or `ABORTED`

Outputs `HOLD`

Robot should stop.

---

# 14. Behavior status message

The node also builds and publishes a detailed `BehaviorStatus`.

This usually contains:

* current state
* reason code
* whether target is valid
* whether target is publishable
* target age/staleness
* whether facing is active
* whether safety is inhibiting
* possibly timing/state age

This is mainly for:

* downstream monitoring
* debugging
* UI/status reporting

So `BehaviorStatus` tells you **why** the node decided what it decided.

---

# 15. Why this file is important

This file is important because tracking alone is not enough.

Even if tracker knows where the person is, you still need policy like:

* should robot move now?
* should it stop because target is too close?
* should it search?
* should it rotate?
* should it abort because the human stopped interaction?

That is exactly what this file handles.

So this file turns raw target state into robot-level behavior logic.

---

# 16. How it connects to `nav_follow_interface.py`

This is the key interface:

`follow_behavior_manager.py`
publishes:

* `BehaviorStatus`
* `MotionIntent`

Then:

`nav_follow_interface.py`
reads `MotionIntent` and turns it into:

* `cmd_vel`
* Nav2 goals
* rotate actions
* search actions

So behavior manager says:

* “what should happen”

and nav interface says:

* “how to execute it”

That separation is good architecture.

---

# 17. Strengths of this implementation

What is good about this file:

## It is layered cleanly

It does not mix tracking and movement control.

## It uses progressive recovery

Occlusion does not instantly become failure.

## It respects social geometry

Distance and facing influence hold/stop behavior.

## It respects safety

Safety state can override motion.

## It provides cautious mode

Not everything is binary follow/stop.

So this is a thoughtful behavior layer.

---

# 18. Limitations / things to watch

A few practical points:

## 1. Good tuning matters

Behavior depends heavily on:

* follow distance
* hold distance
* timeout values
* confidence thresholds
* face-hold duration

Bad tuning can make robot feel:

* too hesitant
* too aggressive
* too easy to stop
* too slow to search

---

## 2. Behavior depends on tracker quality

If upstream target is unstable, behavior can only react to that instability.

It cannot fix wrong target identity by itself.

---

## 3. Facing behavior depends on facing signal quality

If facing estimation is noisy, hold/abort behavior may trigger too often or too rarely.

So the quality of `facing_robot` matters a lot here.

---

# 19. Plain-English summary

The easiest way to understand `follow_behavior_manager.py` is:

It watches the selected target, checks whether following is enabled and whether safety allows motion, then decides if the robot should wait, follow normally, follow cautiously, hold position, try to reacquire, search locally, stop safely, or abort the session.

So in one sentence:

**`follow_behavior_manager.py` is the policy state machine that turns selected-target status into follow/search/hold/stop decisions.`**

---

# 20. Super short logic flow

The internal logic is basically:

selected target + facing + follow enable + safety
→ decide behavior state
→ convert state into motion intent
→ publish status and intent

---

# 21. Best mental model

If `person_track_manager.py` answers:

**“Who is my person?”**

and `target_lock_manager.py` answers:

**“Is that selected target stable enough to trust?”**

then `follow_behavior_manager.py` answers:

**“Given that trusted target, what should the robot do right now?”**

The next file after this is `nav_follow_interface.py`, which is the execution layer.


————————————————————————————————————————————nav_follow_interface.py——————————————————————————————————————————————————————————————————————————————
Yes — next is `nav_follow_interface.py`.

This file is the **execution bridge** between behavior and real robot motion.

A simple way to place it in the full pipeline:

* `person_track_manager.py` → tracks and selects the person
* `target_lock_manager.py` → republishes one stable target
* `follow_behavior_manager.py` → decides what the robot should do
* **`nav_follow_interface.py`** → actually turns that decision into robot movement

So this file is not about perception and not about behavior policy.

Its job is:

**selected target + motion intent + safety + TF/Nav2 availability**
→ publish `cmd_vel` when needed
→ send/cancel Nav2 goals when needed
→ expose nav/TF health

The easiest summary is:

**`nav_follow_interface.py` is the motion executor for person following.**

---

# 1. What this node subscribes to

It subscribes to four main inputs.

## `target_topic`

Default:

* `/person_target_locked`

Type:

* `TrackedPose`

This is the selected target from `target_lock_manager.py`.

From this message, the node uses:

* target pose
* validity
* publishable
* predicted/reacquire flags
* staleness
* facing info

This is the geometric input for motion.

---

## `motion_intent_topic`

Default:

* `/person_follow/motion_intent`

Type:

* `MotionIntent`

This comes from `follow_behavior_manager.py`.

This is the most important control input.

It tells the execution node what mode to perform, such as:

* HOLD
* ROTATE
* FOLLOW_POSE
* SEARCH

So this node does not decide behavior.
It only obeys the motion intent.

---

## `safety_topic`

Default:

* `/person_follow/safety_status`

Type:

* `SafetyStatus`

This is used to block movement when system safety is not okay.

So even if motion intent says follow, safety can still inhibit motion.

---

## `follow_enable_topic`

Default:

* `/person_follow/enable`

Type:

* `std_msgs/Bool`

This is the master session enable.

If disabled:

* cancel motion
* stop robot

---

# 2. What this node publishes

It publishes several outputs.

## `cmd_vel_topic`

Default:

* `/person_follow/cmd_vel_raw`

Type:

* `geometry_msgs/Twist`

This is for direct local motion, mainly:

* hold = zero twist
* rotate = in-place turning
* some search behavior

This topic usually goes into `twist_mux`.

---

## `nav_available_topic`

Default:

* `/person_follow/nav_available`

Type:

* `std_msgs/Bool`

Tells the system whether Nav2 action server is reachable.

Used by `safety_supervisor.py`.

---

## `tf_ok_topic`

Default:

* `/person_follow/tf_ok`

Type:

* `std_msgs/Bool`

Tells whether required TF lookups are currently working.

Also used by `safety_supervisor.py`.

---

## Nav2 action goal

This node also acts as a Nav2 action client:

* usually `NavigateToPose`

So it does not publish goals as a topic.
It sends them through the Nav2 action interface.

---

# 3. Main purpose of this file

This file answers:

**Given the current motion intent, how exactly should the robot move right now?**

Examples:

* `HOLD` → cancel Nav2 and publish zero twist
* `ROTATE` → cancel Nav2 and rotate in place
* `FOLLOW_POSE` → compute standoff goal and send Nav2 goal
* `SEARCH` → go to last known target area or rotate search

So this file converts abstract behavior into concrete motion commands.

---

# 4. High-level structure of the file

The file contains:

* parameter setup
* subscriber callbacks
* TF and Nav2 client setup
* timer loop
* availability checks
* motion mode handlers
* Nav2 goal generation helpers
* local rotate helpers
* stop/cancel helpers

Compared to `person_track_manager.py`, this file is much more operational.

It is a motion adapter.

---

# 5. Parameters in detail

The parameters define how execution behaves.

---

## A. Topic parameters

* `target_topic`
* `motion_intent_topic`
* `safety_topic`
* `follow_enable_topic`
* `cmd_vel_topic`
* `nav_available_topic`
* `tf_ok_topic`

These are just wiring.

---

## B. Frame parameters

* `map_frame`
* `base_frame`
* sometimes target/track frame assumptions

These are very important because Nav2 goals need map-frame pose, while target often arrives in robot-centered tracking frame.

Typical idea:

* target pose is relative to robot or tracking frame
* goal must be converted into map frame

---

## C. Nav2 action parameters

* action name like `/navigate_to_pose`
* goal resend timing
* minimum goal update distance
* cancel cooldown
* search resend timing

These control how frequently new Nav2 goals are sent and when old goals are canceled.

This matters because you do not want to spam Nav2 with tiny goal updates every cycle.

---

## D. Follow geometry parameters

* `follow_distance_m`
* `max_goal_distance_m`
* maybe goal deadbands

These define how far behind the target the robot should aim.

This is very important:
the robot does not chase the exact target point.
It aims for a **standoff pose**.

---

## E. Rotate control parameters

* `yaw_deadband_deg`
* `min_rotate_speed_radps`
* `max_rotate_speed_radps`

These are for local in-place turning.

Used in modes like:

* locking orientation
* reacquire
* local search

---

## F. Safety / execution timing

* send rates
* stale thresholds
* goal cooldowns

These reduce motion jitter and action spam.

---

# 6. Internal state variables

This node keeps cached latest inputs plus motion execution memory.

Important ones typically include:

* latest target
* latest motion intent
* latest safety status
* latest follow enable
* last sent Nav2 goal
* last goal send time
* last cancel time
* last known target map pose
* whether Nav2 is currently available
* whether TF is currently okay

This memory is needed because motion execution is not stateless.

For example:

* you must know whether a goal is already active
* you must know last target pose for search fallback
* you must avoid cancel/send spam

---

# 7. Subscriber callbacks

These callbacks are mostly cache updates.

---

## `_target_cb(msg)`

Stores the latest selected target.

This is the geometry reference for motion.

It may also update the “last known target” memory when target is good enough.

That last known target is useful for search mode.

---

## `_motion_intent_cb(msg)`

Stores latest motion intent from behavior manager.

This is the main driver of what mode the execution node will run.

---

## `_safety_cb(msg)`

Stores latest safety status.

If safety says motion should be inhibited, later timer logic can stop the robot or suppress goal sending.

---

## `_follow_enable_cb(msg)`

Stores session enable.

If follow is disabled:

* node should stop motion and cancel Nav2 goal

---

# 8. TF and Nav2 setup

This node creates:

* TF buffer + listener
* Nav2 action client

These are core dependencies.

---

## TF use

TF is used to:

* get current robot pose in map frame
* convert target-relative motion into map-frame goal

Without TF, the node cannot reliably create follow goals.

---

## Nav2 action client

Used to:

* wait for Nav2 server
* send `NavigateToPose`
* cancel goals

So this file is the glue between person-following logic and Nav2 navigation.

---

# 9. Main timer loop

This is the main runtime engine.

At each cycle, the node usually does:

1. check Nav2 availability
2. check TF health
3. publish nav/tf status
4. inspect follow enable + safety
5. inspect current motion intent
6. choose execution path:

   * hold
   * rotate
   * follow pose
   * search
7. send/cancel/publish motion accordingly

So the timer loop is the operational controller.

---

# 10. Availability checks

These are important for system health.

---

## Nav2 availability

The node checks whether the Nav2 action server is reachable.

If not:

* `nav_available = False`

This gets published and later seen by `safety_supervisor.py`.

---

## TF availability

The node checks whether necessary transforms are available.

For example:

* `map -> base_frame`

If not:

* `tf_ok = False`

Also published for safety supervisor.

So this node is one of the main health reporters for downstream safety.

---

# 11. Motion modes

This is the most important conceptual part.

The node executes different modes depending on `MotionIntent`.

---

## Mode 1: HOLD

This means:

* robot should stop moving
* cancel any active Nav2 goal
* publish zero `Twist`

Typical reasons:

* target too close
* waiting
* safe stop
* aborted
* interaction hold

### What the node does

1. cancel active Nav2 goal if needed
2. publish zero twist

This is the safest mode.

---

## Mode 2: ROTATE

This means:

* robot should rotate in place locally
* no Nav2 translation goal should remain active

Typical reasons:

* locking onto target
* temporary occlusion
* reacquire
* search-orient behavior

### What the node does

1. cancel Nav2 goal
2. compute yaw/bearing error to target
3. publish angular `Twist` only

So rotate is a local low-level motion mode.

---

## Mode 3: FOLLOW_POSE

This is the main normal follow mode.

This is where the robot follows by sending a Nav2 goal.

Important:
it does not send the target position itself as goal.

It computes a **standoff goal**.

### What that means

If the person is at position `(x, y)` relative to robot, the node:

* finds the direction toward that person
* subtracts desired follow distance along that direction
* creates a goal behind the person, not on top of them

This is the correct follow strategy.

### Then it:

1. transform robot pose into map frame
2. convert standoff goal into map coordinates
3. send `NavigateToPose`

So this mode uses global/local navigation instead of raw direct driving.

---

## Mode 4: SEARCH

This is used when behavior manager wants recovery/search.

### What the node does

If it has a remembered last known target map pose:

* send Nav2 goal there

If not:

* fallback to rotate search

So search mode is a bridge between behavior recovery and navigation execution.

---

# 12. Why FOLLOW uses Nav2 instead of raw cmd_vel

This is a very important design choice.

Instead of directly commanding velocity toward the target, the node sends a navigation goal.

Benefits:

* obstacle avoidance comes from Nav2
* planner/controller handle path generation
* robot can move around objects while following
* integrates better with mapped navigation

So this file is designed for a robot that should follow socially and safely, not just blindly chase.

---

# 13. Goal generation for follow mode

This is one of the most important parts.

The follow goal is usually generated like this:

### Step 1: read target position relative to robot

Suppose target is at:

* `tx, ty`

### Step 2: compute distance to target

* `r = sqrt(tx^2 + ty^2)`

### Step 3: compute unit direction to target

* `ux = tx / r`
* `uy = ty / r`

### Step 4: compute standoff point in robot frame

* `goal_x = tx - follow_distance * ux`
* `goal_y = ty - follow_distance * uy`

This means:
go toward the person, but stop short by follow distance.

### Step 5: clamp excessive goal step

If target is very far, do not send an excessively far goal.
Clamp using `max_goal_distance_m`.

### Step 6: transform that point into map frame

Using current robot pose in map:

* convert robot-frame standoff point into map-frame goal

### Step 7: choose goal orientation

Usually face toward the person or toward the direction of travel.

Then send Nav2 goal.

So the robot follows by navigating to a moving standoff waypoint.

---

# 14. Rotate logic

Rotate mode is local, not Nav2-based.

The node usually does:

### Step 1: compute bearing error

Use target relative angle.

### Step 2: if inside deadband

Stop rotating.

### Step 3: otherwise command angular speed

Angular speed depends on error magnitude, with limits:

* at least min rotate speed
* at most max rotate speed

So rotate is a bounded proportional-like controller.

This is useful because:

* small target bearing changes do not need full Nav2 goal updates
* local orientation is faster and smoother than replanning

---

# 15. Search logic

When motion intent is `SEARCH`, execution usually does one of two things.

## Case A: last known target map pose exists

Send Nav2 goal to that remembered target location.

This is smarter than random spin, because it uses the most recent useful target location.

## Case B: no remembered target position

Fallback to local rotate search.

So search mode can degrade gracefully.

---

# 16. Goal resend and spam control

A good execution node should not send Nav2 goals every tiny frame update.

This file usually includes logic like:

* minimum distance change before sending a new goal
* minimum resend interval
* cooldown before cancel
* separate resend timing for search goals

Why this matters:
if you send too many goals:

* Nav2 gets spammed
* motion becomes jittery
* logs become messy
* controller may oscillate

So this file likely tries to be conservative about goal updates.

---

# 17. Cancel logic

This is also important.

When mode switches from follow/search to hold/rotate:

* current Nav2 goal must be canceled

Why:
you do not want Nav2 still driving while local rotate or hold is active.

So the file acts like an arbitration layer:

* either Nav2 controls translation
* or local twist control does
* not both at once

That is very good architecture.

---

# 18. Zero-twist publish

In hold and stop conditions, the node publishes zero velocity.

This is necessary because:

* canceling Nav2 alone does not always guarantee immediate stop
* twist mux may still need an explicit zero command

So the file likely has a helper to publish zero `Twist`.

---

# 19. Last known target memory

This is subtle but important.

The node usually remembers the last useful target position in map frame.

This memory is used for:

* search mode
* recovery after short target loss

So even if the target disappears briefly, the robot can still go toward the last known person location.

That makes the system feel more persistent and less fragile.

---

# 20. Interaction with safety

This node respects safety status.

If safety says movement should be inhibited:

* it should not keep sending follow/search motion
* it should stop or hold

This is important because the behavior layer may want to follow, but execution must still obey system safety.

So this node is not blind to motion intent.

---

# 21. Interaction with `twist_mux`

This is also very important in your architecture.

This node publishes raw follow control to:

* `/person_follow/cmd_vel_raw`

Then `twist_mux` decides priority among:

* joystick
* follow
* nav

So `nav_follow_interface.py` is not necessarily writing directly to the wheel base.

It is writing into the shared velocity arbitration system.

That is correct design.

---

# 22. Why this file is important

Without this file, behavior manager would only say:

* follow
* rotate
* hold
* search

But there would be no actual logic for:

* how to send Nav2 goals
* how to cancel them
* how to rotate locally
* how to track TF and Nav2 health

So this file is the concrete execution layer.

---

# 23. Strengths of this design

What is good about this file:

## It separates behavior from execution

Behavior says what should happen.
This file says how to make it happen.

## It uses Nav2 for follow/search

That gives obstacle-aware motion.

## It uses local twist for rotate/hold

That is fast and simple.

## It reports health

Nav2 availability and TF health are published for safety.

## It remembers last known target location

Useful for search/recovery.

So overall this is a strong architecture.

---

# 24. Limitations / things to watch

A few practical concerns:

## 1. Follow quality depends on TF quality

If `map -> base` or target-frame assumptions are inconsistent, follow goal generation can be wrong.

## 2. Too-frequent goal updates can cause jitter

So resend thresholds must be tuned well.

## 3. Search quality depends on last known target memory

If the remembered map pose is stale or wrong, search may be poor.

## 4. Rotate behavior depends on good target bearing

If target pose is noisy, rotate can oscillate.

So execution tuning still matters a lot.

---

# 25. Plain-English summary

The easiest way to understand `nav_follow_interface.py` is:

It listens to the selected person and the behavior manager’s motion intent, then either stops the robot, rotates it locally, sends a follow waypoint behind the person through Nav2, or sends a search goal to the last known target area, while also reporting whether Nav2 and TF are healthy.

So in one sentence:

**`nav_follow_interface.py` is the motion execution adapter that turns follow behavior into real robot commands and Nav2 goals.`**

---

# 26. Super short logic flow

The file’s runtime logic is basically:

selected target + motion intent + safety
→ check TF/Nav2 health
→ choose hold / rotate / follow / search
→ publish zero twist or rotate twist or send/cancel Nav2 goal
→ publish nav/tf status

---

# 27. Best mental model

If `follow_behavior_manager.py` answers:

**“What should the robot do?”**

then `nav_follow_interface.py` answers:

**“How do I physically make the robot do that right now?”**

The next logical file after this is `safety_supervisor.py`, because that one watches the whole chain and decides whether it is safe/healthy enough to continue.


————————————————————————————————————————————————————————————————safety_supervisor.py———————————————————————————————————————————————————————————————————
Yes — `safety_supervisor.py` is the **watchdog / gatekeeper** of the `person_following` package.

This file is not doing:

* detection
* tracking
* behavior planning
* motion execution

Its job is simpler and very important:

**watch the whole person-follow pipeline and decide whether the system is healthy enough to continue**

A good one-line summary is:

**`safety_supervisor.py` turns many subsystem conditions into one final safety status and stop reason.**

---

# 1. Where it sits in the pipeline

The flow around it is:

* `target_lock_manager.py` → selected target
* `follow_behavior_manager.py` → behavior state
* `nav_follow_interface.py` → reports TF and Nav2 health
* `safety_supervisor.py` → combines all that into one safety decision

Then downstream:

* `follow_behavior_manager.py` and `person_follow_server.py` can react to this safety result

So this node is like the system-level referee.

---

# 2. What it subscribes to

It listens to 7 inputs.

## `follow_enable_topic`

Default:

* `/person_follow/enable`

Type:

* `std_msgs/Bool`

This tells it whether the session is enabled at all.

If follow is disabled, safety does not report “OK”; it reports a waiting/disabled style state.

---

## `tracks_state_topic`

Default:

* `/person_tracks_state`

Type:

* `TrackedPoseArray`

This is used mainly as a **tracker heartbeat**.

Important detail:
this file does **not** inspect every track deeply.
It mainly uses the message arrival time to know whether the tracker is still alive.

---

## `target_topic`

Default:

* `/person_target_locked`

Type:

* `TrackedPose`

This is the selected target.

The node uses it to determine:

* whether a target exists
* whether it is valid
* how stale it is

---

## `behavior_status_topic`

Default:

* `/person_follow/behavior_status`

Type:

* `BehaviorStatus`

This is used mainly to detect:

* whether behavior has entered `STATE_ABORTED`

So if behavior says the session is aborted, safety escalates that into a fault-like stop reason.

---

## `nav_available_topic`

Default:

* `/person_follow/nav2_available`

Type:

* `std_msgs/Bool`

This comes from `nav_follow_interface.py`.

It tells the supervisor whether Nav2 is currently available.

---

## `tf_ok_topic`

Default:

* `/person_follow/tf_ok`

Type:

* `std_msgs/Bool`

Also from `nav_follow_interface.py`.

It tells the supervisor whether required TF is currently healthy.

---

## `obstacle_inhibit_topic`

Default:

* `/person_follow/obstacle_inhibit`

Type:

* `std_msgs/Bool`

This is an external inhibit signal.

If this is true:

* motion should be blocked
* and if it lasts too long, it becomes a fault timeout

So this is the direct obstacle/safety brake input.

---

# 3. What it publishes

It publishes two outputs.

## `safety_status_topic`

Default:

* `/person_follow/safety_status`

Type:

* `SafetyStatus`

This is the main output.

It contains:

* health state
* inhibit flag
* stop reason code
* stop source code
* human-readable labels
* tracker age
* target age
* TF age
* Nav2 availability
* target validity
* obstacle status

This is the core system safety report.

---

## `stop_reason_topic`

Default:

* `/person_follow/stop_reason`

Type:

* `std_msgs/String`

This just publishes the text label of the stop reason.

Important detail:
it publishes `status.stop_reason`, not the numeric code.

So this is mostly for UI/debugging and for quick terminal checking.

---

# 4. Main purpose of the file

This file answers:

**“Is the current person-follow session healthy enough to continue, and if not, why not?”**

Examples:

* follow disabled
* tracker heartbeat died
* TF disappeared
* Nav2 not ready
* obstacle inhibit active
* no selected target yet
* selected target too stale
* behavior explicitly aborted

All of these become one unified `SafetyStatus`.

---

# 5. Stop reasons and stop sources

At the top of the file, there are two mappings.

## `STOP_REASON_LABELS`

Maps numeric codes to readable strings like:

* `disabled_by_server`
* `tracker_heartbeat_timeout`
* `tf_unavailable_timeout`
* `nav2_pending`
* `nav2_unavailable`
* `obstacle_inhibit`
* `obstacle_inhibit_timeout`
* `waiting_for_target`
* `selected_target_pending`
* `selected_target_stale_timeout`
* `behavior_aborted`

So this node has a very clear fault taxonomy.

---

## `STOP_SOURCE_LABELS`

Maps source codes to who caused the stop:

* `server`
* `tracker`
* `tf`
* `navigation`
* `safety`
* `target_lock`
* `behavior`

That is useful because it separates:

* what happened
  from
* which subsystem caused it

---

# 6. Parameters in detail

The important parameters are these timing thresholds.

## `tracker_timeout_sec`

Default:

* `1.0`

If no `TrackedPoseArray` arrives for longer than this:
→ tracker heartbeat timeout fault

This is how the node detects tracker death.

---

## `target_soft_timeout_sec`

Default:

* `1.2`

If the selected target age is above this:
→ target is considered pending / waiting, not yet hard-faulted

This is a softer target-loss threshold.

---

## `target_hard_timeout_sec`

Default:

* `2.5`

If target age goes above this:
→ selected target stale timeout fault

So target loss is treated progressively:

* first pending
* then hard stale timeout

---

## `tf_timeout_sec`

Default:

* `0.75`

If TF has not been healthy within this time:
→ TF unavailable timeout fault

---

## `nav_timeout_sec`

Default:

* `1.5`

If Nav2 is currently unavailable and the last known good Nav2 time is older than this:
→ Nav2 unavailable fault

There is also a separate “pending” case before this hard timeout.

---

## `obstacle_fault_timeout_sec`

Default:

* `3.0`

If obstacle inhibit stays active beyond this:
→ obstacle inhibit timeout fault

Before that, obstacle inhibit is just an inhibited state, not yet a hard fault.

---

# 7. Internal state variables

The node keeps simple state memory.

Important ones:

* `follow_enabled`
* `last_tracker_rx_sec`
* `last_target_rx_sec`
* `latest_target`
* `latest_behavior`
* `nav_available`
* `last_nav_ok_sec`
* `tf_healthy`
* `last_tf_ok_sec`
* `obstacle_inhibit`
* `obstacle_start_sec`

This is enough to compute:

* freshness
* timeout age
* whether problem is current or prolonged

So this file is time-based monitoring, not heavy logic.

---

# 8. Input callbacks

These are very simple.

## `_follow_enable_cb`

Stores whether follow is enabled.

---

## `_tracks_cb`

Does not inspect track contents.
It only updates:

* `last_tracker_rx_sec`

So this topic is treated as a heartbeat.

That is an important detail.

---

## `_target_cb`

Stores:

* latest target message
* last target receive time

So target staleness can later be computed.

---

## `_behavior_cb`

Stores the latest behavior message.

Used mainly to detect `STATE_ABORTED`.

---

## `_nav_cb`

Stores whether Nav2 is currently available.

If Nav2 is available:

* updates `last_nav_ok_sec`

This is smart because even if Nav2 drops briefly later, the file still remembers when it was last healthy.

---

## `_tf_cb`

Stores whether TF is currently healthy.

If TF is healthy:

* updates `last_tf_ok_sec`

Same idea as Nav2:
use last-known-good timestamp, not only instantaneous bool.

---

## `_obstacle_cb`

Tracks obstacle inhibit state.

If inhibit just became active:

* record `obstacle_start_sec`

If inhibit becomes false:

* clear obstacle timer

So the node can distinguish:

* short active obstacle inhibit
  from
* obstacle inhibit stuck for too long

---

# 9. Main timer loop

The timer runs every `0.1 sec`, so 10 Hz.

In `_timer_cb()` it does only two things:

1. call `_build_status(now)`
2. publish:

   * `SafetyStatus`
   * stop reason string

So the real logic is all inside `_build_status()`.

---

# 10. `_build_status()` — the core logic

This is the heart of the file.

It builds the `SafetyStatus` in priority order.

That priority order is very important.

---

## Step 1: fill generic measured fields

Before making decisions, it fills:

* `follow_enabled`
* `nav_available`
* `tf_healthy`
* `obstacle_inhibit`
* `tracker_age_sec`
* `target_age_sec`
* `tf_age_sec`
* `target_valid`

So every status message includes raw monitoring context.

---

## Step 2: follow disabled

```python
if not self.follow_enabled:
```

Return:

* `HEALTH_WAITING`
* inhibit = `True`
* reason = `STOP_DISABLED_BY_SERVER`
* source = `SOURCE_SERVER`

Important:
disabled follow is not treated as a system fault.
It is treated as a waiting state.

That is correct behavior.

---

## Step 3: behavior aborted

If latest behavior state is `STATE_ABORTED`:

Return:

* `HEALTH_FAULT`
* inhibit = `True`
* reason = `STOP_BEHAVIOR_ABORTED`
* source = `SOURCE_BEHAVIOR`

So if behavior layer says “abort,” safety turns that into a hard stop condition.

---

## Step 4: tracker heartbeat timeout

If tracker message never arrived, or tracker age is above timeout:

Return:

* `HEALTH_FAULT`
* inhibit = `True`
* reason = `STOP_TRACKER_HEARTBEAT_TIMEOUT`
* source = `SOURCE_TRACKER`

So tracker death is a fault.

---

## Step 5: TF timeout

If TF never became healthy, or last TF good time is too old:

Return:

* `HEALTH_FAULT`
* inhibit = `True`
* reason = `STOP_TF_UNAVAILABLE_TIMEOUT`
* source = `SOURCE_TF`

So TF failure is also a hard fault.

---

## Step 6: Nav2 pending

If Nav2 has never yet been available and `nav_available` is false:

Return:

* `HEALTH_WAITING`
* inhibit = `True`
* reason = `STOP_NAV2_PENDING`
* source = `SOURCE_NAVIGATION`

This is different from a fault.

It means:
“Nav2 has not come up yet.”

That is a startup/wait condition, not yet an error.

---

## Step 7: Nav2 unavailable timeout

If Nav2 is false, and last known good Nav2 time is older than `nav_timeout_sec`:

Return:

* `HEALTH_FAULT`
* inhibit = `True`
* reason = `STOP_NAV2_UNAVAILABLE`
* source = `SOURCE_NAVIGATION`

So:

* never available yet → waiting
* was available before, then gone too long → fault

That is a good distinction.

---

## Step 8: obstacle inhibit

If obstacle inhibit is active:

### if active too long

If obstacle active duration > `obstacle_fault_timeout_sec`:

Return:

* `HEALTH_FAULT`
* inhibit = `True`
* reason = `STOP_OBSTACLE_INHIBIT_TIMEOUT`
* source = `SOURCE_SAFETY`

### else

Return:

* `HEALTH_INHIBITED`
* inhibit = `True`
* reason = `STOP_OBSTACLE_INHIBIT`
* source = `SOURCE_SAFETY`

This is a nice design:

* temporary obstacle → inhibited
* long stuck obstacle → fault

---

## Step 9: no selected target yet

If latest target is missing, or `selected` is false:

Return:

* `HEALTH_WAITING`
* inhibit = `True`
* reason = `STOP_WAITING_FOR_TARGET`
* source = `SOURCE_TARGET_LOCK`

So no chosen target is not a fault.
It is a waiting state.

---

## Step 10: hard target stale timeout

If target age exceeds `target_hard_timeout_sec`:

Return:

* `HEALTH_FAULT`
* inhibit = `True`
* reason = `STOP_SELECTED_TARGET_STALE_TIMEOUT`
* source = `SOURCE_TARGET_LOCK`

So long target disappearance becomes a real fault-like stop reason.

---

## Step 11: soft target pending

If either:

* target age exceeds `target_soft_timeout_sec`
* or target is not valid

Return:

* `HEALTH_WAITING`
* inhibit = `True`
* reason = `STOP_SELECTED_TARGET_PENDING`
* source = `SOURCE_TARGET_LOCK`

So soft target issues are treated as waiting/pending, not fault.

---

## Step 12: all good

If none of the above triggered:

Return:

* `HEALTH_OK`
* inhibit = `False`
* reason = `STOP_NONE`
* source = `SOURCE_NONE`

This is the healthy normal operating state.

---

# 11. The priority order matters

The node checks conditions in a very specific order.

The effective priority is:

1. disabled by server
2. behavior aborted
3. tracker timeout
4. TF timeout
5. Nav2 pending/unavailable
6. obstacle inhibit
7. no target / target pending / target stale
8. OK

This means, for example:

* if both TF is bad and target is missing, TF fault wins first
* if follow is disabled, everything else is ignored and it reports disabled/waiting

That is exactly how a watchdog should behave.

---

# 12. `_target_valid()`

This helper says target is valid only if:

* latest target exists
* `selected == True`
* `valid == True`

It does **not** check `publishable`.

So in this file:

* `valid` means selected target object is structurally valid
* staleness and soft/hard timeout are handled separately

That is an important design detail.

---

# 13. `_fill()`

This helper writes the final status fields:

* `health_state`
* `stop_reason_code`
* `stop_source_code`
* `inhibit_active`
* `stop_reason`
* `stop_source`
* `health_label`

So `_build_status()` decides what happened, and `_fill()` formats the message.

---

# 14. `_health_label()`

Maps numeric health state to text:

* `HEALTH_OK` → `OK`
* `HEALTH_WAITING` → `WAITING`
* `HEALTH_INHIBITED` → `INHIBITED`
* else → `FAULT`

So health has four practical levels:

* OK
* WAITING
* INHIBITED
* FAULT

That is clean and easy to reason about.

---

# 15. `_age()` and `_target_age()`

These are small but important helpers.

## `_age(now, stamp)`

Returns:

* `0.0` if stamp invalid
* else time difference

Used for tracker, TF, Nav2, obstacle timing.

---

## `_target_age(now)`

This one is slightly special.

If latest target has `staleness_sec > 0`:

* use that value directly

Else:

* use time since last target message receive

That is a very good design.

Why:
the upstream target message may already contain its own internal staleness estimate, which is more meaningful than just message transport age.

So this watchdog respects upstream target age if available.

---

# 16. What this node is really doing conceptually

This file is not making navigation decisions.

It is not saying:

* follow left
* rotate right
* go search kitchen

Instead, it is saying:

**“Given the state of all critical subsystems, should the session be allowed to continue?”**

That is why it is a supervisor.

---

# 17. Why this file is useful

Without this file, every other node would need to individually decide:

* is tracker alive?
* is TF okay?
* is Nav2 ready?
* is target stale?
* is obstacle inhibit active?

That would make the whole package messy.

This file centralizes all of that into one decision message.

That is good architecture.

---

# 18. Strengths of this implementation

What is good here:

## It is simple

The file is small and easy to reason about.

## It distinguishes waiting vs fault

This is very important.
Not everything should be treated as a hard error.

Examples:

* Nav2 still starting up → waiting
* no target yet → waiting
* target softly stale → waiting

while:

* TF timeout
* tracker timeout
* behavior aborted
* long obstacle inhibit
  become faults.

## It uses time-based supervision

This is realistic for ROS systems where temporary glitches happen.

## It has clear reason and source labels

Very useful for debugging.

---

# 19. Limitations / things to note

A few practical points:

## 1. It only uses track-state topic as a heartbeat

It does not inspect whether the tracker output is semantically good, only whether it is alive.

So if tracker publishes nonsense but keeps publishing, this node will still think tracker heartbeat is okay.

---

## 2. It trusts upstream target validity

If upstream target says valid but is wrong identity, this node cannot detect that.

It only checks freshness and validity flags.

---

## 3. Nav2 topic default name must match your launch

Here the default is:

* `/person_follow/nav2_available`

So if another node publishes `/person_follow/nav_available` instead, they would not connect unless launch remaps it.

This is the kind of mismatch worth checking in your actual launch.

---

## 4. Disabled state inhibits motion

Even though disabled is not a fault, `inhibit_active=True` is still set.
That is correct, but worth remembering.

---

# 20. Plain-English summary

The easiest way to understand `safety_supervisor.py` is:

It watches whether follow is enabled, whether the tracker is still alive, whether TF and Nav2 are available, whether an obstacle inhibit is active, whether a selected target exists and is still fresh, and whether behavior has aborted; then it combines all of that into one final safety status and stop reason.

So in one sentence:

**`safety_supervisor.py` is the watchdog that decides whether the person-follow session is healthy, waiting, inhibited, or faulted.`**

---

# 21. Super short logic flow

Its runtime logic is basically:

follow enable + tracker heartbeat + target + behavior + Nav2 + TF + obstacle inhibit
→ compute ages and validity
→ choose WAITING / INHIBITED / FAULT / OK
→ publish `SafetyStatus` and stop reason

---

# 22. Best mental model

If `follow_behavior_manager.py` answers:

**“What should the robot do?”**

and `nav_follow_interface.py` answers:

**“How do I move the robot to do that?”**

then `safety_supervisor.py` answers:

**“Am I even allowed to continue right now?”**

The next file after this is `person_follow_server.py`, which is the operator/API layer for starting, stopping, selecting, and monitoring the full follow session.


———————————————————————————————————————————————————————————————person_follow_server.py—————————————————————————————————————————————————————————————————
Yes — next is `person_follow_server.py`.

This file is the **operator / API control layer** of the whole `person_following` package.

A simple way to place it in the pipeline:

* `person_track_manager.py` → tracks people
* `target_lock_manager.py` → publishes stable selected target
* `follow_behavior_manager.py` → decides follow/hold/search
* `nav_follow_interface.py` → executes movement
* `safety_supervisor.py` → decides if the system is healthy enough
* **`person_follow_server.py`** → gives you one clean way to start, stop, select, monitor, and control the whole follow session

So this file is not the tracking brain and not the motion executor.

Its job is:

**wrap the person-follow system into actions + services + session control**

Best one-line summary:

**`person_follow_server.py` is the command-and-control interface for the person-following stack.`**

---

# 1. What this node does at a high level

This file gives you three control styles:

### 1. Action interface

For starting and supervising a follow session over time.

Action:

* `/person_follow/control`

### 2. Services

For direct commands like:

* select a person
* clear selected person
* stop follow
* get follow status
* set follow parameters

### 3. Topic control

It publishes:

* `/person_target_select_id`
* `/person_follow/enable`
* `/person_follow/session_active`

So this node is like the front desk of the whole package.

---

# 2. Main role of the file

This file answers questions like:

* “Start person following now.”
* “Pick track ID 3.”
* “Stop following.”
* “Tell me current follow status.”
* “Change follow distance and face-hold timeout.”
* “While action is running, publish feedback to client.”
* “If safety says terminal stop, end the action properly.”

That is the real purpose.

---

# 3. Important imports and what they tell you

This file imports:

* `ActionServer`, `GoalResponse`, `CancelResponse`
* `MultiThreadedExecutor`
* `SetParameters`
* `Trigger`
* `PersonFollow` action
* `BehaviorStatus`, `SafetyStatus`, `TrackedPose`, `TrackedPoseArray`
* `ClearPerson`, `GetPersonFollowStatus`, `SelectPerson`, `SetFollowParams`

That tells you immediately:

### This node is an action server

It owns the long-running “follow session” action.

### This node also calls another node’s parameter service

It uses:

* `rcl_interfaces/srv/SetParameters`

So it can dynamically change parameters of another node, specifically:

* `follow_behavior_manager`

That is important.

---

# 4. Class-level stop-code sets

At the top of the class, there are three important sets.

## `ABORT_STOP_CODES`

These are safety stop reasons that should make the active action **abort**.

They include:

* tracker heartbeat timeout
* TF unavailable timeout
* Nav2 unavailable
* selected target stale timeout
* obstacle inhibit timeout
* behavior aborted

So if safety reports one of these, the follow action ends as failure.

---

## `CANCEL_STOP_CODES`

These are stop reasons that should make the active action **cancel**.

Currently:

* disabled by server

So if the session is disabled externally, the action gets canceled instead of aborted.

---

## `SUCCESS_STOP_CODES`

Currently empty.

So in the current code, there is no stop reason that ends the action as a success.

That means most terminal endings are either:

* canceled
* aborted

This is an important design detail.

---

## `TERMINAL_STOP_CODES`

This is just the union of all terminal stop types.

Used to detect:
“Should this active action session end now?”

---

# 5. `__init__()` — node setup

This is where the whole control interface is created.

The node name is:

* `person_follow_server`

It also uses:

* `ReentrantCallbackGroup`

That matters because:

* action callbacks
* services
* subscriptions
  can run concurrently

And the file runs with:

* `MultiThreadedExecutor(num_threads=4)`

So this node is built for concurrent control, not single-threaded blocking only.

---

# 6. Parameters in detail

The node declares these parameters.

## Topic parameters

* `tracks_state_topic`
* `target_topic`
* `facing_topic`
* `behavior_status_topic`
* `safety_status_topic`
* `select_topic`
* `follow_enable_topic`
* `session_active_topic`

These are just wiring to the rest of the stack.

---

## Timing/control parameters

* `tracks_state_timeout_sec`
* `feedback_hz`

These shape how the server behaves.

### `tracks_state_timeout_sec`

Used when validating whether track list is fresh enough for person selection.

### `feedback_hz`

The action feedback loop frequency.

Interesting detail:
it is clamped between `5.0` and `10.0`.

So even if parameter says something else, server keeps feedback in that safe range.

---

## Behavior-node parameter control

* `behavior_node_name`

Default:

* `/follow_behavior_manager`

This tells the server where to send dynamic parameter updates.

That is how this node changes:

* `follow_distance_m`
* `face_hold_sec`

inside the behavior node.

---

## Default follow parameters

* `default_follow_distance_m`
* `default_face_hold_sec`

These become the server’s remembered defaults.

---

# 7. Internal state variables

This file keeps a lot of cached control/session state.

Important ones:

* `last_tracks_state_rx_sec`
* `latest_track_ids`
* `latest_track_positions`
* `latest_target`
* `facing_robot`
* `session_active`
* `follow_enabled`
* `selected_track_id`
* `behavior_state`
* `safety_state`
* `inhibit_active`
* `stop_reason`
* `stop_reason_code`
* `_active_goal_handle`

So the server always knows:

* whether a session is active
* who is selected
* what safety currently says
* what behavior currently says
* whether an action goal is already running

That last one is especially important.

---

# 8. Subscribers

This node subscribes to five main live status topics.

## `tracks_state_cb`

Input:

* `/person_tracks_state`

Purpose:

* cache available track IDs
* cache their positions
* record freshness time

This is used mostly for:

* validating a requested selected track ID
* auto-selecting nearest track

It is not doing full track interpretation.

---

## `target_cb`

Input:

* `/person_target_locked`

Purpose:

* cache latest selected target
* update `selected_track_id`

Important detail:
`selected_track_id` becomes:

* `msg.track_id` if `msg.selected`
* otherwise `0`

So this server trusts the selected target output to reflect current selection.

---

## `facing_cb`

Input:

* `/person_target_facing_robot`

Purpose:

* store `facing_robot` bool

This is mainly used for status/action feedback.

---

## `behavior_cb`

Input:

* `/person_follow/behavior_status`

Purpose:

* store current behavior state label

Again, mostly for status and feedback.

---

## `safety_cb`

Input:

* `/person_follow/safety_status`

Purpose:

* cache:

  * health label
  * inhibit flag
  * stop reason
  * stop reason code

This is extremely important because the action loop depends on it.

Also, if:

* `session_active == True`
* and stop reason code is terminal

then the callback immediately sets:

* `session_active = False`
  and republishes session-active topic.

So safety can asynchronously shut down the session.

That is a very important design point.

---

# 9. Publishers

This node publishes three direct control topics.

## `pub_select`

Topic:

* `/person_target_select_id`

Type:

* `Int32`

Used to request selection of a specific track ID.

---

## `pub_follow_enable`

Topic:

* `/person_follow/enable`

Type:

* `Bool`

This is the master enable switch for the rest of the pipeline.

---

## `pub_session_active`

Topic:

* `/person_follow/session_active`

Type:

* `Bool`

This indicates whether the server currently considers a follow session active.

This is different from `follow_enable`, but related.

---

# 10. Services provided

This file creates five service servers.

---

## `SelectPerson`

Service:

* `/person_follow/select_person`

Handler:

* `handle_select_person`

Purpose:
select a specific track ID.

---

## `ClearPerson`

Service:

* `/person_follow/clear_person`

Handler:

* `handle_clear_person`

Purpose:
clear the selected target.

---

## `GetPersonFollowStatus`

Service:

* `/person_follow/get_status`

Handler:

* `handle_get_status`

Purpose:
return current session/follow state.

---

## `Trigger`

Service:

* `/person_follow/stop`

Handler:

* `handle_stop_follow`

Purpose:
stop follow immediately.

---

## `SetFollowParams`

Service:

* `/person_follow/set_params`

Handler:

* `handle_set_params`

Purpose:
update follow behavior parameters.

So this node exposes a complete small API.

---

# 11. Action server

This is the main long-running control interface.

Action:

* `/person_follow/control`

Callbacks:

* `goal_callback`
* `cancel_callback`
* `execute_callback`

This is the cleanest way for a higher-level task manager to say:

“Start a person-follow session, keep giving me feedback, and finish when the session ends.”

---

# 12. Parameter client to behavior node

This is one of the most important design choices.

The file creates a parameter client to:

* `/{behavior_node_name}/set_parameters`

So this node can remotely modify parameters of `follow_behavior_manager`.

In current code it updates exactly:

* `follow_distance_m`
* `face_hold_sec`

That means the operator/API layer does not hardcode these into motion itself.

Instead, it updates the behavior node that actually uses them.

That is a very good design.

---

# 13. Startup behavior

At the end of `__init__()`:

* it logs startup
* calls `_set_follow_enable(False)`
* calls `_publish_session_active()`

So on boot:

* follow is disabled
* session_active is false

This is the safe default.

---

# 14. Small helper functions

These are simple but important.

---

## `_now_sec()`

Returns current ROS time in seconds.

Used for freshness checks.

---

## `_publish_session_active()`

Publishes current `session_active` bool.

---

## `_set_follow_enable(enabled)`

Updates internal `follow_enabled` and publishes the enable topic.

So this is the main way the server turns following on/off.

---

## `_publish_select(track_id)`

Publishes a target-select request.

---

## `_is_tracks_state_fresh()`

Returns true if latest track-state heartbeat is recent enough.

Uses:

* `last_tracks_state_rx_sec`
* `tracks_state_timeout_sec`

This is used before allowing manual selection.

That is good because you should not select from stale track list.

---

## `_choose_nearest_track_id()`

Chooses the nearest available track by Euclidean distance from robot origin.

Uses cached `latest_track_positions`.

This is for:

* auto-select-if-needed

Important:
it only uses position magnitude, nothing about confidence or identity.
So this is a simple nearest-track auto-selection policy.

---

# 15. Parameter validation and application

---

## `_validate_follow_params(follow_distance_m, face_hold_sec)`

Checks:

* `follow_distance_m > 0`
* `face_hold_sec > 0`

Very simple validation.

---

## `_apply_follow_params(follow_distance_m, face_hold_sec)`

This is more important.

Flow:

1. validate values
2. wait for behavior node parameter service
3. build parameter update request
4. call async service
5. spin until future completes
6. check results

If successful:

* returns `(True, 'ok')`

If failed:

* returns `(False, reason)`

This is how action goals and `set_params` service actually change runtime behavior.

---

# 16. `tracks_state_cb()`

This callback processes `TrackedPoseArray`.

What it does:

1. update `last_tracks_state_rx_sec`
2. extract all track IDs > 0
3. cache their `(x, y)` positions

So this callback is mainly a lightweight track index.

It is used for:

* checking whether requested track exists
* choosing nearest track

It does not inspect confidence, selected flag, or track state deeply.

---

# 17. `target_cb()`

This callback caches the latest selected target.

Logic:

* `latest_target = msg`
* `selected_track_id = msg.track_id` if `msg.selected`
* else `0`

So the server always knows what the current selected target is.

This is important for:

* status service
* action feedback

---

# 18. `facing_cb()`

Very simple:

* cache `facing_robot`

Used for:

* status service
* action feedback

---

# 19. `behavior_cb()`

Stores:

* `behavior_state = msg.state_label`

Again, mostly for:

* get status
* action feedback

---

# 20. `safety_cb()`

This is more important.

It stores:

* `safety_state`
* `inhibit_active`
* `stop_reason`
* `stop_reason_code`

Then it has one key behavior:

If:

* `session_active`
* and `stop_reason_code` is terminal

then:

* `session_active = False`
* publish updated session_active

So safety can asynchronously terminate the session, even before the action loop notices.

This is a good design because it keeps session state synchronized with safety.

---

# 21. Manual-selection service

## `handle_select_person(req, res)`

This handles `/person_follow/select_person`.

Flow:

### Check 1: track ID > 0

If not:

* fail

### Check 2: tracks state must be fresh

If not:

* fail

### Check 3: requested track ID must exist in latest track set

If not:

* fail

### If all pass

* publish selection topic
* update local `selected_track_id`
* return success

So this is a clean validation layer for manual person selection.

---

# 22. Clear-selection service

## `handle_clear_person(req, res)`

Flow:

* publish select `0`
* set `selected_track_id = 0`
* set `latest_target = None`
* return success

This tells the rest of the system:
no person is selected anymore.

---

# 23. Action-finalization helper

## `_finalize_goal(goal_handle, stop_reason_code, stop_reason)`

This is a very important internal function.

It decides how to end the active action.

### If stop reason is in cancel codes

* `goal_handle.canceled()`
* result.success = False

### If stop reason is in success codes

* `goal_handle.succeed()`
* result.success = True

### Else

* `goal_handle.abort()`
* result.success = False

Then clears `_active_goal_handle`.

So this function maps system stop reason into action result semantics.

Important current behavior:
because `SUCCESS_STOP_CODES` is empty,
the action will currently end only as:

* canceled
* aborted

never success.

---

# 24. Stop-follow service

## `handle_stop_follow(req, res)`

Flow:

* `_set_follow_enable(False)`
* publish select `0`
* clear selected track
* set `session_active = False`
* publish session_active
* return success with message `follow_stopped`

So this is the immediate manual stop button.

---

# 25. Status service

## `handle_get_status(req, res)`

This returns a snapshot of current server view.

It computes:

### target visibility

If latest target exists:

* visible only if `camera_visible and valid`

### target age

From `latest_target.staleness_sec`

Then fills response:

* `active`
* `inhibit_active`
* `selected_track_id`
* `target_visible`
* `target_age_sec`
* `facing_robot`
* `state`
* `safety_state`
* `stop_reason`

So this service is the easiest way for another node/UI to get current follow status.

---

# 26. Set-params service

## `handle_set_params(req, res)`

This updates follow behavior parameters.

Flow:

1. start from current server-stored values
2. if request gives positive `follow_distance_m`, use it
3. if request gives positive `face_hold_sec`, use it
4. if nothing valid provided → fail
5. call `_apply_follow_params()`
6. if success:

   * update local copies
   * return success
7. if failure:

   * return apply-failed message

So this is a runtime tuning API for the behavior layer.

Important:
this node stores the updated values locally after successful remote apply.

---

# 27. Action goal acceptance

## `goal_callback(goal_request)`

Very simple logic:

* if `_active_goal_handle` already exists → reject new goal
* otherwise accept

So this server supports only **one active follow action at a time**.

That is exactly what you want.

---

# 28. Action cancel acceptance

## `cancel_callback(goal_handle)`

Always accepts cancel requests.

So a client can always cancel the running follow session.

---

# 29. `execute_callback()` — the core of the file

This is the main action-session logic.

It is the most important function.

Let’s break it carefully.

---

## Step 1: mark current goal as active

```python
self._active_goal_handle = goal_handle
```

So no second goal can be accepted while this one runs.

---

## Step 2: read requested parameters from action goal

The action goal may include:

* `follow_distance_m`
* `face_hold_sec`

If provided and > 0, these override current values for this session.

If not, current/default values stay.

So the action can start a session with custom follow behavior.

---

## Step 3: apply parameters to behavior node

Calls `_apply_follow_params(new_follow_distance, new_face_hold_sec)`.

If that fails:

* disable follow
* set session inactive
* publish session_active
* abort action
* return failure result

So the session will not start if behavior parameters could not be applied.

That is good defensive design.

---

## Step 4: update local stored parameters

If apply succeeds:

* update `self.follow_distance_m`
* update `self.face_hold_sec`

So the server now reflects current active settings.

---

## Step 5: optional auto-select nearest

If action goal says:

* `auto_select_if_needed == True`

and:

* no track currently selected
* track state is fresh

then:

* choose nearest track ID
* publish selection

So this is an automatic start-follow convenience feature.

Important:
it is purely nearest-track based.

---

## Step 6: mark session active and enable follow

The code does:

* `session_active = True`
* publish session_active
* `_set_follow_enable(True)`

This is the actual start of the session.

So from this moment, the rest of the stack is told to run.

---

## Step 7: action feedback loop

The loop runs at:

* `1 / feedback_hz`

Inside the loop it repeatedly checks:

### A. cancel request

If client requested cancel:

* disable follow
* set session inactive
* publish session_active
* mark action canceled
* return canceled result

### B. current target info

It computes:

* `target_visible`
* `target_age`

Visibility rule:

* target exists
* and `camera_visible`
* and `valid`

### C. publish feedback

It fills `PersonFollow.Feedback` with:

* `active`
* `selected_track_id`
* `target_visible`
* `target_age_sec`
* `facing_robot`
* `stop_reason`

So the action client gets live session updates.

---

## Step 8: check terminal stop condition

This is the critical exit logic.

If:

* `session_active == False`
* and `stop_reason_code` is terminal

then:

* disable follow
* call `_finalize_goal(...)`
* return final action result

If `_finalize_goal()` somehow returns None, it falls back to:

* unsuccessful result with stop reason

This means the action session ends when safety/session logic says it ended.

---

## Step 9: sleep until next feedback cycle

It uses:

* `time.sleep(period)`

This is a simple loop style.

Not the most elegant ROS style, but functional because the node uses a multi-threaded executor and reentrant callback group.

---

## Step 10: finally block

No matter how loop exits:

* `_active_goal_handle = None`

So future goals can be accepted.

---

# 30. Important design behavior of the action loop

The action loop itself does **not** perform the following.

It does not:

* track people
* move the robot
* select best target continuously
* compute safety

It only:

* starts the session
* optionally applies params
* optionally auto-selects nearest
* publishes feedback
* ends the session when safety/session says so

That is exactly the correct role for an action server.

---

# 31. Main design strengths

What is good about this file:

## It cleanly separates control from execution

This node does not try to drive the robot directly.

## It exposes both action and services

Good for both:

* high-level long-running tasks
* quick manual control commands

## It integrates runtime parameter tuning

Very useful for changing follow distance and face-hold behavior without restarting nodes.

## It respects safety as the source of terminal session stop

That is good architecture.

## It supports auto-select nearest

Useful for demo/task startup.

---

# 32. Practical limitations / weaknesses

A few real things to notice.

## 1. `SUCCESS_STOP_CODES` is empty

So action currently never finishes as success due to a stop reason.

That means even a “normal end” would not currently map to success unless you add a success stop code later.

This is one of the most important design limitations in this file.

---

## 2. Target visibility is camera-based

In feedback/status, target visible is defined as:

* `camera_visible and valid`

So if target is maintained through LiDAR-only or prediction, visibility may report false even if selected target still exists.

That may be okay, but you should know what it means.

---

## 3. Auto-select uses nearest track only

No confidence or quality filtering.

So in crowded scenes, nearest track may not always be the person you want.

---

## 4. `time.sleep()` inside execute loop

It works, but it is a bit crude in ROS compared to a more timer/rate-driven approach.

Still acceptable here because of multi-threaded executor.

---

## 5. Only one behavior node parameters are controlled

Currently only:

* `follow_distance_m`
* `face_hold_sec`

So this is not a general runtime tuning server for the whole stack, only a focused one.

---

# 33. Plain-English summary

The easiest way to understand `person_follow_server.py` is:

It is the control center for the person-following session: it lets other nodes or users start a follow action, stop it, choose a person, clear the selection, ask for current status, change a few follow parameters, and receive live feedback until safety or cancellation ends the session.

So in one sentence:

**`person_follow_server.py` is the action/service API layer that manages the lifecycle of a person-follow session.`**

---

# 34. Super short logic flow

Its runtime logic is basically:

client sends action/service request
→ validate and maybe apply params
→ publish follow enable / selected target / session active
→ monitor target, behavior, and safety
→ publish feedback
→ cancel/abort action when session ends

---

# 35. Best mental model

If `safety_supervisor.py` answers:

**“Is the system allowed to continue?”**

then `person_follow_server.py` answers:

**“How do I expose this whole follow system as one controllable session to the outside world?”**

The next small file after this is `person_follow_client.py`, which is just the CLI/testing side of this API.



————————————————————————————————————————————————————————person_follow_client.py————————————————————————————————————————————————————————————————————
Yes — the next one is `person_follow_client.py`.

This file is much smaller than the others.

It is **not part of the main follow logic**.
It is a **CLI/testing client** for `person_follow_server.py`.

So the simplest summary is:

**`person_follow_client.py` is a terminal tool to control and test the person-follow server.`**

---

# 1. What this file is for

This file gives you a command-line way to do things like:

* start a follow session
* stop following
* select a track ID
* clear the selected person
* query current status
* update follow parameters

So instead of writing another ROS node, you can just run this client from terminal.

It is mainly for:

* testing
* debugging
* demos
* manual operator control

---

# 2. What it talks to

This client talks to `person_follow_server.py`.

So it uses the same interfaces exposed by the server:

## Action

* `/person_follow/control`

## Services

* `/person_follow/stop`
* `/person_follow/select_person`
* `/person_follow/clear_person`
* `/person_follow/get_status`
* `/person_follow/set_params`

So this client is basically a human-friendly wrapper around those APIs.

---

# 3. Main structure of the file

The file has 3 main parts:

* `PersonFollowCLI` class
* helper methods for each command
* `main()` that parses CLI arguments and runs the requested command

So it is a normal ROS 2 command-line node.

---

# 4. `PersonFollowCLI.__init__()`

This sets up all ROS clients.

It creates:

## Action client

For:

* `PersonFollow` action on `/person_follow/control`

## Service clients

For:

* stop
* select person
* clear person
* get status
* set params

So as soon as the node starts, it has all the handles needed to talk to the server.

---

# 5. `_wait_cli()`

This is a helper that waits for a service or action server to become available.

It takes:

* a client
* a name
* optional timeout

If the service/server is not ready:

* it waits and logs a message

This is important because ROS services/actions may not be up instantly.

So before sending requests, the CLI makes sure the server is available.

---

# 6. `cmd_start()`

This is the most important command in the client.

It sends a `PersonFollow.Goal` action request.

The goal can include:

* `auto_select_if_needed`
* `follow_distance_m`
* `face_hold_sec`

So from CLI you can do things like:

* start following with defaults
* start following and auto-select nearest person
* start following with a custom follow distance
* start following with a custom face-hold timeout

---

## What `cmd_start()` does

### Step 1

Wait for action server.

### Step 2

Build `PersonFollow.Goal`.

### Step 3

Send goal asynchronously.

### Step 4

Handle whether goal was accepted.

### Step 5

While action runs:

* print feedback from server

Feedback includes things like:

* active
* selected track ID
* target visible
* target age
* facing robot
* stop reason

### Step 6

When action finishes:

* print final result

So `cmd_start()` is how you run a live follow session from terminal.

---

# 7. `_feedback_cb()`

This is the action feedback callback.

It prints or logs the live feedback coming from `person_follow_server.py`.

That feedback usually includes:

* whether session is active
* selected track ID
* whether target is visible
* target age
* whether person is facing robot
* stop reason

So while follow is running, this callback gives you real-time session updates.

---

# 8. `cmd_stop()`

This calls the stop service:

* `/person_follow/stop`

It tells the server to:

* disable follow
* clear selected target
* set session inactive

So this is the terminal stop button.

Use this when you want to end follow immediately.

---

# 9. `cmd_select(track_id)`

This calls:

* `/person_follow/select_person`

with a requested track ID.

So if the tracker currently has, for example:

* track 1
* track 2
* track 3

you can manually tell the system:

* follow track 2

Important:
the server validates that:

* tracks state is fresh
* track ID exists

So this client is only requesting selection, not forcing it blindly.

---

# 10. `cmd_clear()`

This calls:

* `/person_follow/clear_person`

It clears the selected target.

So after this:

* no person is selected anymore

This is useful for resetting the session without fully stopping everything.

---

# 11. `cmd_status()`

This calls:

* `/person_follow/get_status`

and prints the returned follow status.

Typical fields include:

* active
* inhibit_active
* selected_track_id
* target_visible
* target_age_sec
* facing_robot
* behavior state
* safety state
* stop reason

This is one of the most useful commands for debugging, because it quickly tells you what the system currently thinks is happening.

---

# 12. `cmd_set_params()`

This calls:

* `/person_follow/set_params`

It can update:

* `follow_distance_m`
* `face_hold_sec`

So from terminal you can change behavior parameters without restarting nodes.

This is very useful for live tuning.

Example use:

* increase follow distance
* reduce face-hold timeout
* test different interaction behavior

---

# 13. `main()` function

This is where CLI arguments are parsed.

It usually supports subcommands like:

* `start`
* `stop`
* `select`
* `clear`
* `status`
* `set-params`

So usage is roughly:

* start a session
* stop a session
* select a person by track ID
* clear selection
* read current status
* change follow params

This is the operator-facing shell interface.

---

# 14. How this file fits the whole package

The flow is:

## Server side

`person_follow_server.py`
owns the real logic for:

* action handling
* services
* session control

## Client side

`person_follow_client.py`
is just a terminal user of that API

So this file does **not**:

* track people
* move robot
* compute safety
* decide behavior

It only sends requests and prints responses.

---

# 15. Why this file is useful

Even though it is small, it is very useful because it gives you a fast way to test the whole package.

Instead of building a GUI or another node, you can quickly do:

* start follow
* select target
* stop
* query status

That is especially useful during:

* lab testing
* robot bringup
* debugging
* demos

---

# 16. Strengths of this file

What is good about it:

## Simple

Very easy to use for testing.

## Uses official server API

So it tests the real interfaces, not some shortcut.

## Supports both action and services

So you can test:

* long-running follow sessions
* one-shot commands

## Good for debugging

Especially `status`, `select`, and `set-params`.

---

# 17. Limitations

A few things to know:

## It is only a client

If the server is broken, this file cannot fix anything.

## It is terminal-oriented

Not a GUI or rich operator dashboard.

## It depends on track IDs

So to use `select`, you need to know the current available track IDs from the system.

---

# 18. Plain-English summary

The easiest way to understand `person_follow_client.py` is:

It is a command-line remote control for the person-follow server, letting you start and stop follow sessions, choose which tracked person to follow, clear selection, inspect current status, and adjust a few behavior parameters from the terminal.

So in one sentence:

**`person_follow_client.py` is the CLI remote controller for the person-following system.`**

---

# 19. Super short logic flow

terminal command
→ action/service request
→ `person_follow_server.py`
→ response/feedback printed in terminal

---

# 20. Best mental model

If `person_follow_server.py` is the **API layer**,
then `person_follow_client.py` is the **human test console** for that API.

If you want, I can now do one of these next:

1. explain `person_perception.launch.py`
2. explain `person_follow_nav2.launch.py`
3. summarize the entire `person_following` package as one full end-to-end flow


——————————————————————————————————————————————summary——————————————————————————————————————————————————————————————————————————————————————————
Yes — here is the **full end-to-end flow** of the `person_following` package as one complete system.

I’ll explain it like a pipeline from:
**user starts follow**
→ **robot detects person**
→ **robot locks target**
→ **robot follows**
→ **robot handles loss / stop / safety**

---

# 1. What the whole package is trying to do

The `person_following` package is a full ROS 2 subsystem that makes the robot do this:

1. see people with the camera
2. see person-like blobs with LiDAR
3. fuse both into tracked people
4. choose one selected person
5. decide whether to follow, hold, rotate, search, or stop
6. send movement commands through Nav2 or raw `cmd_vel`
7. stop safely if the system becomes unhealthy

So the package is not just a detector.
It is a full **perception → tracking → behavior → execution → safety** stack.

---

# 2. The full node chain

When you use the full launch:

* `person_follow_nav2.launch.py`

the main runtime chain is:

1. `vision_node.py`
2. `follow_person.py`
3. `person_track_manager.py`
4. `target_lock_manager.py`
5. `follow_behavior_manager.py`
6. `nav_follow_interface.py`
7. `safety_supervisor.py`
8. `person_follow_server.py`

That is the full follow-person pipeline.

---

# 3. End-to-end data flow in one line

The cleanest single-line summary is:

**RGB-D + LiDAR**
→ **camera detections + LiDAR candidates**
→ **fused tracks**
→ **stable selected target**
→ **behavior state + motion intent**
→ **Nav2 goals / local rotate / stop**
→ **safety supervision**
→ **session control feedback**

---

# 4. Step 0: user or task starts the session

This is the top of the whole flow.

Usually this happens through:

* `person_follow_server.py`
* optionally from `person_follow_client.py`

The server can:

* enable follow
* optionally auto-select nearest person
* apply follow parameters like follow distance
* mark session active

So the package does not automatically follow all the time.
A session has to be started.

Main control topics:

* `/person_follow/enable`
* `/person_follow/session_active`
* `/person_target_select_id`

---

# 5. Step 1: camera perception with `vision_node.py`

This is the first real sensing stage.

## Inputs

* RGB image
* aligned depth image
* camera info
* locked target topic for GUI overlay

## Processing

`vision_node.py` does:

1. synchronize RGB and depth
2. run YOLO person detection
3. for each person box, compute median depth from center ROI
4. convert pixel + depth into 3D point
5. transform that point into robot tracking frame
6. optionally run RetinaFace for face/head pose
7. estimate whether the person is facing the robot
8. extract appearance cues:

   * torso histogram
   * upper-body histogram
   * lower-body histogram
   * identity quality
   * detection quality
   * image-center motion cue

## Output

* `/cam_detections`
* `/cam_detections_markers`

So after this stage, the package has:
**camera-based 3D person detections with appearance and facing cues**

---

# 6. Step 2: LiDAR person-like candidate extraction with `follow_person.py`

This is the second sensing stage.

## Input

* `/scan`

## Processing

`follow_person.py` does:

1. convert LiDAR ranges into XY points
2. transform points into `base_frame`
3. keep only ROI in front of robot
4. cluster nearby scan points
5. reject clusters too small / too large / too noisy
6. keep short-term candidate memory across frames
7. score each candidate for “person-likeness”

## Output

* `/people_candidates`
* `/people_candidates_markers`

So after this stage, the package has:
**stable LiDAR person-like centroids**

Important:
LiDAR side does not know identity.
It only knows:
“this blob looks like a stable human-sized object.”

---

# 7. Step 3: fusion + multi-person tracking in `person_track_manager.py`

This is the core brain.

It takes the two sensor streams and turns them into tracked people.

## Inputs

* `/cam_detections`
* `/people_candidates`
* latest RGB image for optional ReID crop
* `/person_target_select_id`

## Main processing

### A. Build fused measurements

It combines:

* camera detections
* LiDAR candidates

Possible measurement types:

* `cam_lidar`
* `camera_only`
* `lidar_only`

If camera and LiDAR agree:

* use fused measurement

If only camera exists:

* can still use camera-only

If selected target is briefly missing in camera:

* may create special selected-target LiDAR bridge measurement

---

### B. Predict existing tracks

For each current track:

* predict forward in time using IMM-style motion model

This handles:

* moving
* turning
* slowing / stopping

---

### C. Associate measurements to tracks

This is not simple nearest matching.

It uses:

* geometry
* motion consistency
* appearance similarity
* ReID similarity
* histogram similarity

And it does something very important:

**selected target gets matched first**

So the chosen person is protected before the rest of the multi-target tracker resolves other people.

---

### D. Update matched tracks

For matched tracks it updates:

* position and velocity
* covariance
* track score
* appearance memory
* histogram references
* facing info
* embedding gallery

---

### E. Age unmatched tracks

Tracks that do not match become:

* occluded
* lost
* deleted

depending on how long they are missing.

---

### F. Spawn new tracks

New unmatched good measurements become tentative new tracks.

---

### G. Manage selected target lock

This is the special logic for person-following.

The file tries hard to answer:
**“Is this still my chosen person?”**

It uses:

* hard hold windows
* reacquire windows
* relock cooldown
* contradiction detection
* front-occluder rejection
* crossover detection
* identity gallery
* stable anchor embedding
* bbox/range/bearing history

This is what makes it much stronger than a basic tracker.

## Outputs

* `/person_tracks`
* `/person_tracks_poses`
* `/person_tracks_state`
* `/person_target_lock_debug`
* `/person_track_debug`
* optional direct selected target output

So after this stage, the package has:
**a live set of tracked people plus one internally protected selected target**

---

# 8. Step 4: stable selected-target republish in `target_lock_manager.py`

This is the target stabilization layer.

## Inputs

* `/person_tracks_state`
* `/person_target_lock_debug`

## Processing

This node does:

1. find which track is currently marked `selected=True`
2. debounce fast lock switches
3. apply relock cooldown
4. compute public confidence wrappers:

   * geometry confidence
   * visibility confidence
   * freshness confidence
   * combined confidence
5. decide whether the target is publishable

## Outputs

* `/person_target_locked`
* `/person_target_facing_robot`

So after this stage, the package has:
**one stable public selected target message for the rest of the stack**

This is the point where the complex tracker output becomes a cleaner public target interface.

---

# 9. Step 5: behavior decision in `follow_behavior_manager.py`

Now the system stops thinking about tracking and starts thinking about robot behavior.

## Inputs

* `/person_target_locked`
* `/person_target_facing_robot`
* `/person_follow/enable`
* `/person_follow/safety_status`

## Processing

This node decides current behavior state.

Possible states include:

* `IDLE`
* `WAIT_FOR_TARGET`
* `LOCKING_TARGET`
* `FOLLOWING`
* `FOLLOWING_CAUTIOUS`
* `HOLD_LOCKED`
* `TEMP_OCCLUDED`
* `REACQUIRE`
* `LOCAL_SEARCH`
* `SAFE_STOP`
* `ABORTED`

It uses:

* whether follow is enabled
* whether target exists
* whether target is fresh
* whether target is strong/publishable
* target distance
* target bearing
* whether person is facing robot
* safety inhibit / fault status

## Examples

### If no follow session

→ `IDLE`

### If follow enabled but no usable target

→ `WAIT_FOR_TARGET`

### If target just appeared

→ `LOCKING_TARGET`

### If target is valid and strong

→ `FOLLOWING`

### If target exists but weaker

→ `FOLLOWING_CAUTIOUS`

### If target too close

→ `HOLD_LOCKED`

### If target is near and facing robot

→ `HOLD_LOCKED`

### If target is lost briefly

→ `TEMP_OCCLUDED`

### If target lost longer

→ `REACQUIRE`

### If still not found

→ `LOCAL_SEARCH`

### If safety blocks movement

→ `SAFE_STOP`

### If stop/interaction condition reached

→ `ABORTED`

## Outputs

* `/person_follow/behavior_status`
* `/person_follow/motion_intent`

So after this stage, the package has:
**a high-level decision about what the robot should do next**

---

# 10. Step 6: motion execution in `nav_follow_interface.py`

This node turns behavior into actual movement.

## Inputs

* `/person_target_locked`
* `/person_follow/motion_intent`
* `/person_follow/safety_status`
* `/person_follow/enable`

## Processing by motion mode

### A. `HOLD`

* cancel active Nav2 goal
* publish zero twist

### B. `ROTATE`

* cancel active Nav2 goal
* rotate in place toward target bearing

### C. `FOLLOW_POSE`

* compute standoff point behind the person
* convert that goal into map frame
* send `NavigateToPose` to Nav2

### D. `SEARCH`

* go to last known target map position if available
* otherwise rotate search locally

This is a hybrid design:

* local orientation uses raw `cmd_vel`
* following/search translation uses Nav2

## Extra outputs

This node also publishes health info:

* nav availability
* TF health

## Outputs

* `/person_follow/cmd_vel_raw`
* Nav2 action goals
* `/person_follow/nav_available` or nav2 available topic
* `/person_follow/tf_ok`

So after this stage, the package has:
**real movement commands being executed**

---

# 11. Step 7: safety supervision in `safety_supervisor.py`

This node watches the whole stack.

## Inputs

* `/person_follow/enable`
* `/person_tracks_state`
* `/person_target_locked`
* `/person_follow/behavior_status`
* Nav2 availability topic
* TF health topic
* obstacle inhibit topic

## Processing

It checks:

* is follow enabled?
* is tracker still alive?
* is TF healthy?
* is Nav2 available?
* is obstacle inhibit active?
* does selected target exist?
* is selected target stale?
* did behavior abort?

Then it assigns:

* `HEALTH_OK`
* `HEALTH_WAITING`
* `HEALTH_INHIBITED`
* `HEALTH_FAULT`

and also sets:

* stop reason
* stop source
* inhibit flag

## Outputs

* `/person_follow/safety_status`
* `/person_follow/stop_reason`

So after this stage, the package has:
**one system-level health decision**

Important:
this safety status goes back into behavior and server logic.

So the package has a feedback loop:
tracking/behavior/execution affect safety, and safety can stop behavior/execution.

---

# 12. Step 8: session control in `person_follow_server.py`

This is the operator/API layer on top of everything.

## Inputs

It listens to:

* `/person_tracks_state`
* `/person_target_locked`
* `/person_target_facing_robot`
* `/person_follow/behavior_status`
* `/person_follow/safety_status`

## It provides

### Action

* `/person_follow/control`

### Services

* select person
* clear person
* stop follow
* get status
* set follow params

## Processing

When an action goal comes in, it can:

* enable follow
* optionally auto-select nearest person
* apply new behavior params
* set session active
* monitor safety and behavior continuously
* publish action feedback
* end session if terminal stop reason occurs

## Outputs

* `/person_target_select_id`
* `/person_follow/enable`
* `/person_follow/session_active`

So this node wraps the whole package into a clean session-level API.

---

# 13. The full topic flow from start to motion

Here is the complete topic path in simple order:

## Perception

RGB-D → `vision_node.py` → `/cam_detections`
LiDAR → `follow_person.py` → `/people_candidates`

## Fusion / tracking

`/cam_detections` + `/people_candidates`
→ `person_track_manager.py`
→ `/person_tracks_state`

## Stable selected target

`/person_tracks_state`
→ `target_lock_manager.py`
→ `/person_target_locked`
→ `/person_target_facing_robot`

## Behavior

`/person_target_locked` + facing + follow enable + safety
→ `follow_behavior_manager.py`
→ `/person_follow/behavior_status`
→ `/person_follow/motion_intent`

## Execution

target + motion intent + safety
→ `nav_follow_interface.py`
→ raw `cmd_vel` or Nav2 goal

## Safety

tracker + target + behavior + nav + tf + obstacle
→ `safety_supervisor.py`
→ `/person_follow/safety_status`

## Session control

operator / task manager
→ `person_follow_server.py`
→ enable / select / stop / feedback

---

# 14. Full runtime story: a normal successful follow

Here is the normal case as a story.

## 1. Session starts

User or task sends action goal to `person_follow_server.py`

## 2. Follow enabled

Server publishes:

* follow enable true
* session active true

## 3. Perception begins

Camera sees people, LiDAR sees blobs.

## 4. Tracker creates tracks

`person_track_manager.py` fuses detections and creates person tracks.

## 5. One person becomes selected

Either manually selected or auto-selected nearest.

## 6. Lock stabilizes

`target_lock_manager.py` republishes stable selected target.

## 7. Behavior enters `LOCKING_TARGET`

Short confirmation state.

## 8. Behavior enters `FOLLOWING`

Target is valid, publishable, and strong.

## 9. Motion intent becomes `FOLLOW_POSE`

`nav_follow_interface.py` computes standoff goal behind person.

## 10. Robot moves with Nav2

Robot follows while avoiding obstacles.

## 11. Safety remains `OK`

Session continues.

That is the normal happy path.

---

# 15. Runtime story: brief occlusion

Now the recovery path.

## 1. Person briefly blocked

Camera loses visual contact for a moment.

## 2. Tracker still predicts selected target

`person_track_manager.py` may use:

* prediction
* LiDAR bridge
* selected history

## 3. Target may stay valid briefly

Or behavior may downgrade into `TEMP_OCCLUDED`

## 4. Motion may hold or rotate

Instead of immediately stopping forever.

## 5. If person reappears quickly

Tracker matches back to same person.

## 6. Behavior returns to `FOLLOWING`

Robot continues.

So the package is designed to survive short loss.

---

# 16. Runtime story: longer loss and search

If loss lasts longer:

## 1. Target becomes stale

Behavior moves from:

* `TEMP_OCCLUDED`
  to
* `REACQUIRE`

## 2. If still not found

Behavior moves to:

* `LOCAL_SEARCH`

## 3. Execution enters `SEARCH`

`nav_follow_interface.py`:

* goes to last known target map position
  or
* rotates locally

## 4. If target still not recovered

Safety may eventually report target stale timeout.

## 5. Session stops

Server ends session because terminal stop reason occurred.

So recovery is staged, not immediate failure.

---

# 17. Runtime story: person stops and faces robot

This is the social interaction path.

## 1. Person comes close

Selected target distance becomes small.

## 2. Facing becomes true

From vision/tracker/target-lock path.

## 3. Behavior enters `HOLD_LOCKED`

Robot holds position.

## 4. If this face-hold condition persists

Longer than `face_hold_sec`

## 5. Behavior may enter `ABORTED`

Meaning interaction/session should end.

## 6. Safety sees behavior aborted

Safety returns terminal stop reason.

## 7. Server ends session

Action aborts.

So the package interprets “person close and facing robot for a while” as a possible intentional stop/end signal.

---

# 18. Runtime story: subsystem failure

For example TF failure.

## 1. `nav_follow_interface.py` detects TF bad

Publishes `tf_ok = False`

## 2. `safety_supervisor.py` sees TF unavailable too long

Returns:

* `HEALTH_FAULT`
* reason = TF unavailable timeout

## 3. Behavior receives safety fault

Moves to safe stop style behavior

## 4. Server sees terminal stop code

Ends follow session

So the package is safety-closed-loop, not just open-loop following.

---

# 19. Best mental model for the whole package

The best way to mentally divide the package is into 6 layers.

## Layer 1: sensing

* `vision_node.py`
* `follow_person.py`

## Layer 2: fusion / tracking

* `person_track_manager.py`

## Layer 3: stable selected target interface

* `target_lock_manager.py`

## Layer 4: behavior policy

* `follow_behavior_manager.py`

## Layer 5: motion execution

* `nav_follow_interface.py`

## Layer 6: safety + session control

* `safety_supervisor.py`
* `person_follow_server.py`

That is the full architecture.

---

# 20. Which files are the real “brains”

If I rank the most important intelligence in the package:

## 1. `person_track_manager.py`

This is the biggest brain:

* fusion
* identity memory
* selected-target protection
* reacquire logic

## 2. `follow_behavior_manager.py`

This is the decision brain:

* follow / hold / search / stop

## 3. `nav_follow_interface.py`

This is the execution brain:

* how to physically move

## 4. `safety_supervisor.py`

This is the watchdog brain:

* whether system is allowed to continue

The others support these.

---

# 21. Plain-English full summary

If I explain the entire package in one plain-English story:

The robot first detects people in the camera and person-like blobs in LiDAR. Then it fuses those into tracked people and tries to keep one chosen person locked, even through brief occlusion or crowd crossings. That chosen person is republished as a stable target. The behavior layer then decides whether the robot should follow, hold, rotate, search, or stop. The execution layer turns that decision into either Nav2 goals or local twist commands. Meanwhile a safety supervisor keeps checking whether tracking, TF, Nav2, target freshness, and obstacle conditions are healthy enough. Finally, a session-control server lets an operator or higher-level task start, stop, select, and monitor the whole process.

---

# 22. Shortest possible full-system summary

**`person_following` is a layered ROS 2 subsystem that senses people, tracks and locks one target, decides how the robot should react, executes that motion through Nav2/local control, and supervises the whole session for safety and control.**

If you want, the next best thing is I can make you a **single diagram-style architecture map** of the whole `person_following` package with nodes, topics, and arrows.


————————————————————————————————————————————————————————————diagram architecture——————————————————————————————————————————————————————————————————————
Yes — here is a **diagram-style architecture map** of the whole `person_following` package.

---

# 1. Full node architecture map

```text
                         ┌──────────────────────────────┐
                         │     person_follow_server     │
                         │  action + services control   │
                         └──────────────┬───────────────┘
                                        │
                         /person_follow/enable
                         /person_follow/session_active
                         /person_target_select_id
                                        │
                                        v

 RGB image ───────┐
 Depth image ─────┼────> ┌──────────────────────────────┐
 CameraInfo ──────┘      │         vision_node          │
                         │ YOLO + depth + facing + hist │
                         └──────────────┬───────────────┘
                                        │
                                        v
                                /cam_detections
                                /cam_detections_markers

 LiDAR /scan ─────────────> ┌──────────────────────────────┐
                            │        follow_person         │
                            │ LiDAR clustering + candidates│
                            └──────────────┬───────────────┘
                                           │
                                           v
                                   /people_candidates
                                   /people_candidates_markers


 /cam_detections ───────────┐
 /people_candidates ────────┼────> ┌──────────────────────────────┐
 RGB image (for ReID) ──────┤      │     person_track_manager     │
 /person_target_select_id ──┘      │ fusion + tracking + lock     │
                                    └───────┬─────────┬───────────┘
                                            │         │
                                            │         └────> /person_target_lock_debug
                                            │
                                            └────> /person_tracks_state
                                                   /person_tracks
                                                   /person_tracks_poses
                                                   /person_track_debug


 /person_tracks_state ───────┐
 /person_target_lock_debug ──┼──> ┌──────────────────────────────┐
                             │    │      target_lock_manager     │
                             │    │ stable selected target       │
                             │    └──────────────┬───────────────┘
                             │                   │
                             │                   ├──> /person_target_locked
                             │                   └──> /person_target_facing_robot
                             │

 /person_target_locked ─────────────┐
 /person_target_facing_robot ───────┼──> ┌──────────────────────────────┐
 /person_follow/enable ─────────────┤    │   follow_behavior_manager    │
 /person_follow/safety_status ──────┘    │ behavior state + intent      │
                                          └──────────────┬───────────────┘
                                                         │
                                                         ├──> /person_follow/behavior_status
                                                         └──> /person_follow/motion_intent


 /person_target_locked ─────────────┐
 /person_follow/motion_intent ──────┼──> ┌──────────────────────────────┐
 /person_follow/safety_status ──────┤    │    nav_follow_interface      │
 /person_follow/enable ─────────────┘    │ Nav2 goal / rotate / hold    │
                                          └──────────────┬───────────────┘
                                                         │
                                                         ├──> /person_follow/cmd_vel_raw
                                                         ├──> Nav2 NavigateToPose action
                                                         ├──> /person_follow/nav_available
                                                         └──> /person_follow/tf_ok


 /person_follow/enable ─────────────┐
 /person_tracks_state ──────────────┼──> ┌──────────────────────────────┐
 /person_target_locked ─────────────┤    │      safety_supervisor       │
 /person_follow/behavior_status ────┤    │ watchdog / stop reasons      │
 /person_follow/nav_available ──────┤    └──────────────┬───────────────┘
 /person_follow/tf_ok ──────────────┤                   │
 /person_follow/obstacle_inhibit ───┘                   ├──> /person_follow/safety_status
                                                        └──> /person_follow/stop_reason
```

---

# 2. Flow in plain English

## Layer A: perception

`vision_node.py`

* sees people from RGB-D
* gives 3D positions, facing cue, appearance cues

`follow_person.py`

* sees person-like LiDAR blobs
* gives stable candidate positions

## Layer B: fusion and tracking

`person_track_manager.py`

* fuses camera + LiDAR
* tracks multiple people
* protects selected target from switching

## Layer C: stable selected target

`target_lock_manager.py`

* takes tracker output
* republishes one stable selected target

## Layer D: behavior

`follow_behavior_manager.py`

* decides:

  * wait
  * follow
  * cautious follow
  * hold
  * reacquire
  * search
  * stop

## Layer E: execution

`nav_follow_interface.py`

* turns motion intent into:

  * zero twist
  * rotate in place
  * Nav2 follow goal
  * search goal

## Layer F: safety and control

`safety_supervisor.py`

* checks if stack is healthy enough

`person_follow_server.py`

* lets operator or task start/stop/select/status

---

# 3. Most important topics

If you only remember a few topics, remember these:

## Sensor outputs

* `/cam_detections`
* `/people_candidates`

## Tracker output

* `/person_tracks_state`

## Stable selected target

* `/person_target_locked`
* `/person_target_facing_robot`

## Behavior output

* `/person_follow/behavior_status`
* `/person_follow/motion_intent`

## Execution output

* `/person_follow/cmd_vel_raw`

## Safety output

* `/person_follow/safety_status`
* `/person_follow/stop_reason`

## Session control

* `/person_follow/enable`
* `/person_follow/session_active`
* `/person_target_select_id`

---

# 4. Best way to mentally split the package

You can think of the package as 6 boxes:

```text
[ Camera + LiDAR perception ]
            ↓
[ Fusion + multi-person tracking ]
            ↓
[ Stable selected target ]
            ↓
[ Behavior decision ]
            ↓
[ Motion execution ]
            ↓
[ Safety + session control ]
```

---

# 5. Single-sentence summary

**`person_following` is a layered ROS 2 system that detects people, tracks and locks one target, decides how the robot should react, executes that motion through Nav2/local control, and supervises the whole session for safety and operator control.**

If you want, I can do the next one as a **clean node-topic table** for the whole package.








