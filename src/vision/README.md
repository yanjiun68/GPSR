# Vision Package Notes

## Python Environment

Every executable in `src/vision/scripts` runs with the computer-vision virtual
environment at `~/ai/qwen3v1/.venv` by default. This also applies after CMake
installs the scripts for `ros2 run` and `ros2 launch`.

Use that environment's Python for all vision package installation commands so
that `pip` and the runtime interpreter always match:

```bash
~/ai/qwen3v1/.venv/bin/python -m pip install <package>
```

For a temporary alternative environment, set `VISION_PYTHON` to its Python
executable before starting ROS:

```bash
export VISION_PYTHON=/path/to/another/.venv/bin/python
```

## MediaPipe Human Pose

The `human_skeleton_detection_node.py` node uses MediaPipe Tasks Pose Landmarker Full in `VIDEO` mode.

Install Python dependency if needed:

```bash
~/ai/qwen3v1/.venv/bin/python -m pip install mediapipe
```

Download the Full model into the package:

```bash
bash src/vision/tools/download_pose_landmarker_full_model.sh
```

Build and source:

```bash
colcon build --packages-select putrabot_interfaces vision
source install/setup.bash
```

Launch:

```bash
ros2 launch vision human_skeleton_detection.launch.py
```

Launch with GUI monitoring and pose/gesture TTS:

```bash
ros2 launch vision human_skeleton_detection.launch.py show_gui:=true enable_tts:=true start_voice_tts:=true
```

If MediaPipe reports two people when only one person is visible, first try the post-filter defaults. For strict one-person scenes, force the best single candidate:

```bash
ros2 launch vision human_skeleton_detection.launch.py num_poses:=4 prefer_single_person_mode:=true show_gui:=true
```

Useful inspection commands:

```bash
ros2 topic echo /human_pose/poses --once
ros2 topic echo /human_pose/gestures
ros2 topic echo /perception/human_poses
rqt_image_view /human_pose/debug_image
```

Main topics:

- `/human_pose/poses`: `putrabot_interfaces/msg/HumanPoseArray` with 33 MediaPipe landmarks per detected person.
- `/human_pose/gestures`: JSON GPSR gesture summary with legacy gesture fields plus per-person `left_arm_state`, `right_arm_state`, `raised_arm_state`, `wave_state`, `confidence`, and `timestamp`.
- `/human_pose/debug_image`: `sensor_msgs/msg/Image` with skeleton connections, arm-joint reliability, wrist trajectories, direction changes, confidence, person ID, body pose, and gesture.
- `/perception/human_poses`: compact JSON pose and gesture summary for quick terminal inspection.
- `/perception/human_skeletons`: legacy 17-keypoint compatibility topic for count-people pose sorting.

Main monitoring options:

- `show_gui:=true`: opens a local OpenCV monitor window when `DISPLAY` is available.
- `publish_debug_image:=true`: publishes `/human_pose/debug_image` for `rqt_image_view`.
- `enable_tts:=true`: speaks detected pose and stabilized gesture status through `/voice/tts/say`.
- `start_voice_tts:=true`: starts the local `voice/tts_node.py` service from the launch file.
- `min_usable_landmarks`, `min_torso_landmark_confidence`, `min_pose_bbox_area_ratio`: reject weak pose candidates before publishing.
- `duplicate_iou_threshold`, `duplicate_center_distance_ratio`: suppress duplicate MediaPipe candidates for the same person.
- `prefer_single_person_mode:=true`: publishes only the best pose candidate while keeping MediaPipe `num_poses` configurable.
- `gesture_detection_enabled:=true`: enables RoboCup GPSR gesture classification.
- `gesture_mirror_input:=true`: reverses `pointing_left`/`pointing_right` for mirrored camera feeds only.
- `gesture_stability_frames:=3`: requires a gesture candidate to persist before it is published or spoken.
- `arm_landmark_min_visibility:=0.55`: independently validates each arm's shoulder, elbow, and wrist.
- `raised_arm_vertical_margin:=0.04`, `raised_arm_elbow_margin:=0.08`: configure shoulder-relative raised-arm geometry.
- `wave_window_seconds:=1.5`, `wave_min_horizontal_range:=0.12`, `wave_min_direction_changes:=2`: configure temporal horizontal oscillation.
- `wave_max_vertical_drift:=0.15`, `wave_history_timeout_seconds:=2.0`: reject excessive vertical motion and expire missing-person histories.

## GPSR Qwen World Sensing

The GPSR object detector uses the model-loading and camera-inference pattern from
`/home/yanjiun68/ai/qwen3v1/test_camera.py` through the lifecycle-managed
`gpsr_object_detection_node`. The node is inactive until the GPSR lifecycle
manager starts an object skill such as `FIND_OBJECT` or `COUNT_OBJECTS`.

```text
GPSR planner
    |
    v
FIND_OBJECT / COUNT_OBJECTS lifecycle activation
    |
    v
UnifoLM-ER-1 world sensing
    |
    v
/perception/raw_object_detections
    |
    v
existing GPSR perception and YASMIN skill servers
```

Qwen publishes structured semantic object names and image ROIs through the
existing `putrabot_interfaces/msg/ObjectDetections` contract. The human detector
remains on YOLO and does not share the Qwen object backend.

Build the integrated backend with:

```bash
colcon build --packages-select putrabot_interfaces vision gpsr_task_scheduler
source install/setup.bash
```

The GPSR scheduler launches Qwen object sensing by default. To inspect the
resulting detections during a GPSR skill:

```bash
ros2 topic echo /perception/raw_object_detections
ros2 topic echo /perception/object_detector_ready
```

The Qwen prompt and output contract are installed from
`src/vision/config/qwen_world_prompt.txt`. YOLO remains available by setting the
object detector's `inference_backend` parameter to `yolo` for deployments that
explicitly require that fallback.

### Unified Object Query for FIND_OBJECT and COUNT_OBJECTS

Object characteristics and relations do not create additional GPSR skills. The
planner sends one typed filter list with either `FIND_OBJECT` or
`COUNT_OBJECTS`. `FIND_OBJECT` transports it as `query_json`; `COUNT_OBJECTS`
transports it inside `filter_json`. Each skill publishes the same validated
query on `/perception/object_detector_query`, and the Qwen detector uses it only
to prioritize visible candidates.

```text
FIND_OBJECT(query_json) -----------+
                                   |
COUNT_OBJECTS(filter_json) --------+
        |
        v
/perception/object_detector_query
        |
        v
Qwen candidates + ROI attributes
        |
        +-------------------------------+
        |                               |
        v                               v
local FIND candidate filter/ranker   local COUNT candidate filter/count
        |                               |
        v                               v
FindObject.selection_json            CountObjects.result_json
```

Supported query filters are `object_color`, `shape`, `orientation`,
`size_relation` (`smallest` or `largest`), and image-space
`spatial_relation` (`left_of`, `right_of`, `above`, `below`, `near`, or
`far_from`) with a configured `reference_object`. The local server verifies
relations from ROIs and calculates a 3-by-3 `image_position`; Qwen does not
provide metric distance or final action decisions.

For a count command such as “count the red cups” or “count bottles to the left
of the cup,” the count service dispatches the typed Qwen query and production
launches wait up to `qwen_query_wait_timeout_sec` for the next detector frame.
It counts only temporally confirmed candidates that pass the local label,
attribute, and image-relation checks. `near` and `far_from` use the configurable
`near_relation_distance_ratio` of the image diagonal. The count result includes
the dispatched query, whether a fresh frame arrived, matched ROI attributes, and
their image-relative positions for traceability.

`smallest` and `largest` currently compare image bounding-box area. Use the
RGB-D target-geometry pipeline when a physical-size or metric-distance decision
is required.

## Semantic Target Mesh and Grasp Geometry

The target geometry node is perception-only. It does not create Nav2, MoveIt, IK,
arm, or gripper clients.

```text
/vision/target_object
        |
        v
on-demand Qwen grounding --> YOLOE instance mask
                                  |
aligned RGB + depth + CameraInfo -+
                                  v
                     target-only metric surface
                                  |
                                  v
                 organized triangle mesh + normals
                                  |
                                  v
                    OBB + candidate grasp geometry
                                  |
                 +----------------+----------------+
                 v                v                v
       /vision/target_mask  /vision/grasp_pose  /vision/target_markers
```

The launch file reuses the existing aligned RealSense streams and therefore does
not start a second camera pipeline. Build, source, and launch it after the camera:

```bash
colcon build --packages-select vision
source install/setup.bash
ros2 launch vision target_grasp_geometry.launch.py start_rviz:=true
```

Request or clear a target:

```bash
ros2 topic pub --once /vision/target_object std_msgs/msg/String "{data: 'red cup'}"
ros2 topic pub --once /vision/target_object std_msgs/msg/String "{data: ''}"
```

Inspect the machine-readable state and candidate grasp:

```bash
ros2 topic echo /vision/target_status
ros2 topic echo /vision/grasp_pose --once
```

Defaults are in `config/target_grasp_geometry.yaml`. Qwen is lazy-loaded only
after a non-empty request. YOLOE provides the actual binary object mask; the Qwen
box and point are selection hints only. Point-cloud publication is disabled by
default, while the standard RViz display uses a `TRIANGLE_LIST` polygon surface.
