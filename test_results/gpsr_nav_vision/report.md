# GPSR Navigation + Vision Validation Report

Date: 2026-10-15  
Workspace revision: `27c9d5b5`  
Overall result: **FAIL — navigation is operational, but neither vision pattern is fully operational.**

## Environment and launch

- ROS 2 Humble; Gazebo Fortress 6.18.0.
- World: `robocup_home_2025_vision.sdf`, launched through the existing `my_robot_nav home25_nav_all.launch.py` entrypoint in headless mode.
- Robot spawn: `(-2.06, -0.72, yaw 0)`; map: `vision_geometric_reference.yaml`.
- A clean final run used `ROS_DOMAIN_ID=186` and simulation time.
- Gazebo, robot state publisher, AMCL, all required Nav2 lifecycle nodes, odometry, lidar, RGB, depth, camera info, TF, scheduler, executor, lifecycle manager, TTS, and vision lifecycle nodes started.
- `map -> odom -> base_footprint` was available. `gripper_link` was present in robot-state-publisher output.
- RGB/depth/camera-info streams were live. The object detector consumed the simulated RGB stream; no Gazebo model state was used as a perception result.
- GPU: NVIDIA GeForce RTX 4060 Laptop GPU, 8,188 MiB total and 7,589 MiB free before inference.

The first attempted final run was invalidated by a stale earlier Home25 launch on the same ROS domain. It produced duplicate `/clock`, Nav2, and lifecycle nodes. Both launch trees were stopped, and all reported end-to-end evidence below comes from the clean domain-186 rerun.

## Actual execution chain

```text
/gpsr/schedule_command (ScheduleCommand)
  -> gpsr_task_scheduler
  -> /execute_action_queue (ExecuteActionQueue)
  -> NAVIGATE
     -> /nav/navigate_named_pose (NavigateNamedPose)
     -> /navigate_to_pose (nav2_msgs/NavigateToPose)
  -> FIND_OBJECT or COUNT_OBJECTS (only after NAVIGATE terminal success)
     -> /gpsr_lifecycle/prepare_skill
     -> /gpsr/find_object (FindObject), or /gpsr/count_objects (CountObjects)
     -> /gpsr_object_detection_node using current Gazebo camera frames
  -> action-queue result
  -> scheduler service result and GPSR status topics
  -> return NAVIGATE to instruction_point
```

The generated plans contained both mandatory actions and explicit dependencies. Live timestamps proved that perception lifecycle activation began only after Nav2 reported goal success. Perception failures remained failures and were not promoted to task success.

## Automated/static results

| Check | Result | Evidence |
|---|---:|---|
| Scheduler, UI, Home25 simulation, and robot-description tests | PASS | `69 passed in 1.52s` |
| Scheduler tests after confirmation-timeout fix | PASS | `28 passed in 0.89s` |
| Changed packages (`my_robot_nav`, `robocup_home_2025_sim`, `gpsr_task_scheduler`, `robot_yasmin_tasks`) | PASS | all four built successfully |
| Vision package installation of corrected detector | PASS | installed detector resolves `UnifoLM-ER-Flow`; `cmake --install build/vision` completed |
| Vision Python tests under system Python | FAIL | 6 passed, 3 failed because OpenCV 4.10 was compiled for NumPy 1.x while system NumPy is 2.2.6 |
| Vision environment compatibility check | PARTIAL | inference venv has NumPy 1.26.4/OpenCV 4.10 and imports correctly, but does not contain pytest |
| `colcon build --packages-select vision` | FAIL | stale `build/vision/ament_cmake_python/*` directories prevent CMake from creating symlinks; source/install update was completed without deleting build artifacts |

## Runtime results

| ID | Command / condition | Navigation | Vision | Final GPSR result | Status |
|---|---|---|---|---|---:|
| N01 | Direct `living_room` named-pose goal | Nav2 success; robot moved from about `(-2.01,-0.71)` toward `(-2.60,-0.60)` | n/a | `Reached waypoint "living_room"` | PASS |
| P1 | `Go to the living room and find the cola.` | Success, ~9.06 s; perception started afterward | Qwen model loaded and activated, but generated malformed JSON: `Expecting ',' delimiter`; zero detection frames | Accepted, then `partial_success`; `FIND_OBJECT: recoverable_failure`; returned to instruction point | FAIL |
| P2 | `Count the spoons in the dining room.` | Success, ~17.57 s to target `(1.00,1.25)`; perception started afterward | Qwen generated malformed JSON: `Expecting property name enclosed in double quotes`; no fresh detections within 5 s | Accepted, then `partial_success`; `COUNT_OBJECTS: failed`; returned to instruction point | FAIL |
| VU1 | Vision detector unavailable because configured model path did not exist (pre-fix run) | Navigation succeeded | Lifecycle prepare returned `skill_unavailable` | Task failed and returned; no false detection success | PASS (failure handling) |
| IV1 | `Go to the coffee table and find the cola.` | Not started | Not started | Rejected with structured `UNKNOWN_LOCATION` | PASS (failure handling) |
| S01 | P1 immediately followed by P2 | Both commands got their own target and result | Distinct `coke`/find and `spoon`/count requests; no prior result reused | Both returned to instruction point and scheduler returned to `WAITING_FOR_COMMAND` | PASS (state isolation; vision outcomes still failed) |

Direct navigation ended at approximately `(-2.44,-0.62)` for the `(-2.60,-0.60)` target: about 0.162 m position error and about 2 degrees heading error, within the configured Nav2 completion behavior. During P2, the start of the return leg reported `(0.86,1.13)` against target `(1.00,1.25)`, about 0.184 m position error. A precise heading sample was not captured for P2.

P1 took about 67.8 s from command intake through failed-task return. Model load/activation consumed about 24.7 s, and the asynchronous inference error appeared about 16.6 s after the image query. P2 took about 78.3 s through return; target navigation took about 17.6 s and model configuration/activation about 17.1 s.

## Required scenario matrix

The prescribed fixture cases were not relabeled as passes. The normal Home25 world does not expose the official furniture names as navigable waypoints, and real inference is currently unable to publish a valid detection frame.

| Cases | Status | Reason |
|---|---:|---|
| F01–F05 | BLOCKED | `coffee_table`, `shelves`, `office_cabinet`, and `dining_table` are absent from the five-waypoint Home25 navigation file; successful detections are also blocked by malformed Qwen JSON. |
| F06 | BLOCKED | An object-absent fixture would not distinguish absence from the current detector-output failure. |
| F07 | PASS | The pre-fix unavailable-model run propagated `skill_unavailable`; it did not claim success. |
| F08 | NOT EXECUTED | No temporary unreachable waypoint was added because the success-path perception blocker already prevents completion; invalid location rejection was tested separately as IV1. |
| C01–C08 | BLOCKED | Required reproducible quantities/official surface waypoints are not present, and the detector provides no valid frames for independent count comparison. |
| C09 | NOT EXECUTED | Camera-unavailable injection was not needed to establish the active detector failure; live camera availability itself was verified. |
| C10 | NOT EXECUTED | No temporary unreachable navigation fixture was created. |
| C11–C12 | BLOCKED | Occlusion/coverage and deduplication cannot be evaluated until valid instance detections are produced. |

No temporary object scenes were created because the current backend fails before producing even one parseable observation; running the full fixture matrix would not yield meaningful object-accuracy evidence. The original world was left unchanged.

## Object and waypoint alignment

- The expected `robocup_my2025_gpsr_objects/config/object_catalog.json` path is not present. Home25 instead contains `robocup_malaysia_2025_objects.json` with all 20 official objects and categories.
- The Home25 waypoint file contains only `instruction_point`, `living_room`, `kitchen`, `bedroom`, and `dining_room`.
- `object_knowledge_2025.yaml` maps drinks to `living_room`, while food, snacks, and tableware map to `dining_room`.
- This differs from the specified official arrangement: drinks/coffee table, food/shelves, snacks/office cabinet, tableware/dining table.
- The scheduler loads `ObjectKnowledge.default_path()` rather than receiving the Home25 knowledge file explicitly, so deployment knowledge alignment needs a dedicated correction before official commands can be accepted.

## UI and observability

- Runtime topics exist for `/gpsr/task_plan`, `/gpsr/action_queue_status_json`, `/gpsr/state`, and `/gpsr/command_result`.
- The action queue and scheduler service returned the current action outcomes and exact failure reasons for both live commands.
- The `gpsr_sys_ui` automated tests are included in the 69-test passing suite.
- A live listener on `/gpsr/command_result` received no message for the planning-time `UNKNOWN_LOCATION` rejection during the observation window; the service response did contain the structured failure. Thus service clients can display it, but topic-only UI consumers may miss planning rejections.

## Minimal fixes applied

1. `gpsr_task_scheduler/scheduler_node.py`: increased the mandatory confirmation exchange timeout from 6 s to 20 s. Real Piper playback consumed the original window before the four-second listening period began. The fixed flow completed SAY and ASK_AND_LISTEN successfully in both clean live commands.
2. `vision/scripts/gpsr_object_detection_node.py`: changed the stale default model path from nonexistent `UnifoLM-ER-1` to installed `UnifoLM-ER-Flow`. The corrected checkpoint loaded on the GPU and reached ACTIVE in both tests.

Neither fix fabricates perception output or changes the action interfaces.

## Remaining defects and assessment

1. **Blocking:** UnifoLM-ER-Flow often emits invalid JSON for the current world-sensing prompt. `parse_qwen_detections()` correctly fails closed, but no detection frame is published, so find/count cannot succeed.
2. **Blocking:** official furniture-level commands cannot resolve against the current Home25 waypoint vocabulary.
3. **High:** object/waypoint knowledge is not aligned with the official category-location mapping.
4. **Medium:** loading and cleaning the large detector for every command adds roughly 17–25 s and allows the skill timeout to occur while inference is still running.
5. **Medium:** the vision test environment has incompatible system NumPy/OpenCV versions, and the dedicated inference venv lacks pytest.
6. **Medium:** the vision package build tree contains stale Python install artifacts that block a normal incremental colcon build.
7. **Low/observability:** planning rejection was returned by the scheduling service but not observed on `/gpsr/command_result`.

Final assessment:

- Navigate → Find Object: **FAIL / not fully operational**. Correct sequence and navigation pass; real visual identification does not.
- Navigate → Count Objects: **FAIL / not fully operational**. Correct sequence and navigation pass; no valid instances are available to count.
- Failure propagation, return-to-instruction-point behavior, and consecutive-command state isolation: **PASS for the exercised paths**.

## Reproduction commands

```bash
cd /home/yanjiun68/workspace/WS1/PutrabotUPM-project
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=186

ros2 launch my_robot_nav home25_nav_all.launch.py \
  start_sim:=true headless:=true use_rviz:=false

ros2 launch gpsr_task_scheduler gpsr_scheduler.launch.py \
  use_sim_time:=true dry_run:=false start_executor:=true \
  start_find_object_pipeline:=true start_count_objects_pipeline:=true \
  start_count_people_pipeline:=false start_find_person_pipeline:=false \
  start_find_person_attribute_pipeline:=false start_voice_tts:=true \
  start_voice_stt:=false start_voice_command:=false enable_tts:=true \
  enable_scheduler_viewer:=false rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  camera_info_topic:=/camera/color/camera_info show_gui:=false \
  return_after_completion:=false

# In a third terminal, provide the operator confirmation transcript.
ros2 topic pub -r 1 /voice/stt/transcript std_msgs/msg/String '{data: confirm}'

ros2 service call /gpsr/schedule_command \
  robot_ai_interfaces/srv/ScheduleCommand \
  "{command_id: 'find_01', instruction: 'Go to the living room and find the cola.', dry_run: false, execute: true, overall_timeout_sec: 240.0}"

ros2 service call /gpsr/schedule_command \
  robot_ai_interfaces/srv/ScheduleCommand \
  "{command_id: 'count_01', instruction: 'Count the spoons in the dining room.', dry_run: false, execute: true, overall_timeout_sec: 240.0}"
```

Detailed ROS logs from this run were written under `/tmp/putrabot-gpsr-clean-sim`, `/tmp/putrabot-gpsr-clean-scheduler`, and `/tmp/putrabot-gpsr-clean-client` (temporary paths).
