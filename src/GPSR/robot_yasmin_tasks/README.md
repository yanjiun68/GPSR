# GPSR YASMIN Package

## Voice Command Flow

```text
voice_tts prompt
      |
      v
voice_stt transcript
      |
      v
gpsr_voice_command plans symbolic queue
      |
      v
/execute_action_queue
      |
      v
ActionQueueExecutorSM -> skill state machines
```

TTS is used for prompts and spoken status. STT receives the operator command. The parser only creates symbolic actions; robot execution stays inside deterministic YASMIN skills.

## Build

```bash
cd /home/yanjiun68/workspace/WS1/PutrabotUPM-project
colcon build --packages-select voice_interfaces voice robot_ai_interfaces robot_yasmin_tasks
source install/setup.bash
```

## Run Spoken GPSR

```bash
ros2 launch robot_yasmin_tasks gpsr_executor.launch.py use_local_voice:=true
```

The launch starts `voice_tts`, `voice_stt`, `gpsr_lifecycle_manager`, `gpsr_executor`, and
`gpsr_voice_command`. Every symbolic action is prepared and cleaned through the manager.
Actions with persistent dependencies have an empty lifecycle-node group; transient perception
actions require the managed skill processes from the full runtime below.

Do not run this standalone executor together with `gpsr_scheduler.launch.py` or
`gpsr_managed_runtime.launch.py`: those integrated launches already own the scheduler,
executor, lifecycle manager, and `voice_tts`. The executor launch defaults
`use_local_voice:=false` so it cannot create a second voice owner accidentally.

## Managed FIND_OBJECT Runtime

```bash
ros2 launch putrabot_bringup gpsr_managed_runtime.launch.py
```

The managed runtime starts the scheduler, lifecycle manager, transient perception skills,
and executor. `FIND_OBJECT` is triggered through the `/gpsr/find_object` action. The latest
target-specific status is retained on `/gpsr/find_object/target_status` as JSON in
`std_msgs/msg/String`; it reports the target, search phase, detection and verification
state, confidence, final result code, object ID, room, and viewpoint.

Lifecycle `ACTIVE` is only a transition state, not the perception-readiness barrier. The
executor first navigates to a waypoint from `config/find_object_search.yaml`, then submits a
bounded strict-location observation to `/gpsr/find_object`. Configuration performs one
non-published detector warm-up inference; live readiness is asserted only after the first
real camera inference. The local observation allowance follows the configured confirmation
window (3 seconds by default).

The lifecycle manager uses one 60-second absolute deadline for a complete group operation.
The executor waits an additional 10-second response margin. If prepare remains uncertain,
it queues explicit cleanup for `FIND_OBJECT` and does not abandon lifecycle ownership.

The competition scheduler publishes its command-cycle state on `/gpsr/state`, listens for
human transcripts only in `WAITING_FOR_COMMAND`, and uses `/nav/navigate_named_pose` to
confirm initial arrival at and final return to the configured `instruction_point`.

## Waypoint Search Routes

```text
FIND_PERSON
    -> name-only or name-plus-visible-attribute query
    -> persistent tracked-person observations
    -> mandatory spoken raised-hand request and stable track transition

FIND_PERSON_BY_ATTRIBUTE
    -> deterministic tracked pose/gesture/appearance evidence first
    -> structured VLM fallback only for unresolved visible attributes
```

Both routes load `config/find_person_search.yaml` and own their Nav2 waypoint loop:

```text
STRICT_LOCATION -> requested waypoint -> local person search -> result

The attribute server publishes unresolved candidate tracks on
`/perception/person_vlm_query` only after deterministic matching fails. A VLM adapter
may answer on `/perception/person_vlm_result` with a correlated JSON object such as
`{"request_id":"...","status":"MATCH_FOUND","track_key":"session:4","confidence":0.89}`.
The adapter is perception-only; the server validates the track key and confidence before
persisting the candidate, and no VLM message is connected to a motion interface.

GLOBAL -> kitchen -> living_room -> dining_room -> bedroom -> entrance -> exit
             |             |               |
             +-- NOT_FOUND +---- NOT_FOUND +---- continue until found/exhausted
```

Edit `person_search.global_waypoints` to change the shared route. Navigation failures skip
the unreachable waypoint; perception failure, cancellation, and timeout remain terminal.

`FIND_OBJECT` follows the same navigate-observe-scan pattern. Its global route, scan
headings, navigation timeout, and local observation timeout come from
`config/find_object_search.yaml`; an explicit location creates a one-waypoint strict route.
The `retry.max_not_found_retries` policy repeats one complete route only after a clean
`NOT_FOUND` pass. The production value is `1`, meaning at most two complete passes.
Timeout, cancellation, perception, lifecycle, and navigation failures do not consume this
semantic retry, and `skill_retries.FIND_OBJECT` remains `0` so the executor cannot create a
second full two-pass mission.

Open `docs/find_search_visualizer.html` in a browser for a hardware-free interactive model
of strict/global object and person searches, hidden target positions, and failure branches.

## Standalone FIND_OBJECT Launch Test

Use the dedicated test launch for a hardware-free lifecycle and action check. It includes the
production `find_object_skill.launch.py`, publishes synthetic detector readiness and matching
`bottle` frames, then sends one `/gpsr/find_object` goal:

```bash
ros2 launch robot_yasmin_tasks test_find_object_skill.launch.py
```

The test should report a successful `FindObject` result. To test a real camera and detector instead,
disable the synthetic streams and enable the detector:

```bash
ros2 launch robot_yasmin_tasks test_find_object_skill.launch.py \
  publish_test_detections:=false start_object_detector:=true send_test_goal:=false
```

## GPSR tracked appearance and guest memory

The GPSR launch owns a RAM-only `gpsr_guest_memory_node`. Its YASMIN blackboard
is process-local; every other process must use these ROS interfaces:

```bash
ros2 service call /gpsr/guest_memory/get robot_ai_interfaces/srv/GetGpsrGuest "{memory_id: ''}"
ros2 service call /gpsr/guest_memory/list robot_ai_interfaces/srv/ListGpsrGuests "{}"
ros2 service call /gpsr/guest_memory/clear std_srvs/srv/Trigger "{}"
```

`/gpsr/reset_state` does not clear guest memory. Clearing it is always explicit.
The lifecycle manager activates canonical identity tracking, skeleton perception,
tracked appearance, and the selected person skill server in dependency order.
It cleans them in reverse order after the ActionStep; stored guest snapshots remain
available for subsequent commands and for `EXTRACT_PERSON_INFO`.

Glasses matching is enabled by default. Hat and face-mask matching stay disabled
until a validated positive-class model is added to the tracked appearance model
paths, the scheduler `supported_person_attributes`, and the find server's
`enabled_wearable_predicates` parameter.

For `selector.type: name`, `FindPersonSkillSM` queries
`/appearance/lookup_identity_profile`. A usable profile selects registered SFace
re-identification through `/appearance/recognize_human_id`. A missing or unusable profile
selects candidate-by-candidate verbal confirmation through the existing voice topics. The
question is always `Are you <name>?`; yes/no parsing is deterministic and an unclear answer
gets one retry. A verbally confirmed face is enrolled best-effort through
`/appearance/capture_named_profile`. Enrollment failure is reported but does not discard the
confirmed person. Non-name selectors are rejected by this state machine so they cannot enter
identity resolution accidentally.

The lifecycle `find_person_by_attribute_skill_server` accepts `pose`, `gesture`,
`shirt_color`, enabled positive wearables, `any_person`, and logical-AND combinations. It
waits for fresh `/appearance/tracked_people` observations and requires three matching frames
for the same canonical YOLO track before saving the match. Free-form descriptions and
negative wearables remain unsupported and return `invalid_input`; no real execution path
fabricates a person result.

Runtime interfaces:

- `/appearance/lookup_identity_profile`
  (`receptionist_interfaces/srv/LookupIdentityProfile`)
- `/appearance/recognize_human_id`
  (`receptionist_interfaces/action/RecognizeHumanId`)
- `/appearance/capture_named_profile`
  (`receptionist_interfaces/action/CaptureNamedProfile`)
- `/appearance/current_person_observation` (`std_msgs/msg/String`, JSON candidate state)
- `/voice/tts/say` (`voice_interfaces/srv/SayText`)
- `/voice/stt/start` (`std_msgs/msg/Float32`)
- `/voice/stt/transcript` (`std_msgs/msg/String`)
- `/perception/human_poses` (`std_msgs/msg/String`, JSON pose/gesture summary)

Build and launch the integrated runtime:

```bash
cd /home/yanjiun68/workspace/WS1/PutrabotUPM-project
colcon build --packages-up-to receptionist_interfaces face_detection robot_yasmin_tasks gpsr_task_scheduler
source install/setup.bash
ros2 launch gpsr_task_scheduler gpsr_scheduler.launch.py dry_run:=false start_find_person_pipeline:=true start_find_person_attribute_pipeline:=true start_voice_stt:=true
```

Inspect the mode-selection and candidate interfaces:

```bash
ros2 service type /appearance/lookup_identity_profile
ros2 service call /appearance/lookup_identity_profile receptionist_interfaces/srv/LookupIdentityProfile "{name: Alex}"
ros2 topic echo /appearance/current_person_observation std_msgs/msg/String
ros2 action info /appearance/recognize_human_id
ros2 action info /appearance/capture_named_profile
```

Successful queue results include `person_ref`, `identification_mode_used`,
`profile_registered`, `fallback_reason`, enrollment diagnostics, and the task-local candidate
registry. The normalized blackboard keys are `person_ref`, `target_person`,
`profile_registered`, and `identification_mode_used`.

Manual queue submission is still supported:

```bash
ros2 action send_goal /execute_action_queue robot_ai_interfaces/action/ExecuteActionQueue "{queue: {task_id: 'demo_gpsr', steps: [{action_type: 'SAY', parameters_json: '{\"text\":\"I am ready for GPSR.\"}', timeout_sec: 5.0}]}}"
```

Disable spoken intake for manual testing:

```bash
ros2 launch robot_yasmin_tasks gpsr_executor.launch.py use_voice_command:=false
```

Spoken GPSR command intake waits 12 seconds for a transcript by default. Override the window
when required with `command_input_timeout_sec:=<seconds>`.

Enable optional LLM parsing only after `/llm` is available:

```bash
ros2 launch robot_yasmin_tasks gpsr_executor.launch.py use_llm_parser:=true
```

## Add a Skill

```text
Add action type -> create skill file -> register lifecycle policy -> update dispatcher -> add tests
```

Every canonical executable primitive must have one entry in `config/skill_registry.yaml`.
List transient lifecycle nodes in dependency order and declare `implementation` plus
`production_enabled`. Empty lifecycle lists are valid for persistent real endpoints, but a
missing or simulated implementation must remain disabled in production. The scheduler and
executor both reject disabled primitives. The queue executor always calls manager
`prepare_skill` before dispatch and `cleanup_skill` before committing the result.
