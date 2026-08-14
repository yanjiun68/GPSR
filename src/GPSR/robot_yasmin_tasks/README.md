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
ros2 launch robot_yasmin_tasks gpsr_executor.launch.py
```

The launch starts `voice_tts`, `voice_stt`, `gpsr_lifecycle_manager`, `gpsr_executor`, and
`gpsr_voice_command`. Every symbolic action is prepared and cleaned through the manager.
Actions with persistent dependencies have an empty lifecycle-node group; transient perception
actions require the managed skill processes from the full runtime below.

## Managed FIND_OBJECT Runtime

```bash
ros2 launch putrabot_bringup gpsr_managed_runtime.launch.py
```

The managed runtime starts the scheduler, lifecycle manager, transient perception skills,
and executor. `FIND_OBJECT` is triggered through the `/gpsr/find_object` action. The latest
target-specific status is retained on `/gpsr/find_object/target_status` as JSON in
`std_msgs/msg/String`; it reports the target, search phase, detection and verification
state, confidence, final result code, object ID, room, and viewpoint.

The competition scheduler publishes its command-cycle state on `/gpsr/state`, listens for
human transcripts only in `WAITING_FOR_COMMAND`, and uses `/nav/navigate_named_pose` to
confirm initial arrival at and final return to the configured `instruction_point`.

## Two-Mode FIND_PERSON

For `selector.type: name`, `FindPersonSkillSM` queries
`/appearance/lookup_identity_profile`. A usable profile selects registered SFace
re-identification through `/appearance/recognize_human_id`. A missing or unusable profile
selects candidate-by-candidate verbal confirmation through the existing voice topics. The
question is always `Are you <name>?`; yes/no parsing is deterministic and an unclear answer
gets one retry. A verbally confirmed face is enrolled best-effort through
`/appearance/capture_named_profile`. Enrollment failure is reported but does not discard the
confirmed person. Existing description, pose, gesture, clothing, and any-person selectors
continue through the legacy selection adapter.

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

Build and launch the integrated runtime:

```bash
cd /home/yanjiun68/workspace/WS1/PutrabotUPM-project
colcon build --packages-up-to receptionist_interfaces face_detection robot_yasmin_tasks gpsr_task_scheduler
source install/setup.bash
ros2 launch gpsr_task_scheduler gpsr_scheduler.launch.py dry_run:=false start_find_person_pipeline:=true start_voice_stt:=true
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

Enable optional LLM parsing only after `/llm` is available:

```bash
ros2 launch robot_yasmin_tasks gpsr_executor.launch.py use_llm_parser:=true
```

## Add a Skill

```text
Add action type -> create skill file -> register lifecycle policy -> update dispatcher -> add tests
```

Every dispatcher action type must have one entry in `config/skill_registry.yaml`. List
transient lifecycle nodes in dependency order; use an empty list only for persistent or
simulation-only dependencies. The queue executor always calls manager `prepare_skill` before
dispatch and `cleanup_skill` before committing the result.
