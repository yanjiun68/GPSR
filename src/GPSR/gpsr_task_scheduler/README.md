# GPSR Task Scheduler

Section: Deterministic GPSR scheduler front end.

This package parses constrained GPSR commands without an LLM, plans the physical task order, and submits reusable symbolic skills to the existing `robot_yasmin_tasks` executor.

Section: Build.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_ai_interfaces robot_yasmin_tasks gpsr_task_scheduler
```

Section: Dry-run launch.

```bash
source install/setup.bash
ros2 launch gpsr_task_scheduler gpsr_scheduler.launch.py dry_run:=true command_input_mode:=topic
```

Section: Full-system launch.

```bash
source install/setup.bash
ros2 launch gpsr_task_scheduler gpsr_scheduler.launch.py dry_run:=false command_input_mode:=topic
```

Section: Full GPSR scheduler, executor, and CV perception launch.

```bash
source install/setup.bash
ros2 launch gpsr_task_scheduler gpsr_scheduler.launch.py \
  dry_run:=false \
  start_executor:=true \
  start_find_object_pipeline:=true \
  start_count_objects_pipeline:=true \
  start_count_people_pipeline:=true \
  start_find_person_pipeline:=true \
  start_find_person_attribute_pipeline:=true
```

This starts the scheduler, the central GPSR lifecycle manager, `/execute_action_queue`,
`/gpsr/find_object`, `/gpsr/count_people`, `/gpsr/count_objects`, their perception services,
and the transient person/object/pose detector nodes. The transient nodes start
`UNCONFIGURED`; the manager activates only the selected state-machine resource group and
returns it to `UNCONFIGURED` before the queue advances.

Set `enable_depth:=false` when only an RGB camera is available. In that mode FIND_OBJECT
still performs named-waypoint navigation, scanning, and stable visual verification, but it
does not wait for aligned depth or CameraInfo, does not publish a localized object map pose,
and skips the final stand-off approach. The action result reports `approached=false`.

`lifecycle_transition_timeout_sec` defaults to 60 seconds and bounds the entire manager
operation rather than each individual transition. `lifecycle_operation_timeout_margin_sec`
adds a 10-second executor response margin. A timed-out prepare is followed by explicit
cleanup, preventing a late service completion from leaving an unowned ACTIVE group.

Section: Runtime state-machine selection.

```text
/gpsr/command
      |
      v
scheduler parser and planner
      |
      v
ActionStep.action_type
      |
      v
executor dispatcher -> matching SkillSM
      |
      v
lifecycle manager -> configure -> activate -> execute -> deactivate -> cleanup
```

Every pipeline switch defaults to `true`, so the single scheduler launch can accept any
implemented perception command without a restart. `FIND_OBJECT` and `COUNT_OBJECTS` have
independent switches but share one `/gpsr_object_detection_node`; the detector process starts
when either pipeline is enabled. Lifecycle activation still occurs only for the state machine
selected by the executor.

Section: Per-command return policy.

The default scheduler freezes the newest valid `map`-frame `/amcl_pose` when a command is
accepted. The pose is normalized and remains unchanged until that command finishes. If no
fresh valid pose arrives within three seconds, the scheduler freezes the named
`instruction_point` fallback instead.

For an exhaustive `FIND_OBJECT` `NOT_FOUND` result, the scheduler first returns to this
frozen command-start pose and then reports `I could not find the <target>.` through the
normal `SAY` skill. This report is automatic even when the original instruction did not
explicitly request speech. If return navigation fails, speech remains pending until guarded
return recovery succeeds.

```text
WAITING_FOR_COMMAND
        |
        v
CAPTURING_RETURN_POSE -- no valid AMCL pose in 3 s --> named fallback
        |
        v
PLANNING -> EXECUTING -> RETURNING_TO_COMMAND_START
                              | success
                              v
                      REPORTING_RESULT (when requested)
                              |
                              v
                   WAITING_FOR_COMMAND / FINISHED
```

Raw captured poses use Nav2 `/navigate_to_pose`; ordinary semantic locations and the
fallback use `/nav/navigate_named_pose`. Startup navigation is disabled by default. Set
`return_target_mode:=named_location` for the prior named-only policy, or explicitly set
`navigate_to_instruction_point_on_start:=true` to restore startup motion. Dry-run mode does
not wait for AMCL and publishes only the configured return policy.

Section: Primitive-only ActionQueue vocabulary.

Natural-language verbs are parser aliases and never enter `ActionStep.action_type`. The
planner emits only the canonical primitives declared as constants in
`robot_ai_interfaces/msg/ActionStep` and registered in
`robot_yasmin_tasks/config/skill_registry.yaml`. Compound commands such as `take` and
`bring` are expanded into ordered find, pick, navigate, and handover primitives by the
planner. If any required primitive is not a real production-enabled capability, the plan is
retained for diagnostics but rejected with `primitive_unavailable` before queue submission.

Temporary queue aliases (`PICK`, `PLACE`, `GIVE`, `FOLLOW`, `GUIDE`, `ASK`, `REPORT`,
`GREET`, and `RETURN_TO_INSTRUCTION_POINT`) are normalized at executor admission and emit a
deprecation warning. Newly generated scheduler queues use only explicit primitive names.

Name and attribute person searches use separate symbolic actions and lifecycle groups:

```text
selector.type=name
      |
      v
FIND_PERSON -> FIND_PERSON_SKILL_SM

selector.type=pose|gesture|any_person
      |
      v
FIND_PERSON_BY_ATTRIBUTE -> FIND_PERSON_BY_ATTRIBUTE_SM
```

Both person routes are compound search skills. An explicit task location sets
`search_scope=STRICT_LOCATION`; a location-free command sets `search_scope=GLOBAL` and the
executor loops through `find_person_search.yaml`. The planner therefore does not prepend a
separate `NAVIGATE` step to either person-finding action.

`start_find_person_pipeline` controls face identity and verbal confirmation.
`start_find_person_attribute_pipeline` independently controls the shared human detector,
MediaPipe pose detector, and real `/perception/human_poses` attribute evidence. A disabled
route fails lifecycle preparation with `skill_unavailable`; it never uses simulated success.

Section: Required dependent nodes for full execution.

Navigation action servers and any persistent manipulation, identity, or voice dependencies
must be running before non-dry-run execution. The scheduler launch owns the executor,
lifecycle manager, and transient GPSR perception/action servers by default.

Section: Error recovery.

When an initial or post-command return to the Instruction Point fails, the scheduler enters
`ERROR`, waits for the configured delay, and transitions through
`RECOVERING_TO_INSTRUCTION_POINT`. Command admission remains closed until a recovery return
succeeds. Success restores `WAITING_FOR_COMMAND`; failure returns to `ERROR` for another
bounded attempt. Configure this behavior with `enable_error_recovery`,
`error_recovery_max_attempts`, and `error_recovery_retry_delay_sec`. A maximum of zero retries
indefinitely. `executor_goal_response_timeout_sec` controls the executor acknowledgement wait
and defaults to 30 seconds. Dry-run startup always skips physical initial navigation.

Reset an exhausted or unwanted `ERROR` state back to command intake:

```bash
ros2 service call /gpsr/reset_state std_srvs/srv/Trigger "{}"
```

The service accepts only while the scheduler is `ERROR`. It clears stale command ownership
and changes the cycle directly to `WAITING_FOR_COMMAND`; it does not command recovery motion.

Section: YASMIN scheduler-cycle viewer.

```text
INITIALIZING -> GO_TO_INSTRUCTION_POINT -> WAITING_FOR_COMMAND
                                           |
                                           v
                                     PLANNING -> EXECUTING
                                           |          |
                                           |          v
                                           +----> RETURNING_TO_INSTRUCTION_POINT
                                                        | failure
                                                        v
                                                      ERROR
                                                        | /gpsr/reset_state
                                                        v
                                               WAITING_FOR_COMMAND
```

`gpsr_scheduler.launch.py` publishes this supervisory machine as
`GPSR_SCHEDULER_CYCLE` on `/fsm_viewer` by default. Start the web viewer with:

```bash
ros2 run yasmin_viewer yasmin_viewer_node
```

Open `http://localhost:5000`, then select `GPSR_SCHEDULER_CYCLE`. Set
`enable_scheduler_viewer:=false` only when visualization is not wanted.

Section: Dry-run command example.

```bash
ros2 topic pub --once /gpsr/command std_msgs/msg/String "{data: 'count people in the kitchen go to the living room tell me'}"
```

Section: Dry-run service command example.

```bash
ros2 service call /gpsr/schedule_command robot_ai_interfaces/srv/ScheduleCommand "{command_id: 'test_001', instruction: 'count people in the kitchen go to the living room tell me', dry_run: true, execute: false, overall_timeout_sec: 60.0}"
```

Section: Execution service command example.

```bash
ros2 service call /gpsr/schedule_command robot_ai_interfaces/srv/ScheduleCommand "{command_id: 'gpsr_001', instruction: 'count people in the kitchen go to the living room tell me', dry_run: false, execute: true, overall_timeout_sec: 300.0}"
```

Expected dry-run plan:

```text
NAVIGATE(target_location=kitchen)
COUNT_PEOPLE(target_location=kitchen)
NAVIGATE(target_location=living_room)
SAY(text='I counted {result.count} people.')
```

Section: Exhaustive and random scheduler-plan validation.

The validator checks every phrase ordering of the supported task templates and then
checks additional seeded random orderings. For each accepted command it verifies the
task family, physical skill order, linear dependency chain, positive execution
policy, and lossless conversion to an executor-compatible `ActionStep` dictionary.
It does not command robot hardware.

```bash
source install/setup.bash
gpsr_validate_sequences --seed 20260805 --random-cases 1000
```

The default run covers 96 exhaustive permutations (four supported task families ×
24 phrase orders) plus the requested random cases. A nonzero exit status and JSON
failure list identify any command whose plan is not executable by the action queue.

Section: Recipient command example.

```bash
ros2 topic pub --once /gpsr/command std_msgs/msg/String "{data: 'count people in the kitchen go to the living room tell john'}"
```

Expected dry-run plan:

```text
NAVIGATE(target_location=kitchen)
COUNT_PEOPLE(target_location=kitchen)
NAVIGATE(target_location=living_room)
FIND_PERSON(target_person=john, target_location=living_room)
SAY(text='I counted {result.count} people.')
```

Section: Launch parameters.

The scheduler launch additionally exposes `return_target_mode`, `amcl_pose_topic`,
`amcl_pose_max_age_sec`, `amcl_pose_capture_timeout_sec`,
`return_fallback_named_location`, `navigate_to_pose_action`,
`navigate_to_named_pose_action`, and `navigate_to_instruction_point_on_start`. See
`gpsr_scheduler.launch.py --show-args` for the complete perception, voice, executor, and
recovery parameter list.
