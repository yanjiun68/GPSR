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
ros2 launch gpsr_task_scheduler gpsr_scheduler.launch.py dry_run:=false start_executor:=true start_count_people_pipeline:=true start_count_objects_pipeline:=true
```

This starts the scheduler, the central GPSR lifecycle manager, `/execute_action_queue`,
`/gpsr/find_object`, `/gpsr/count_people`, `/gpsr/count_objects`, their perception services,
and the transient person/object/pose detector nodes. The transient nodes start
`UNCONFIGURED`; the manager activates only the selected state-machine resource group and
returns it to `UNCONFIGURED` before the queue advances.

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

Re-arm recovery after an exhausted attempt budget:

```bash
ros2 service call /gpsr/reset_state std_srvs/srv/Trigger "{}"
```

The service accepts only while the scheduler is `ERROR`. It does not force
`WAITING_FOR_COMMAND`; it clears the recovery attempt budget and requires a verified return
through `RECOVERING_TO_INSTRUCTION_POINT` before command intake reopens.

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

The scheduler launch exposes `use_sim_time`, `dry_run`, `command_input_mode`, `command_topic`, `schedule_command_service`, `start_executor`, `start_count_people_pipeline`, `start_count_objects_pipeline`, `start_voice_tts`, `rgb_topic`, `show_gui`, `enable_count_report_tts`, `yolo_model_path`, `confirmation_window_sec`, `confirmation_required_ratio`, `confirmation_confidence_threshold`, `confirmation_last_seen_timeout_sec`, `confirmation_min_frames`, `instruction_point`, `return_after_completion`, `return_after_failure`, `navigation_action`, `find_object_action`, `find_person_action`, `tts_action`, `default_skill_timeout_sec`, `default_skill_retries`, `enable_navigation`, `enable_perception`, `enable_manipulation`, `enable_tts`, `enable_error_recovery`, `error_recovery_max_attempts`, and `error_recovery_retry_delay_sec`.
