# GPSR Verb/Action Grouping Audit

Audit date: 2026-08-16  
Workspace: `/home/yanjiun68/workspace/WS1/PutrabotUPM-project`  
Method: static, read-only inspection of the current working tree plus direct calls to the deterministic parser/planner. No ROS nodes, hardware, or motion interfaces were launched. The working tree already contained modifications and untracked files; this report evaluates that current state.

## 1. Executive summary

The workspace does **not** currently satisfy the requested invariant across all three layers.

```text
spoken command
      |
      v
main GPSR parser: only FIND / COUNT task families
      |
      v
planner: FIND_OBJECT, FIND_PERSON[_BY_ATTRIBUTE], COUNT_*, plus helper NAVIGATE/SAY
      |
      v
ActionStep.action_type: unrestricted string
      |
      v
executor allow-list + dictionary dispatcher: 19 action identifiers
      |
      +--> real ROS-backed skills: navigation, find, count, speech/dialogue
      |
      `--> simulation-only success: pick/place/give/follow/guide/etc.
```

The dispatcher and its Python allow-list are internally aligned, but the task scheduler exposes only four task types. Thirteen of the fifteen required natural-language examples are rejected before an `ActionQueue` exists. The executor includes `PICK`, `PLACE`, `GIVE`, `FOLLOW`, and `GUIDE`, but the production executor does not replace their default simulation clients, and the lifecycle registry explicitly labels those interfaces `simulation_only`.

The highest-risk issue is therefore not synonym spelling alone. It is a capability-contract gap: queue actions can be accepted and return success without calling the physical manipulation/following system. No P0 wrong-motion path was found in static inspection, because these disconnected actions do not issue physical commands. They can nevertheless break GPSR missions and falsely report completion (P1).

Overall result:

| Layer | Result | Reason |
|---|---|---|
| Scheduler grouping | **MISMATCH** | Only find/count are primary tasks; most reference verbs are unsupported. |
| ActionQueue grouping | **PARTIAL** | Stable Python strings exist, but they are lower-level names and the ROS field is free-form. |
| Dispatcher grouping | **PARTIAL** | Complete for its own 19-string allow-list, but several mappings terminate in simulation-only clients. |
| Skill-SM grouping | **PARTIAL** | Most executor identifiers have an SM wrapper; several lack a real robot contract, and canonical semantic names differ. |

Severity totals in this report: **P0: 0, P1: 4, P2: 7, P3: 3, P4: 1**.

## 2. Relevant packages/files discovered

| Component | Actual package/file | Purpose |
|---|---|---|
| Task scheduler | `src/GPSR/gpsr_task_scheduler/gpsr_task_scheduler/parser.py` | Deterministic command classification and role extraction. |
| Task planner/compiler | `src/GPSR/gpsr_task_scheduler/gpsr_task_scheduler/planner.py` | Converts four task types into `SkillStep` and then `ActionStep`-compatible dictionaries. |
| Scheduler facade/node | `src/GPSR/gpsr_task_scheduler/gpsr_task_scheduler/scheduler.py`, `scheduler_node.py` | Runs parse/plan and builds/publishes the ROS queue goal. |
| Scheduler vocabulary config | `src/GPSR/gpsr_task_scheduler/config/scheduler.yaml` | Known locations, objects, categories, people, timeouts, and executor action name. |
| Alternate/legacy parser | `src/GPSR/robot_yasmin_tasks/robot_yasmin_tasks/gpsr/voice_command_parser.py` | Parses LLM pairs or a few transcript regexes directly into `ActionStepData`; no production caller was found outside tests. |
| ActionQueue schema | `src/GPSR/robot_ai_interfaces/msg/ActionQueue.msg`, `ActionStep.msg` | Ordered ROS action steps and free-form action identifier. |
| Queue action endpoint | `src/GPSR/robot_ai_interfaces/action/ExecuteActionQueue.action` | Top-level queue goal/result/feedback contract. |
| Executor | `src/GPSR/robot_yasmin_tasks/robot_yasmin_tasks/gpsr/action_queue_executor_sm.py` | Lifecycle start/dispatch/cleanup/result/recovery loop and `/execute_action_queue` server. |
| Allowed identifiers | `src/GPSR/robot_yasmin_tasks/robot_yasmin_tasks/gpsr/action_types.py` | Nineteen Python string constants plus a set. |
| Dispatcher | `src/GPSR/robot_yasmin_tasks/robot_yasmin_tasks/gpsr/action_dispatcher_state.py` | Dictionary/factory map from identifier to YASMIN SM. |
| Lifecycle map | `src/GPSR/robot_yasmin_tasks/config/skill_registry.yaml` | Per-action nodes, conflict group, and execution endpoint. |
| Skill SMs | `src/GPSR/robot_yasmin_tasks/robot_yasmin_tasks/skills/` | Thin and compound YASMIN state machines. |
| Lifecycle skill servers | `src/GPSR/robot_yasmin_tasks/robot_yasmin_tasks/ros_servers/` | Find/count/attribute lifecycle action servers. |
| Physical follow stack | `src/NAVIGATION/person_following/person_following/nodes/person_follow_server.py` | Separate `/person_follow/control` `putrabot_interfaces/PersonFollow` action server, not wired to GPSR `FOLLOW`. |
| GPSR launch | `src/GPSR/gpsr_task_scheduler/launch/gpsr_scheduler.launch.py` | Scheduler, executor, lifecycle manager, and supporting pipelines. |

No separate `gpsr_executor` package was found. The executor is the `robot_yasmin_tasks` entry point implemented by `action_queue_executor_sm.py`.

## 3. Current scheduler vocabulary

Actual primary task vocabulary in the launched deterministic scheduler:

```yaml
task_scheduler_actions:
  find_object:
    surface_aliases: [find, locate, look for]
    target_rule: known object or known category
  find_person:
    surface_aliases: [find, locate, look for]
    target_rule: person/man/woman/someone/guest or configured person name
  count_objects:
    surface_aliases: [count, how many]
    target_rule: not classified as people
  count_people:
    surface_aliases: [count, how many]
    target_rule: people/persons/humans
```

Additional words are modifiers rather than primary intents:

- `tell`, `report`, `say`, and `answer` split a search clause, identify a configured report recipient, request reporting, and select an output template. They do not create `COMMUNICATE` or `ANSWER_QUERY` task types.
- `go`, `navigate`, and `return` can help resolve a report location. A standalone `go` command is not a supported task.
- Person attribute aliases include arm/hand-raised variants, poses, shirt colors, and model-gated positive wearables.

Configured closed vocabularies include people `operator`, `john`, and `mary`; **Alex is not configured**. Locations are `kitchen`, `living_room`, `bedroom`, `dining_room`, `entrance`, `operator`, and `instruction_point`; **table is not a configured location**.

The alternate `voice_command_parser.py` has a different vocabulary:

```yaml
legacy_llm_pair_aliases:
  navigate: NAVIGATE
  find_obj: FIND_OBJECT
  find_pers: FIND_PERSON_BY_ATTRIBUTE
  count_people/count_persons/count_humans: COUNT_PEOPLE
  grasp/pick: PICK
  handover: GIVE
  speak/say: SAY
legacy_transcript_patterns:
  go/navigate/move to: NAVIGATE
  how many/count people: [optional NAVIGATE, COUNT_PEOPLE]
  go-to + get/grab/take/pick-up + bring/give/handover: [NAVIGATE, FIND_OBJECT, PICK, NAVIGATE, GIVE]
  find <description> person in <room>: [NAVIGATE, FIND_PERSON_BY_ATTRIBUTE]
```

Static search found this alternate parser imported only by its tests. It is not the scheduler node's parser, so its aliases are not treated as active scheduler support.

## 4. Current ActionQueue vocabulary

`robot_yasmin_tasks/gpsr/action_types.py` defines the queue/executor vocabulary:

```yaml
action_queue_actions:
  - NAVIGATE
  - FIND_OBJECT
  - FIND_PERSON
  - FIND_PERSON_BY_ATTRIBUTE
  - COUNT_OBJECTS
  - COUNT_PEOPLE
  - COMPARE_OBJECTS
  - EXTRACT_PERSON_INFO
  - PICK
  - PLACE
  - DISPOSE_TRASH
  - GIVE
  - FOLLOW
  - GUIDE
  - GREET
  - SAY
  - ASK
  - REPORT
  - RETURN_TO_INSTRUCTION_POINT
```

These are Python string constants, not ROS message constants or an enum. `ActionStep.action_type` is a plain `string`; the executor uppercases incoming identifiers and checks the Python allow-list.

## 5. Current dispatcher vocabulary

The dictionary in `action_dispatcher_state.py` maps exactly the same 19 identifiers:

```yaml
dispatcher_actions:
  NAVIGATE: NavigationSkillSM
  FIND_OBJECT: FindObjectSkillSM
  FIND_PERSON: FindPersonSkillSM
  FIND_PERSON_BY_ATTRIBUTE: FindPersonByAttributeSM
  COUNT_OBJECTS: CountObjectsSkillSM
  COUNT_PEOPLE: CountPeopleSkillSM
  COMPARE_OBJECTS: CompareObjectsSkillSM
  EXTRACT_PERSON_INFO: ExtractPersonInfoSkillSM
  PICK: PickObjectSkillSM
  PLACE: PlaceObjectSkillSM
  DISPOSE_TRASH: DisposeTrashSkillSM
  GIVE: HandoverSkillSM
  FOLLOW: FollowPersonSkillSM
  GUIDE: GuidePersonSkillSM
  GREET: GreetPersonSkillSM
  SAY: SpeechSkillSM
  ASK: DialogueSkillSM
  REPORT: ReportSkillSM
  RETURN_TO_INSTRUCTION_POINT: ReturnInstructionPointSM
```

The translation `GIVE -> HandoverSkillSM` is explicit. All other mappings are direct factory entries. A test asserts that the dispatcher key set equals `ALLOWED_ACTION_TYPES`, which protects internal coverage but not semantic agreement with the scheduler.

## 6. Current robot-skill/state-machine vocabulary

```yaml
robot_skill_actions:
  real_or_ros_backed:
    - NAVIGATE
    - RETURN_TO_INSTRUCTION_POINT
    - FIND_OBJECT
    - FIND_PERSON
    - FIND_PERSON_BY_ATTRIBUTE
    - EXTRACT_PERSON_INFO
    - COUNT_OBJECTS
    - COUNT_PEOPLE
    - SAY
    - ASK
    - GREET
    - REPORT
  simulation_only_in_production_executor:
    - COMPARE_OBJECTS
    - PICK
    - PLACE
    - DISPOSE_TRASH
    - GIVE
    - FOLLOW
    - GUIDE

state_machine_names:
  NAVIGATE: NAVIGATION_SKILL_SM
  RETURN_TO_INSTRUCTION_POINT: RETURN_INSTRUCTION_POINT_SM
  FIND_OBJECT_executor_wrapper: FIND_OBJECT_ACTION_WRAPPER
  FIND_OBJECT_server: FIND_OBJECT_STRATEGY_2B
  FIND_PERSON: FIND_PERSON_SKILL_SM
  FIND_PERSON_BY_ATTRIBUTE_executor_wrapper: FIND_PERSON_BY_ATTRIBUTE_SM
  FIND_PERSON_BY_ATTRIBUTE_server: FIND_PERSON_BY_ATTRIBUTE_SERVER_SM
  EXTRACT_PERSON_INFO_executor_wrapper: EXTRACT_PERSON_INFO_SKILL_SM
  EXTRACT_PERSON_INFO_server: EXTRACT_PERSON_INFO_SERVER_SM
  COUNT_OBJECTS: COUNT_OBJECTS_SKILL_SM
  COUNT_PEOPLE: COUNT_PEOPLE_SKILL_SM
  COMPARE_OBJECTS: COMPARE_OBJECTS_SKILL_SM
  PICK: PICK_OBJECT_SKILL_SM
  PLACE: PLACE_OBJECT_SKILL_SM
  DISPOSE_TRASH: DISPOSE_TRASH_SKILL_SM
  GIVE: HANDOVER_SKILL_SM
  FOLLOW: FOLLOW_PERSON_SKILL_SM
  GUIDE: GUIDE_PERSON_SKILL_SM
  GREET: GREET_PERSON_SKILL_SM
  SAY: SPEECH_SKILL_SM
  ASK: DIALOGUE_SKILL_SM
  REPORT: REPORT_SKILL_SM
```

The simple SMs use `VALIDATE_INPUT -> EXECUTE_CLIENT`. Compound state graphs are explicit for find-object, named-person search, attribute search, and person-info extraction.

## 7. End-to-end command flow

The table below records direct calls to `schedule_command()` using the checked-in `scheduler.yaml`. `MISSING` means the layer is not reached or no implementation was found; it is not inferred.

| Input | Parser/scheduler representation | Canonical intent | ActionQueue | Dispatcher | ROS interface -> skill/SM |
|---|---|---|---|---|---|
| `go to the kitchen` | invalid: `unknown_task` | NAVIGATE desired | **MISSING** | **MISSING** | **MISSING** |
| `find Alex` | classified toward `find_object`, then invalid: `missing_target`; Alex is not a configured person | FIND_PERSON desired | **MISSING** | **MISSING** | **MISSING** |
| `find the coke` | valid `find_object`, target `coke`, global scope | FIND_OBJECT | `FIND_OBJECT` | `FindObjectSkillSM` | `/gpsr/find_object` -> lifecycle `find_object_skill_server` -> `FIND_OBJECT_STRATEGY_2B` |
| `meet Alex` | invalid: `unknown_task` | MEET_PERSON desired | **MISSING** | **MISSING** | **MISSING** |
| `take the coke` | invalid: `unknown_task` | ACQUIRE_OBJECT desired | **MISSING** | **MISSING** | **MISSING** |
| `bring me the coke` | invalid: `unknown_task` | DELIVER_OBJECT desired | **MISSING** | **MISSING** | **MISSING** |
| `deliver the coke to Alex` | invalid: `unknown_task`, `unknown_location: alex` | DELIVER_OBJECT desired | **MISSING** | **MISSING** | **MISSING** |
| `tell Alex the time` | invalid: `unknown_task` | COMMUNICATE desired | **MISSING** | **MISSING** | **MISSING** |
| `talk to Alex` | invalid: `unknown_task`, `unknown_location: alex` | COMMUNICATE desired | **MISSING** | **MISSING** | **MISSING** |
| `count the people in the kitchen` | valid `count_people`, location `kitchen` | COUNT_PEOPLE | `NAVIGATE(kitchen) -> COUNT_PEOPLE` | `NavigationSkillSM -> CountPeopleSkillSM` | `/nav/navigate_named_pose` -> `/gpsr/count_people` -> lifecycle count server -> `/perception/count_people` |
| `count the drinks on the table` | detects `count_objects`/category `drink`, then invalid: `missing_task_location`; `table` is not a location and `on` is not checked by unknown-location diagnostic | COUNT_OBJECTS desired | **MISSING** | **MISSING** | **MISSING** |
| `describe the person` | invalid: `unknown_task` | DESCRIBE_PERSON desired | **MISSING** | **MISSING** | **MISSING** |
| `follow Alex` | invalid: `unknown_task` | FOLLOW_PERSON desired | **MISSING** | **MISSING** | **MISSING** |
| `guide Alex to the kitchen` | invalid: `unknown_task` | GUIDE_PERSON desired | **MISSING** | **MISSING** | **MISSING** |
| `accompany Alex` | invalid: `unknown_task` | ACCOMPANY_PERSON desired | **MISSING** | **MISSING** | **MISSING** |

For supported named-person commands, for example `find john in the living room`, the flow is `find_person/name -> NAVIGATE -> FIND_PERSON -> FindPersonSkillSM -> identity/profile services`. For supported attribute commands, the action is `FIND_PERSON_BY_ATTRIBUTE` and the server owns its lifecycle YASMIN SM and guest-memory save.

## 8. Canonical grouping comparison

The reference model uses high-level intents such as `ACQUIRE_OBJECT`, `DELIVER_OBJECT`, and `COMMUNICATE`. The implementation mixes levels:

```text
reference semantic layer       current queue layer
------------------------       -------------------
ACQUIRE_OBJECT                 PICK
PLACE_OBJECT                   PLACE
DELIVER_OBJECT                 no identifier; GIVE is only handover
COMMUNICATE                    SAY / REPORT (output), ASK (prompt/listen)
ANSWER_QUERY                   no identifier
FOLLOW_PERSON                  FOLLOW
GUIDE_PERSON                   GUIDE
```

`FIND` and `COUNT` specialization is implemented correctly for the subset recognized by the main parser. The remaining reference groups either exist only at the executor layer, use a lower-level action name, or are missing entirely.

## 9. Consistency matrix

| Canonical intent | Scheduler | ActionQueue | Dispatcher | Skill/SM | Status |
|---|---|---|---|---|---|
| NAVIGATE | Helper step only; standalone `go` rejected | `NAVIGATE` | `NavigationSkillSM` | real `/nav/navigate_named_pose` | PARTIAL |
| FIND | `find/locate/look for` specialized | no generic `FIND` | explicit person/object variants | separate real person/object SMs | ALIGNED |
| MEET_PERSON | missing | missing | missing | missing | MISSING |
| ACQUIRE_OBJECT | missing in main scheduler | `PICK` | `PickObjectSkillSM` | simulation-only client | MISMATCH |
| PLACE_OBJECT | missing | `PLACE` | `PlaceObjectSkillSM` | simulation-only client | PARTIAL |
| DELIVER_OBJECT | missing | missing; only low-level `GIVE` | `GIVE -> HandoverSkillSM` | simulation-only handover | MISSING |
| COMMUNICATE | `tell/report/say/answer` only as report modifiers | `SAY`, `REPORT`; no shared intent | separate speech/report SMs | real TTS | MISMATCH |
| ANSWER_QUERY | `answer` treated as report modifier | no answer identifier; `ASK` means ask-and-listen | `ASK -> DialogueSkillSM` | STT/TTS dialogue | MISMATCH |
| GREET_PERSON | missing | `GREET` | `GreetPersonSkillSM` | real TTS wrapper | PARTIAL |
| OFFER | missing | missing | missing | missing | MISSING |
| STORE_INFORMATION | missing | missing | missing | GPSR guest memory exists, but no remember task/skill | MISSING |
| COUNT | count/how-many specialized | `COUNT_OBJECTS`, `COUNT_PEOPLE` | separate count SMs | real lifecycle count servers | ALIGNED |
| DESCRIBE | missing | missing | missing | missing | MISSING |
| FOLLOW_PERSON | missing | `FOLLOW` | `FollowPersonSkillSM` | simulation-only; physical follow stack disconnected | MISMATCH |
| GUIDE_PERSON | missing | `GUIDE` | `GuidePersonSkillSM` | simulation-only | MISMATCH |
| ACCOMPANY_PERSON | missing | missing | missing | missing | MISSING |

Every non-`ALIGNED` result follows from a missing main-parser route, a naming/semantic-level disagreement, or a non-physical skill contract.

## 10. Mismatches

| ID | Severity | Finding | Impact |
|---|---|---|---|
| F-01 | P1 | Main scheduler supports only four task types; 13/15 required phrases produce no queue. | Most GPSR verbs cannot execute. |
| F-02 | P1 | `FOLLOW` and `GUIDE` return simulated success and do not call the available physical follow subsystem or a guide server. | Mission can falsely complete without motion. |
| F-03 | P1 | `PICK`, `PLACE`, `GIVE`, `DISPOSE_TRASH`, and `COMPARE_OBJECTS` use default in-memory clients in the production executor and are registry-marked `simulation_only`. | Manipulation missions can falsely complete. |
| F-04 | P1 | No `DELIVER_OBJECT` parser/compiler/action exists, and no prerequisite expansion exists in the main scheduler. | Bring/deliver commands cannot run. |
| F-05 | P2 | High-level canonical concepts are represented by low-level identifiers (`PICK`, `GIVE`, `FOLLOW`, `GUIDE`). | Upstream intent and executable primitive are conflated. |
| F-06 | P2 | Communication is fragmented into `SAY`, `REPORT`, `ASK`, and `GREET`; `tell` and `answer` are parser modifiers, while `talk` is unsupported. | Similar utterances route inconsistently or fail. |
| F-07 | P2 | `ActionStep.action_type` is a free-form ROS string, while validation constants exist only in Python. | Other producers can send spelling/case/semantic variants; only uppercase normalization is provided. |
| F-08 | P2 | Two parsers have different alias sets and expansion behavior; the more capable legacy parser has no production scheduler caller. | Tests or alternate entry points can disagree with deployed behavior. |
| F-09 | P2 | Unknown named people after `find` fall through to object search and yield `missing_target`, rather than an unknown-person diagnostic. | User correction and future open-vocabulary identity are unreliable. |
| F-10 | P2 | `count ... on the table` recognizes count/category but not `table` as task location; unknown-location detection omits `on`. | Valid surface structure produces an imprecise `missing_task_location` failure. |
| F-11 | P2 | `MEET_PERSON`, `DESCRIBE_*`, `ACCOMPANY_PERSON`, `OFFER`, and `STORE_INFORMATION` are absent at all routing layers. | Reference GPSR intent coverage is incomplete. |
| F-12 | P3 | ROS actions `PickObject`, `PlaceObject`, `Handover`, `FollowPerson`, and `GuidePerson` are defined but not used by the respective GPSR clients. | Interface definitions imply capability that executor wiring does not provide. |
| F-13 | P3 | `SkillStep.skill_name` is copied directly into `ActionStep.action_type`; there is no typed semantic-intent-to-executable-action compiler boundary. | Adding synonyms or compound tasks encourages direct string coupling. |
| F-14 | P3 | The standalone physical `person_following` package exposes a different action type/name (`putrabot_interfaces/PersonFollow`, `/person_follow/control`) than GPSR's unused `robot_ai_interfaces/FollowPerson`. | Duplicate contracts complicate integration and maintenance. |
| F-15 | P4 | Names vary (`NAVIGATE` vs `NAVIGATION_SKILL_SM`, `GIVE` vs `HANDOVER_SKILL_SM`, `SAY` vs `SPEECH_SKILL_SM`). | Readability issue; explicit dispatcher translations prevent lookup ambiguity. |

No P0 issue was demonstrated by static inspection. The critical disconnected clients are non-motion simulations, not mappings that visibly invoke the wrong physical behavior.

## 11. Duplicate aliases

- Main scheduler search aliases: `find`, `locate`, `look for`.
- Main scheduler count aliases: `count`, `how many`.
- Report-clause words: `tell`, `report`, `say`, `answer` all affect output, but are not normalized to a first-class communication intent.
- Legacy LLM-pair parser: `grasp`/`pick -> PICK`; `speak`/`say -> SAY`; `count_people`/`count_persons`/`count_humans -> COUNT_PEOPLE`.
- Legacy bring-pattern parser accepts `get`, `grab`, `take`, `pick up` and terminal `bring`, `give`, `handover`, but only inside one rigid compound regex.
- Navigation has surface `go`/`navigate`/`move` in the legacy parser and action `NAVIGATE`; the deployed main parser supports those only as location/report clues.

There are no duplicate dispatcher keys. The duplication is between parser implementations and semantic/action naming levels.

## 12. Missing mappings

Completely missing end-to-end mappings:

```yaml
missing:
  - MEET_PERSON
  - DELIVER_OBJECT
  - ANSWER_QUERY
  - OFFER
  - STORE_INFORMATION
  - DESCRIBE_PERSON
  - DESCRIBE_OBJECT
  - ACCOMPANY_PERSON
```

Scheduler-to-existing-executor mappings missing:

```yaml
parser_compiler_missing_for_existing_queue_actions:
  - standalone NAVIGATE
  - PICK
  - PLACE
  - DISPOSE_TRASH
  - GIVE
  - FOLLOW
  - GUIDE
  - GREET
  - SAY
  - ASK
  - REPORT
  - COMPARE_OBJECTS
```

Real endpoint mappings missing for existing executor actions:

```yaml
physical_client_missing:
  - PICK
  - PLACE
  - DISPOSE_TRASH
  - GIVE
  - FOLLOW
  - GUIDE
  - COMPARE_OBJECTS
```

## 13. Ambiguous mappings

- `find Alex`: because Alex is not configured, the parser's final generic-find branch chooses `find_object`; validation then removes the request. The command is semantically person-like to a human but is not represented that way in code.
- `answer`: the parser treats it as a reporting marker, while the queue's `ASK` is an ask-and-listen operation. There is no model for answering a query.
- `GIVE`: dispatcher maps it to handover. It may be a valid delivery terminal primitive, but it is not a complete delivery intent.
- `REPORT` and `SAY`: both use TTS and overlap; the main scheduler generates `SAY`, not `REPORT`.
- `FIND_PERSON_BY_ATTRIBUTE` and `FIND_PERSON`: the distinction is explicit and safe, but the planner defaults `find_person` to attribute search unless the selector type is exactly `name`.
- `table`: could be a semantic viewpoint/furniture target or a location. Current config and parser do not define which.

## 14. Follow vs guide assessment

The YASMIN validation contracts preserve the conceptual direction:

```text
FOLLOW: requires target_person
GUIDE:  requires target_person + destination
```

They are separate dispatcher entries and separate SM classes, so no static inversion was found. However, both call methods on `ManipulationClients`; those methods immediately return success and issue no ROS request. The lifecycle registry labels both `simulation_only`.

The workspace contains a real follow stack with `/person_follow/control`, where the robot follows a selected human. GPSR `FOLLOW` does not call it. No comparable physical guide endpoint was found in the audited GPSR path. `ACCOMPANY_PERSON` and leader-resolution logic are **MISSING**.

Conclusion: the names preserve leadership, but behavior is not integrated; status **MISMATCH**, severity P1 for false success.

## 15. Find person vs find object assessment

This is the strongest-aligned semantic area:

```text
find + person word/configured name -> find_person
  selector=name                 -> FIND_PERSON
  selector=attribute/any_person -> FIND_PERSON_BY_ATTRIBUTE

find + remaining target -> find_object -> FIND_OBJECT
```

There is no generic `FIND` dispatcher key, so it cannot accidentally select a perception pipeline at dispatch time. Separate action identifiers, lifecycle resources, action endpoints, and SMs exist.

The remaining defect is unknown-name handling: unconfigured Alex falls through to the object branch rather than producing `unknown_person` or an open-vocabulary person selector. `FIND_PERSON` success means identified/found; no `MEET_PERSON` state adds an interaction-ready/approach contract.

## 16. Bring/deliver assessment

In the production scheduler, both are unsupported and therefore do not normalize to a shared action. `DELIVER_OBJECT` is absent from the allow-list and dispatcher.

The alternate legacy regex groups `bring`, `give`, and `handover` only as terminal words in a rigid phrase and expands:

```text
NAVIGATE -> FIND_OBJECT -> PICK -> NAVIGATE(operator) -> GIVE
```

That expansion is not called by the scheduler node and cannot establish deployed support. `GIVE -> HandoverSkillSM` is an explicit low-level translation but its client is simulation-only. Destination/recipient arguments are therefore not represented by a real canonical delivery contract.

## 17. Talk/tell assessment

`talk` is unsupported. `tell` is not a task; it modifies a supported find/count command by causing optional report navigation, recipient lookup, and `SAY`. For example, supported forms can compile as:

```text
COUNT_PEOPLE -> NAVIGATE(report location) -> FIND_PERSON(recipient) -> SAY(template)
```

There is no shared `COMMUNICATE` representation and no parser route for `tell Alex the time`. `SAY`, `REPORT`, `GREET`, and `ASK` have related voice dependencies but separate dispatcher entries. This is a semantic mismatch, not unnecessary duplicate physical TTS code: the real clients reuse `/voice/tts/say`, while dialogue additionally uses `/voice/stt/start` and `/voice/stt/transcript`.

## 18. Count/describe specialization assessment

Count specialization is aligned for supported targets:

- person terms force `count_people -> COUNT_PEOPLE -> CountPeopleSkillSM -> /gpsr/count_people`;
- other count phrases use `count_objects -> COUNT_OBJECTS -> CountObjectsSkillSM -> /gpsr/count_objects`;
- a known object/category target and known task location are required for object count.

The exact table example fails because `table` is not configured as a location and is not diagnosed through the `on` preposition.

Describe specialization is entirely **MISSING**: no parser task types, queue identifiers, dispatcher entries, ROS action contracts, or describe SMs were found for `DESCRIBE_PERSON` or `DESCRIBE_OBJECT`.

## 19. ActionQueue schema analysis

```yaml
action_queue:
  definition_file: src/GPSR/robot_ai_interfaces/msg/ActionQueue.msg
  step_definition_file: src/GPSR/robot_ai_interfaces/msg/ActionStep.msg
  language: ROS 2 IDL (.msg), consumed primarily from Python
  ROS_interface: robot_ai_interfaces/msg/ActionQueue
  fields:
    ActionQueue: [task_id, steps]
    ActionStep:
      - step_id
      - action_type
      - task_id
      - task_family
      - target_object
      - target_person
      - target_location
      - target_category
      - parameters_json
      - timeout_sec
      - max_retries
      - requires_success
      - requires_data
      - result_variable
  action_identifier_field: ActionStep.action_type
  identifier_representation: free-form string, uppercased and checked against Python constants at executor selection
  argument_storage: typed target strings plus duplicated/arbitrary parameters_json
  result_storage: ExecuteActionQueue.Result.results[] of SkillResult; result_variable stores selected result data in executor blackboard
  dependencies: requires_success[] and requires_data[]
  timeout_support: timeout_sec
  retry_support: max_retries, enforced by executor recovery policy
```

The schema offers good dependency, timeout, retry, and result plumbing. Its key weakness is the untyped action identifier and duplicated argument representation. `parameters_json` can disagree with typed target fields; normalization establishes precedence in Python rather than the ROS contract.

## 20. Dispatcher architecture analysis

The dispatcher is a static dictionary of factories, not `if/elif`, plugin discovery, or dynamic imports. The executor checks allowed action strings, starts lifecycle resources, instantiates/runs the mapped SM, tears resources down, commits the result, and performs bounded recovery.

```text
SELECT_NEXT_ACTION
        |
CHECK_PREREQUISITES
        |
START_SKILL_RESOURCES
        |
DISPATCH_ACTION -- dictionary lookup --> skill YASMIN SM
        |
STOP_SKILL_RESOURCES
        |
STORE_SKILL_RESULT --> completion or recovery
```

Runtime factory overrides provide real ROS clients for navigation, find, count, speech/dialogue, greet/report, and extraction. The untouched defaults remain simulations. The lifecycle registry covers all 19 identifiers, but registry presence does not prove real execution; seven entries explicitly say `simulation_only`.

## 21. State-machine mapping analysis

| Queue action | Package/file | SM | Trigger/result contract | Dependencies |
|---|---|---|---|---|
| NAVIGATE | `robot_yasmin_tasks/skills/navigation_skill_sm.py` | `NAVIGATION_SKILL_SM` | `/nav/navigate_named_pose` (`NavigateNamedPose`) -> `SkillResult` | named-pose server/Nav2 |
| FIND_OBJECT | `skills/find_object_skill_sm.py`, `ros_servers/find_object_skill_server.py` | wrapper + `FIND_OBJECT_STRATEGY_2B` | `/gpsr/find_object` (`FindObject`) | object detection, navigation, TF/depth/costmap |
| FIND_PERSON | `skills/find_person_skill_sm.py` | `FIND_PERSON_SKILL_SM` | identity/profile services and observations -> `SkillResult` | identity resource, TTS/STT fallback |
| FIND_PERSON_BY_ATTRIBUTE | wrapper + `ros_servers/person_attribute_skill_servers.py` | wrapper + server SM | `/gpsr/find_person_by_attribute` -> guest-memory save -> action result | identity tracker, skeleton, tracked appearance, memory |
| EXTRACT_PERSON_INFO | wrapper + same lifecycle server file | wrapper + server SM | `/gpsr/extract_guest_info` -> memory get/save -> action result | tracked appearance, profiles, memory |
| COUNT_OBJECTS | `skills/count_objects_skill_sm.py`, lifecycle count server | `COUNT_OBJECTS_SKILL_SM` | `/gpsr/count_objects` -> `/perception/count_objects` | object detector/count perception |
| COUNT_PEOPLE | `skills/count_people_skill_sm.py`, lifecycle count server | `COUNT_PEOPLE_SKILL_SM` | `/gpsr/count_people` -> `/perception/count_people` | human/skeleton detector/count perception |
| SAY/GREET/REPORT | respective skill files | speech/greet/report SMs | `/voice/tts/say` -> `SkillResult` | local TTS service |
| ASK | `skills/dialogue_skill_sm.py` | `DIALOGUE_SKILL_SM` | TTS plus `/voice/stt/start`, `/voice/stt/transcript` | local TTS/STT |
| RETURN_TO_INSTRUCTION_POINT | `skills/return_instruction_point_sm.py` | `RETURN_INSTRUCTION_POINT_SM` | same named-pose action | named-pose server/Nav2 |
| COMPARE_OBJECTS | `skills/compare_objects_skill_sm.py` | `COMPARE_OBJECTS_SKILL_SM` | in-memory simulated result | none |
| PICK/PLACE/DISPOSE/GIVE | respective skill files | respective simple SM; GIVE uses `HANDOVER_SKILL_SM` | in-memory simulated result | none |
| FOLLOW/GUIDE | respective skill files | separate simple SMs | in-memory simulated result | none; physical follow stack is disconnected |

The dispatcher is the explicit translation layer for naming differences such as `GIVE -> HandoverSkillSM`. Therefore these differences do not cause lookup failure, but they do conflate delivery semantics with a terminal handover primitive.

## 22. Recommended canonical action vocabulary

Use a two-level model so high-level intent is not confused with executable primitives:

```yaml
semantic_intents:
  - NAVIGATE
  - FIND_PERSON
  - FIND_OBJECT
  - MEET_PERSON
  - ACQUIRE_OBJECT
  - PLACE_OBJECT
  - DELIVER_OBJECT
  - COMMUNICATE
  - ANSWER_QUERY
  - GREET_PERSON
  - OFFER
  - STORE_INFORMATION
  - COUNT_PEOPLE
  - COUNT_OBJECTS
  - DESCRIBE_PERSON
  - DESCRIBE_OBJECT
  - FOLLOW_PERSON
  - GUIDE_PERSON
  - ACCOMPANY_PERSON

executable_primitives:
  - NAVIGATE
  - FIND_PERSON
  - FIND_PERSON_BY_ATTRIBUTE
  - FIND_OBJECT
  - PICK_OBJECT
  - PLACE_OBJECT
  - HANDOVER_OBJECT
  - SAY
  - ASK_AND_LISTEN
  - COUNT_PEOPLE
  - COUNT_OBJECTS
  - FOLLOW_PERSON
  - GUIDE_PERSON
```

```text
surface aliases
   |
canonical semantic intent + typed roles
   |
one compiler owns prerequisite expansion
   |
typed executable ActionSteps
   |
one dispatcher entry per real skill contract
```

Recommended alias normalization includes `bring/deliver -> DELIVER_OBJECT`, `talk/tell -> COMMUNICATE`, and `take/pick/pick up/grasp -> ACQUIRE_OBJECT`. `fetch` should resolve from syntax: retrieval-only versus retrieval-and-delivery must not be silently assumed. `ACCOMPANY_PERSON` must require an explicit leader role before compiling to follow or guide.

## 23. Recommended fixes ranked by priority

1. **P1 — Fail closed for non-physical skills.** Do not register production `PICK`, `PLACE`, `GIVE`, `FOLLOW`, `GUIDE`, `DISPOSE_TRASH`, or `COMPARE_OBJECTS` as available until a real client is injected. Simulation must require an explicit launch/test flag and return metadata that cannot be mistaken for physical success.
2. **P1 — Add one canonical intent parser and compiler.** Extend the launched scheduler for the reference verbs, then compile high-level intents to primitives. Do not route production commands through the currently disconnected alternate parser.
3. **P1 — Implement `DELIVER_OBJECT` expansion in one compiler.** Compile source navigation, find, acquire, recipient navigation, and handover with explicit dependency/data edges. Keep `HANDOVER_OBJECT` as a terminal primitive, not a synonym for delivery.
4. **P1 — Connect and verify physical clients.** Wire GPSR follow to the existing physical follow action after reconciling interface types; implement/identify real manipulation and guide endpoints. Add result tests that prove a ROS goal was accepted and completed.
5. **P2 — Establish a typed canonical contract.** Add ROS message constants or a validated enum/code field and a single shared conversion module. Retain strings only as display labels if compatibility requires it.
6. **P2 — Separate communication intents.** Define `COMMUNICATE`, `ANSWER_QUERY`, and `GREET_PERSON` at the semantic layer, compiling them to `SAY`, `ASK_AND_LISTEN`, recipient search, or knowledge lookup as required.
7. **P2 — Correct entity/location diagnostics.** Add unknown-person handling, semantic furniture/viewpoint roles, and `on` to location diagnostics. Decide whether `table` is a waypoint, furniture entity, or search surface.
8. **P2 — Add missing intent contracts.** Implement `MEET_PERSON`, `DESCRIBE_*`, `ACCOMPANY_PERSON`, `OFFER`, and `STORE_INFORMATION` only when success conditions and real endpoints are defined.
9. **P3 — Retire or integrate the alternate parser.** One deployed parser should own aliases; tests should run the same entry point used by `gpsr_scheduler`.
10. **P3/P4 — Normalize names and generated documentation.** Prefer `*_PERSON`/`*_OBJECT` executable identifiers and document every intentional translation.

## 24. Files that would need modification

No source files were modified during this audit. A future implementation would likely touch:

- `src/GPSR/gpsr_task_scheduler/gpsr_task_scheduler/parser.py`: canonical alias and role parsing.
- `src/GPSR/gpsr_task_scheduler/gpsr_task_scheduler/models.py`: typed semantic intent/roles.
- `src/GPSR/gpsr_task_scheduler/gpsr_task_scheduler/planner.py`: single prerequisite-expansion compiler.
- `src/GPSR/gpsr_task_scheduler/config/scheduler.yaml`: people, locations/surfaces, aliases, and timeouts.
- `src/GPSR/robot_ai_interfaces/msg/ActionStep.msg`: typed identifier/constants and clearer argument contract.
- `src/GPSR/robot_ai_interfaces/action/*.action`: reconcile follow/guide/manipulation contracts with real servers.
- `src/GPSR/robot_yasmin_tasks/robot_yasmin_tasks/gpsr/action_types.py`: canonical executable vocabulary.
- `src/GPSR/robot_yasmin_tasks/robot_yasmin_tasks/gpsr/action_dispatcher_state.py`: updated real mappings.
- `src/GPSR/robot_yasmin_tasks/robot_yasmin_tasks/gpsr/action_queue_executor_sm.py`: inject real clients and production fail-closed policy.
- `src/GPSR/robot_yasmin_tasks/config/skill_registry.yaml`: real lifecycle nodes/endpoints only.
- `src/GPSR/robot_yasmin_tasks/robot_yasmin_tasks/ros_clients/manipulation_clients.py`: replace simulation-only behavior with action clients.
- `src/GPSR/robot_yasmin_tasks/robot_yasmin_tasks/skills/*_skill_sm.py`: align validators and names with the chosen primitive contracts.
- `src/NAVIGATION/person_following/person_following/nodes/person_follow_server.py` and/or an adapter: reconcile `/person_follow/control` with GPSR FOLLOW.
- Scheduler, executor, parser, dispatcher, and integration tests under both GPSR packages.

## 25. Commands for user to validate findings

These commands are safe static/unit-level checks; they do not launch hardware:

```bash
cd /home/yanjiun68/workspace/WS1/PutrabotUPM-project

# Show the four deployed scheduler task families.
rg -n "def _detect_task_type|return \"(find|count)_" \
  src/GPSR/gpsr_task_scheduler/gpsr_task_scheduler/parser.py

# Compare the executor allow-list, dispatcher, and lifecycle registry.
sed -n '1,120p' src/GPSR/robot_yasmin_tasks/robot_yasmin_tasks/gpsr/action_types.py
sed -n '30,70p' src/GPSR/robot_yasmin_tasks/robot_yasmin_tasks/gpsr/action_dispatcher_state.py
sed -n '1,120p' src/GPSR/robot_yasmin_tasks/config/skill_registry.yaml

# Confirm which clients the production executor overrides.
sed -n '450,530p' \
  src/GPSR/robot_yasmin_tasks/robot_yasmin_tasks/gpsr/action_queue_executor_sm.py

# Confirm simulation-only methods and the disconnected real follow endpoint.
sed -n '1,90p' \
  src/GPSR/robot_yasmin_tasks/robot_yasmin_tasks/ros_clients/manipulation_clients.py
rg -n "PersonFollow|/person_follow/control" \
  src/NAVIGATION/person_following/person_following/nodes/person_follow_server.py

# Run parser/planner unit tests only.
python3 -m pytest -q src/GPSR/gpsr_task_scheduler/test/test_scheduler_core.py
python3 -m pytest -q src/GPSR/robot_yasmin_tasks/test/test_voice_command_parser.py

# Inspect installed ROS schemas after building/sourcing, without launching nodes.
ros2 interface show robot_ai_interfaces/msg/ActionStep
ros2 interface show robot_ai_interfaces/action/ExecuteActionQueue
ros2 interface show robot_ai_interfaces/action/FollowPerson
ros2 interface show putrabot_interfaces/action/PersonFollow
```

No `USER_ACTION_REQUIRED` item remains: all information needed for this static audit was accessible in the workspace. Installed-interface output would confirm the built overlay version but is not needed to establish the source-level findings.

## Architecture ratings

```yaml
ratings:
  verb_normalization: 2/5
  semantic_consistency: 2/5
  action_queue_consistency: 3/5
  dispatcher_consistency: 4/5
  skill_mapping_consistency: 2/5
  ambiguity_handling: 2/5
  extensibility: 3/5
```

- Verb normalization (2): find/count aliases are deterministic, but most reference verbs and shared synonym groups are absent; the alternate parser disagrees.
- Semantic consistency (2): find/count are good; high-level delivery/acquisition/communication concepts are missing or conflated with primitives.
- ActionQueue consistency (3): one executor vocabulary and good dependency fields exist, but the ROS identifier is free-form and semantic/primitives levels are mixed.
- Dispatcher consistency (4): its dictionary completely covers its allow-list with explicit translations; the lost point is that several mappings are simulation-only.
- Skill mapping consistency (2): every allowed string has an SM class, but seven production paths do not reach real robot interfaces.
- Ambiguity handling (2): conflicting person selectors are rejected safely, but unknown people, table/surface roles, answer/tell, and accompany leadership are unresolved.
- Extensibility (3): factories, lifecycle registry, and message structure are extendable, but duplicated parsers and string coupling increase change cost.

## Final invariant verdict

```text
ONE SEMANTIC CONCEPT          PARTIAL
        |
ONE CANONICAL INTENT          NOT YET
        |
ONE STABLE ACTION IDENTIFIER  PARTIAL (Python constants; ROS string)
        |
ONE EXPLICIT DISPATCH MAP     YES for the current 19 executor strings
        |
ONE CLEAR ROBOT CONTRACT      NO for simulation-only/disconnected skills
```

The current workspace therefore **does not satisfy the invariant end to end**.
