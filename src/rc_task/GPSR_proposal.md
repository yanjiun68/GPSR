# GPSR Proposal: LLM Planner + SMACC2 High-Level Orchestration + Behavior Tree Skill Recovery

## 1. Purpose

This document proposes a practical software architecture for a RoboCup@Home GPSR-style robot system that can respond to random, spontaneous human commands.

The proposed design combines:

1. **LLM Planner** for natural language understanding and high-level task planning.
2. **SMACC2 high-level task orchestration** for explicit state control, event tracing, and visualization.
3. **Behavior Tree / BT-style recovery** for fallback behavior inside individual robot skills.
4. **ROS 2 skill modules** for navigation, perception, manipulation, speech, memory, and safety.

The key design principle is:

```text
LLM decides what should be done.
SMACC2 controls what phase the robot is currently executing.
Behavior Tree logic decides what to try when a skill fails.
ROS 2 modules perform the actual robot actions.
```

The LLM must **not** directly control the robot's motors, arm, or `/cmd_vel`. It should only generate structured plans that are validated before execution.

---

## 2. Background and Motivation

General Purpose Service Robot tasks require a robot to understand and execute commands involving different robot abilities such as:

- Task planning
- Object detection and recognition
- Person detection and recognition
- Object feature recognition
- Object manipulation
- Navigation
- Natural human-robot interaction

In RoboCup@Home GPSR-like tasks, the command order is not fixed. For example:

```text
"Go to the kitchen, find the coke, and bring it to the person sitting on the couch."
```

Another command may be:

```text
"Find John, ask what drink he wants, then bring it from the kitchen."
```

A purely hard-coded flat state machine is too rigid for this because the sequence of actions changes depending on the command. An LLM can help interpret flexible language and produce a high-level task plan. However, the LLM is not reliable enough to directly execute robot behavior. Therefore, it should be constrained by a skill registry, a validator, and a deterministic execution layer.

---

## 3. Main Design Philosophy

### 3.1 Separation of Responsibility

The system should be divided into clear layers:

| Layer | Responsibility |
|---|---|
| LLM Planner | Understand command and generate structured task plan |
| Plan Validator | Check whether the generated plan is safe, valid, and executable |
| SMACC2 Task Executive | Execute the plan step-by-step with clear state transitions |
| BT-style Skill Recovery | Handle retry, fallback, and recovery inside individual skills |
| ROS 2 Skill Modules | Provide concrete robot capabilities through actions, services, and topics |
| Hardware Layer | Base, arm, gripper, cameras, LiDAR, microphone, speaker |

### 3.2 Why Not Let the LLM Control Everything?

The LLM is useful for:

- Understanding flexible human commands
- Decomposing tasks into steps
- Asking clarification questions
- Choosing a high-level strategy
- Explaining what the robot is doing

But the LLM is weak for:

- Real-time control
- Guaranteed safety
- Exact robot state tracking
- Deterministic execution
- Timing-critical behavior
- Direct sensor interpretation
- Motor control

Therefore, the LLM should be treated as a **planner**, not a controller.

---

## 4. Overall System Architecture

```text
┌────────────────────────────────────────────┐
│ Human Operator                              │
│ Example: "Bring me the coke from kitchen"  │
└─────────────────────┬──────────────────────┘
                      │
                      v
┌────────────────────────────────────────────┐
│ ASR / Speech-to-Text                        │
│ Converts spoken command into text           │
└─────────────────────┬──────────────────────┘
                      │
                      v
┌────────────────────────────────────────────┐
│ LLM Planner                                 │
│ Converts text command into JSON plan         │
└─────────────────────┬──────────────────────┘
                      │
                      v
┌────────────────────────────────────────────┐
│ Plan Validator                              │
│ Checks skill availability, safety, world map │
└─────────────────────┬──────────────────────┘
                      │
                      v
┌────────────────────────────────────────────┐
│ SMACC2 High-Level Task Executive            │
│ Executes each validated plan step            │
└─────────────────────┬──────────────────────┘
                      │
                      v
┌────────────────────────────────────────────┐
│ BT-style Skill Recovery Layer               │
│ Retry, fallback, ask help, skip, recover     │
└─────────────────────┬──────────────────────┘
                      │
                      v
┌────────────────────────────────────────────┐
│ ROS 2 Skill Servers                         │
│ Nav2, MoveIt, Vision, Voice, Memory, Safety │
└─────────────────────┬──────────────────────┘
                      │
                      v
┌────────────────────────────────────────────┐
│ Robot Hardware                              │
│ Base, arm, sensors, microphone, speaker      │
└────────────────────────────────────────────┘
```

---

## 5. Component Roles

## 5.1 LLM Planner

The LLM Planner receives a natural language command and converts it into a structured task plan.

Example input:

```text
"Go to the kitchen, find the red cup, and bring it to John."
```

Example LLM output:

```json
{
  "task_type": "fetch_and_deliver",
  "steps": [
    {
      "skill": "navigate_to",
      "target_location": "kitchen"
    },
    {
      "skill": "find_object",
      "object": "cup",
      "attributes": ["red"]
    },
    {
      "skill": "pick_object",
      "object": "cup"
    },
    {
      "skill": "find_person",
      "person_name": "John"
    },
    {
      "skill": "handover_object",
      "object": "cup"
    }
  ]
}
```

The LLM output must be machine-readable and constrained. Free-form natural language plans should be avoided because they are difficult to validate.

### LLM Planner Responsibilities

- Parse command intent.
- Extract objects, people, locations, and constraints.
- Generate ordered task steps.
- Ask clarification when the command is ambiguous.
- Replan only when given summarized feedback from the executor.

### LLM Planner Restrictions

The LLM must not:

- Publish velocity commands directly.
- Directly control the arm or gripper.
- Call arbitrary ROS 2 services.
- Invent unavailable skills.
- Ignore the robot's safety limits.

---

## 5.2 Plan Validator

The Plan Validator is a safety gate between the LLM and the robot executor.

It checks whether the LLM-generated plan is valid before the robot moves.

```text
┌──────────────────────────────┐
│ LLM JSON Plan                 │
└───────────────┬──────────────┘
                v
┌──────────────────────────────┐
│ Plan Validator                │
├──────────────────────────────┤
│ Check skill exists            │
│ Check object is allowed       │
│ Check location is known       │
│ Check person reference valid  │
│ Check manipulation is safe    │
│ Check recovery option exists  │
│ Check command is competition-safe │
└───────────────┬──────────────┘
                v
        Valid plan or clarification request
```

### Validation Rules

The validator should check:

1. **Skill existence**
   - Every requested skill must exist in the robot's skill registry.

2. **Known location**
   - Example: `kitchen`, `living_room`, `dining_table`, `instruction_point`.

3. **Known object or object category**
   - Example: `cup`, `coke`, `bottle`, `snack`, `fruit`.

4. **Safe manipulation**
   - The robot should not attempt actions outside its manipulation capability.

5. **Allowed human interaction**
   - Human assistance should be requested only as a fallback, not as the default solution.

6. **Time feasibility**
   - Long plans should be shortened if the test duration is limited.

7. **Ambiguity handling**
   - If the command contains unclear references like `it`, `there`, or `him`, the robot should ask clarification.

---

## 5.3 SMACC2 High-Level Task Executive

SMACC2 should be used as the high-level task orchestration layer.

It is responsible for:

- Managing the execution state.
- Running one validated plan step at a time.
- Receiving success/failure/timeout events from skills.
- Handling pause, cancel, timeout, emergency, and recovery events.
- Providing strong visualization of the robot's current task phase.

### Proposed SMACC2 Plan Executor

```text
WAIT_FOR_COMMAND
      │
      v
TRANSCRIBE_COMMAND
      │
      v
GENERATE_LLM_PLAN
      │
      v
VALIDATE_PLAN
      │
      v
EXECUTE_PLAN
      │
      v
REPORT_RESULT
      │
      v
RETURN_TO_INSTRUCTION_POINT
```

### SMACC2 Execution Detail

Inside `EXECUTE_PLAN`, the state machine can execute each plan step sequentially:

```text
EXECUTE_STEP_N
├── EvSkillSuccess  → EXECUTE_STEP_N_PLUS_1
├── EvSkillFailure  → RECOVERY_DECISION
├── EvSkillTimeout  → RECOVERY_DECISION
├── EvUnsafe        → SAFE_STOP
└── EvPlanComplete  → REPORT_RESULT
```

### Why SMACC2 Is Useful Here

SMACC2 provides strong advantages for RoboCup debugging:

- Clear visualization of active state.
- Explicit event transitions.
- Good support for orthogonals.
- Good separation between navigation, voice, vision, manipulation, and safety clients.
- Easier diagnosis during competition runs.

Example orthogonals:

```text
┌──────────────────────────────────────┐
│ State: ExecuteSkillStep               │
├──────────────────────────────────────┤
│ OrNavigation  → Nav2 action client     │
│ OrManipulation → MoveIt / arm client   │
│ OrVision      → object/person detector │
│ OrVoice       → ASR/TTS client         │
│ OrMemory      → world model client     │
│ OrSafety      → emergency monitor      │
└──────────────────────────────────────┘
```

---

## 5.4 Behavior Tree / BT-Style Recovery Layer

Behavior Tree logic is best used inside skills where fallback behavior is needed.

For example, `find_object` should not be a single fragile action. It should include recovery:

```text
Fallback: FindObject
├── DetectObjectOnVisibleTable
├── SearchNearbySurfaces
├── NavigateToDefaultObjectLocation
├── AskHumanForObjectLocation
└── ReportObjectNotFound
```

Similarly, `pick_object` should have fallback:

```text
Fallback: PickObject
├── TryTopGrasp
├── TrySideGrasp
├── MoveBaseCloserAndRetry
├── AdjustArmHeightAndRetry
├── AskHumanForHandoverIfAllowed
└── ReportPickFailure
```

### Why Not Put Every Fallback Directly in SMACC2?

SMACC2 can express fallback behavior, but doing this for every small behavior can create transition explosion.

Example problem:

```text
NavigateToKitchen
├── success
├── failure
├── timeout
├── retry
├── low_battery
├── localization_lost
├── obstacle_blocked
└── emergency_stop
```

If every state has many transitions, the high-level state machine becomes too large.

Therefore:

```text
Use SMACC2 for high-level task phase.
Use BT-style fallback inside individual skills.
```

---

## 6. Proposed Skill Registry

The LLM should only be allowed to use skills from a fixed registry.

### 6.1 Navigation Skills

```yaml
navigation_skills:
  - navigate_to(location)
  - navigate_to_person(person_id)
  - return_to_instruction_point()
  - search_room(room)
  - follow_person(person_id)
  - stop_navigation()
```

### 6.2 Perception Skills

```yaml
perception_skills:
  - find_object(object, attributes)
  - find_person(name_or_description)
  - detect_empty_seat()
  - detect_gesture(gesture_type)
  - verify_object_in_gripper(object)
  - verify_person_present(person_id)
```

### 6.3 Manipulation Skills

```yaml
manipulation_skills:
  - pick_object(object)
  - place_object(target_location)
  - handover_object(object)
  - receive_object_from_human(object)
  - open_door_if_supported()
  - place_next_to_similar_object(object)
```

### 6.4 HRI Skills

```yaml
hri_skills:
  - speak(text)
  - listen_for_command()
  - ask_clarification(question)
  - confirm_understanding(summary)
  - request_human_help(help_type, desired_outcome)
  - thank_human()
```

### 6.5 Memory and World Model Skills

```yaml
memory_skills:
  - store_person(name, attributes)
  - store_object_location(object, location)
  - query_person(name_or_description)
  - query_object_location(object)
  - update_world_state(key, value)
  - clear_task_memory()
```

### 6.6 Safety Skills

```yaml
safety_skills:
  - check_emergency_stop()
  - check_collision_risk()
  - check_battery_level()
  - safe_stop()
  - request_referee_intervention_if_needed()
```

---

## 7. Execution Feedback Loop

The LLM should not receive raw sensor data. It should receive summarized skill feedback.

Bad design:

```text
Camera frames → LLM
LiDAR scans → LLM
Joint states → LLM
```

Better design:

```text
Vision node → "red cup detected on dining table, confidence 0.82"
Navigation → "reached kitchen"
Manipulator → "grasp failed because object is out of reach"
Memory → "John was last seen near couch"
```

### Feedback Message Example

```json
{
  "step": "find_object",
  "target": "coke",
  "status": "failure",
  "reason": "object_not_seen",
  "searched_locations": ["kitchen_table", "counter"],
  "available_recovery": ["search_cabinet", "ask_human"]
}
```

The LLM may then select the next high-level strategy:

```json
{
  "decision": "continue_with_recovery",
  "next_step": {
    "skill": "ask_clarification",
    "question": "I cannot find the coke on the kitchen table. Do you know where it is?"
  }
}
```

---

## 8. Example End-to-End Execution

### Human Command

```text
"Find the person wearing a blue shirt and bring him a drink from the kitchen."
```

### LLM Plan

```json
{
  "task_type": "serve_person",
  "steps": [
    {
      "skill": "find_person",
      "description": "wearing blue shirt"
    },
    {
      "skill": "store_target_person",
      "key": "recipient"
    },
    {
      "skill": "navigate_to",
      "target_location": "kitchen"
    },
    {
      "skill": "find_object",
      "object_category": "drink"
    },
    {
      "skill": "pick_object",
      "object_category": "drink"
    },
    {
      "skill": "navigate_to_person",
      "person_key": "recipient"
    },
    {
      "skill": "handover_object"
    }
  ]
}
```

### SMACC2 State Flow

```text
WAIT_FOR_COMMAND
→ TRANSCRIBE_COMMAND
→ GENERATE_LLM_PLAN
→ VALIDATE_PLAN
→ EXECUTE_STEP_0_FIND_PERSON
→ EXECUTE_STEP_1_STORE_TARGET
→ EXECUTE_STEP_2_NAVIGATE_TO_KITCHEN
→ EXECUTE_STEP_3_FIND_DRINK
→ EXECUTE_STEP_4_PICK_DRINK
→ EXECUTE_STEP_5_NAVIGATE_TO_PERSON
→ EXECUTE_STEP_6_HANDOVER
→ REPORT_SUCCESS
→ RETURN_TO_INSTRUCTION_POINT
```

### Skill-Level Recovery Example

```text
FindPerson failure:
├── rotate and scan room
├── navigate to another likely room
├── ask person to wave
├── ask operator for clarification
└── report failure safely

PickObject failure:
├── retry top grasp
├── retry side grasp
├── move base closer
├── choose easier object from same category
├── ask human for help if allowed
└── report failure safely
```

---

## 9. Recommended ROS 2 Package Layout

```text
putrabot_gpsr/
├── README.md
├── config/
│   ├── skill_registry.yaml
│   ├── gpsr_locations.yaml
│   ├── gpsr_objects.yaml
│   ├── llm_prompts.yaml
│   └── validator_rules.yaml
├── launch/
│   ├── gpsr_system.launch.py
│   ├── llm_planner.launch.py
│   ├── gpsr_skill_servers.launch.py
│   └── sm_gpsr.launch.py
├── putrabot_gpsr/
│   ├── llm_planner_node.py
│   ├── plan_validator_node.py
│   ├── plan_blackboard_node.py
│   ├── skill_feedback_router.py
│   └── json_schema.py
├── behavior_trees/
│   ├── find_object.xml
│   ├── pick_object.xml
│   ├── find_person.xml
│   ├── navigate_with_recovery.xml
│   └── handover_object.xml
├── sm_gpsr/
│   ├── include/
│   ├── src/
│   │   ├── sm_gpsr.cpp
│   │   ├── states/
│   │   └── orthogonals/
│   └── CMakeLists.txt
└── test/
    ├── test_plan_validation.py
    ├── test_llm_json_schema.py
    └── test_skill_registry.py
```

---

## 10. Suggested ROS 2 Interfaces

### 10.1 LLM Planner Action

```text
Action: GeneratePlan

Goal:
- string command_text
- string current_context_json

Feedback:
- string reasoning_summary

Result:
- bool success
- string plan_json
- string error_message
```

### 10.2 Plan Validator Service

```text
Service: ValidatePlan

Request:
- string plan_json

Response:
- bool valid
- string validated_plan_json
- string reason
- string clarification_question
```

### 10.3 Execute Skill Action

```text
Action: ExecuteSkill

Goal:
- string skill_name
- string arguments_json

Feedback:
- string status
- string feedback_json

Result:
- bool success
- string result_json
- string failure_reason
```

---

## 11. Safety and Reliability Rules

### 11.1 LLM Safety Rules

The LLM should only produce plans using approved skills.

```text
Allowed:
- navigate_to(kitchen)
- find_object(coke)
- pick_object(coke)
- handover_object(coke)

Not allowed:
- directly_set_wheel_speed(1.0)
- force_open_unknown_door()
- grasp_person_hand()
- ignore_obstacle_and_continue()
```

### 11.2 Human Assistance Rule

Human assistance should be treated as a last resort.

The robot should first attempt autonomous recovery:

```text
Failure → Retry → Alternative method → Ask clarification → Human assistance → Report failure
```

When requesting help, the robot should:

1. Explain what help is needed.
2. Explain the desired outcome.
3. Supervise the human action if possible.
4. Verify that the result is correct.
5. Thank the human.

### 11.3 Safe Failure

The robot should prefer safe failure over unsafe action.

Example:

```text
"I cannot safely pick up the glass because it is too close to the edge of the table. I will stop and ask for help."
```

---

## 12. Practical Development Roadmap

### Phase 1: Fixed Skill Registry

Create a fixed list of available robot skills.

Deliverables:

- `skill_registry.yaml`
- JSON schema for LLM output
- Simple validator

### Phase 2: LLM Command-to-JSON Planner

Use a local or remote LLM to convert user commands into structured plans.

Deliverables:

- `llm_planner_node.py`
- prompt templates
- JSON schema enforcement
- test command dataset

### Phase 3: SMACC2 Plan Executor

Build a generic SMACC2 executor that can execute a validated list of steps.

Deliverables:

- `sm_gpsr`
- state visualization
- action clients for skill execution
- timeout and failure handling

### Phase 4: Skill Servers

Wrap robot capabilities as ROS 2 action servers.

Deliverables:

- navigation skill server
- object finding skill server
- person finding skill server
- manipulation skill server
- speech/HRI skill server

### Phase 5: BT-Style Recovery

Add fallback behavior inside individual skills.

Deliverables:

- behavior tree XML files or internal recovery state logic
- recovery test cases
- failure simulation

### Phase 6: Replanning

Allow the LLM to replan based on summarized execution feedback.

Deliverables:

- skill feedback router
- world model update
- replanning prompt
- safety constraints for replanning

---

## 13. Example Prompt Strategy for the LLM

The LLM should be prompted as a constrained robot task planner.

Example system instruction:

```text
You are a robot task planner. Convert the user's command into a JSON plan.
You may only use skills from the provided skill registry.
Do not invent skills.
If the command is ambiguous, output a clarification request instead of a plan.
Do not generate motor commands.
Do not generate unsafe actions.
```

Example skill registry given to the LLM:

```json
{
  "allowed_skills": [
    "navigate_to",
    "find_object",
    "pick_object",
    "place_object",
    "find_person",
    "handover_object",
    "speak",
    "ask_clarification",
    "return_to_instruction_point"
  ]
}
```

---

## 14. Advantages of the Proposed Architecture

### 14.1 Advantages of LLM Planner

- Handles spontaneous natural language commands.
- Converts vague instructions into structured steps.
- Can ask clarification.
- Can replan at high level.

### 14.2 Advantages of SMACC2

- Clear visualization of the current robot state.
- Explicit event-driven transitions.
- Good for competition debugging.
- Supports orthogonals for navigation, vision, voice, manipulation, memory, and safety.
- Good for pause, resume, cancel, timeout, and emergency handling.

### 14.3 Advantages of Behavior Tree Recovery

- Compact fallback design.
- Good for retry behavior.
- Good for local skill recovery.
- Avoids transition explosion in the high-level SMACC2 state machine.

### 14.4 Advantages of ROS 2 Skill Modules

- Modular and reusable.
- Easy to test independently.
- Can be used by SMACC2, BT, or other planners.
- Fits navigation, manipulation, perception, and HRI workflows.

---

## 15. Main Risks and Mitigations

| Risk | Example | Mitigation |
|---|---|---|
| LLM hallucinated skill | `open_any_container()` | Fixed skill registry and validator |
| Ambiguous command | "Bring it to him" | Ask clarification |
| Unsafe manipulation | Picking fragile object near edge | Safety validator and manipulation limits |
| Slow LLM response | Large model delay | Use smaller model or template fallback |
| Environment mismatch | Object not in expected location | Perception verification and replanning |
| Transition explosion | Too many SMACC2 recovery states | Put local recovery inside BT-style skills |
| Over-trusting human assistance | Robot asks help too early | Assistance only after autonomous recovery attempts |

---

## 16. Recommended Final Architecture

```text
┌────────────────────────────────────────────┐
│ Human Command                              │
└─────────────────────┬──────────────────────┘
                      v
┌────────────────────────────────────────────┐
│ ASR / Command Input                         │
└─────────────────────┬──────────────────────┘
                      v
┌────────────────────────────────────────────┐
│ LLM Planner                                 │
│ Natural language → JSON plan                │
└─────────────────────┬──────────────────────┘
                      v
┌────────────────────────────────────────────┐
│ Plan Validator                              │
│ Safety + capability + schema check          │
└─────────────────────┬──────────────────────┘
                      v
┌────────────────────────────────────────────┐
│ SMACC2 GPSR Task Executive                  │
│ Executes plan step by step                  │
└─────────────────────┬──────────────────────┘
                      v
┌────────────────────────────────────────────┐
│ ROS 2 Skill Servers                         │
│ Navigation / Vision / Arm / Voice / Memory  │
└─────────────────────┬──────────────────────┘
                      v
┌────────────────────────────────────────────┐
│ BT-Style Recovery Inside Skills             │
│ Retry / fallback / ask help / safe failure  │
└─────────────────────┬──────────────────────┘
                      v
┌────────────────────────────────────────────┐
│ Robot Hardware                              │
└────────────────────────────────────────────┘
```

Final recommended principle:

```text
LLM = flexible command interpretation and high-level planning
SMACC2 = visible, event-driven high-level task orchestration
BT = compact local fallback and recovery behavior
ROS 2 actions/services = executable robot skills
```

---

## 17. Conclusion

The proposed architecture is practical for RoboCup GPSR because it balances flexibility, safety, and debuggability.

A pure state machine is too rigid for random commands. A pure LLM controller is unsafe and unreliable. A pure Behavior Tree can become difficult to monitor at high level during competition. The hybrid approach combines the strengths of each method:

```text
LLM gives flexibility.
SMACC2 gives visualization and control.
BT gives robust fallback behavior.
ROS 2 skill servers give modular robot capability.
```

This architecture should be developed incrementally, starting with a fixed skill registry and JSON plan validation, then adding SMACC2 execution, BT-style recovery, and finally LLM-based replanning.
