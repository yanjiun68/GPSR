# fixed_information_dialogue

Deterministic ROS 2 provider for GPSR fixed-information dialogue.

## Scope

Answers only:
- static competition/team/robot facts stored in YAML;
- local runtime time/date facts.

It intentionally returns `NO_MATCH` for perception, task-memory, scene-memory,
external-live-data, and general-knowledge questions.

## Topics

Input:
- `/fixed_information/query` (`std_msgs/String`) — plain text or JSON request.

Outputs:
- `/fixed_information/answer` — JSON result envelope.
- `/fixed_information/answer_text` — plain TTS-ready text only for `FOUND`.
- `/fixed_information/ready` — transient-local readiness flag.

Reload:
- `/fixed_information/reload` (`std_srvs/Trigger`)

## Build

```bash
cd ~/ros2_ws/src
unzip fixed_information_dialogue.zip
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select fixed_information_dialogue
source install/setup.bash
```

## Run

```bash
ros2 launch fixed_information_dialogue fixed_information_dialogue.launch.py
```

## Test a direct fact trigger

```bash
ros2 topic echo /fixed_information/answer
```

In another terminal:

```bash
ros2 topic pub --once /fixed_information/query std_msgs/msg/String \
  "{data: \"today's date\"}"
```

Direct TTS-oriented output:

```bash
ros2 topic echo /fixed_information/answer_text
```

## Speak answer text directly

Start the existing voice TTS node, fixed-information provider, and the dedicated
answer-text bridge in separate terminals:

```bash
ros2 launch voice voice.launch.py start_stt:=false start_tts:=true
ros2 launch fixed_information_dialogue fixed_information_dialogue.launch.py
ros2 launch fixed_information_dialogue answer_text_tts_bridge.launch.py
```

The bridge subscribes only to `/fixed_information/answer_text` and forwards its
`std_msgs/String.data` value to `/voice/tts/say`. Do not run this bridge together
with `gpsr_dialogue_manager` unless duplicate spoken answers are desired, because
the manager already sends matched answers to TTS.

## Reload edited facts

```bash
ros2 service call /fixed_information/reload std_srvs/srv/Trigger "{}"
```

## Router integration

Recommended routing:

```text
STT
 -> Dialogue/Question Router
    -> FIXED_INFORMATION -> this package
    -> TASK_RESULT       -> TaskMemory
    -> WORLD_STATE       -> QueryWorld / Rediscovery
    -> SCENE_HISTORY     -> SemanticSceneMemory
    -> EXTERNAL_DYNAMIC  -> external provider
```

`NO_MATCH` and `AMBIGUOUS` must be routed onward or clarified; this node never
uses an LLM to invent a fixed answer.

## GPSR skill-state integration

The `robot_yasmin_tasks` executor exposes this provider as the
`FIXED_INFORMATION_SKILL_SM` state. The scheduler emits the canonical
`FIXED_INFORMATION` primitive for provider-owned requests such as:

```text
tell today's date
today's date
meet Axel in the bedroom, tell him today's date
what time is it
what is your team name
who are you
```

Recipient-directed commands remove the recipient from the provider payload. For
example, the compound command above becomes `FIND_PERSON` followed by
`FIXED_INFORMATION(question="today's date")`; it does not become a literal
`SAY` action.

The state correlates the topic response by request ID, preserves `NO_MATCH` and
`AMBIGUOUS`, and sends only a `FOUND` answer to TTS.
