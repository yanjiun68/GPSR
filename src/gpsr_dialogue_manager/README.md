# gpsr_dialogue_manager

P0 integration layer for the existing `voice` and
`fixed_information_dialogue` packages.

## What this fixes

- Owns one complete conversation turn.
- Waits for TTS to finish before starting STT.
- Consumes the STT action result, not `/voice/stt/transcript`, preventing
  duplicate handling of the same utterance.
- Separates obvious GPSR task commands from questions.
- Correlates `/fixed_information/answer` using `request_id`.
- Applies a bounded timeout to fixed-information queries.
- Routes `NO_MATCH`/`NOT_CONFIGURED` onward instead of speaking a false answer.

## Interfaces

Uses:

```text
/voice/tts/say_action
/voice/stt/transcribe_action

/fixed_information/query
/fixed_information/answer
```

Publishes:

```text
/dialogue/task_command
/dialogue/unhandled_question
/dialogue/status
```

Trigger a single turn:

```text
/dialogue/start    std_msgs/msg/Empty
```

## Build

Place these three packages in the same ROS 2 workspace:

```text
voice
fixed_information_dialogue
gpsr_dialogue_manager
```

Then:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select \
  voice_interfaces \
  voice \
  fixed_information_dialogue \
  gpsr_dialogue_manager
source install/setup.bash
```

Package names in your existing voice workspace may differ. If the voice package
is split into multiple packages, build its actual packages plus
`voice_interfaces`.

## Run

Terminal 1: start the voice STT/TTS nodes using the existing voice launch/run
procedure.

Terminal 2:

```bash
ros2 launch fixed_information_dialogue fixed_information_dialogue.launch.py
```

Terminal 3:

```bash
ros2 launch gpsr_dialogue_manager dialogue_manager.launch.py
```

Start one interaction:

```bash
ros2 topic pub --once /dialogue/start std_msgs/msg/Empty "{}"
```

Expected flow:

```text
robot TTS: "What would you like to ask me?"
        ↓ TTS completes
STT records user
        ↓
question -> fixed-information provider
        ↓
FOUND -> TTS speaks answer
```

For:

```text
Find the apple in the kitchen
```

the manager publishes the transcript to:

```text
/dialogue/task_command
```

instead of querying the fixed-information database.

## P0 limitations

This is intentionally not the final dialogue architecture.

Still future work:
- QueryWorld provider
- SemanticSceneMemory provider
- TaskMemory questions
- external-current-data provider
- richer dialogue-act/NLU classification
- confirmation/correction context
- VAD
- immediate subprocess cancellation
- fixed-information service API instead of topic RPC
