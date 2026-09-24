# Qwen VLM ROS 2 text bridge

This package keeps the local Qwen3-VL-compatible UnifoLM-ER-1 model loaded and
replaces the original `message.txt` input with either a final STT transcript or
a dedicated manual debug command.

```text
/voice/stt/transcript ---------\
                                > qwen_text_planner -> /ai/model/qwen_vlm/plan
/ai/model/qwen_vlm/debug/input-/                    -> status/raw/result topics
```

The bridge only publishes proposed semantic plans. It does not submit an
`ActionQueue` or move the robot.

Debug envelopes containing `"submit": true` are rejected. Execution must be
added later through the guarded GPSR scheduler rather than this model node.

## Build and launch

```bash
colcon build --packages-select qwen_vlm_bridge
source install/setup.bash
ros2 launch qwen_vlm_bridge qwen_text_planner.launch.py
```

The launcher uses `/home/yanjiun68/ai/qwen3v1/.venv/bin/python` by default.
Set `QWEN_PYTHON` before launch to select another Python environment.

## Manual debug input

Publish plain text:

```bash
ros2 topic pub --once /ai/model/qwen_vlm/debug/input std_msgs/msg/String \
  "{data: 'Go to the kitchen and find a coke.'}"
```

Publish a correlated JSON envelope:

```bash
ros2 topic pub --once /ai/model/qwen_vlm/debug/input std_msgs/msg/String \
  '{data: "{\"request_id\":\"debug_001\",\"instruction\":\"Go to the kitchen\",\"dry_run\":true,\"submit\":false}"}'
```

Observe outputs:

```bash
ros2 topic echo /ai/model/qwen_vlm/status
ros2 topic echo /ai/model/qwen_vlm/debug/result
ros2 topic echo /ai/model/qwen_vlm/plan
```

## One-shot CLI compatibility

```bash
ros2 run qwen_vlm_bridge qwen_text_cli.py --text "Go to the kitchen"
```

Legacy message files remain supported with `--message path/to/message.txt`.
