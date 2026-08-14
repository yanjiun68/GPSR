# Implement `Receptionist_CV` Non-Biometric Guest Recognition

## Summary
Create a new package at `rc_task/Receptionist_CV` with ROS package name `receptionist_cv`. It will not use face embeddings or biometric identity matching. It will reuse the existing appearance/object detection outputs from the current pipeline and add an appearance-signature matcher for guest recognition.

```text
[Existing camera + YOLO/object detection]
        |
        v
[/appearance/current_estimate] + [/perception/recognized_items] + [/cam_detections]
        |
        v
[receptionist_cv signature_builder]
        |
        v
[appearance signature: shirt + accessories + items + spatial cues]
        |
        v
[receptionist_cv appearance_matcher]
        |
        v
[/appearance/recognition_status] + /appearance/current_person_observation
```

## Key Changes
- Create `rc_task/Receptionist_CV` as an `ament_python` package:
  - ROS package name: `receptionist_cv`
  - Console scripts:
    - `appearance_signature_node`
    - `appearance_matcher_node`
    - optional `appearance_profile_capture_node`
- Reuse existing message types:
  - `receptionist_interfaces/msg/GuestAppearance`
  - `receptionist_interfaces/msg/PersonProfile`
  - `receptionist_interfaces/msg/RecognitionStatus`
  - `std_msgs/msg/String` for JSON debug/signature topics
- Do not use:
  - face embeddings
  - cosine similarity
  - `IdentityStore.recognize_profile(...)`
  - biometric match score

## Behavior
```text
[Capture guest]
        |
        v
Read current GuestAppearance + stable detected items + person position
        |
        v
Store non-biometric signature in guest memory
        |
        v
Recognition later compares current signature against stored guest signatures
```

- Signature fields:
  - `shirt_color`
  - `shirt_color_confidence`
  - `glasses`, `hat`, `face_mask`
  - stable detected items from `/perception/recognized_items`
  - optional person position/distance from `/cam_detections`
  - optional seat/register-order fallback from guest memory
- Matching score:
  - shirt color: high weight
  - stable objects/items: high weight
  - glasses/hat/mask: medium weight
  - position/seat/order: fallback weight
- If score is low or tied, publish unknown/ambiguous instead of forcing a wrong identity.

## Package Integration
- Keep `face_detection` appearance/object nodes running for now, but only as detection providers.
- Disable biometric identification by launch parameter:
  - `enable_identification:=false`
- Enable developed object detection:
  - `enable_detected_items_node:=true`
  - `enable_stable_item_memory:=true`
- Add new launch file in `Receptionist_CV`:
  - starts `appearance_signature_node`
  - starts `appearance_matcher_node`
  - remaps topics to current pipeline defaults

```text
[appearance_pipeline.launch.py]
        |
        +-- existing appearance_node
        +-- existing detected_items_node
        +-- existing stable_detected_items_node
        |
        +-- new receptionist_cv matcher nodes
```

## Test Plan
- Build:
  - `colcon build --packages-select receptionist_cv receptionist_interfaces receptionist_nodes face_detection`
- Runtime checks:
  - `ros2 topic echo /appearance/current_estimate --once`
  - `ros2 topic echo /perception/recognized_items --once`
  - `ros2 topic echo /appearance/current_guest_signature --once`
  - `ros2 topic echo /appearance/recognition_status --once`
- Scenarios:
  - Two guests with different shirt colors should match correctly.
  - Same shirt color but different carried objects should use object cues.
  - Same shirt color and no stable items should return ambiguous/unknown.
  - Biometric identification disabled should not break object/appearance recognition.

## Assumptions
- Directory will be `rc_task/Receptionist_CV`; ROS package name will be lowercase `receptionist_cv`.
- “Do not use InsightFace” means no biometric face identity recognition, but current `face_detection` object/appearance detection topics may still be reused.
- Existing object detection should be retained and promoted as a major identity cue.
- No new custom interface package is required for the first version; use JSON debug topics plus existing `RecognitionStatus`.

## To view pkl
python3 - <<'PY'
import pickle
from pathlib import Path

path = Path.home() / ".ros" / "face_detection" / "guest_db.pkl"

with path.open("rb") as file:
    database = pickle.load(file)

for profile_id, profile in database.get("profiles", {}).items():
    print({
        "profile_id": profile_id,
        "name": profile.get("name", ""),
        "role": profile.get("role", ""),
        "favorite_drink": profile.get("favorite_drink", ""),
        "age": profile.get("age", 0),
        "shirt_color": profile.get("shirt_color", ""),
        "template_count": len(profile.get("templates", [])),
        "embedding_dimension": profile.get("embedding_dimension", 0),
    })
PY
