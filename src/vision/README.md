# Vision Package Notes

## MediaPipe Human Pose

The `human_skeleton_detection_node.py` node uses MediaPipe Tasks Pose Landmarker Full in `VIDEO` mode.

Install Python dependency if needed:

```bash
python3 -m pip install mediapipe
```

Download the Full model into the package:

```bash
bash src/vision/tools/download_pose_landmarker_full_model.sh
```

Build and source:

```bash
colcon build --packages-select putrabot_interfaces vision
source install/setup.bash
```

Launch:

```bash
ros2 launch vision human_skeleton_detection.launch.py
```

Launch with GUI monitoring and pose/gesture TTS:

```bash
ros2 launch vision human_skeleton_detection.launch.py show_gui:=true enable_tts:=true start_voice_tts:=true
```

If MediaPipe reports two people when only one person is visible, first try the post-filter defaults. For strict one-person scenes, force the best single candidate:

```bash
ros2 launch vision human_skeleton_detection.launch.py num_poses:=2 prefer_single_person_mode:=true show_gui:=true
```

Useful inspection commands:

```bash
ros2 topic echo /human_pose/poses --once
ros2 topic echo /human_pose/gestures
ros2 topic echo /perception/human_poses
rqt_image_view /human_pose/debug_image
```

Main topics:

- `/human_pose/poses`: `putrabot_interfaces/msg/HumanPoseArray` with 33 MediaPipe landmarks per detected person.
- `/human_pose/gestures`: JSON GPSR gesture summary with legacy gesture fields plus per-person `left_arm_state`, `right_arm_state`, `raised_arm_state`, `wave_state`, `confidence`, and `timestamp`.
- `/human_pose/debug_image`: `sensor_msgs/msg/Image` with skeleton connections, arm-joint reliability, wrist trajectories, direction changes, confidence, person ID, body pose, and gesture.
- `/perception/human_poses`: compact JSON pose and gesture summary for quick terminal inspection.
- `/perception/human_skeletons`: legacy 17-keypoint compatibility topic for count-people pose sorting.

Main monitoring options:

- `show_gui:=true`: opens a local OpenCV monitor window when `DISPLAY` is available.
- `publish_debug_image:=true`: publishes `/human_pose/debug_image` for `rqt_image_view`.
- `enable_tts:=true`: speaks detected pose and stabilized gesture status through `/voice/tts/say`.
- `start_voice_tts:=true`: starts the local `voice/tts_node.py` service from the launch file.
- `min_usable_landmarks`, `min_torso_landmark_confidence`, `min_pose_bbox_area_ratio`: reject weak pose candidates before publishing.
- `duplicate_iou_threshold`, `duplicate_center_distance_ratio`: suppress duplicate MediaPipe candidates for the same person.
- `prefer_single_person_mode:=true`: publishes only the best pose candidate while keeping MediaPipe `num_poses` configurable.
- `gesture_detection_enabled:=true`: enables RoboCup GPSR gesture classification.
- `gesture_mirror_input:=true`: reverses `pointing_left`/`pointing_right` for mirrored camera feeds only.
- `gesture_stability_frames:=3`: requires a gesture candidate to persist before it is published or spoken.
- `arm_landmark_min_visibility:=0.55`: independently validates each arm's shoulder, elbow, and wrist.
- `raised_arm_vertical_margin:=0.04`, `raised_arm_elbow_margin:=0.08`: configure shoulder-relative raised-arm geometry.
- `wave_window_seconds:=1.5`, `wave_min_horizontal_range:=0.12`, `wave_min_direction_changes:=2`: configure temporal horizontal oscillation.
- `wave_max_vertical_drift:=0.15`, `wave_history_timeout_seconds:=2.0`: reject excessive vertical motion and expire missing-person histories.
