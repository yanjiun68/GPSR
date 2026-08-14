# YuNet/SFace ROS usage

## Runtime architecture

The package is named `face_detection`. The runtime face backend is OpenCV YuNet +
SFace and has no InsightFace fallback.

```text
sensor_msgs/Image -> latest-frame cache -> 5 Hz timer
                                          |
                       largest/static ROI/existing tracked-person ROI
                                          |
                              YuNet (OpenCV DNN, CPU)
                                          |
                         bbox + five global-frame landmarks
                                          |
                              SFace alignCrop + feature
                                          |
                             normalized float32 embedding
                                          |
                  enrollment/templates or open-set temporal voting
                                          |
                              Receptionist / GPSR actions
```

`tracked_person` mode reuses `/human_identity/recognized_people`
(`vision/msg/RecognizedPerson`) and its `track_id` + `bbox_xyxy`. `largest` remains
the compatibility default. No new tracker is started by this launch file.

## Requirements and models

Use system/ROS OpenCV 4.5.4 or newer. It must expose both
`cv2.FaceDetectorYN` and `cv2.FaceRecognizerSF`. The face-only Python manifest
contains NumPy; it intentionally does not install an OpenCV wheel or a special
virtual environment.

Place OpenCV Zoo-compatible models in `src/rc_task/face_detection/models/` before
building:

- `face_detection_yunet_2023mar.onnx`
- `face_recognition_sface_2021dec.onnx`

The search order is explicit ROS path, installed package `models/`, then
`face_model_dir`. Runtime never downloads a model. Verify APIs and model loading:

```bash
source /opt/ros/humble/setup.bash
python3 src/rc_task/face_detection/scripts/check_python_environment.py \
  --yunet-model src/rc_task/face_detection/models/face_detection_yunet_2023mar.onnx \
  --sface-model src/rc_task/face_detection/models/face_recognition_sface_2021dec.onnx
```

## Database compatibility

Schema version 2 now stores `embedding_backend=opencv_sface`, the SFace model
filename, and the dimension measured from real model output. Legacy InsightFace
pickles and any backend/model/dimension mismatch are rejected and never compared.
Move an old database aside, then re-enroll every person:

```bash
mkdir -p ~/.ros/face_detection
test ! -f ~/.ros/face_detection/guest_db.pkl || \
  mv ~/.ros/face_detection/guest_db.pkl ~/.ros/face_detection/guest_db.insightface.bak
```

Writes use a temporary file, flush, `fsync`, and atomic `os.replace`. Enrollment
requires distinct fresh timestamps, five consistent frames by default, and has an
eight-second timeout.

## Build and launch

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select receptionist_interfaces putrabot_interfaces face_detection
source install/setup.bash
```

Start the RealSense publisher separately so only one driver owns the camera:

```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

Then start the face/appearance pipeline:

```bash
ros2 launch face_detection appearance_pipeline.launch.py \
  image_topic:=/camera/camera/color/image_raw \
  enable_depth:=true \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw
```

For an existing RGB publisher on `/camera/image_raw`:

```bash
ros2 launch face_detection appearance_pipeline.launch.py personal_test_mode:=true
```

The existing standalone webcam publisher remains available:

```bash
ros2 run face_detection detection_receptionist --ros-args \
  -p camera_backend:=webcam -p camera_index:=/dev/video0 \
  -p image_topic:=/camera/image_raw -p enable_depth:=false
```

To bind faces to the nearest ROI published by the existing tracker:

```bash
ros2 launch face_detection appearance_pipeline.launch.py \
  face_selection_mode:=tracked_person \
  tracked_person_topic:=/human_identity/recognized_people \
  target_track_id:=0
```

This optional mode also requires the repository's `vision` package and its own
declared dependencies to be built and sourced.

## Enrollment and recognition

Enroll a named guest from new observations:

```bash
ros2 action send_goal /appearance/capture_named_profile \
  receptionist_interfaces/action/CaptureNamedProfile \
  "{profile_id: guest_1, role: guest, name: Morgan}" --feedback
```

Recognize from new temporal evidence rather than cached state:

```bash
ros2 action send_goal /appearance/recognize_human_id \
  receptionist_interfaces/action/RecognizeHumanId \
  "{require_known_match: true}" --feedback
```

Action result messages begin with `SUCCESS_KNOWN`, `SUCCESS_UNKNOWN`, `NO_FACE`,
`TIMEOUT`, or `CANCELLED` while preserving the existing action definition.

## ROS interfaces

Published topics:

- `/perception/largest_face` (`std_msgs/msg/String`): isolated compatibility JSON
  conversion with bbox, landmarks, normalized embedding, backend/model/dimension,
  depth, selected ROI, stable timestamp, and optional `track_id`.
- `/appearance/current_estimate` (`receptionist_interfaces/msg/GuestAppearance`).
- `/appearance/current_capture_status` (`receptionist_interfaces/msg/CurrentGuestStatus`).
- `/appearance/recognition_status` (`receptionist_interfaces/msg/RecognitionStatus`).
- `/appearance/recognized_human_identity` (`std_msgs/msg/String`): dedicated JSON
  identity state for real-time consumers and the appearance GUI; known identities are
  emitted only after temporal verification.

Inspect the dedicated recognition output while the pipeline is running:

```bash
ros2 topic echo /appearance/recognized_human_identity std_msgs/msg/String
```
- `/appearance/current_person_observation` (`std_msgs/msg/String`): candidate and
  stable identity details, top-two margin, and associated appearance.
- `/dialogue/current_guest_profile` and `/dialogue/current_host_profile`
  (`receptionist_interfaces/msg/PersonProfile`).

Consumed topics:

- configurable color/depth `sensor_msgs/msg/Image` topics;
- `/human_identity/recognized_people` (`vision/msg/RecognizedPerson`) in
  `tracked_person` mode;
- `/appearance/current_estimate` in enrollment/identity nodes.

Services retained:

- `/appearance/capture_profile_snapshot` (`receptionist_interfaces/srv/CaptureAppearance`).
- `/appearance/lookup_identity_profile`
  (`receptionist_interfaces/srv/LookupIdentityProfile`): case-insensitive lookup by name;
  reports whether a persisted identity has usable face templates without returning embeddings.
- existing guest-memory services used by profile capture.

Actions retained:

- `/appearance/capture_named_profile` (`CaptureNamedProfile`).
- `/appearance/recognize_human_id` (`RecognizeHumanId`).

Important parameters include `yunet_model_path`, `sface_model_path`,
`face_model_dir`, `yunet_score_threshold`, `yunet_nms_threshold`, `yunet_top_k`,
`opencv_dnn_backend`, `opencv_dnn_target`, `face_inference_rate_hz`,
`recognition_rate_hz`, `max_face_age_sec`, `face_selection_mode`,
`tracked_person_topic`, `target_track_id`, `min_enrollment_samples`,
`max_enrollment_samples`, `profile_capture_timeout_sec`,
`embedding_similarity_threshold`, `recognition_threshold`, `recognition_margin`,
`identity_window_size`, `identity_min_votes`, `recognition_action_timeout_sec`,
`recognition_action_min_votes`, `debug_mode`, and `enable_inference_timing`.

The high-frequency face transfer remains JSON for interface compatibility; the
conversion is isolated in `face_perception.py`. A future typed message is still
recommended once Receptionist/GPSR consumers can be migrated together.
