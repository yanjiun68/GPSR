# Receptionist Context Window Log Record

Generated: 2026-06-25 15:14:01 +08  
Workspace: `/home/yanjiun68/workspace/WS1/PutrabotUPM-project/src`  
Role context: ROS2 autonomous system developer

## Context Window Summary

This log records the explored decisions, implemented package work, and verification results from the current development window. The work focused on ROS2 navigation map and waypoint handling, SMACC2 state-machine inspection, and a new InsightFace feature-summary state machine that speaks detected appearance features through the existing TTS service.

The final implemented feature in this window is a new lifecycle-managed SMACC2 package named `insightface_feature_summary_smacc2`. It launches the InsightFace appearance pipeline, waits for a visible face, collects appearance messages, stores detected features in SMACC2 global data, generates flexible speech text, calls the TTS service once, and cleans up the launched InsightFace process tree.

```text
[Context Window Work]
        |
        +--> Navigation map and waypoint usage
        |
        +--> Rviz/Nav2 goal pose and orientation discussion
        |
        +--> Named waypoint action flow
        |
        +--> SMACC2/RTA viewer inspection
        |
        +--> InsightFace feature-summary SMACC2 implementation
        |
        +--> TTS service-call design and verification
```

## Navigation And Mapping Work

The navigation discussion covered how `static_map_nav.launch.py` and mapping-related launch files are used in the robot navigation workflow. The map argument in the static navigation launch command selects the saved map YAML file loaded by Nav2 map server. That YAML points to the map image and defines metadata such as resolution, origin, occupied threshold, and free threshold.

For map export, the expected ROS2 workflow is to run SLAM or mapping, then save the map using the Nav2 map saver CLI:

```bash
ros2 run nav2_map_server map_saver_cli -f /path/to/map_name
```

This produces:

```text
map_name.yaml
map_name.pgm
```

The YAML is then passed back into the static navigation launch file with a `map:=...` argument.

```text
[SLAM / Mapping Running]
        |
        v
[map_saver_cli -f map_name]
        |
        v
[map_name.yaml + map_name.pgm]
        |
        v
[static_map_nav.launch.py map:=map_name.yaml]
        |
        v
[Nav2 map server publishes static map]
```

## Waypoint YAML And Named-Pose Action Flow

The waypoint workflow moved toward storing target poses in a YAML file so named locations can be reused without manually clicking in RViz every time. The current intended source is the active waypoint YAML under `NAVIGATION/my_robot_nav/maps/`, not stale waypoints from an older map.

A target pose in the map frame needs:

```yaml
position:
  x: <meters_in_map_frame>
  y: <meters_in_map_frame>
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: <quaternion_z>
  w: <quaternion_w>
```

For 2D navigation, the pose is usually interpreted as:

```text
x, y, yaw
```

The orientation in RViz is represented internally as a quaternion. If only 2D coordinates are visible in a terminal output, the orientation can be checked by echoing the full goal pose topic/action payload or by converting quaternion `z,w` into yaw:

```text
yaw = atan2(2*w*z, 1 - 2*z*z)
```

The named-pose action flow discussed in this window was:

```bash
ros2 action send_goal /nav/navigate_named_pose \
  receptionist_interfaces/action/NavigateNamedPose \
  "{target_name: 6}" \
  --feedback
```

If the command remains at:

```text
Waiting for an action server to become available...
```

then the named-pose action server is not running, has a different action name, or the workspace was not sourced correctly.

```text
[waypoints.yaml]
        |
        v
[NavigateNamedPose action server loads names]
        |
        v
[ros2 action send_goal /nav/navigate_named_pose]
        |
        v
[Server resolves target_name to PoseStamped]
        |
        v
[Nav2 NavigateToPose action]
        |
        v
[Robot moves to waypoint]
```

## SMACC2 And RTA Viewer Investigation

The state-machine inspection focused on the receptionist SMACC2 package and the ability to view the state-machine flow. The referenced launch file was:

```text
rc_task/sm_receptionist/launch/smacc2_receptionist.launch.py
```

The intended RTA viewer workflow is to launch the SMACC2 state machine with its introspection support enabled, then open the runtime analyzer/viewer to inspect states, transitions, and active orthogonals.

The existing `insightface_camera_switch_smacc2` package was used as the reference style for lifecycle-supervised SMACC2 execution. It contains:

```text
rc_task/insightface_camera_switch_smacc2/src/camera_switch_smacc2_node.cpp
rc_task/insightface_camera_switch_smacc2/src/camera_switch_lifecycle_node.cpp
rc_task/insightface_camera_switch_smacc2/include/insightface_camera_switch_smacc2/process_utils.hpp
```

That package pattern uses:

```text
[Lifecycle node]
        |
        v
[Detached SMACC2 process]
        |
        v
[Detached InsightFace launch process]
        |
        v
[Process group registry in /tmp]
        |
        v
[Clean shutdown by process group]
```

## InsightFace Feature-Summary SMACC2 Package

A new package was implemented:

```text
rc_task/insightface_feature_summary_smacc2
```

Created files:

```text
rc_task/insightface_feature_summary_smacc2/CMakeLists.txt
rc_task/insightface_feature_summary_smacc2/package.xml
rc_task/insightface_feature_summary_smacc2/include/insightface_feature_summary_smacc2/process_utils.hpp
rc_task/insightface_feature_summary_smacc2/launch/feature_summary.launch.py
rc_task/insightface_feature_summary_smacc2/src/feature_summary_lifecycle_node.cpp
rc_task/insightface_feature_summary_smacc2/src/feature_summary_smacc2_node.cpp
```

The launch file exposes the following important defaults:

```text
personal_test_mode:=true
start_tts:=true
tts_service_name:=/voice/tts/say
face_topic:=/perception/largest_face
appearance_topic:=/appearance/current_estimate
```

The SMACC2 flow is:

```text
[StLaunchInsightFace]
        |
        v
[StWaitForFace]
        |
        v
[StCollectAppearance]
        |
        v
[StBuildSummary]
        |
        v
[StSpeakSummary]
        |
        v
[StStopInsightFaceAndFinish]
```

Failure from launch, face wait, appearance collection, summary generation, or speech output transitions to:

```text
[StAborted]
        |
        v
[Stop registered InsightFace process group]
        |
        v
[Shutdown]
```

The state machine stores detected features in SMACC2 global data:

```text
shirt_color
shirt_color_confidence
glasses_state
glasses_confidence
hat_state
hat_confidence
face_mask_state
face_mask_confidence
feature_summary_text
```

The appearance message used by the package is:

```text
receptionist_interfaces/msg/GuestAppearance
```

The collected fields are:

```text
shirt_color
glasses
hat
face_mask
confidence values for each feature
```

## TTS Service-Call Logic

The speech output uses:

```text
voice_interfaces/srv/SayText
```

Service shape:

```text
string text
---
bool success
string message
```

Default service name:

```text
/voice/tts/say
```

The text generation is flexible because it is generated after appearance aggregation. The state machine chooses sentences based on detected, not detected, or unknown states.

Example output shape:

```text
I can see a person wearing a blue shirt. I detect glasses. I do not detect a hat. I do not detect a face mask.
```

If the feature cannot be confidently determined, the generated text avoids overclaiming:

```text
I can see a person, but I cannot confidently determine the shirt color.
I cannot confidently determine whether the person is wearing glasses.
```

The TTS service is called exactly once in the speech state:

```text
[Aggregated appearance global data]
        |
        v
[Build feature_summary_text]
        |
        v
[CbSpeakFeatureSummary]
        |
        v
[One SayText request to /voice/tts/say]
```

## Build And Verification Results

The package was discovered by `colcon`:

```bash
colcon list --packages-select insightface_feature_summary_smacc2
```

The package and required dependencies were built from the workspace root:

```bash
colcon build --packages-up-to insightface_feature_summary_smacc2
```

Result:

```text
6 packages finished
insightface_feature_summary_smacc2 built successfully
```

The installed executables were verified:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg executables insightface_feature_summary_smacc2
```

Result:

```text
insightface_feature_summary_smacc2 feature_summary_lifecycle_node
insightface_feature_summary_smacc2 feature_summary_smacc2_node
```

The launch arguments were checked with:

```bash
ROS_LOG_DIR=/tmp/roslogs \
ros2 launch insightface_feature_summary_smacc2 feature_summary.launch.py --show-args
```

The check confirmed the expected defaults, including:

```text
personal_test_mode default: true
start_tts default: true
tts_service_name default: /voice/tts/say
```

## Known Limitations And Next Steps

Full hardware runtime testing was not completed in this context window. Specifically, the camera stack, live InsightFace detection, physical robot motion, and audible Piper playback were not run end-to-end during verification.

Recommended next runtime tests:

```bash
source /home/yanjiun68/workspace/WS1/PutrabotUPM-project/install/setup.bash
ros2 launch insightface_feature_summary_smacc2 feature_summary.launch.py
```

For RealSense mode:

```bash
ros2 launch insightface_feature_summary_smacc2 feature_summary.launch.py personal_test_mode:=false
```

Runtime checks to perform:

```bash
ros2 topic echo /perception/largest_face --once
ros2 topic echo /appearance/current_estimate --once
ros2 service list | grep /voice/tts/say
```

If the state machine times out waiting for face or appearance data, check:

```text
camera device availability
InsightFace model dependencies
topic names
workspace sourcing
ROS_DOMAIN_ID
TTS node availability
```

## Microsoft Word Compatibility Notes

This log is maintained as Markdown for version control and future editing. A `.docx` export is generated for Microsoft Word compatibility and user-manual preparation.

Recommended document policy:

```text
Markdown file = editable source of truth
DOCX file = exported manual/reference copy
```

The `.docx` file should open in Microsoft Word and LibreOffice Writer. Future updates should be made in the Markdown source first, then exported again.
