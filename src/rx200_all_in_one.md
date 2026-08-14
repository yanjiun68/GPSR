# RX200 All-In-One Launch Notes

This note summarizes the last two discussions about the `rx200_handbag_grasp` launch model and the end-to-end topic architecture around `all_in_one.launch.py`.

## 1. Launch Mindset Model

`all_in_one.launch.py` is an orchestrator. It does not directly define robot behavior. It stages three subsystems in sequence:

1. Step 1: robot, lift, ros2_control, state merge, MoveIt
2. Step 2: RealSense, YOLO, selector, camera TF
3. Step 3: grasp execution

Reference:
- [all_in_one.launch.py](/home/yanjiun68/workspace/WS_phIV/src/ARM_LIFTING/rx200_handbag_grasp/launch/all_in_one.launch.py)

## 2. Inverse Kinematics Configuration

IK is configured in `config/kinematics.yaml` and injected into MoveIt by `step1_startup.launch.py` under `robot_description_kinematics`.

Configuration:

- `interbotix_arm`
  - `lma_kinematics_plugin/LMAKinematicsPlugin`
  - `position_only_ik: true`
- `interbotix_gripper`
  - `kdl_kinematics_plugin/KDLKinematicsPlugin`
- `interbotix_lift`
  - `kdl_kinematics_plugin/KDLKinematicsPlugin`
- `interbotix_arm_lift`
  - `lma_kinematics_plugin/LMAKinematicsPlugin`
  - `position_only_ik: true`

Interpretation:

- The RX200 is treated as a position-driven manipulator rather than a fully orientation-constrained 6-DOF arm.
- The grasp code later simplifies orientation handling to fit the real 5-DOF capability.

References:
- [kinematics.yaml](/home/yanjiun68/workspace/WS_phIV/src/ARM_LIFTING/rx200_handbag_grasp/config/kinematics.yaml)
- [step1_startup.launch.py](/home/yanjiun68/workspace/WS_phIV/src/ARM_LIFTING/rx200_handbag_grasp/launch/step1_startup.launch.py)

## 3. Collision Model Added for the RX200 + Lift System

Collision configuration is defined in `config/combined.srdf.xacro`.

Key design intent:

- Disable only adjacent or structurally unavoidable collision pairs.
- Keep meaningful arm-vs-lift collision checking enabled.

Disabled examples:

- `lift_base_link` vs `lift_top_link`
- `lift_top_link` vs `rx200/base_link`
- `rx200/base_link` vs `lift_base_link`
- `rx200/shoulder_link` vs lift links
- finger links vs `lift_base_link` for folded poses
- camera mount vs immediately adjacent gripper links

Still collision-checked:

- upper arm vs lift
- forearm vs lift
- wrist vs lift
- gripper body vs lift

Interpretation:

- The project explicitly preserves safety-critical arm-body vs lift collision checking instead of disabling everything around the lift.

Reference:
- [combined.srdf.xacro](/home/yanjiun68/workspace/WS_phIV/src/ARM_LIFTING/rx200_handbag_grasp/config/combined.srdf.xacro)

## 4. How the Lift Is Added into the URDF

The lift is built directly into the combined robot model in `urdf/combined.urdf.xacro`.

Structure:

- `world -> lift_base_link`
- `lift_base_link -> lift_top_link` via `lift_joint` prismatic Z axis
- `lift_top_link -> rx200/base_link` via fixed joint `lift_to_robot`

Important points:

- The lift is not treated as an external accessory at planning level.
- MoveIt sees one combined robot model.
- The arm is mounted on the moving top platform of the lift.

There is also an eye-in-hand camera mounted on the gripper side of the arm in the same URDF.

References:
- [combined.urdf.xacro](/home/yanjiun68/workspace/WS_phIV/src/ARM_LIFTING/rx200_handbag_grasp/urdf/combined.urdf.xacro)

## 5. How the Arm Uses RealSense Depth to Grab the Bag

The bag-grasping pipeline is:

1. RealSense publishes color, camera info, and aligned depth.
2. YOLO detects handbag handles or straps in RGB.
3. The selector reduces detections to the chosen handbag target.
4. `grasp_node.py` synchronizes:
   - selected detections
   - aligned depth
   - camera intrinsics
   - optional segmentation mask
5. The grasp node samples depth in the ROI, preferably within the mask.
6. It rejects bad depth values, estimates a robust median depth, and back-projects the 2D detection into a 3D point in camera coordinates.
7. It applies calibrated grasp offsets.
8. It transforms the pose from camera frame into `lift_base_link` using TF.
9. It publishes grasp and pregrasp poses.
10. It uses MoveIt plus visual servo refinement before final descent and closing.

Important implementation details:

- `camera_color_optical_frame` is the reference frame.
- `lift_base_link` is the planning base frame.
- Mask-based depth sampling and outlier rejection are used.
- Visual servoing is used to refine lateral and depth alignment before grasp execution.

References:
- [step2_perception.launch.py](/home/yanjiun68/workspace/WS_phIV/src/ARM_LIFTING/rx200_handbag_grasp/launch/step2_perception.launch.py)
- [grasp_params.yaml](/home/yanjiun68/workspace/WS_phIV/src/ARM_LIFTING/rx200_handbag_grasp/config/grasp_params.yaml)
- [grasp_node.py](/home/yanjiun68/workspace/WS_phIV/src/ARM_LIFTING/rx200_handbag_grasp/scripts/grasp_node.py)

## 6. Diagnosis of `xsarm_control.launch.py`

The command tested was:

```bash
ros2 launch interbotix_xsarm_control xsarm_control.launch.py \
  robot_model:=rx200 \
  port:=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT2XP6Z8-if00-port0
```

Observed behavior:

- `xs_sdk` started and found the motors.
- `/rx200/sdk_joint_states` published correctly.
- `/rx200/joint_states` existed only as a subscribed topic and had no publisher.
- Runtime bus errors also appeared:
  - `Failed syncRead`
  - `There is no status packet`
  - `Incorrect status packet`

Root causes:

1. `port:=...` is not a valid launch argument for `xsarm_control.launch.py`.
   - The launch file accepts `motor_configs`, not `port`.
   - So the extra `port:=...` is ignored.
   - The actual port still comes from the motor config YAML.

2. For RX200, the motor config intentionally publishes `sdk_joint_states`, not `joint_states`.
   - This is defined in `interbotix_xsarm_control/config/rx200.yaml`.

3. `xsarm_control.launch.py` does not start a `joint_state_publisher` node.
   - It brings up `xs_sdk`, `robot_state_publisher`, and optionally RViz.

4. `robot_state_publisher` subscribes to `/rx200/joint_states`, but nothing in that launch publishes it.
   - Therefore `/rx200/joint_states` stays silent.

5. The runtime `syncRead` errors indicate bus communication instability.
   - This is separate from the topic-name issue.
   - Likely causes include serial quality, U2D2 timing, cable quality, power, or port contention.

Conclusion:

- `interbotix_xsarm_control xsarm_control.launch.py` is only the low-level arm driver plus description.
- In this workspace, the correct higher-level model is `step1_startup.launch.py`, where `/joint_states` is produced by `joint_state_merger`.

References:
- [rx200.yaml](/home/yanjiun68/workspace/WS_phIV/src/ARM_LIFTING/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_control/config/rx200.yaml)
- [step1_startup.launch.py](/home/yanjiun68/workspace/WS_phIV/src/ARM_LIFTING/rx200_handbag_grasp/launch/step1_startup.launch.py)

## 7. Compact Topic Architecture Map

The effective end-to-end data flow under `all_in_one.launch.py` is:

1. Hardware and planning state
2. Perception
3. 3D grasp estimation
4. Motion execution
5. Gripper and lift actuation

### 7.1 Robot State Layer

From Step 1:

- `xs_sdk` publishes `/rx200/sdk_joint_states`
- `lift_serial_driver` publishes `/lift/joint_state`
- `joint_state_merger` publishes `/joint_states`
- `robot_state_publisher` consumes `/joint_states` and produces `/tf` and `/tf_static`
- `move_group` consumes `/joint_states`

Interpretation:

- `/joint_states` is the real planning-state topic for this workspace.

### 7.2 Camera and Detection Layer

From Step 2:

- RealSense color topic:
  - `/camera/camera/color/image_raw/compressed`
- Camera intrinsics:
  - `/camera/camera/color/camera_info`
- Aligned depth:
  - `/camera/camera/aligned_depth_to_color/image_raw`

Then:

- `yolo_handle_detector.py`
  - subscribes to color image
  - publishes `/yolo/detections`
  - publishes `/yolo/mask`
- `hand_bag_selector.py`
  - consumes `/yolo/detections`
  - publishes `/handbag/selected_detections`
- `camera_tf_publisher.py`
  - ensures the runtime camera TF chain is available

### 7.3 Grasp Estimation Layer

`grasp_node.py` subscribes to:

- `/camera/camera/color/camera_info`
- `/camera/camera/aligned_depth_to_color/image_raw`
- `/handbag/selected_detections`
- `/yolo/mask`
- `/joint_states`

It publishes:

- `/grasp/target_pose`
- `/grasp/pregrasp_pose`
- `/grasp/debug_markers`
- `/grasp/busy`

Internal logic:

- sync detections with aligned depth
- robust depth extraction in ROI
- optional mask-constrained depth sampling
- back-project to 3D
- apply offsets
- transform into `lift_base_link`
- publish motion targets

### 7.4 TF Layer

The intended frame logic is:

- reference frame: `camera_color_optical_frame`
- base planning frame: `lift_base_link`
- end effector link: `rx200/ee_gripper_link`

The TF chain is conceptually:

```text
lift_base_link
  -> ... robot links ...
  -> rx200/gripper_prop_link
  -> rx200/camera_mount_link
  -> camera_link
  -> camera_color_optical_frame
```

`grasp_node.py` transforms camera-frame grasp poses into `lift_base_link`.

### 7.5 MoveIt and Execution Layer

MoveIt groups:

- `interbotix_arm`
- `interbotix_lift`
- `interbotix_arm_lift`

Execution sequence in `grasp_node.py`:

1. move to pregrasp
2. visual-servo centering
3. wrist alignment
4. camera-to-gripper offset shift
5. descent
6. gripper close
7. lift / retract

Service entrypoint:

- `/grasp/execute`

### 7.6 Final Actuation Outputs

- Arm motion goes through MoveIt and ros2_control controllers
- Gripper command topic:
  - `/rx200/commands/joint_single`
- Lift action:
  - `/lift_controller/follow_joint_trajectory`

## 8. Compact End-to-End Flow

```text
RealSense color/depth
  -> YOLO handle detector
  -> selected handbag detection
  -> grasp_node syncs detection + aligned depth + camera_info
  -> masked depth sampling + 3D back-projection
  -> TF transform to lift_base_link
  -> publish grasp/pregrasp pose
  -> MoveIt plan
  -> visual servo correction
  -> arm controller motion
  -> gripper command on /rx200/commands/joint_single
  -> lift action on /lift_controller/follow_joint_trajectory
```
