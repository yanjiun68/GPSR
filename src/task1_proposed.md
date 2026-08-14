# ============================================================
# SMACC2 + ROS 2 LIFECYCLE INTEGRATION PLAN
# Task: CARRY MY LUGGAGE (Flowchart V4)
# PutrabotUPM Project
# ============================================================

## 📋 Table of Contents

1. [Thinking Process](#1-thinking-process)
2. [Architecture Overview](#2-architecture-overview)
3. [Package Discovery (src/ only)](#3-package-discovery)
4. [SMACC2 Concepts Used](#4-smacc2-concepts-used)
5. [ROS 2 Lifecycle Integration](#5-ros-2-lifecycle-integration)
6. [State → Package Mapping](#6-state-package-mapping)
7. [SMACC2 State Machine Structure](#7-smacc2-state-machine-structure)
8. [Process Flow Diagram](#8-process-flow-diagram)
9. [Implementation Steps](#9-implementation-steps)
10. [File Structure](#10-file-structure)

---

## 1. Thinking Process

### Step 1: Understanding the Task
The "Carry My Luggage" task is a **deterministic reactive state machine** with 14 states (S0–S13).
The robot must:
- Detect a fingertip pointing at a bag
- Select and navigate to the nearest bag
- Grasp the bag with a robotic arm
- Follow the operator while avoiding obstacles
- Handover or decline, then return home

### Step 2: Survey Available Packages in `src/`
I examined every package under `src/` to understand what ROS 2 nodes, topics, services,
and actions are already available. The goal is to **reuse existing code as SMACC2 clients**
rather than rewriting functionality.

### Step 3: Why SMACC2?
SMACC2 (State Machine Asynchronous Composition and Control) is ideal because:
- **Orthogonal states**: Multiple subsystems (vision, arm, navigation) run simultaneously
- **Client behaviors**: Each ROS 2 node maps to a SMACC2 client (action/service/topic)
- **State reactors**: Handle asynchronous events (obstacle detected, operator facing)
- **Lifecycle integration**: Manage node startup/teardown per state

### Step 4: Why ROS 2 Lifecycle?
Lifecycle (managed) nodes allow:
- **Controlled startup**: Sensors only activate when their state begins
- **Resource management**: Close unnecessary nodes between states (save CPU)
- **Error recovery**: Transition to error states on failure
- **Deterministic shutdown**: Clean teardown at S13

### Step 5: Mapping States to Packages
Each state was mapped to the specific packages and nodes in `src/` that provide
the required functionality. See Section 6 for the full mapping.

---

## 2. Architecture Overview

```
┌──────────────────────────────────────────────────────────┐
│                    SMACC2 STATE MACHINE                  │
│              (carry_my_luggage_sm package)                │
│                   [in INTEGRATION/]                       │
│                                                          │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌─────────┐ │
│  │ VISION   │  │ ARM/GRASP│  │NAVIGATION│  │  VOICE  │ │
│  │ Client   │  │ Client   │  │ Client   │  │ Client  │ │
│  │(Orthog.1)│  │(Orthog.2)│  │(Orthog.3)│  │(Orthog.4│ │
│  └─────┬────┘  └─────┬────┘  └─────┬────┘  └────┬────┘ │
│        │              │              │             │      │
│  ┌─────▼────┐  ┌─────▼────┐  ┌─────▼────┐  ┌────▼────┐ │
│  │Lifecycle │  │Lifecycle │  │Lifecycle │  │Lifecycl│  │
│  │ Manager  │  │ Manager  │  │ Manager  │  │Manager │  │
│  └─────┬────┘  └─────┬────┘  └─────┬────┘  └────┬────┘ │
└────────┼──────────────┼──────────────┼────────────┼──────┘
         │              │              │            │
    ┌────▼─────┐  ┌─────▼────┐  ┌─────▼────┐ ┌────▼─────┐
    │vision_   │  │grasp_    │  │Nav2 +    │ │voice_   │
    │node.py   │  │node.py   │  │return_   │ │node     │
    │          │  │lift_     │  │home.py   │ │(TTS/STT)│
    │YOLO+Depth│  │serial_   │  │putrabot_ │ │         │
    │          │  │driver.py │  │base      │ │         │
    └──────────┘  └──────────┘  └──────────┘ └─────────┘
         ▲              ▲              ▲
    ┌────┴─────┐  ┌─────┴────┐  ┌─────┴────┐
    │realsense │  │interbotix│  │rplidar_  │
    │-ros      │  │_ros_*    │  │ros /     │
    │(camera)  │  │(RX200)   │  │sllidar   │
    └──────────┘  └──────────┘  └──────────┘
```

---

## 3. Package Discovery

### Packages found in `src/` folder:

| # | Category       | Package Path                                 | Key Nodes / Scripts                        | ROS 2 Type     |
|---|----------------|----------------------------------------------|--------------------------------------------|----------------|
| 1 | **VISION**     | `VISION/`                                    | `vision_node.py` (YOLO + depth + TF)       | Python, rclpy  |
| 2 | **VISION**     | `VISION/msg/`                                | `CamDetection.msg`, `CamDetections.msg`    | Custom msg     |
| 3 | **ARM/GRASP**  | `ARM_LIFTING/rx200_handbag_grasp/`           | `grasp_node.py` (MoveIt + visual servo)    | Python, rclpy  |
| 4 | **ARM/GRASP**  | `ARM_LIFTING/rx200_handbag_grasp/`           | `yolo_handle_detector.py` (handle detect)  | Python, rclpy  |
| 5 | **ARM/GRASP**  | `ARM_LIFTING/rx200_handbag_grasp/`           | `camera_tf_publisher.py` (static TF)       | Python, rclpy  |
| 6 | **ARM/LIFT**   | `ARM_LIFTING/lift_bridge/`                   | `lift_serial_driver.py` (linear lift)      | Python, rclpy  |
| 7 | **ARM/CORE**   | `ARM_LIFTING/interbotix_ros_core/`           | Interbotix drivers, SDK                    | C++/Python     |
| 8 | **ARM/MANIP**  | `ARM_LIFTING/interbotix_ros_manipulators/`   | Arm manipulation toolbox                   | C++/Python     |
| 9 | **ARM/TOOLS**  | `ARM_LIFTING/interbotix_ros_toolboxes/`      | Common arm utilities                       | C++/Python     |
|10 | **ARM/CONFIG** | `ARM_LIFTING/lift_moveit_config/`            | MoveIt configuration                       | Config         |
|11 | **ARM/DESC**   | `ARM_LIFTING/lift_description/`              | Lift URDF                                  | URDF           |
|12 | **ARM/VIS**    | `ARM_LIFTING/moveit_visual_tools/`           | RViz debugging tools                       | C++            |
|13 | **ARM/TELEOP** | `ARM_LIFTING/ps4_teleop/`                    | PS4 manual control                         | Python         |
|14 | **ARM/GRASP2** | `ARM_LIFTING/rx200_official_grasp/`          | Official grasp (alt. method)               | Python         |
|15 | **NAV**        | `NAVIGATION/my_robot_nav/`                   | Nav2 config, return_home.py, SLAM          | Python/Config  |
|16 | **NAV/BASE**   | `NAVIGATION/putrabot_base/`                  | cmd_vel, wheel odometry, serial_bridge     | Python, rclpy  |
|17 | **NAV/DESC**   | `NAVIGATION/robot_description/`              | Robot URDF + meshes                        | URDF           |
|18 | **VOICE**      | `VOICE/`                                     | ament_cmake package (TBD: TTS/STT)         | C++ (skeleton) |
|19 | **SENSOR**     | `realsense-ros/`                             | Intel RealSense D400 driver                | C++            |
|20 | **SENSOR**     | `rplidar_ros/`                               | RPLIDAR LiDAR driver                       | C++            |
|21 | **SENSOR**     | `sllidar_ros2/`                              | Slamtec LiDAR driver                       | C++            |
|22 | **INTEG.**     | `INTEGRATION/`                               | **EMPTY** – target for SMACC2 package      | –              |

---

## 4. SMACC2 Concepts Used

### 4.1 State Machine (SmaccStateMachine)
- **Top-level class**: `SmCarryMyLuggage` – the main SMACC2 state machine
- Contains all 14 states as C++ classes inheriting from `smacc2::SmaccState`

### 4.2 Orthogonal Components (Orthogonals)
Each orthogonal runs **concurrently** within a state. We define 4 orthogonals:

| Orthogonal        | Purpose                        | Packages Used                                      |
|-------------------|--------------------------------|----------------------------------------------------|
| `OrVision`        | Camera, YOLO, detection        | `VISION/`, `realsense-ros`                         |
| `OrArmGrasp`      | Arm control, gripper, lift     | `rx200_handbag_grasp/`, `lift_bridge/`, `interbotix_ros_*` |
| `OrNavigation`    | Nav2, base driver, LiDAR       | `my_robot_nav/`, `putrabot_base/`, `rplidar_ros`   |
| `OrVoice`         | Text-to-speech, speech-to-text | `VOICE/`                                           |

### 4.3 Client Behaviors (CbXxx)
Client behaviors are the **actions** that run inside each orthogonal per state:

| Client Behavior           | Orthogonal    | Description                              |
|---------------------------|---------------|------------------------------------------|
| `CbDetectFingertip`       | OrVision      | Subscribe to VISION detections, find fingertip |
| `CbDetectBags`            | OrVision      | Subscribe to CamDetections, filter bags  |
| `CbSelectNearestBag`      | OrVision      | Compute distances, select closest to fingertip |
| `CbNavigateToTarget`      | OrNavigation  | Send Nav2 goal to bag/origin position    |
| `CbFollowOperator`        | OrNavigation  | Continuous follow behavior               |
| `CbReroutePath`           | OrNavigation  | Re-plan around obstacles                 |
| `CbMoveArmToGrasp`        | OrArmGrasp    | Visual servo to grasp pose               |
| `CbCloseGripper`          | OrArmGrasp    | Close gripper command                    |
| `CbLiftObject`            | OrArmGrasp    | Lift via lift_serial_driver              |
| `CbVerifyGrasp`           | OrArmGrasp    | Check finger position for success        |
| `CbExtendArm`             | OrArmGrasp    | Extend arm for handover                  |
| `CbOpenGripper`           | OrArmGrasp    | Open gripper to release                  |
| `CbRetractArm`            | OrArmGrasp    | Retract arm after handover               |
| `CbSpeak`                 | OrVoice       | Text-to-speech output                    |
| `CbListenForResponse`     | OrVoice       | Speech-to-text input                     |
| `CbDetectObstacle`        | OrNavigation  | LiDAR-based obstacle detection           |
| `CbTrackOperator`         | OrVision      | Person tracking + re-ID                  |
| `CbDetectMultipleHumanoids`| OrVision     | Multiple person detection                |

### 4.4 State Reactors & Event Generators
- **EvFingertipDetected**: Fires when fingertip is found → transition S1→S2
- **EvBagsDetected**: Fires when ≥2 bags detected → transition S2→S3
- **EvAtTarget**: Fires when Nav2 goal reached → S4→S5 or S12→S13
- **EvGraspSuccess**: Fires when grasp verified → S5→S6
- **EvGraspFailed**: Fires when grasp fails → retry in S5
- **EvObstacleDetected**: Fires during follow → S6→S7
- **EvOperatorFacing**: Fires when operator faces robot → S6→S9
- **EvMultipleHumanoids**: Fires in obstacle handler → S7→S8
- **EvSpeechYes**: Fires on "YES" response → S9→S10
- **EvSpeechNo**: Fires on "NO" response → S9→S11
- **EvHandoverComplete**: Fires after grip detected + timer → S10→S12
- **EvArrivedOrigin**: Fires when back at start → S12→S13

---

## 5. ROS 2 Lifecycle Integration

### 5.1 Why Lifecycle?
Per the user's requirement to **"close unnecessary programs after use to conserve CPU"**,
we use ROS 2 Lifecycle (managed) nodes to control when each subsystem is active.

### 5.2 Lifecycle States
```
                ┌──────────────┐
                │  UNCONFIGURED│  ← Node created but not ready
                └──────┬───────┘
                       │ configure()
                ┌──────▼───────┐
                │   INACTIVE   │  ← Configured but not processing
                └──────┬───────┘
                       │ activate()
                ┌──────▼───────┐
                │    ACTIVE    │  ← Processing data, subscriptions active
                └──────┬───────┘
                       │ deactivate()
                ┌──────▼───────┐
                │   INACTIVE   │  ← Paused, resources still allocated
                └──────┬───────┘
                       │ cleanup()
                ┌──────▼───────┐
                │  UNCONFIGURED│  ← Resources freed
                └──────┬───────┘
                       │ shutdown()
                ┌──────▼───────┐
                │   FINALIZED  │  ← Node destroyed
                └──────────────┘
```

### 5.3 Lifecycle Management per State

| State | Nodes ACTIVE                                               | Nodes INACTIVE / UNCONFIGURED               |
|-------|------------------------------------------------------------|----------------------------------------------|
| S0    | All nodes → configure + activate                           | –                                            |
| S1    | vision_node, realsense camera                              | grasp_node, Nav2, voice_node                 |
| S2    | vision_node, realsense camera                              | grasp_node, Nav2, voice_node                 |
| S3    | vision_node, **voice_node** (for speak)                    | grasp_node, Nav2                             |
| S4    | Nav2, putrabot_base, rplidar, vision_node                  | grasp_node, voice_node                       |
| S5    | grasp_node, yolo_handle_detector, lift_bridge, interbotix  | Nav2 (deactivated), voice_node               |
| S6    | Nav2, putrabot_base, vision_node, rplidar                  | grasp_node (deactivated), voice_node         |
| S7    | vision_node, Nav2 (replanning), rplidar                    | grasp_node, voice_node                       |
| S8    | vision_node (re-ID)                                        | Nav2 (paused), grasp_node, voice_node        |
| S9    | **voice_node** (speak + listen), vision_node               | Nav2 (paused), grasp_node                    |
| S10   | grasp_node, lift_bridge, interbotix                        | Nav2, voice_node                             |
| S11   | voice_node (optional announcement)                         | Nav2, grasp_node                             |
| S12   | Nav2, putrabot_base, rplidar                               | grasp_node, voice_node, vision_node          |
| S13   | ALL → deactivate → cleanup → shutdown                      | ALL → FINALIZED                              |

### 5.4 SMACC2 Lifecycle Client
SMACC2 provides `cl_lifecyclenode` client to manage lifecycle transitions:

```cpp
// Example: In StS1DetectFingertip::onEntry()
// Activate vision, deactivate arm/nav/voice

// Get the lifecycle clients from orthogonals
auto vision_lc = this->getOrthogonal<OrVision>()
    ->getClientBehavior<CbLifecycleActivate>();

auto arm_lc = this->getOrthogonal<OrArmGrasp>()
    ->getClientBehavior<CbLifecycleDeactivate>();

auto nav_lc = this->getOrthogonal<OrNavigation>()
    ->getClientBehavior<CbLifecycleDeactivate>();
```

---

## 6. State → Package Mapping

```
┌─────────┬───────────────────────────┬──────────────────────────────────────────────────┐
│ State   │ Name                      │ Packages from src/                               │
├─────────┼───────────────────────────┼──────────────────────────────────────────────────┤
│ S0      │ START                     │ ALL packages (system bringup)                    │
│         │                           │ • robot_description/ (URDF)                      │
│         │                           │ • realsense-ros/ (camera)                        │
│         │                           │ • rplidar_ros/ or sllidar_ros2/ (LiDAR)          │
│         │                           │ • interbotix_ros_* (arm drivers)                 │
│         │                           │ • lift_bridge/ (lift actuator)                   │
│         │                           │ • putrabot_base/ (motors)                        │
├─────────┼───────────────────────────┼──────────────────────────────────────────────────┤
│ S1      │ DETECT FINGERTIP          │ • VISION/scripts/vision_node.py                  │
│         │                           │   (YOLO body keypoint detection)                 │
│         │                           │ • realsense-ros/ (RGB-D images)                  │
├─────────┼───────────────────────────┼──────────────────────────────────────────────────┤
│ S2      │ DETECT BAGS               │ • VISION/scripts/vision_node.py                  │
│         │                           │   (YOLO object detection, "bag" class)           │
│         │                           │ • VISION/msg/CamDetection.msg (x,y,z,conf)       │
│         │                           │ • VISION/msg/CamDetections.msg (array)            │
│         │                           │ • realsense-ros/ (depth for 3D positions)        │
├─────────┼───────────────────────────┼──────────────────────────────────────────────────┤
│ S3      │ SELECT NEAREST BAG        │ • VISION/scripts/vision_node.py (3D positions)   │
│         │                           │ • VOICE/ (speak: "I will carry this bag")        │
├─────────┼───────────────────────────┼──────────────────────────────────────────────────┤
│ S4      │ MOVE TO SELECTED BAG      │ • NAVIGATION/my_robot_nav/ (Nav2 action)         │
│         │                           │ • NAVIGATION/putrabot_base/ (cmd_vel, odom)      │
│         │                           │ • rplidar_ros/ or sllidar_ros2/ (obstacle avoid)  │
├─────────┼───────────────────────────┼──────────────────────────────────────────────────┤
│ S5      │ GRASP BAG                 │ • ARM_LIFTING/rx200_handbag_grasp/grasp_node.py  │
│         │                           │   (move_arm_to → close_gripper → verify_grasp)   │
│         │                           │ • ARM_LIFTING/rx200_handbag_grasp/               │
│         │                           │   yolo_handle_detector.py (handle location)      │
│         │                           │ • ARM_LIFTING/lift_bridge/lift_serial_driver.py   │
│         │                           │   (lift_object via GOTO command)                 │
│         │                           │ • ARM_LIFTING/interbotix_ros_manipulators/        │
│         │                           │ • ARM_LIFTING/lift_moveit_config/                 │
├─────────┼───────────────────────────┼──────────────────────────────────────────────────┤
│ S6      │ FOLLOW OPERATOR           │ • NAVIGATION/my_robot_nav/ (Nav2 follow mode)    │
│         │                           │ • NAVIGATION/putrabot_base/ (base control)       │
│         │                           │ • VISION/scripts/vision_node.py (operator track)  │
│         │                           │ • rplidar_ros/ or sllidar_ros2/ (obstacle scan)   │
├─────────┼───────────────────────────┼──────────────────────────────────────────────────┤
│ S7      │ OBSTACLE HANDLER          │ • VISION/scripts/vision_node.py (humanoid detect) │
│         │                           │ • NAVIGATION/my_robot_nav/ (path replanning)     │
│         │                           │ • rplidar_ros/ or sllidar_ros2/                   │
├─────────┼───────────────────────────┼──────────────────────────────────────────────────┤
│ S8      │ SELECT CORRECT HUMANOID   │ • VISION/scripts/vision_node.py                  │
│         │                           │   (person re-ID via embedding field in           │
│         │                           │    CamDetection.msg)                             │
├─────────┼───────────────────────────┼──────────────────────────────────────────────────┤
│ S9      │ ASK OPERATOR FOR HANDOVER │ • VOICE/ (TTS: "Do you want to take the bag?")  │
│         │                           │ • VOICE/ (STT: listen for YES/NO)                │
├─────────┼───────────────────────────┼──────────────────────────────────────────────────┤
│ S10     │ HANDOVER SEQUENCE         │ • ARM_LIFTING/rx200_handbag_grasp/grasp_node.py  │
│         │                           │   (extend_arm, detect_grab, open_gripper,        │
│         │                           │    retract_arm)                                  │
│         │                           │ • ARM_LIFTING/lift_bridge/lift_serial_driver.py   │
│         │                           │ • ARM_LIFTING/interbotix_ros_manipulators/        │
├─────────┼───────────────────────────┼──────────────────────────────────────────────────┤
│ S11     │ OPERATOR DECLINES         │ • VOICE/ (announce: "Transfer complete")         │
│         │                           │ • ARM_LIFTING/rx200_handbag_grasp/ (optional:     │
│         │                           │   place bag down)                                │
├─────────┼───────────────────────────┼──────────────────────────────────────────────────┤
│ S12     │ RETURN TO ORIGIN          │ • NAVIGATION/my_robot_nav/src/return_home.py     │
│         │                           │   (Nav2 action client → origin (0,0))            │
│         │                           │ • NAVIGATION/putrabot_base/                       │
│         │                           │ • rplidar_ros/ or sllidar_ros2/                   │
├─────────┼───────────────────────────┼──────────────────────────────────────────────────┤
│ S13     │ END                       │ ALL → Lifecycle shutdown → terminate_task()       │
└─────────┴───────────────────────────┴──────────────────────────────────────────────────┘
```

---

## 7. SMACC2 State Machine Structure

### 7.1 State Machine Class Hierarchy

```cpp
// ============================================================
// Top-level State Machine
// ============================================================
struct SmCarryMyLuggage : smacc2::SmaccStateMachineBase<SmCarryMyLuggage, StS0Start>
{
    // Define orthogonals (concurrent subsystems)
    void onInitialize() override
    {
        this->createOrthogonal<OrVision>();       // Camera + YOLO
        this->createOrthogonal<OrArmGrasp>();     // RX200 arm + gripper + lift
        this->createOrthogonal<OrNavigation>();   // Nav2 + base driver
        this->createOrthogonal<OrVoice>();        // TTS + STT
    }
};

// ============================================================
// Individual State Definitions (one class per state)
// ============================================================

// S0: START – Initialize all systems
struct StS0Start : smacc2::SmaccState<StS0Start, SmCarryMyLuggage>
{
    typedef mpl::list<
        Transition<EvAllInitialized, StS1DetectFingertip>  // → S1
    > reactions;

    void onEntry() {
        // Configure and activate all lifecycle nodes
    }
};

// S1: DETECT FINGERTIP
struct StS1DetectFingertip : smacc2::SmaccState<StS1DetectFingertip, SmCarryMyLuggage>
{
    typedef mpl::list<
        Transition<EvFingertipDetected, StS2DetectBags>    // → S2
    > reactions;

    void onEntry() {
        // Activate: OrVision with CbDetectFingertip
        // Deactivate: OrArmGrasp, OrNavigation, OrVoice
    }
};

// S2: DETECT BAGS
struct StS2DetectBags : smacc2::SmaccState<StS2DetectBags, SmCarryMyLuggage>
{
    typedef mpl::list<
        Transition<EvBagsDetected, StS3SelectNearestBag>   // → S3
    > reactions;

    void onEntry() {
        // Activate: OrVision with CbDetectBags (requires ≥2)
    }
};

// S3: SELECT NEAREST BAG
struct StS3SelectNearestBag : smacc2::SmaccState<StS3SelectNearestBag, SmCarryMyLuggage>
{
    typedef mpl::list<
        Transition<EvBagSelected, StS4MoveToSelectedBag>   // → S4
    > reactions;

    void onEntry() {
        // OrVision: CbSelectNearestBag
        // OrVoice: CbSpeak("I will carry this bag.")
    }
};

// S4: MOVE TO SELECTED BAG
struct StS4MoveToSelectedBag : smacc2::SmaccState<StS4MoveToSelectedBag, SmCarryMyLuggage>
{
    typedef mpl::list<
        Transition<EvAtTarget, StS5GraspBag>               // → S5
    > reactions;

    void onEntry() {
        // Activate: OrNavigation with CbNavigateToTarget
        // Deactivate: OrArmGrasp, OrVoice
    }
};

// S5: GRASP BAG
struct StS5GraspBag : smacc2::SmaccState<StS5GraspBag, SmCarryMyLuggage>
{
    typedef mpl::list<
        Transition<EvGraspSuccess, StS6FollowOperator>,    // → S6
        Transition<EvGraspFailed, StS5GraspBag>            // retry (self)
    > reactions;

    void onEntry() {
        // Activate: OrArmGrasp with CbMoveArmToGrasp, CbCloseGripper,
        //           CbLiftObject, CbVerifyGrasp
        // Deactivate: OrNavigation
    }
};

// S6: FOLLOW OPERATOR
struct StS6FollowOperator : smacc2::SmaccState<StS6FollowOperator, SmCarryMyLuggage>
{
    typedef mpl::list<
        Transition<EvObstacleDetected,  StS7ObstacleHandler>,   // → S7
        Transition<EvOperatorFacing,    StS9AskOperator>        // → S9
    > reactions;

    void onEntry() {
        // Activate: OrNavigation (CbFollowOperator),
        //           OrVision (CbTrackOperator + CbDetectObstacle)
        // Deactivate: OrArmGrasp, OrVoice
    }
};

// S7: OBSTACLE HANDLER
struct StS7ObstacleHandler : smacc2::SmaccState<StS7ObstacleHandler, SmCarryMyLuggage>
{
    typedef mpl::list<
        Transition<EvMultipleHumanoids, StS8SelectCorrectHumanoid>,  // → S8
        Transition<EvPathRerouted,      StS6FollowOperator>          // → S6
    > reactions;

    void onEntry() {
        // OrVision: CbDetectMultipleHumanoids
        // OrNavigation: CbReroutePath (if no humanoids)
    }
};

// S8: SELECT CORRECT HUMANOID
struct StS8SelectCorrectHumanoid : smacc2::SmaccState<StS8SelectCorrectHumanoid, SmCarryMyLuggage>
{
    typedef mpl::list<
        Transition<EvCorrectOperatorFound, StS6FollowOperator>  // → S6
    > reactions;

    void onEntry() {
        // OrVision: CbIdentifyCorrectOperator (embedding matching)
    }
};

// S9: ASK OPERATOR FOR HANDOVER
struct StS9AskOperator : smacc2::SmaccState<StS9AskOperator, SmCarryMyLuggage>
{
    typedef mpl::list<
        Transition<EvSpeechYes, StS10HandoverSequence>,    // → S10
        Transition<EvSpeechNo,  StS11OperatorDeclines>     // → S11
    > reactions;

    void onEntry() {
        // Activate: OrVoice (CbSpeak + CbListenForResponse)
        // Deactivate: OrNavigation, OrArmGrasp
    }
};

// S10: HANDOVER SEQUENCE
struct StS10HandoverSequence : smacc2::SmaccState<StS10HandoverSequence, SmCarryMyLuggage>
{
    typedef mpl::list<
        Transition<EvHandoverComplete, StS12ReturnToOrigin>  // → S12
    > reactions;

    void onEntry() {
        // Activate: OrArmGrasp (CbExtendArm → wait for grab → CbOpenGripper → CbRetractArm)
    }
};

// S11: OPERATOR DECLINES
struct StS11OperatorDeclines : smacc2::SmaccState<StS11OperatorDeclines, SmCarryMyLuggage>
{
    typedef mpl::list<
        Transition<EvTransferIndicated, StS12ReturnToOrigin>  // → S12
    > reactions;

    void onEntry() {
        // OrVoice: CbSpeak("Transfer complete")
    }
};

// S12: RETURN TO ORIGIN
struct StS12ReturnToOrigin : smacc2::SmaccState<StS12ReturnToOrigin, SmCarryMyLuggage>
{
    typedef mpl::list<
        Transition<EvArrivedOrigin, StS13End>  // → S13
    > reactions;

    void onEntry() {
        // Activate: OrNavigation (CbNavigateToTarget with origin_pose)
        // Deactivate: OrVision, OrArmGrasp, OrVoice
    }
};

// S13: END
struct StS13End : smacc2::SmaccState<StS13End, SmCarryMyLuggage>
{
    void onEntry() {
        // Shutdown all lifecycle nodes
        // terminate_task()
    }
};
```

---

## 8. Process Flow Diagram

```
                        ┌─────────────────────┐
                        │   SmCarryMyLuggage   │
                        │   (State Machine)    │
                        └──────────┬──────────┘
                                   │
            ┌──────────┬───────────┼───────────┬──────────┐
            │          │           │           │          │
      ┌─────▼─────┐ ┌─▼──────┐ ┌─▼────────┐ ┌▼────────┐│
      │ OrVision  │ │OrArm   │ │OrNavigat.│ │OrVoice  ││
      │           │ │Grasp   │ │          │ │         ││
      │ Clients:  │ │Clients:│ │ Clients: │ │Clients: ││
      │•ClVision  │ │•ClGrasp│ │•ClNav2   │ │•ClTTS   ││
      │•ClRealsns │ │•ClLift │ │•ClBase   │ │•ClSTT   ││
      │           │ │•ClArm  │ │•ClLidar  │ │         ││
      └───────────┘ └────────┘ └──────────┘ └─────────┘│
                                                        │
                    S T A T E   T R A N S I T I O N S   │
                    ─────────────────────────────────────┘

  ┌───────┐     ┌───────┐     ┌───────┐     ┌───────┐
  │  S0   │────▶│  S1   │────▶│  S2   │────▶│  S3   │
  │ START │     │FINGER │     │ BAGS  │     │SELECT │
  │       │     │ TIP   │     │DETECT │     │NEAREST│
  └───────┘     └───────┘     └───────┘     └───┬───┘
                                                 │
  ┌───────┐     ┌───────┐                   ┌───▼───┐
  │  S6   │◀────│  S5   │◀──────────────────│  S4   │
  │FOLLOW │     │GRASP  │                   │MOVE TO│
  │OPERATR│     │ BAG   │◀─ retry           │  BAG  │
  └─┬──┬──┘     └───────┘                   └───────┘
    │  │
    │  │          ┌───────┐     ┌───────┐
    │  └─────────▶│  S7   │────▶│  S8   │────┐
    │  obstacle   │OBSTACL│     │CORRECT│    │
    │             │HANDLER│     │HUMANOD│    │
    │             └───┬───┘     └───────┘    │
    │                 │ reroute              │
    │                 └──────────────────────┘
    │                          ↓ back to S6
    │
    │  operator facing
    │  ┌───────┐     ┌───────┐     ┌───────┐
    └─▶│  S9   │────▶│  S10  │────▶│  S12  │
       │  ASK  │ YES │HANDOVR│     │RETURN │
       │OPERATR│     │SEQUNCE│     │ORIGIN │
       └───┬───┘     └───────┘     └───┬───┘
           │ NO                        │
       ┌───▼───┐                   ┌───▼───┐
       │  S11  │──────────────────▶│  S13  │
       │DECLINE│                   │  END  │
       └───────┘                   └───────┘
```

---

## 9. Implementation Steps

### Phase 1: Create SMACC2 Package
1. Create `INTEGRATION/carry_my_luggage_sm/` package
2. Define `CMakeLists.txt` with SMACC2 dependencies
3. Define `package.xml` with all required dependencies

### Phase 2: Define Orthogonals
4. Create `orthogonals/or_vision.hpp`
5. Create `orthogonals/or_arm_grasp.hpp`
6. Create `orthogonals/or_navigation.hpp`
7. Create `orthogonals/or_voice.hpp`

### Phase 3: Define Clients
8. Wrap existing nodes as SMACC2 clients
   - `ClVisionNode` → wraps `vision_node.py` topics
   - `ClGraspNode` → wraps `grasp_node.py` actions/services
   - `ClLiftBridge` → wraps `lift_serial_driver.py` actions/services
   - `ClNav2` → wraps Nav2 `navigate_to_pose` action
   - `ClBaseDriver` → wraps `putrabot_base/` topics
   - `ClVoice` → wraps TTS/STT services (to be implemented)

### Phase 4: Define States (S0–S13)
9. Create state classes in `states/` directory
10. Define transitions and client behaviors per state
11. Implement lifecycle management per state transition

### Phase 5: Define Events
12. Create event classes for all transitions
13. Wire event generators to SMACC2 state reactors

### Phase 6: Launch & Testing
14. Create launch file for the state machine
15. Test each state transition individually
16. Run full integration test

---

## 10. File Structure

```
src/INTEGRATION/carry_my_luggage_sm/
├── CMakeLists.txt
├── package.xml
├── include/carry_my_luggage_sm/
│   ├── sm_carry_my_luggage.hpp          # Main state machine
│   ├── orthogonals/
│   │   ├── or_vision.hpp                # Vision orthogonal
│   │   ├── or_arm_grasp.hpp             # Arm + gripper orthogonal
│   │   ├── or_navigation.hpp            # Navigation orthogonal
│   │   └── or_voice.hpp                 # Voice orthogonal
│   ├── clients/
│   │   ├── cl_vision_node.hpp           # Wraps VISION/vision_node.py
│   │   ├── cl_grasp_node.hpp            # Wraps rx200_handbag_grasp/grasp_node.py
│   │   ├── cl_lift_bridge.hpp           # Wraps lift_bridge/lift_serial_driver.py
│   │   ├── cl_nav2.hpp                  # Wraps Nav2 navigate_to_pose action
│   │   ├── cl_base_driver.hpp           # Wraps putrabot_base
│   │   └── cl_voice.hpp                 # Wraps VOICE TTS/STT
│   ├── client_behaviors/
│   │   ├── cb_detect_fingertip.hpp      # S1 behavior
│   │   ├── cb_detect_bags.hpp           # S2 behavior
│   │   ├── cb_select_nearest_bag.hpp    # S3 behavior
│   │   ├── cb_navigate_to_target.hpp    # S4, S12 behavior
│   │   ├── cb_move_arm_to_grasp.hpp     # S5 behavior
│   │   ├── cb_close_gripper.hpp         # S5 behavior
│   │   ├── cb_lift_object.hpp           # S5 behavior
│   │   ├── cb_verify_grasp.hpp          # S5 behavior
│   │   ├── cb_follow_operator.hpp       # S6 behavior
│   │   ├── cb_reroute_path.hpp          # S7 behavior
│   │   ├── cb_detect_obstacle.hpp       # S6, S7 behavior
│   │   ├── cb_track_operator.hpp        # S6, S8 behavior
│   │   ├── cb_detect_humanoids.hpp      # S7 behavior
│   │   ├── cb_identify_operator.hpp     # S8 behavior
│   │   ├── cb_speak.hpp                 # S3, S9, S11 behavior
│   │   ├── cb_listen_response.hpp       # S9 behavior
│   │   ├── cb_extend_arm.hpp            # S10 behavior
│   │   ├── cb_open_gripper.hpp          # S10 behavior
│   │   ├── cb_retract_arm.hpp           # S10 behavior
│   │   └── cb_lifecycle_manage.hpp      # Lifecycle transitions
│   ├── states/
│   │   ├── st_s0_start.hpp
│   │   ├── st_s1_detect_fingertip.hpp
│   │   ├── st_s2_detect_bags.hpp
│   │   ├── st_s3_select_nearest_bag.hpp
│   │   ├── st_s4_move_to_selected_bag.hpp
│   │   ├── st_s5_grasp_bag.hpp
│   │   ├── st_s6_follow_operator.hpp
│   │   ├── st_s7_obstacle_handler.hpp
│   │   ├── st_s8_select_correct_humanoid.hpp
│   │   ├── st_s9_ask_operator.hpp
│   │   ├── st_s10_handover_sequence.hpp
│   │   ├── st_s11_operator_declines.hpp
│   │   ├── st_s12_return_to_origin.hpp
│   │   └── st_s13_end.hpp
│   └── events/
│       ├── ev_all_initialized.hpp
│       ├── ev_fingertip_detected.hpp
│       ├── ev_bags_detected.hpp
│       ├── ev_bag_selected.hpp
│       ├── ev_at_target.hpp
│       ├── ev_grasp_success.hpp
│       ├── ev_grasp_failed.hpp
│       ├── ev_obstacle_detected.hpp
│       ├── ev_operator_facing.hpp
│       ├── ev_multiple_humanoids.hpp
│       ├── ev_correct_operator_found.hpp
│       ├── ev_path_rerouted.hpp
│       ├── ev_speech_yes.hpp
│       ├── ev_speech_no.hpp
│       ├── ev_handover_complete.hpp
│       ├── ev_transfer_indicated.hpp
│       └── ev_arrived_origin.hpp
├── src/
│   └── sm_carry_my_luggage_node.cpp     # Main ROS 2 node entry point
├── launch/
│   └── carry_my_luggage.launch.py       # Launch file
└── config/
    └── state_machine_params.yaml        # Parameters for all states
```

---

## Summary

| Aspect           | Decision                                                       |
|------------------|----------------------------------------------------------------|
| **Framework**    | SMACC2 (C++ state machine, wraps Python ROS 2 nodes)          |
| **Lifecycle**    | ROS 2 Lifecycle nodes – activate/deactivate per state          |
| **Location**     | `src/INTEGRATION/carry_my_luggage_sm/`                         |
| **Orthogonals**  | 4: OrVision, OrArmGrasp, OrNavigation, OrVoice                |
| **States**       | 14 (S0–S13), each as a C++ SmaccState class                   |
| **Events**       | 17 custom events for state transitions                         |
| **Behaviors**    | 20+ client behaviors wrapping existing package functionality   |
| **CPU Saving**   | Lifecycle deactivate/cleanup for unused nodes per state        |
| **Packages Used**| ALL `src/` packages (22 packages, see Section 3)               |
