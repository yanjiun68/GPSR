# Navigation Behavior Trees

This folder contains package-local Nav2 behavior trees used by `my_robot_nav`.
They customize how the robot plans, follows paths, replans, and performs recovery
actions during navigation.

## Files

| File | Main use | Key behavior |
| --- | --- | --- |
| `navigate_to_pose_no_backup.xml` | Single-goal navigation | Fast replanning, no blind reverse |
| `navigate_through_poses_no_backup.xml` | Multi-waypoint navigation | Removes passed goals, no blind reverse |
| `navigate_smooth_recovery.xml` | Single-goal navigation with smoother motion | Smooths path and allows backup recovery |

## Behavior Tree Node Legend

```text
[RecoveryNode]      Try the main child first; if it fails, run recovery and retry.
[PipelineSequence]  Tick planning and control together so replanning can happen while moving.
[Fallback]          Try children from left to right until one succeeds.
[ReactiveSequence]  Re-check conditions every tick before continuing.
[ReactiveFallback]  Recovery branch that can stop early when the goal changes.
[RoundRobin]        Rotate through recovery actions across repeated failures.
```

## High-Level Navigation Flow

```text
Navigation goal received
        |
        v
Check whether current path can still be used
        |
        +-- Path valid and goal unchanged --> Continue following path
        |
        +-- Path invalid or goal changed --> Compute a new path
                                             |
                                             v
                                      Follow the path
                                             |
                         +-------------------+-------------------+
                         |                                       |
                      Success                                 Failure
                         |                                       |
                      Finish                         Run recovery actions
                                                                 |
                                                                 v
                                                         Retry navigation
```

## `navigate_to_pose_no_backup.xml`

This tree is for normal single-goal navigation. It replans quickly and avoids
blind reverse recovery, which makes it safer for cluttered RoboCup-style indoor
spaces.

### Tree Structure

```text
NavigateRecovery Retry x6
├── NavigateWithReplanning PipelineSequence
│   ├── RateControllerComputePathToPose 3.0 Hz
│   │   └── RecoveryComputePathToPose Retry x1
│   │       ├── FallbackComputePathToPose
│   │       │   ├── CheckIfNewPathNeeded ReactiveSequence
│   │       │   │   ├── Inverter
│   │       │   │   │   └── GlobalUpdatedGoal
│   │       │   │   └── IsPathValid path="{path}"
│   │       │   └── ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"
│   │       └── ClearGlobalCostmap-Context
│   └── RecoveryFollowPath Retry x1
│       ├── FollowPath path="{path}" controller_id="FollowPath"
│       └── ClearLocalCostmap-Context
└── FallbackRecoveries ReactiveFallback
    ├── GoalUpdated
    └── RecoveryActions RoundRobin
        ├── ClearingActions Sequence
        │   ├── ClearLocalCostmap-Subtree
        │   └── ClearGlobalCostmap-Subtree
        └── WaitAndSpin Sequence
            ├── WaitBeforeSpin 2s
            └── SpinRecovery 1.57 rad
```

### Transitions

```text
Start single-goal navigation
        |
        v
RateController ticks planning at 3.0 Hz
        |
        v
FallbackComputePathToPose
        |
        +-- Goal unchanged and path valid --> Return success without replanning
        |
        +-- Goal changed or path invalid --> ComputePathToPose
                                             |
                                             v
                                      FollowPath
                                             |
        +------------------------------------+------------------------------------+
        |                                                                         |
   FollowPath succeeds                                                       FollowPath fails
        |                                                                         |
   Navigation succeeds                                       Clear local costmap and retry once
                                                                                  |
                                                                                  v
                                                                   If still failing, use global recovery
```

### Recovery Behavior

```text
Navigation pipeline fails
        |
        v
FallbackRecoveries
        |
        +-- GoalUpdated succeeds --> Stop current recovery and restart with new goal
        |
        +-- No new goal ---------> RoundRobin recovery
                                   |
                      +------------+------------+
                      |                         |
                      v                         v
             Clear local/global maps       Wait 2s, then spin 1.57 rad
```

## `navigate_through_poses_no_backup.xml`

This tree is for waypoint missions. It navigates through multiple poses, removes
already-passed goals, replans when the path becomes invalid, and also avoids
blind reverse recovery.

### Tree Structure

```text
NavigateRecovery Retry x6
├── NavigateWithReplanning PipelineSequence
│   ├── RateControllerComputePathThroughPoses 2.0 Hz
│   │   └── RecoveryComputePathThroughPoses Retry x1
│   │       ├── FallbackComputePathThroughPoses
│   │       │   ├── CheckIfNewPathNeeded ReactiveSequence
│   │       │   │   ├── Inverter
│   │       │   │   │   └── GlobalUpdatedGoal
│   │       │   │   └── IsPathValid path="{path}"
│   │       │   └── ComputeActiveWaypointPath ReactiveSequence
│   │       │       ├── RemovePassedGoals radius="0.7"
│   │       │       └── ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="GridBased"
│   │       └── ClearGlobalCostmap-Context
│   └── RecoveryFollowPath Retry x1
│       ├── FollowPath path="{path}" controller_id="FollowPath"
│       └── ClearLocalCostmap-Context
└── FallbackRecoveries ReactiveFallback
    ├── GoalUpdated
    └── RecoveryActions RoundRobin
        ├── ClearingActions Sequence
        │   ├── ClearLocalCostmap-Subtree
        │   └── ClearGlobalCostmap-Subtree
        └── WaitAndSpin Sequence
            ├── WaitBeforeSpin 2s
            └── SpinRecovery 1.57 rad
```

### Transitions

```text
Start waypoint navigation
        |
        v
RateController ticks planning at 2.0 Hz
        |
        v
FallbackComputePathThroughPoses
        |
        +-- Goal list unchanged and path valid --> Continue using current path
        |
        +-- Goal list changed or path invalid --> RemovePassedGoals within 0.7 m
                                                |
                                                v
                                         ComputePathThroughPoses
                                                |
                                                v
                                           FollowPath
                                                |
        +---------------------------------------+--------------------------------------+
        |                                                                              |
   All waypoints reached                                                        Failure occurs
        |                                                                              |
   Mission succeeds                                             Clear costmaps, wait/spin, and retry
```

### Recovery Behavior

```text
Waypoint pipeline fails
        |
        v
FallbackRecoveries
        |
        +-- GoalUpdated succeeds --> Restart behavior around updated waypoint list
        |
        +-- No new goal ---------> RoundRobin recovery
                                   |
                      +------------+------------+
                      |                         |
                      v                         v
             Clear local/global maps       Wait 2s, then spin 1.57 rad
```

## `navigate_smooth_recovery.xml`

This tree is for single-goal navigation where smoother motion is desired. It
computes a path, smooths it, then follows the smoothed path. It includes backup
recovery, so it should be used carefully on a real robot around people or
furniture.

### Tree Structure

```text
NavigateRecovery Retry x6
├── PipelineSequence
│   ├── RateControllerComputePathToPose 1.0 Hz
│   │   └── RecoveryComputePathToPose Retry x1
│   │       ├── FallbackComputePathToPose
│   │       │   ├── CheckIfNewPathNeeded ReactiveSequence
│   │       │   │   ├── Inverter
│   │       │   │   │   └── GlobalUpdatedGoal
│   │       │   │   └── IsPathValid path="{path}"
│   │       │   └── ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"
│   │       └── ClearGlobalCostmap-Context
│   └── RecoveryFollowPath Retry x1
│       ├── FollowPathContext Sequence
│       │   ├── SmoothPath unsmoothed_path="{path}" smoothed_path="{smoothed_path}" smoother_id="simple_smoother"
│       │   └── FollowPath path="{smoothed_path}" controller_id="FollowPath"
│       └── ClearLocalCostmap-Context
└── FallbackRecoveries ReactiveFallback
    ├── GoalUpdated
    └── RecoveryActions RoundRobin
        ├── ClearingActions Sequence
        │   ├── ClearLocalCostmap-Subtree
        │   └── ClearGlobalCostmap-Subtree
        ├── WaitAndSpin Sequence
        │   ├── WaitBeforeSpin 3s
        │   └── SpinRecovery 1.57 rad
        └── BackUpRecovery backup_dist="0.30" backup_speed="0.05"
```

### Transitions

```text
Start single-goal navigation
        |
        v
RateController ticks planning at 1.0 Hz
        |
        v
FallbackComputePathToPose
        |
        +-- Goal unchanged and path valid --> Reuse current path
        |
        +-- Goal changed or path invalid --> ComputePathToPose
                                             |
                                             v
                                         SmoothPath
                                             |
                                             v
                                   Follow smoothed path
                                             |
        +------------------------------------+------------------------------------+
        |                                                                         |
   FollowPath succeeds                                                       FollowPath fails
        |                                                                         |
   Navigation succeeds                                       Clear local costmap and retry once
                                                                                  |
                                                                                  v
                                                                   If still failing, use global recovery
```

### Recovery Behavior

```text
Navigation pipeline fails
        |
        v
FallbackRecoveries
        |
        +-- GoalUpdated succeeds --> Stop recovery and restart with new goal
        |
        +-- No new goal ---------> RoundRobin recovery
                                   |
             +---------------------+---------------------+---------------------+
             |                                           |                     |
             v                                           v                     v
    Clear local/global maps                    Wait 3s, then spin       Back up 0.30 m
```

## Comparison

```text
                         Top-level RecoveryNode x6
                                      |
          +---------------------------+---------------------------+
          |                           |                           |
          v                           v                           v
navigate_to_pose_no_backup   navigate_through_poses_no_backup   navigate_smooth_recovery
Single goal                  Multiple waypoints                 Single goal
3.0 Hz replanning            2.0 Hz replanning                  1.0 Hz replanning
No backup                    No backup                          Has backup
No path smoothing            No path smoothing                  SmoothPath before FollowPath
Spin recovery                Spin recovery                      Spin and BackUp recovery
```

## RoboCup Usage Recommendation

```text
RoboCup navigation task
        |
        +-- Single destination task -----> navigate_to_pose_no_backup.xml
        |
        +-- Multi-waypoint task ---------> navigate_through_poses_no_backup.xml
        |
        +-- Need smoother path tracking -> navigate_smooth_recovery.xml
                                          |
                                          v
                                  Verify backup is safe first
```

For real RoboCup runs, prefer `navigate_to_pose_no_backup.xml` for single-goal
navigation and `navigate_through_poses_no_backup.xml` for waypoint tasks. These
two trees avoid blind reverse recovery, which is safer in indoor environments
with people, furniture, and narrow passages.

Use `navigate_smooth_recovery.xml` when path smoothness is more important, but
review the `BackUpRecovery` behavior before enabling it on the real robot.
