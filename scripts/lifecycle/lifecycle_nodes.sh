#!/usr/bin/env bash
# Section: Canonical lifecycle-node inventory reviewed from the workspace launch files.
#
# This file is sourced by lifecycle helper scripts.  It declares possible node
# names; it never starts, stops, configures, or activates a node by itself.

# Section: Nav2 nodes managed by localization_launch.py, navigation_launch.py, and the collision manager.
NAV2_LIFECYCLE_NODES=(
    "/map_server"
    "/amcl"
    "/controller_server"
    "/smoother_server"
    "/planner_server"
    "/behavior_server"
    "/bt_navigator"
    "/waypoint_follower"
    "/velocity_smoother"
    "/collision_monitor"
)

# Section: SLAM Toolbox can expose a lifecycle interface in mapping and SLAM-navigation modes.
SLAM_LIFECYCLE_NODES=(
    "/slam_toolbox"
)

# Section: Lifecycle resources declared by the GPSR scheduler and focused skill launches.
GPSR_LIFECYCLE_NODES=(
    "/count_people_skill_server"
    "/count_objects_skill_server"
    "/gpsr_human_detection_node"
    "/count_people_perception_server"
    "/gpsr_object_detection_node"
    "/count_objects_perception_server"
    "/human_skeleton_detection_node"
    "/gpsr_human_identity_node"
    "/gpsr_tracked_appearance_node"
    "/find_person_by_attribute_skill_server"
    "/extract_person_info_skill_server"
    "/find_object_skill_server"
    "/follow_person_skill_server"
    "/skill_availability_resource"
)

# Section: Optional Receptionist and InsightFace task lifecycle wrappers.
TASK_LIFECYCLE_NODES=(
    "/insightface_camera_switch_lifecycle"
    "/insightface_feature_summary_lifecycle"
    "/task_receptionist_lifecycle"
    "/sm_receptionist_lifecycle"
)

# Section: All reviewed workspace lifecycle names; discovery remains authoritative at runtime.
ALL_REVIEWED_LIFECYCLE_NODES=(
    "${NAV2_LIFECYCLE_NODES[@]}"
    "${SLAM_LIFECYCLE_NODES[@]}"
    "${GPSR_LIFECYCLE_NODES[@]}"
    "${TASK_LIFECYCLE_NODES[@]}"
)

# Section: Ordered dependencies for the dedicated COUNT_OBJECTS helpers.
COUNT_OBJECTS_LIFECYCLE_NODES=(
    "/gpsr_object_detection_node"
    "/count_objects_perception_server"
    "/count_objects_skill_server"
)
