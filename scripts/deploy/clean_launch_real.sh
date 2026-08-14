#!/usr/bin/env bash
# Real-robot launch. Kills stale hardware / Nav2 / IAIG-Nav processes
# then brings up the reactive `real_robot_iaig.launch.py` stack
# (planner_bridge -> goal_manager -> inference_node -> safety_wrapper),
# which matches the main Gazebo IAIG validation path more closely.
#
# Usage:  bash scripts/deploy/clean_launch_real.sh [extra ros2 launch args…]
#
# Common extras:
#   map:=<path>                 use a different SLAM map
#   real_speed_scale:=0.3       peak speed multiplier (default 0.2)
#   tracker:=true               enable pedestrian tracking again
#   vision:=true                enable YOLO camera pipeline again

set +u

_THIS=${BASH_SOURCE[0]:-$0}
ROS2_WS="$(cd "$(dirname "${_THIS}")/../.." && pwd)"

for PAT in \
        robot_state_publisher \
        rplidar_node sllidar \
        realsense2_camera_node rs_launch \
        base_driver cmd_vel_to_targets omni_odometry imu_decimator \
        scan_self_filter ekf_node ekf_filter_node \
        breadcrumb_teach_repeat \
        map_server amcl \
        planner_server controller_server smoother_server behavior_server \
        bt_navigator waypoint_follower velocity_smoother \
        collision_monitor lifecycle_manager \
        nav2_autostart_gate amcl_global_localizer \
        initial_pose_relay initial_pose_publisher \
        bootstrap_lidar_loc bootstrap_pose_promoter \
        semantic_navigator \
        vision_node pedestrian_tracker scan_preprocessor goal_manager \
        inference_node safety_wrapper \
        planner_bridge goal_approach_controller iaig_follow_path_server \
        rviz2 ; do
    pkill -f "${PAT}" >/dev/null 2>&1 || true
done

sleep 2

source "${ROS2_WS}/scripts/deploy/setup_env.sh"

exec ros2 launch iaig_nav_ros real_robot_iaig.launch.py "$@"
