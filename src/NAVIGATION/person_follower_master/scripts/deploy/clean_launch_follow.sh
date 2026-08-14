#!/usr/bin/env bash
# Person-follower launch. Kills any stale hardware / IAIG-Nav / follow
# processes, sources the workspace, and brings up the single-master
# person-following stack.
#
# Usage:  bash scripts/deploy/clean_launch_follow.sh [extra ros2 launch args…]

set +u

_THIS=${BASH_SOURCE[0]:-$0}
# scripts/deploy/clean_launch_follow.sh lives under src/person_follower_master/,
# so the workspace root is five levels up.
ROS2_WS="$(cd "$(dirname "${_THIS}")/../../../.." && pwd)"

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
        reid_enricher_node master_tracker_node follow_controller_node \
        rviz2 ; do
    pkill -f "${PAT}" >/dev/null 2>&1 || true
done

sleep 2

# Auto-select a conda env that actually exists. setup_env.sh defaults to
# ~/miniconda3/envs/iaig_nav, but this host may instead carry the deps
# (torch, torchreid, ultralytics, …) under `robocup` or another env. We
# pick the first one that has a site-packages directory, unless the user
# already exported CONDA_ENV_IAIG_NAV.
if [[ -z "${CONDA_ENV_IAIG_NAV:-}" ]]; then
    for _env in iaig_nav robocup; do
        _cand="${HOME}/miniconda3/envs/${_env}"
        if [[ -d "${_cand}/lib/python3.10/site-packages" ]]; then
            export CONDA_ENV_IAIG_NAV="${_cand}"
            echo "[clean_launch_follow] using conda env: ${_cand}"
            break
        fi
    done
fi

source "${ROS2_WS}/scripts/deploy/setup_env.sh"

# Ensure user-local pip installs (torch, scipy, ultralytics, …) are
# visible even when running under bash -l, which some shells strip.
_USER_SITE="${HOME}/.local/lib/python3.10/site-packages"
if [[ -d "${_USER_SITE}" ]]; then
    case ":${PYTHONPATH:-}:" in
        *":${_USER_SITE}:"*) ;;
        *) export PYTHONPATH="${_USER_SITE}${PYTHONPATH:+:$PYTHONPATH}" ;;
    esac
fi

exec ros2 launch person_follower_master person_follow.launch.py "$@"
