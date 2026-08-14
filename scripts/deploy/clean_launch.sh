#!/usr/bin/env bash
# Gazebo sim launch. Kills stale processes from any previous run, then
# brings up `gazebo_hunav.launch.py`.
#
# Usage:  bash scripts/deploy/clean_launch.sh [extra ros2 launch args…]

set +u

_THIS=${BASH_SOURCE[0]:-$0}
ROS2_WS="$(cd "$(dirname "${_THIS}")/../.." && pwd)"

for PAT in \
        'gz sim' \
        'ruby.*gz' \
        parameter_bridge \
        robot_state_publisher \
        hunav_agent \
        hunav_evaluator \
        inference_node \
        vision_node \
        pedestrian_tracker \
        safety_wrapper \
        scan_preprocessor \
        goal_manager \
        map_server \
        planner_server \
        lifecycle_manager \
        planner_bridge \
        goal_approach_controller \
        rviz2 ; do
    pkill -f "${PAT}" >/dev/null 2>&1 || true
done

sleep 2

source "${ROS2_WS}/scripts/deploy/setup_env.sh"

exec ros2 launch iaig_nav_ros gazebo_hunav.launch.py "$@"
