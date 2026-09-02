#!/usr/bin/env bash
set -u

echo '=== ROS DISTRO ==='
echo "${ROS_DISTRO:-not sourced}"

echo
echo '=== NODES ==='
ros2 node list 2>/dev/null || true

echo
echo '=== TOPICS + TYPES ==='
ros2 topic list -t 2>/dev/null || true

echo
echo '=== LIFECYCLE STATES ==='
while IFS= read -r node; do
  [[ -z "$node" ]] && continue
  if ros2 service list 2>/dev/null | grep -qx "${node%/}/get_state"; then
    printf '%-55s ' "$node"
    timeout 2 ros2 lifecycle get "$node" 2>/dev/null || echo 'unavailable'
  fi
done < <(ros2 node list 2>/dev/null)

echo
echo '=== ACTIONS + TYPES ==='
ros2 action list -t 2>/dev/null || true

echo
echo '=== SERVICES + TYPES ==='
ros2 service list -t 2>/dev/null || true
