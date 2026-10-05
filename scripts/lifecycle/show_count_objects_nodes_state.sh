#!/usr/bin/env bash
# Section: Display the current state of every COUNT_OBJECTS lifecycle node.

set -eo pipefail

# Section: Resolve and source the ROS 2 workspace used by this script.
SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIRECTORY}/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "${PROJECT_ROOT}/install/setup.bash"
# Section: Import the reviewed lifecycle inventory and COUNT_OBJECTS dependency order.
source "${SCRIPT_DIRECTORY}/lifecycle_nodes.sh"
set -u

# Section: Keep displayed nodes aligned with the reviewed COUNT_OBJECTS lifecycle registry.
LIFECYCLE_NODES=("${COUNT_OBJECTS_LIFECYCLE_NODES[@]}")

# Special function: Query one node and print a stable name-to-state row.
show_node_state() {
    local node_name="$1"
    local state_output
    state_output="$(ros2 lifecycle get "${node_name}" 2>&1)" || {
        printf '%-42s %s\n' "${node_name}" "UNAVAILABLE (${state_output})"
        return 1
    }
    printf '%-42s %s\n' "${node_name}" "${state_output}"
    [[ "${state_output}" == *"active [3]"* ]]
}

# Section: Print every state while retaining an overall pipeline readiness result.
printf '%-42s %s\n' "NODE" "LIFECYCLE STATE"
printf '%-42s %s\n' "------------------------------------------" "----------------"
all_active=true
for node_name in "${LIFECYCLE_NODES[@]}"; do
    if ! show_node_state "${node_name}"; then
        all_active=false
    fi
done

# Section: Summarize whether the complete dependency chain can process requests.
if [[ "${all_active}" == true ]]; then
    echo "COUNT_OBJECTS pipeline status: READY (all nodes active)."
    exit 0
fi

echo "COUNT_OBJECTS pipeline status: NOT READY (one or more nodes are not active)."
exit 1
