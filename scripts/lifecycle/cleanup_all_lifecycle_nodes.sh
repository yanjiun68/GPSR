#!/usr/bin/env bash
# Section: Deactivate and clean every currently discoverable ROS 2 lifecycle node, one node at a time.
#
# Usage:
#   ./cleanup_all_lifecycle_nodes.sh
#
# Exit status:
#   0 = every discovered node is unconfigured or was cleaned successfully
#   1 = one or more nodes could not be queried or cleaned
#   2 = ROS 2 is unavailable or no lifecycle nodes were discovered
#
# This script intentionally never sends the shutdown transition.  Cleanup leaves
# the node process running and returns its lifecycle state to unconfigured.

# Section: Enable strict handling while preserving per-node failures for the final summary.
set -uo pipefail

# Section: Resolve and source the ROS 2 workspace used by this script.
SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIRECTORY}/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "${PROJECT_ROOT}/install/setup.bash"

# Section: Bound each lifecycle CLI operation so one unavailable service cannot stall the batch.
LIFECYCLE_TIMEOUT_SECONDS=15

# Special function: Print an error message to standard error.
error() {
    echo "ERROR: $*" >&2
}

# Special function: Read one lifecycle node state with a bounded service call.
get_lifecycle_state() {
    local node_name="$1"
    timeout "${LIFECYCLE_TIMEOUT_SECONDS}" ros2 lifecycle get "${node_name}" 2>&1
}

# Special function: Request one named lifecycle transition with a bounded service call.
set_lifecycle_state() {
    local node_name="$1"
    local transition="$2"
    timeout "${LIFECYCLE_TIMEOUT_SECONDS}" ros2 lifecycle set "${node_name}" "${transition}" 2>&1
}

# Special function: Return success only when a node reports the requested primary lifecycle state.
has_lifecycle_state() {
    local state_output="$1"
    local expected_state="$2"
    [[ "${state_output}" == *"${expected_state} ["* ]]
}

# Special function: Deactivate, when required, then clean one node without ever shutting it down.
cleanup_node() {
    local node_name="$1"
    local state_output

    if ! state_output="$(get_lifecycle_state "${node_name}")"; then
        error "Cannot query ${node_name}: ${state_output}"
        return 1
    fi

    if has_lifecycle_state "${state_output}" "active"; then
        echo "Deactivating ${node_name}..."
        if ! set_lifecycle_state "${node_name}" deactivate; then
            error "Failed to deactivate ${node_name}."
            return 1
        fi

        if ! state_output="$(get_lifecycle_state "${node_name}")" || ! has_lifecycle_state "${state_output}" "inactive"; then
            error "${node_name} did not reach inactive: ${state_output}"
            return 1
        fi
    elif has_lifecycle_state "${state_output}" "inactive"; then
        echo "${node_name} is already inactive."
    elif has_lifecycle_state "${state_output}" "unconfigured"; then
        echo "${node_name} is already unconfigured."
        return 0
    else
        error "${node_name} is not in a cleanable primary state: ${state_output}"
        return 1
    fi

    echo "Cleaning up ${node_name}..."
    if ! set_lifecycle_state "${node_name}" cleanup; then
        error "Failed to clean up ${node_name}."
        return 1
    fi

    if ! state_output="$(get_lifecycle_state "${node_name}")" || ! has_lifecycle_state "${state_output}" "unconfigured"; then
        error "${node_name} did not reach unconfigured: ${state_output}"
        return 1
    fi

    echo "Cleaned ${node_name}: ${state_output}"
}

# Section: Ensure the ROS 2 CLI is available before attempting discovery.
if ! command -v ros2 >/dev/null 2>&1; then
    error "ros2 command not found after sourcing ROS 2 and the workspace."
    exit 2
fi

# Section: Discover lifecycle nodes and reverse their lexical order for dependency-friendly teardown.
# Runtime discovery is deliberate: it includes every lifecycle node visible in this ROS domain,
# including optional Nav2, GPSR, and Receptionist nodes that are not active in every launch mode.
mapfile -t LIFECYCLE_NODES < <(
    ros2 lifecycle nodes 2>/dev/null \
        | sed '/^[[:space:]]*$/d' \
        | sort -r
)

# Section: Stop early when there is no batch to clean.
if [[ ${#LIFECYCLE_NODES[@]} -eq 0 ]]; then
    error "No lifecycle nodes discovered."
    exit 2
fi

# Section: Process every discovered node sequentially and report all failures together.
failed_nodes=()
for node_name in "${LIFECYCLE_NODES[@]}"; do
    echo
    echo "Processing ${node_name}..."
    if ! cleanup_node "${node_name}"; then
        failed_nodes+=("${node_name}")
    fi
done

# Section: Return a nonzero status only after every eligible node has received a cleanup attempt.
if [[ ${#failed_nodes[@]} -gt 0 ]]; then
    error "Cleanup failed for ${#failed_nodes[@]} node(s): ${failed_nodes[*]}"
    exit 1
fi

echo
echo "All ${#LIFECYCLE_NODES[@]} lifecycle nodes are unconfigured."
