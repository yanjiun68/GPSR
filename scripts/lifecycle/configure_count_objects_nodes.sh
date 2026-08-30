#!/usr/bin/env bash
# Section: Configure the COUNT_OBJECTS lifecycle dependency group in registry order.

set -eo pipefail

# Section: Resolve and source the ROS 2 workspace used by this script.
SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIRECTORY}/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "${PROJECT_ROOT}/install/setup.bash"
set -u

# Section: Keep node order aligned with the COUNT_OBJECTS lifecycle registry.
LIFECYCLE_NODES=(
    "/gpsr_object_detection_node"
    "/count_objects_perception_server"
    "/count_objects_skill_server"
)

# Special function: Read one lifecycle node state and preserve ROS error output.
get_lifecycle_state() {
    local node_name="$1"
    ros2 lifecycle get "${node_name}" 2>&1
}

# Special function: Configure one unconfigured node or accept an already configured node.
configure_node() {
    local node_name="$1"
    local state_output
    state_output="$(get_lifecycle_state "${node_name}")" || {
        echo "ERROR: Cannot query ${node_name}: ${state_output}" >&2
        return 1
    }

    case "${state_output}" in
        *"unconfigured"*)
            echo "Configuring ${node_name}..."
            ros2 lifecycle set "${node_name}" configure
            ;;
        *"inactive"*)
            echo "${node_name} is already configured and inactive."
            ;;
        *"active"*)
            echo "ERROR: ${node_name} is active; deactivate it before configuring." >&2
            return 1
            ;;
        *)
            echo "ERROR: ${node_name} has an unsupported state: ${state_output}" >&2
            return 1
            ;;
    esac

    state_output="$(get_lifecycle_state "${node_name}")"
    if [[ "${state_output}" != *"inactive [2]"* ]]; then
        echo "ERROR: ${node_name} did not reach inactive: ${state_output}" >&2
        return 1
    fi
    echo "Verified ${node_name}: ${state_output}"
}

# Section: Configure detector, perception service, then GPSR action server.
for node_name in "${LIFECYCLE_NODES[@]}"; do
    configure_node "${node_name}"
done

echo "COUNT_OBJECTS lifecycle nodes are configured and inactive."
