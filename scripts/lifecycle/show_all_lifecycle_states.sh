#!/usr/bin/env bash

# Show the state of every ROS 2 lifecycle node currently visible.
#
# Usage:
#   ./show_all_lifecycle_states.sh
#
# Optional:
#   ./show_all_lifecycle_states.sh --watch
#   ./show_all_lifecycle_states.sh --require-active
#   ./show_all_lifecycle_states.sh --known
#
# Exit status:
#   0 = all discovered lifecycle nodes are in healthy primary states
#   1 = one or more nodes are transitional, finalized, or unreachable
#   2 = no lifecycle nodes found / ROS 2 unavailable

# Section: Resolve and source the workspace so direct execution uses the current overlay.
SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIRECTORY}/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "${PROJECT_ROOT}/install/setup.bash"
# Section: Import the reviewed inventory used by the explicit --known audit mode.
source "${SCRIPT_DIRECTORY}/lifecycle_nodes.sh"

# Section: Enable unset-variable checking only after ROS setup has initialized its optional variables.
set -u

WATCH_MODE=false
REQUIRE_ACTIVE=false
KNOWN_MODE=false
INTERVAL=1

# Section: Parse monitoring options without treating healthy idle nodes as failures.
for argument in "$@"; do
    case "$argument" in
        --watch)
            WATCH_MODE=true
            ;;
        --require-active)
            REQUIRE_ACTIVE=true
            ;;
        --known)
            KNOWN_MODE=true
            ;;
        *)
            echo "ERROR: unknown option: $argument"
            exit 2
            ;;
    esac
done

# ANSI colors
GREEN="\033[0;32m"
YELLOW="\033[0;33m"
RED="\033[0;31m"
CYAN="\033[0;36m"
GRAY="\033[0;90m"
RESET="\033[0m"

check_ros2() {
    if ! command -v ros2 >/dev/null 2>&1; then
        echo "ERROR: ros2 command not found."
        echo "Source ROS 2 and your workspace first."
        return 2
    fi
}

# Special function: Print every reviewed lifecycle name exactly once in lexical order.
reviewed_lifecycle_nodes() {
    printf '%s\n' "${ALL_REVIEWED_LIFECYCLE_NODES[@]}" | sort -u
}

state_color() {
    local state="$1"

    case "$state" in
        active*)
            printf "%b" "$GREEN"
            ;;
        inactive*)
            printf "%b" "$YELLOW"
            ;;
        unconfigured*)
            printf "%b" "$YELLOW"
            ;;
        not_running*)
            printf "%b" "$GRAY"
            ;;
        finalized*)
            printf "%b" "$RED"
            ;;
        error*|unknown*|unreachable*)
            printf "%b" "$RED"
            ;;
        *)
            printf "%b" "$CYAN"
            ;;
    esac
}

show_states() {
    local all_healthy=true
    local all_active=true

    # Section: Always discover first so optional known nodes never incur an unavailable-service timeout.
    mapfile -t discovered_nodes < <(
        ros2 lifecycle nodes 2>/dev/null \
        | sed '/^[[:space:]]*$/d' \
        | sort
    )

    # Section: Runtime discovery covers every running node; --known audits all reviewed launch declarations.
    if [[ "$KNOWN_MODE" == true ]]; then
        mapfile -t nodes < <(reviewed_lifecycle_nodes)
    else
        nodes=("${discovered_nodes[@]}")
    fi

    echo
    printf "%-55s %-24s\n" "NODE" "LIFECYCLE STATE"
    printf "%-55s %-24s\n" \
        "-------------------------------------------------------" \
        "------------------------"

    if [[ ${#nodes[@]} -eq 0 ]]; then
        printf "${RED}No lifecycle nodes available for this audit.${RESET}\n"
        return 2
    fi

    local active_count=0
    local inactive_count=0
    local unconfigured_count=0
    local finalized_count=0
    local not_running_count=0
    local other_count=0

    for node in "${nodes[@]}"; do

        # Section: Known-but-optional nodes are reported without querying a service that is not present.
        if [[ "$KNOWN_MODE" == true ]] && ! printf '%s\n' "${discovered_nodes[@]}" | grep -Fxq "$node"; then
            state="not_running"
            all_active=false
            ((not_running_count++))
        else
            # Section: Prevent one broken discovered lifecycle service from blocking the script for more than five seconds.
            state="$(
                timeout 5 ros2 lifecycle get "$node" 2>/dev/null
            )"

            rc=$?

            if [[ $rc -ne 0 || -z "$state" ]]; then
                state="unreachable"
                all_healthy=false
                all_active=false
                ((other_count++))
            else
                case "$state" in
                    active*)
                        ((active_count++))
                        ;;
                    inactive*)
                        ((inactive_count++))
                        all_active=false
                        ;;
                    unconfigured*)
                        ((unconfigured_count++))
                        all_active=false
                        ;;
                    finalized*)
                        ((finalized_count++))
                        all_healthy=false
                        all_active=false
                        ;;
                    *)
                        ((other_count++))
                        all_healthy=false
                        all_active=false
                        ;;
                esac
            fi
        fi

        color="$(state_color "$state")"

        printf "%-55s ${color}%-24s${RESET}\n" \
            "$node" \
            "$state"
    done

    echo
    echo "Summary:"
    printf "  ${GREEN}Active        : %d${RESET}\n" "$active_count"
    printf "  ${YELLOW}Inactive      : %d${RESET}\n" "$inactive_count"
    printf "  ${YELLOW}Unconfigured  : %d${RESET}\n" "$unconfigured_count"
    printf "  ${RED}Finalized     : %d${RESET}\n" "$finalized_count"
    printf "  ${GRAY}Not running   : %d${RESET}\n" "$not_running_count"
    printf "  ${RED}Other/Error   : %d${RESET}\n" "$other_count"
    echo "  Total         : ${#nodes[@]}"

    echo

    if [[ "$REQUIRE_ACTIVE" == true && "$all_active" != true ]]; then
        printf "${RED}LIFECYCLE STATUS: NOT ALL NODES ACTIVE${RESET}\n"
        return 1
    elif [[ "$all_healthy" == true ]]; then
        printf "${GREEN}LIFECYCLE STATUS: HEALTHY (ACTIVE OR IDLE)${RESET}\n"
        return 0
    else
        printf "${RED}LIFECYCLE STATUS: ERROR OR UNREACHABLE${RESET}\n"
        return 1
    fi
}

main() {
    check_ros2 || exit $?

    if [[ "$WATCH_MODE" == true ]]; then
        while true; do
            clear
            echo "ROS 2 Lifecycle Monitor"
            echo "Refresh interval: ${INTERVAL}s"
            echo "Press Ctrl+C to exit."

            show_states
            sleep "$INTERVAL"
        done
    else
        show_states
        exit $?
    fi
}

main


#Do not ever shutdown any nodes, only deactivate then clean up. 

# If shutdown , a process id deep kill is required. 
#pkill -f 
#kill -9 
