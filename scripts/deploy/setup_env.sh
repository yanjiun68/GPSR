#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# Source this file to prepare a shell for launching the IAIG-Nav ROS 2 stack.
#
#     source <ros2_ws>/scripts/deploy/setup_env.sh
#
# Self-contained: does NOT assume the outer IAIG-NAV repo is present. The
# only requirement is that `ros2_ws/` is next to (or contains) this script,
# and that the conda env `iaig_nav` (or a Python env exposing torch,
# torch_geometric, ultralytics, opencv, scipy, cv_bridge) is installed
# on the host.
#
# What it does:
#   1) Sources ROS 2 Humble.
#   2) Sources the colcon overlay at <ros2_ws>/install.
#   3) Bridges the conda env site-packages into PYTHONPATH without
#      activating conda (conda's bin on PATH breaks colcon).
#   4) Adds NVIDIA cuda libs shipped in the conda env to LD_LIBRARY_PATH.
# -----------------------------------------------------------------------------
# Do NOT `set -u` — /opt/ros/humble setup.bash references unset env vars
# (e.g. AMENT_TRACE_SETUP_FILES) and nounset aborts sourcing.
: "${COLCON_TRACE:=}"
export COLCON_TRACE

# Resolve ros2_ws root from this script's location (ros2_ws/scripts/deploy/X.sh)
_THIS=${BASH_SOURCE[0]:-$0}
ROS2_WS="$(cd "$(dirname "${_THIS}")/../.." && pwd)"
export ROS2_WS

for _candidate in \
    "${CONDA_ENV_IAIG_NAV:-}" \
    "${CONDA_PREFIX:-}" \
    "${HOME}/miniconda3/envs/iaig_nav"
do
    if [[ -n "${_candidate}" && -d "${_candidate}" ]]; then
        CONDA_ENV="${_candidate}"
        break
    fi
done
: "${CONDA_ENV:=${HOME}/miniconda3/envs/iaig_nav}"
CONDA_SITE="${CONDA_ENV}/lib/python3.10/site-packages"

# 1) ROS 2 Humble
if [[ -z "${ROS_DISTRO:-}" ]]; then
    source /opt/ros/humble/setup.bash
fi

# 2) colcon overlay
if [[ -f "${ROS2_WS}/install/setup.bash" ]]; then
    source "${ROS2_WS}/install/setup.bash"
else
    echo "[setup_env] warn: ${ROS2_WS}/install not found — run build_ws.sh first" >&2
fi

# 3) conda site-packages on PYTHONPATH (torch stack)
if [[ -d "${CONDA_SITE}" ]]; then
    case ":${PYTHONPATH:-}:" in
        *":${CONDA_SITE}:"*) ;;
        *) export PYTHONPATH="${CONDA_SITE}${PYTHONPATH:+:$PYTHONPATH}" ;;
    esac
fi

# 4) NVIDIA CUDA libs from conda wheels
NVIDIA_LIB="${CONDA_SITE}/nvidia"
if [[ -d "${NVIDIA_LIB}" ]]; then
    for sub in ${NVIDIA_LIB}/*/lib; do
        case ":${LD_LIBRARY_PATH:-}:" in
            *":${sub}:"*) ;;
            *) export LD_LIBRARY_PATH="${sub}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;;
        esac
    done
fi

set +u
