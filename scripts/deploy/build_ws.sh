#!/usr/bin/env bash
# Clean-env colcon build for the IAIG-Nav ros2_ws.
# Keeps conda's python off PATH (conda bin breaks colcon).
set -euo pipefail

_THIS=${BASH_SOURCE[0]:-$0}
ROS2_WS="$(cd -L "$(dirname "${_THIS}")/../.." && pwd -L)"
cd "${ROS2_WS}"

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

env_args=(
    "HOME=$HOME"
    "USER=$USER"
    "PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    "TERM=${TERM:-xterm}"
)

# Preserve UTF-8 locale for workspaces under non-ASCII paths.
for locale_var in LANG LANGUAGE LC_ALL LC_CTYPE LC_NUMERIC LC_TIME LC_COLLATE LC_MONETARY LC_MESSAGES LC_PAPER LC_NAME LC_ADDRESS LC_TELEPHONE LC_MEASUREMENT LC_IDENTIFICATION; do
    if [[ -n "${!locale_var:-}" ]]; then
        env_args+=("${locale_var}=${!locale_var}")
    fi
done

# Keep conda's Python packages visible without putting conda's bin on PATH.
if [[ -d "${CONDA_SITE}" ]]; then
    env_args+=("PYTHONPATH=${CONDA_SITE}")
fi

colcon_global_args=()
colcon_build_args=(
    --symlink-install
    --continue-on-error
    --cmake-clean-cache
    --cmake-force-configure
)

# Keep the build tree on an ASCII-only path when the workspace itself
# contains non-ASCII characters. This avoids ROSIDL path mangling.
if printf '%s' "${ROS2_WS}" | LC_ALL=C grep -q '[^ -~]'; then
    ASCII_TMP_ROOT="/tmp/ros2_ws_build_${UID}"
    colcon_global_args+=(--log-base "${ASCII_TMP_ROOT}/log")
    colcon_build_args+=(--build-base "${ASCII_TMP_ROOT}/build")
    echo "[build_ws] note: using ${ASCII_TMP_ROOT} for colcon build/log because ${ROS2_WS} contains non-ASCII characters" >&2
fi

printf -v colcon_global_q '%q ' "${colcon_global_args[@]}"
printf -v colcon_build_q '%q ' "${colcon_build_args[@]}"

env -i "${env_args[@]}" \
    bash -lc "source /opt/ros/humble/setup.bash && \
              colcon ${colcon_global_q}build ${colcon_build_q}"
