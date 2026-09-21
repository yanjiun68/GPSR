#!/usr/bin/env -S bash -c 'exec "${QWEN_PYTHON:-/home/yanjiun68/ai/qwen3v1/.venv/bin/python}" "$0" "$@"'
"""ROS executable for the persistent device-local Qwen text planner."""

# Section: Keep the executable thin so CLI and ROS paths share the installed implementation.
from qwen_vlm_bridge.node import main


if __name__ == "__main__":
    # Section: Enter the ROS node only when invoked as an executable.
    main()
