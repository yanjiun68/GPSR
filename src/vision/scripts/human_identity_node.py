#!/usr/bin/env -S bash -c 'exec "${VISION_PYTHON:-${HOME}/ai/qwen3v1/.venv/bin/python}" "$0" "$@"'
"""Section: Executable entry point for the standalone human identity node."""

from human_identity.human_identity.node import main


if __name__ == "__main__":
    main()
