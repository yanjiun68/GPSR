#!/usr/bin/env -S bash -c 'exec "${QWEN_PYTHON:-/home/yanjiun68/ai/qwen3v1/.venv/bin/python}" "$0" "$@"'
"""Command-line compatibility frontend for file-based or direct-text Qwen tests."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from qwen_vlm_bridge.contract import DEFAULT_ALLOWED_SKILLS
from qwen_vlm_bridge.engine import QwenPlannerEngine


def _read_message(message_path: str) -> str:
    """Special function: Read and validate a legacy message file."""
    # Section: Preserve the original test_text.py file mode for regression testing.
    path = Path(message_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"message file does not exist: {path}")
    text = path.read_text(encoding="utf-8").strip()
    if not text:
        raise ValueError(f"message file is empty: {path}")
    return text


def main() -> None:
    """Special function: Run one Qwen request from --text or the legacy --message file."""
    # Section: Accept direct text for prototyping while retaining backwards-compatible file input.
    parser = argparse.ArgumentParser(description="Run one local Qwen semantic planning request.")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--text", help="Direct user instruction.")
    source.add_argument("--message", help="Path to a user-message text file.")
    parser.add_argument(
        "--system",
        default="/home/yanjiun68/ai/qwen3v1/system_prompt_actionqueue_test.txt",
        help="Path to the Qwen system-prompt file.",
    )
    parser.add_argument(
        "--model",
        default="/home/yanjiun68/ai/models/Qwen3-VL-4B-Instruct",
        help="Path to local Qwen model weights.",
    )
    parser.add_argument("--max-new-tokens", type=int, default=256)
    parser.add_argument("--no-4bit", action="store_true")
    args = parser.parse_args()

    # Section: Resolve direct text before the expensive model is allocated.
    instruction = str(args.text).strip() if args.text is not None else _read_message(args.message)
    if not instruction:
        raise SystemExit("instruction is empty")

    # Section: Use exactly the same engine and validation path as the ROS node.
    engine = QwenPlannerEngine(
        model_path=args.model,
        system_prompt_path=args.system,
        max_new_tokens=args.max_new_tokens,
        load_in_4bit=not args.no_4bit,
        local_files_only=True,
        allowed_skills=DEFAULT_ALLOWED_SKILLS,
    )
    engine.load()
    plan, _raw_response = engine.generate_plan(instruction)
    print(json.dumps(plan, indent=2, ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
    # Section: Execute one-shot inference only when called from the command line.
    main()
