"""Section: Hardware-free GPSR canonicalization and queue inspection CLI."""

from __future__ import annotations

import argparse
import json

from gpsr_task_scheduler.config import load_config
from gpsr_task_scheduler.planner import plan_to_dict
from gpsr_task_scheduler.scheduler import schedule_command


def main(argv: list[str] | None = None) -> int:
    """Special function: Print every deterministic processing stage without dispatching."""
    parser = argparse.ArgumentParser(
        description="Canonicalize and compile one GPSR command without robot execution."
    )
    parser.add_argument("command", help="Raw ASR/operator command text.")
    parser.add_argument(
        "--normalized",
        default=None,
        help="Optional externally normalized text retained separately from raw input.",
    )
    parser.add_argument("--config", default=None, help="Optional scheduler YAML path.")
    args = parser.parse_args(argv)

    # Section: Pure-Python compilation intentionally creates no ROS node or action client.
    result = schedule_command(
        "dry_run",
        args.command,
        load_config(args.config),
        normalized_instruction=args.normalized,
    )
    payload = plan_to_dict(result)
    context = payload.get("command_context") or {}
    sections = [
        ("RAW", context.get("raw_text", "")),
        ("NORMALIZED", context.get("normalized_text", "")),
        ("CLAUSES", context.get("clauses", [])),
        ("ENTITIES", context.get("entities", {})),
        ("SEMANTIC PLAN", context.get("semantic_steps", [])),
        ("FINAL ACTION QUEUE", context.get("action_queue", [])),
        ("VALIDATION", context.get("validation", {})),
    ]
    for title, value in sections:
        print(f"{title}:")
        print(json.dumps(value, indent=2, sort_keys=True))
    return 0 if result.valid else 2


if __name__ == "__main__":
    raise SystemExit(main())
