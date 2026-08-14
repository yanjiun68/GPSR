"""Section: Repeatable command-order validation for GPSR executable plans."""

from __future__ import annotations

import argparse
import itertools
import json
import random
from dataclasses import asdict, dataclass

from gpsr_task_scheduler.config import SchedulerConfig, default_config
from gpsr_task_scheduler.planner import TASK_TO_SKILL, step_to_action_step_dict
from gpsr_task_scheduler.scheduler import schedule_command


# Section: Templates cover every task family that the deterministic parser supports.
COMMAND_TEMPLATES: dict[str, tuple[str, ...]] = {
    "count_people": ("count people", "in the kitchen", "go to the living room", "tell john"),
    "count_objects": ("count coke", "in the kitchen", "go to the living room", "tell john"),
    "find_object": ("find coke", "in the kitchen", "go to the living room", "tell john"),
    "find_person": ("find person", "in the kitchen", "go to the living room", "tell john"),
}


@dataclass(frozen=True, slots=True)
class ValidationReport:
    """Special function: Store the outcome of one repeatable scheduler validation run."""

    seed: int
    requested_random_cases: int
    exhaustive_cases: int
    passed_cases: int
    failures: tuple[str, ...]

    @property
    def valid(self) -> bool:
        """Special function: Indicate whether every generated command produced an executable plan."""
        return not self.failures


def iter_exhaustive_commands() -> list[tuple[str, str]]:
    """Special function: Generate every phrase ordering for each supported task template."""
    # Section: Retain the task-family label to verify parser classification after reordering.
    commands: list[tuple[str, str]] = []
    # Section: Enumerate permutations rather than using a random generator for full coverage.
    for task_type, parts in COMMAND_TEMPLATES.items():
        for permutation in itertools.permutations(parts):
            commands.append((task_type, " ".join(permutation)))
    return commands


def iter_random_commands(seed: int, count: int) -> list[tuple[str, str]]:
    """Special function: Generate reproducible random phrase orderings across task families."""
    # Section: Use a local generator so the caller's random state is never modified.
    generator = random.Random(seed)
    commands: list[tuple[str, str]] = []
    # Section: Sample task templates and shuffle a fresh copy of their command phrases.
    for _ in range(count):
        task_type = generator.choice(sorted(COMMAND_TEMPLATES))
        parts = list(COMMAND_TEMPLATES[task_type])
        generator.shuffle(parts)
        commands.append((task_type, " ".join(parts)))
    return commands


def validate_plan(task_type: str, instruction: str, config: SchedulerConfig) -> list[str]:
    """Special function: Verify a plan can be delivered as an ordered executor action queue."""
    # Section: Ask the public scheduler facade for the same result used by the ROS node.
    result = schedule_command("sequence_validation", instruction, config)
    errors: list[str] = []
    # Section: Reject parser or planner failures with their exact command context.
    if not result.valid:
        return [f"{task_type}: {instruction!r}: invalid result: {[issue.code for issue in result.issues]}"]
    # Section: Verify that phrase ordering did not alter the intended task classification.
    if result.task_request.task_type != task_type:
        errors.append(f"{task_type}: {instruction!r}: parsed as {result.task_request.task_type!r}")
    # Section: Validate deterministic physical task ordering and its dependency chain.
    # Section: Compound FIND_OBJECT owns its initial search navigation.
    expected_skills = (
        ["FIND_OBJECT", "NAVIGATE", "FIND_PERSON", "SAY"]
        if task_type == "find_object"
        else ["NAVIGATE", TASK_TO_SKILL[task_type], "NAVIGATE", "FIND_PERSON", "SAY"]
    )
    actual_skills = [step.skill_name for step in result.steps]
    if actual_skills != expected_skills:
        errors.append(f"{task_type}: {instruction!r}: skills {actual_skills!r}, expected {expected_skills!r}")
    step_ids = [step.step_id for step in result.steps]
    if len(step_ids) != len(set(step_ids)):
        errors.append(f"{task_type}: {instruction!r}: duplicate step identifiers")
    for index, step in enumerate(result.steps):
        expected_dependencies = [] if index == 0 else [result.steps[index - 1].step_id]
        if step.dependencies != expected_dependencies:
            errors.append(f"{task_type}: {instruction!r}: {step.step_id} dependencies {step.dependencies!r}")
        if step.timeout <= 0.0 or step.retry_count < 0:
            errors.append(f"{task_type}: {instruction!r}: {step.step_id} has invalid execution policy")
        # Section: Confirm every symbolic step preserves its payload through the executor adapter.
        action = step_to_action_step_dict(step, result.command_id)
        if action["action_type"] != step.skill_name or action["task_id"] != result.command_id:
            errors.append(f"{task_type}: {instruction!r}: {step.step_id} action conversion changed identity")
        if action["timeout_sec"] != step.timeout or json.loads(action["parameters_json"]) != step.arguments:
            errors.append(f"{task_type}: {instruction!r}: {step.step_id} action conversion changed payload")
    return errors


def run_validation(seed: int = 20260805, random_cases: int = 1000, config: SchedulerConfig | None = None) -> ValidationReport:
    """Special function: Run exhaustive and seeded-random command-order validation."""
    # Section: Default to the same built-in vocabulary used by the package unit tests.
    active_config = config or default_config()
    exhaustive_commands = iter_exhaustive_commands()
    random_commands = iter_random_commands(seed, random_cases)
    failures: list[str] = []
    # Section: Validate both full permutation coverage and additional random command sequences.
    for task_type, instruction in exhaustive_commands + random_commands:
        failures.extend(validate_plan(task_type, instruction, active_config))
    return ValidationReport(seed, random_cases, len(exhaustive_commands), len(exhaustive_commands) + len(random_commands), tuple(failures))


def main() -> None:
    """Special function: Run validation from the terminal and return a machine-readable report."""
    # Section: Expose seed and sample size so failures can be reproduced exactly.
    parser = argparse.ArgumentParser(description="Validate randomized GPSR task-plan phrase orderings.")
    parser.add_argument("--seed", type=int, default=20260805, help="Seed for reproducible random command ordering.")
    parser.add_argument("--random-cases", type=int, default=1000, help="Number of seeded random commands after exhaustive permutations.")
    arguments = parser.parse_args()
    if arguments.random_cases < 0:
        parser.error("--random-cases must be zero or greater")
    report = run_validation(arguments.seed, arguments.random_cases)
    print(json.dumps(asdict(report), indent=2))
    if not report.valid:
        raise SystemExit(1)


if __name__ == "__main__":
    # Section: Allow direct execution during development without ROS hardware.
    main()
